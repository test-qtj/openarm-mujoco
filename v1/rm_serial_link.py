"""RoboMaster 2027 0x0311 serial framing for simulated joint positions.

The 0x0311 command carries a 60-byte CustomControl protobuf. Its ``data``
field contains this project's compact joint packet: magic, version, sequence,
and eight little-endian float32 joint positions.
"""

from __future__ import annotations

import struct
import threading
import time

import serial

SOF = 0xA5
CMD_CUSTOM_CONTROL = 0x0311
CUSTOM_DATA_SIZE = 60
PROTO_DATA_SIZE = 58
APP_MAGIC = b"OA"
APP_VERSION = 1
APP_STRUCT = struct.Struct("<2sBH8f")
BAUDRATE = 921600


def crc8(data: bytes, initial: int = 0xFF) -> int:
    crc = initial
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ 0x8C if crc & 1 else crc >> 1
    return crc


def crc16(data: bytes, initial: int = 0xFFFF) -> int:
    crc = initial
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ 0x8408 if crc & 1 else crc >> 1
    return crc & 0xFFFF


def encode_custom_control(sequence: int, positions: tuple[float, ...] | list[float]) -> bytes:
    """Serialize the official CustomControl protobuf with this sim's joint payload."""
    if len(positions) != 8:
        raise ValueError("Expected eight right-arm joint positions")
    app_data = APP_STRUCT.pack(APP_MAGIC, APP_VERSION, sequence & 0xFFFF, *positions)
    app_data = app_data.ljust(PROTO_DATA_SIZE, b"\0")
    # CustomControl protobuf v3: optional bytes data = 1. Its value is 58 bytes,
    # making the serialized protobuf exactly the 60 bytes carried by command 0x0311.
    return bytes((0x0A, PROTO_DATA_SIZE)) + app_data


def _serial_frame(sequence: int, command: int, payload: bytes) -> bytes:
    command_data = struct.pack("<H", command) + payload
    header = struct.pack("<BHB", SOF, len(command_data), sequence & 0xFF)
    header += bytes((crc8(header),))
    packet = header + command_data
    return packet + struct.pack("<H", crc16(packet))


def encode_joint_frame(sequence: int, positions: tuple[float, ...] | list[float]) -> bytes:
    custom_control = encode_custom_control(sequence, positions)
    return _serial_frame(sequence, CMD_CUSTOM_CONTROL, custom_control)


def _decode_custom_control(payload: bytes) -> tuple[int, tuple[float, ...]] | None:
    if len(payload) != CUSTOM_DATA_SIZE or payload[:2] != bytes((0x0A, PROTO_DATA_SIZE)):
        return None
    try:
        values = APP_STRUCT.unpack_from(payload, 2)
    except struct.error:
        return None
    magic, version, sequence, *positions = values
    if magic != APP_MAGIC or version != APP_VERSION:
        return None
    return sequence, tuple(positions)


def decode_custom_control(payload: bytes) -> tuple[int, tuple[float, ...]] | None:
    """Decode and validate this simulation's CustomControl payload."""
    return _decode_custom_control(payload)


class RoboMasterFrameParser:
    """Incrementally parse and CRC-check referee serial frames."""

    def __init__(self) -> None:
        self._buffer = bytearray()
        self.crc_errors = 0

    def feed(self, incoming: bytes) -> list[tuple[int, tuple[float, ...]]]:
        self._buffer.extend(incoming)
        decoded = []
        while True:
            try:
                start = self._buffer.index(SOF)
            except ValueError:
                self._buffer.clear()
                break
            if start:
                del self._buffer[:start]
            if len(self._buffer) < 5:
                break
            data_length = int.from_bytes(self._buffer[1:3], "little")
            frame_size = 5 + data_length + 2
            if data_length < 2 or data_length > 512:
                del self._buffer[0]
                self.crc_errors += 1
                continue
            if len(self._buffer) < frame_size:
                break
            candidate = bytes(self._buffer[:frame_size])
            if crc8(candidate[:4]) != candidate[4]:
                del self._buffer[0]
                self.crc_errors += 1
                continue
            expected_crc = int.from_bytes(candidate[-2:], "little")
            if crc16(candidate[:-2]) != expected_crc:
                del self._buffer[0]
                self.crc_errors += 1
                continue
            command = int.from_bytes(candidate[5:7], "little")
            if command == CMD_CUSTOM_CONTROL:
                joint_packet = _decode_custom_control(candidate[7:-2])
                if joint_packet is not None:
                    decoded.append(joint_packet)
            del self._buffer[:frame_size]
        return decoded


class SerialJointWriter:
    def __init__(self, port: str) -> None:
        self.port = serial.Serial(
            port, BAUDRATE, bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE, timeout=0, write_timeout=0.1,
            xonxoff=False, rtscts=False, dsrdtr=False,
        )
        self.sequence = 0

    def send(self, positions: tuple[float, ...] | list[float]) -> None:
        payload = encode_custom_control(self.sequence, positions)
        self.send_custom_control(payload)

    def send_custom_control(self, payload: bytes) -> None:
        if _decode_custom_control(payload) is None:
            raise ValueError("Invalid CustomControl payload for simulated joint positions")
        self.port.write(_serial_frame(self.sequence & 0xFF, CMD_CUSTOM_CONTROL, payload))
        self.sequence = (self.sequence + 1) & 0xFFFF

    def close(self) -> None:
        self.port.close()


class SerialJointReader:
    def __init__(self, port: str) -> None:
        self.port = serial.Serial(
            port, BAUDRATE, bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE, timeout=0, write_timeout=0.1,
            xonxoff=False, rtscts=False, dsrdtr=False,
        )
        self.parser = RoboMasterFrameParser()
        self.latest_positions: tuple[float, ...] | None = None
        self.latest_time = 0.0
        self.error: Exception | None = None
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._read_loop, name="rm-serial-link", daemon=True)
        self._thread.start()

    def _read_loop(self) -> None:
        try:
            while not self._stop.is_set():
                chunk = self.port.read(4096)
                for _, positions in self.parser.feed(chunk):
                    self.latest_positions = positions
                    self.latest_time = time.monotonic()
                if not chunk:
                    self._stop.wait(0.002)
        except Exception as exc:
            self.error = exc

    def close(self) -> None:
        self._stop.set()
        self._thread.join(timeout=1)
        self.port.close()
