"""Read and decode VT03/VT13 remote-control frames over a serial port."""

from __future__ import annotations

import argparse
import threading
import time
from dataclasses import dataclass

FRAME_SIZE = 21
FRAME_HEADER = b"\xA9\x53"
BAUDRATE = 921600
CHANNEL_MIN = 364
CHANNEL_CENTER = 1024
CHANNEL_MAX = 1684


def _bits(data: bytes, offset: int, width: int) -> int:
    """Extract a little-endian bit field using the offsets in the VT13 guide."""
    value = 0
    for index in range(width):
        bit = offset + index
        value |= ((data[bit // 8] >> (bit % 8)) & 1) << index
    return value


def crc16_vt13(data: bytes, initial: int = 0xFFFF) -> int:
    """Calculate the CRC used by DJI's official VT03/VT13 sample code."""
    crc = initial
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ 0x8408 if crc & 1 else crc >> 1
    return crc & 0xFFFF


@dataclass(frozen=True, slots=True)
class VT13Frame:
    channels: tuple[int, int, int, int]
    mode_switch: int
    pause: int
    fn1: int
    fn2: int
    wheel: int
    trigger: int
    mouse_x: int
    mouse_y: int
    mouse_z: int
    mouse_left: int
    mouse_right: int
    mouse_middle: int
    keyboard: int

    @classmethod
    def decode(cls, frame: bytes) -> VT13Frame:
        if len(frame) != FRAME_SIZE or frame[:2] != FRAME_HEADER:
            raise ValueError("A VT13 frame must be 21 bytes and start with A9 53")

        expected_crc = crc16_vt13(frame[:-2])
        received_crc = int.from_bytes(frame[-2:], byteorder="little")
        if expected_crc != received_crc:
            raise ValueError(
                f"VT13 CRC mismatch: expected {expected_crc:04x}, got {received_crc:04x}"
            )

        def signed16(offset: int) -> int:
            value = _bits(frame, offset, 16)
            return value - 0x10000 if value & 0x8000 else value

        return cls(
            channels=(
                _bits(frame, 16, 11),
                _bits(frame, 27, 11),
                _bits(frame, 38, 11),
                _bits(frame, 49, 11),
            ),
            mode_switch=_bits(frame, 60, 2),
            pause=_bits(frame, 62, 1),
            fn1=_bits(frame, 63, 1),
            fn2=_bits(frame, 64, 1),
            wheel=_bits(frame, 65, 11),
            trigger=_bits(frame, 76, 1),
            mouse_x=signed16(80),
            mouse_y=signed16(96),
            mouse_z=signed16(112),
            mouse_left=_bits(frame, 128, 2),
            mouse_right=_bits(frame, 130, 2),
            mouse_middle=_bits(frame, 132, 2),
            keyboard=_bits(frame, 136, 16),
        )


class VT13FrameParser:
    """Synchronize on A9 53, validate CRC, and emit complete 21-byte frames."""

    def __init__(self) -> None:
        self._buffer = bytearray()
        self.crc_errors = 0

    def feed(self, data: bytes) -> list[VT13Frame]:
        self._buffer.extend(data)
        frames: list[VT13Frame] = []

        while True:
            header_index = self._buffer.find(FRAME_HEADER)
            if header_index < 0:
                # Keep a possible first header byte across USB read boundaries.
                keep = 1 if self._buffer.endswith(FRAME_HEADER[:1]) else 0
                if keep:
                    self._buffer[:] = self._buffer[-1:]
                else:
                    self._buffer.clear()
                break

            if header_index:
                del self._buffer[:header_index]
            if len(self._buffer) < FRAME_SIZE:
                break

            candidate = bytes(self._buffer[:FRAME_SIZE])
            try:
                parsed = VT13Frame.decode(candidate)
            except ValueError:
                self.crc_errors += 1
                # Advance one byte and search again so a bad frame cannot lock
                # the parser out of sync.
                del self._buffer[0]
                continue

            frames.append(parsed)
            del self._buffer[:FRAME_SIZE]

        return frames


def normalize_axis(value: int, deadzone: float = 0.05) -> float:
    """Map the documented 364..1684 channel range to -1..1 with a deadzone."""
    if not 0 <= deadzone < 1:
        raise ValueError("deadzone must be in [0, 1)")
    normalized = max(-1.0, min(1.0, (value - CHANNEL_CENTER) / (CHANNEL_MAX - CHANNEL_CENTER)))
    if abs(normalized) <= deadzone:
        return 0.0
    return (1.0 if normalized > 0 else -1.0) * (abs(normalized) - deadzone) / (1 - deadzone)


class VT13SerialReader:
    """Read VT13 frames asynchronously; requires the optional pyserial package."""

    def __init__(self, port: str, baudrate: int = BAUDRATE) -> None:
        self.port = port
        self.baudrate = baudrate
        self.latest_frame: VT13Frame | None = None
        self.latest_frame_time = 0.0
        self.error: Exception | None = None
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._serial = None
        self._parser = VT13FrameParser()

    def start(self) -> VT13SerialReader:
        try:
            import serial
        except ImportError as exc:
            raise RuntimeError(
                "VT13 serial input needs pyserial; install it with `python -m pip install pyserial`."
            ) from exc

        try:
            self._serial = serial.Serial(
                self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.05,
                xonxoff=False,
                rtscts=False,
                dsrdtr=False,
            )
        except Exception as exc:
            raise RuntimeError(f"Cannot open VT13 serial port {self.port}: {exc}") from exc

        self._thread = threading.Thread(target=self._read_loop, name="vt13-serial", daemon=True)
        self._thread.start()
        return self

    def _read_loop(self) -> None:
        assert self._serial is not None
        try:
            while not self._stop.is_set():
                chunk = self._serial.read(256)
                if not chunk:
                    continue
                for frame in self._parser.feed(chunk):
                    self.latest_frame = frame
                    self.latest_frame_time = time.monotonic()
        except Exception as exc:
            self.error = exc

    def close(self) -> None:
        self._stop.set()
        if self._serial is not None:
            self._serial.close()
        if self._thread is not None:
            self._thread.join(timeout=1.0)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="/dev/ttyACM0", help="VT03 transmitter serial port")
    args = parser.parse_args()

    reader = VT13SerialReader(args.port).start()
    print(f"Reading VT13 frames from {args.port} at {BAUDRATE} baud. Press Ctrl+C to stop.")
    last_print_time = 0.0
    try:
        while True:
            frame = reader.latest_frame
            now = time.monotonic()
            if frame is not None and now - last_print_time >= 0.1:
                last_print_time = now
                channels = " ".join(f"{value:4d}" for value in frame.channels)
                mode = ("C", "N", "S", "?")[frame.mode_switch]
                print(
                    f"ch0..3=[{channels}] wheel={frame.wheel:4d} "
                    f"mode={mode} pause={frame.pause} fn1={frame.fn1} fn2={frame.fn2} "
                    f"trigger={frame.trigger}",
                    flush=True,
                )
            if reader.error is not None:
                raise RuntimeError(f"VT13 serial read failed: {reader.error}")
            time.sleep(0.02)
    except KeyboardInterrupt:
        pass
    finally:
        reader.close()


if __name__ == "__main__":
    main()
