"""MQTT publisher for the RoboMaster CustomControl terminal protocol."""

from __future__ import annotations

import time
from threading import Event

from rm_serial_link import SerialJointWriter, decode_custom_control, encode_custom_control


class CustomTerminalPublisher:
    """Publish serialized CustomControl protobuf messages over MQTT QoS 2."""

    def __init__(
        self,
        client_id: str,
        host: str = "192.168.12.1",
        port: int = 3333,
        topic: str = "CustomControl",
        connect_timeout: float = 5.0,
    ) -> None:
        try:
            import paho.mqtt.client as mqtt
        except ImportError as exc:
            raise RuntimeError(
                "MQTT publishing requires paho-mqtt; install with `python -m pip install -e '.[competition]'`."
            ) from exc

        if not client_id:
            raise ValueError("The MQTT client ID must identify the robot, as required by the referee protocol")

        self._mqtt = mqtt
        self.topic = topic
        self.sequence = 0
        self.client = mqtt.Client(
            callback_api_version=mqtt.CallbackAPIVersion.VERSION2,
            client_id=client_id,
            protocol=mqtt.MQTTv311,
        )
        self.client.max_queued_messages_set(2)
        self.client.reconnect_delay_set(min_delay=1, max_delay=5)
        self.client.connect(host, port, keepalive=30)
        self.client.loop_start()

        deadline = time.monotonic() + connect_timeout
        while not self.client.is_connected() and time.monotonic() < deadline:
            time.sleep(0.01)
        if not self.client.is_connected():
            self.close()
            raise TimeoutError(f"MQTT connection to {host}:{port} did not complete within {connect_timeout:g}s")

    def publish_joint_positions(self, positions: tuple[float, ...] | list[float]) -> None:
        payload = encode_custom_control(self.sequence, positions)
        info = self.client.publish(self.topic, payload=payload, qos=2, retain=False)
        if info.rc != self._mqtt.MQTT_ERR_SUCCESS:
            raise RuntimeError(f"MQTT publish to {self.topic!r} failed with code {info.rc}")
        self.sequence = (self.sequence + 1) & 0xFFFF

    def close(self) -> None:
        if self.client.is_connected():
            self.client.disconnect()
        self.client.loop_stop()


class LocalCompetitionBridge:
    """Local test gateway: MQTT CustomControl subscriber to referee UART frames."""

    def __init__(
        self,
        serial_port: str,
        client_id: str,
        host: str,
        port: int,
        topic: str = "CustomControl",
        connect_timeout: float = 10.0,
    ) -> None:
        try:
            import paho.mqtt.client as mqtt
        except ImportError as exc:
            raise RuntimeError(
                "The local MQTT bridge requires paho-mqtt; install with `python -m pip install -e '.[competition]'`."
            ) from exc

        self._mqtt = mqtt
        self._subscribed = Event()
        self.error: Exception | None = None
        self.serial_writer = SerialJointWriter(serial_port)
        self.client = mqtt.Client(
            callback_api_version=mqtt.CallbackAPIVersion.VERSION2,
            client_id=f"{client_id}-sim-bridge",
            protocol=mqtt.MQTTv311,
        )
        self.client.max_queued_messages_set(2)
        self.client.on_connect = lambda client, userdata, flags, reason, properties: self._on_connect(
            topic, client, reason
        )
        self.client.on_subscribe = self._on_subscribe
        self.client.on_message = self._on_message
        self.client.connect(host, port, keepalive=30)
        self.client.loop_start()
        if not self._subscribed.wait(connect_timeout):
            self.close()
            raise TimeoutError(f"MQTT bridge could not subscribe to {topic!r} at {host}:{port}")

    def _on_connect(self, topic: str, client: object, reason_code: object) -> None:
        if reason_code.is_failure:
            self.error = RuntimeError(f"MQTT bridge connection rejected: {reason_code}")
            return
        client.subscribe(topic, qos=2)

    def _on_subscribe(self, client: object, userdata: object, mid: int, reason_code_list: list[object], properties: object) -> None:
        if any(code.is_failure for code in reason_code_list):
            self.error = RuntimeError(f"MQTT bridge subscription rejected: {reason_code_list}")
            return
        self._subscribed.set()

    def _on_message(self, client: object, userdata: object, message: object) -> None:
        try:
            if decode_custom_control(message.payload) is not None:
                self.serial_writer.send_custom_control(message.payload)
        except Exception as exc:
            self.error = exc

    def run_forever(self) -> None:
        while True:
            if self.error is not None:
                raise RuntimeError(f"Local MQTT-to-serial bridge failed: {self.error}") from self.error
            time.sleep(0.1)

    def close(self) -> None:
        if self.client.is_connected():
            self.client.disconnect()
        self.client.loop_stop()
        self.serial_writer.close()
