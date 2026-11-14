"""Run paired OpenArm v1 right arms with viewer controls or VT13 input."""

from __future__ import annotations

import argparse
import subprocess
import sys
import time
from pathlib import Path

import mujoco
import mujoco.viewer

from vt13_serial import VT13SerialReader, normalize_axis
from rm_serial_link import SerialJointReader, SerialJointWriter
from custom_terminal_mqtt import CustomTerminalPublisher, LocalCompetitionBridge


HERE = Path(__file__).resolve().parent
MASTER_SCENE = HERE / "scene_master_right_only.xml"
FOLLOWER_SCENE = HERE / "scene_right_only.xml"
JOINTS = tuple(f"openarm_right_joint{i}" for i in range(1, 8)) + ("openarm_right_finger_joint1",)
ACTUATORS = tuple(f"right_joint{i}_ctrl" for i in range(1, 8)) + ("right_finger1_ctrl",)


def load_scene(path: Path) -> tuple[mujoco.MjModel, mujoco.MjData]:
    model = mujoco.MjModel.from_xml_path(str(path))
    data = mujoco.MjData(model)
    key_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "home")
    if key_id >= 0:
        mujoco.mj_resetDataKeyframe(model, data, key_id)
    mujoco.mj_forward(model, data)
    return model, data


def joint_and_actuator_ids(model: mujoco.MjModel) -> tuple[list[int], list[int]]:
    joint_ids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name) for name in JOINTS]
    actuator_ids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name) for name in ACTUATORS]
    if any(index < 0 for index in joint_ids + actuator_ids):
        raise ValueError("Scene is missing a right-arm joint or actuator")
    return joint_ids, actuator_ids


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--role", choices=("pair", "master", "follower", "bridge"), default="pair")
    parser.add_argument("--fps", type=int, default=60, help="Joint command update rate")
    parser.add_argument("--serial-tx", default=None, help="Serial TX endpoint to the simulated follower")
    parser.add_argument("--serial-rx", default=None, help="Competition-link serial RX endpoint for the follower")
    parser.add_argument("--mqtt-host", default=None, help="Optional RoboMaster custom-terminal MQTT broker host")
    parser.add_argument("--mqtt-port", type=int, default=3333, help="Custom-terminal MQTT broker port")
    parser.add_argument("--mqtt-client-id", default=None, help="Robot ID used as the MQTT client ID")
    parser.add_argument("--mqtt-topic", default="CustomControl", help="Custom-terminal MQTT topic")
    parser.add_argument(
        "--mqtt-bridge-to-serial",
        action="store_true",
        help="For local testing only: forward MQTT CustomControl messages to the follower's serial link",
    )
    parser.add_argument(
        "--vt13-port",
        default=None,
        help="Optional VT03/VT13 serial port; mode switch selects joint groups and trigger controls gripper",
    )
    parser.add_argument(
        "--vt13-speed",
        type=float,
        default=0.35,
        help="Maximum simulated joint speed in rad/s for a full VT13 stick deflection",
    )
    args = parser.parse_args()
    if args.fps <= 0:
        parser.error("--fps must be positive")
    if args.fps > 75:
        parser.error("--fps cannot exceed the RoboMaster 0x0311 limit of 75 Hz")
    if args.vt13_speed <= 0:
        parser.error("--vt13-speed must be positive")
    if not 1 <= args.mqtt_port <= 65535:
        parser.error("--mqtt-port must be between 1 and 65535")
    if args.mqtt_host and not args.mqtt_client_id:
        parser.error("--mqtt-client-id is required when --mqtt-host is set")
    if args.mqtt_client_id and not args.mqtt_host:
        parser.error("--mqtt-host is required when --mqtt-client-id is set")
    if args.mqtt_bridge_to_serial and not args.mqtt_host:
        parser.error("--mqtt-bridge-to-serial requires --mqtt-host")

    if args.role == "pair":
        if not args.serial_rx:
            parser.error("pair mode requires --serial-rx for the follower")
        if args.mqtt_bridge_to_serial and not args.serial_tx:
            parser.error("local MQTT bridge mode also requires --serial-tx")
        if not args.mqtt_host and not args.serial_tx:
            parser.error("serial mode requires --serial-tx and --serial-rx")
        command = [sys.executable, "-u", str(Path(__file__).resolve())]
        children = []
        follower = subprocess.Popen([
            *command, "--role", "follower", "--fps", str(args.fps), "--serial-rx", args.serial_rx,
        ])
        children.append(follower)
        time.sleep(0.5)
        if args.mqtt_bridge_to_serial:
            bridge_command = [
                *command, "--role", "bridge", "--serial-tx", args.serial_tx,
                "--mqtt-host", args.mqtt_host, "--mqtt-port", str(args.mqtt_port),
                "--mqtt-client-id", args.mqtt_client_id, "--mqtt-topic", args.mqtt_topic,
            ]
            bridge = subprocess.Popen(bridge_command)
            children.append(bridge)
            time.sleep(0.5)
        master_command = [*command, "--role", "master", "--fps", str(args.fps)]
        if not args.mqtt_host:
            master_command.extend(("--serial-tx", args.serial_tx))
        if args.vt13_port:
            master_command.extend(("--vt13-port", args.vt13_port, "--vt13-speed", str(args.vt13_speed)))
        if args.mqtt_host:
            master_command.extend((
                "--mqtt-host", args.mqtt_host,
                "--mqtt-port", str(args.mqtt_port),
                "--mqtt-client-id", args.mqtt_client_id,
                "--mqtt-topic", args.mqtt_topic,
            ))
        master = subprocess.Popen(master_command)
        children.append(master)
        if args.vt13_port:
            print(
                f"Master window: VT13 N mode controls J1-J5, C mode controls J6-J7 (J6 inverted), "
                "trigger controls gripper, and S/pause holds."
            )
        else:
            print("Master window: use the Control panel sliders to move the right arm.")
        if args.mqtt_bridge_to_serial:
            print("Local chain: VT13 → MQTT CustomControl → local bridge → RoboMaster 0x0311 serial → follower.")
        elif args.mqtt_host:
            print("Competition path: VT13 → MQTT CustomControl; follower reads its configured robot-side serial port.")
        else:
            print("Direct test path: master sends RoboMaster 0x0311 serial frames to the follower.")
        print("Follower window: the right arm follows joint positions received from the serial link.")
        print("Close either window or press Ctrl+C in this terminal to stop both.")
        try:
            while all(process.poll() is None for process in children):
                time.sleep(0.1)
        except KeyboardInterrupt:
            pass
        finally:
            for process in children:
                if process.poll() is None:
                    process.terminate()
            for process in children:
                try:
                    process.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    process.kill()
        return

    if args.role == "bridge":
        if not args.serial_tx or not args.mqtt_host or not args.mqtt_client_id:
            parser.error("bridge role requires --serial-tx, --mqtt-host, and --mqtt-client-id")
        bridge = LocalCompetitionBridge(
            serial_port=args.serial_tx,
            client_id=args.mqtt_client_id,
            host=args.mqtt_host,
            port=args.mqtt_port,
            topic=args.mqtt_topic,
        )
        print(f"Local MQTT-to-serial bridge subscribed to {args.mqtt_topic}.")
        try:
            bridge.run_forever()
        except KeyboardInterrupt:
            pass
        finally:
            bridge.close()
        return

    is_master = args.role == "master"
    if is_master and not args.serial_tx and not args.mqtt_host:
        parser.error("master role requires --serial-tx")
    if not is_master and not args.serial_rx:
        parser.error("follower role requires --serial-rx")
    scene = MASTER_SCENE if is_master else FOLLOWER_SCENE
    model, data = load_scene(scene)
    joint_ids, actuator_ids = joint_and_actuator_ids(model)
    qpos_indices = [int(model.jnt_qposadr[joint_id]) for joint_id in joint_ids]
    steps_per_frame = max(1, round(1 / args.fps / model.opt.timestep))
    frame_period = 1 / args.fps

    serial_writer = SerialJointWriter(args.serial_tx) if is_master and not args.mqtt_host else None
    serial_reader = SerialJointReader(args.serial_rx) if not is_master else None
    mqtt_publisher = (
        CustomTerminalPublisher(
            client_id=args.mqtt_client_id,
            host=args.mqtt_host,
            port=args.mqtt_port,
            topic=args.mqtt_topic,
        )
        if is_master and args.mqtt_host
        else None
    )

    vt13_reader = None
    if is_master and args.vt13_port:
        vt13_reader = VT13SerialReader(args.vt13_port).start()
        print(
            f"VT13 input {args.vt13_port}: N→J1-J5 (ch1,ch0,ch3,ch2,wheel), "
            f"C→J6-J7 (ch0,ch1), trigger→gripper, S/pause→hold; "
            f"max speed {args.vt13_speed:.2f} rad/s."
        )
    if mqtt_publisher is not None:
        print(
            f"Custom terminal MQTT connected: {args.mqtt_host}:{args.mqtt_port}, "
            f"client ID {args.mqtt_client_id}, topic {args.mqtt_topic}, QoS 2."
        )

    previous_frame_start = time.perf_counter()

    with mujoco.viewer.launch_passive(model, data, show_right_ui=is_master) as viewer:
        last_positions = None
        while viewer.is_running():
            frame_start = time.perf_counter()
            elapsed = min(max(frame_start - previous_frame_start, 0.0), 0.05)
            previous_frame_start = frame_start
            viewer.sync()

            with viewer.lock():
                if is_master:
                    if vt13_reader is not None:
                        if vt13_reader.error is not None:
                            raise RuntimeError(f"VT13 serial read failed: {vt13_reader.error}")
                        frame = vt13_reader.latest_frame
                        frame_age = time.monotonic() - vt13_reader.latest_frame_time
                        if frame is not None and frame_age <= 0.20:
                            axis_commands = ()
                            controls_enabled = not frame.pause
                            if controls_enabled and frame.mode_switch == 1:  # N: arm/base joints
                                ch0, ch1, ch2, ch3 = frame.channels
                                axis_commands = (
                                    (actuator_ids[0], ch1, 1.0),
                                    (actuator_ids[1], ch0, 1.0),
                                    (actuator_ids[2], ch3, 1.0),
                                    (actuator_ids[3], ch2, 1.0),
                                    (actuator_ids[4], frame.wheel, 1.0),
                                )
                            elif controls_enabled and frame.mode_switch == 0:  # C: wrist joints
                                ch0, ch1, _, _ = frame.channels
                                axis_commands = (
                                    (actuator_ids[5], ch0, -1.0),
                                    (actuator_ids[6], ch1, 1.0),
                                )

                            for actuator_id, axis_value, direction in axis_commands:
                                velocity = normalize_axis(axis_value) * direction
                                if velocity == 0.0:
                                    continue
                                low, high = model.actuator_ctrlrange[actuator_id]
                                target = float(data.ctrl[actuator_id]) + velocity * args.vt13_speed * elapsed
                                data.ctrl[actuator_id] = min(float(high), max(float(low), target))

                            # C/N modes: held trigger closes; released trigger opens.
                            # S mode and the pause button hold the current gripper state.
                            if controls_enabled and frame.mode_switch in (0, 1):
                                gripper_id = actuator_ids[7]
                                low, high = model.jnt_range[joint_ids[7]]
                                data.ctrl[gripper_id] = float(low) if frame.trigger else float(high)
                    for _ in range(steps_per_frame):
                        mujoco.mj_step(model, data)
                    last_positions = [float(data.qpos[index]) for index in qpos_indices]
                    if serial_writer is not None:
                        serial_writer.send(last_positions)
                    if mqtt_publisher is not None:
                        mqtt_publisher.publish_joint_positions(last_positions)
                else:
                    if serial_reader.error is not None:
                        raise RuntimeError(f"Competition serial read failed: {serial_reader.error}")
                    if serial_reader.latest_positions is not None and time.monotonic() - serial_reader.latest_time <= 0.25:
                        last_positions = serial_reader.latest_positions
                    if last_positions is not None:
                        for actuator_id, position in zip(actuator_ids, last_positions, strict=True):
                            low, high = model.actuator_ctrlrange[actuator_id]
                            data.ctrl[actuator_id] = min(float(high), max(float(low), position))
                    for _ in range(steps_per_frame):
                        mujoco.mj_step(model, data)

            viewer.sync()
            delay = frame_period - (time.perf_counter() - frame_start)
            if delay > 0:
                time.sleep(delay)
    if vt13_reader is not None:
        vt13_reader.close()
    if serial_writer is not None:
        serial_writer.close()
    if serial_reader is not None:
        serial_reader.close()
    if mqtt_publisher is not None:
        mqtt_publisher.close()


if __name__ == "__main__":
    main()
