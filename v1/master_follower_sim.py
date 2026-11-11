"""Run a mouse-controlled OpenArm v1 master and a joint-following follower."""

from __future__ import annotations

import argparse
import socket
import struct
import subprocess
import sys
import time
from pathlib import Path

import mujoco
import mujoco.viewer

from vt13_serial import VT13SerialReader, normalize_axis


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
    parser.add_argument("--role", choices=("pair", "master", "follower"), default="pair")
    parser.add_argument("--fps", type=int, default=60, help="Joint command update rate")
    parser.add_argument("--port", type=int, default=45871, help="Local UDP port for master-follower angles")
    parser.add_argument(
        "--vt13-port",
        default=None,
        help="Optional VT03/VT13 serial port; channels 0-3 and wheel incrementally control J1-J5",
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
    if not 1 <= args.port <= 65535:
        parser.error("--port must be between 1 and 65535")
    if args.vt13_speed <= 0:
        parser.error("--vt13-speed must be positive")

    if args.role == "pair":
        command = [sys.executable, "-u", str(Path(__file__).resolve())]
        follower = subprocess.Popen([*command, "--role", "follower", "--fps", str(args.fps), "--port", str(args.port)])
        time.sleep(0.5)
        master_command = [*command, "--role", "master", "--fps", str(args.fps), "--port", str(args.port)]
        if args.vt13_port:
            master_command.extend(("--vt13-port", args.vt13_port, "--vt13-speed", str(args.vt13_speed)))
        master = subprocess.Popen(master_command)
        if args.vt13_port:
            print(f"Master window: VT13 on {args.vt13_port} controls J1-J5; use Control sliders for J6, J7, and gripper.")
        else:
            print("Master window: use the Control panel sliders to move the right arm.")
        print("Follower window: the right arm copies the master's measured joint angles.")
        print("Close either window or press Ctrl+C in this terminal to stop both.")
        try:
            while master.poll() is None and follower.poll() is None:
                time.sleep(0.1)
        except KeyboardInterrupt:
            pass
        finally:
            for process in (master, follower):
                if process.poll() is None:
                    process.terminate()
            for process in (master, follower):
                try:
                    process.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    process.kill()
        return

    is_master = args.role == "master"
    scene = MASTER_SCENE if is_master else FOLLOWER_SCENE
    model, data = load_scene(scene)
    joint_ids, actuator_ids = joint_and_actuator_ids(model)
    qpos_indices = [int(model.jnt_qposadr[joint_id]) for joint_id in joint_ids]
    steps_per_frame = max(1, round(1 / args.fps / model.opt.timestep))
    frame_period = 1 / args.fps

    channel = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    if not is_master:
        channel.bind(("127.0.0.1", args.port))
        channel.setblocking(False)

    vt13_reader = None
    if is_master and args.vt13_port:
        vt13_reader = VT13SerialReader(args.vt13_port).start()
        print(
            f"VT13 input {args.vt13_port}: swapped stick axes, "
            f"ch1→J1, ch0→J2, ch3→J3, ch2→J4, wheel→J5; "
            f"centered controls hold position; "
            f"max speed {args.vt13_speed:.2f} rad/s."
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
                            ch0, ch1, ch2, ch3 = frame.channels
                            axes = (ch1, ch0, ch3, ch2, frame.wheel)
                            for actuator_id, axis_value in zip(actuator_ids[:5], axes, strict=True):
                                velocity = normalize_axis(axis_value)
                                if velocity == 0.0:
                                    continue
                                low, high = model.actuator_ctrlrange[actuator_id]
                                target = float(data.ctrl[actuator_id]) + velocity * args.vt13_speed * elapsed
                                data.ctrl[actuator_id] = min(float(high), max(float(low), target))
                    for _ in range(steps_per_frame):
                        mujoco.mj_step(model, data)
                    last_positions = [float(data.qpos[index]) for index in qpos_indices]
                    channel.sendto(struct.pack("!8d", *last_positions), ("127.0.0.1", args.port))
                else:
                    while True:
                        try:
                            packet, _ = channel.recvfrom(1024)
                        except BlockingIOError:
                            break
                        if len(packet) == struct.calcsize("!8d"):
                            last_positions = list(struct.unpack("!8d", packet))
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
    channel.close()


if __name__ == "__main__":
    main()
