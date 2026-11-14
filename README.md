# MuJoCo Description Files (MJCF) for OpenArm
<img height="546" alt="image" src="media/v2.png" />

This repository contains assets for OpenArm v2 (above), Cell, v1 and v0.3 (below) simulation in MuJoCo.

## Usage

To install a release with pip in a virtual environment:

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install openarm-mujoco
```

For the example using `uv`, install [uv](https://docs.astral.sh/uv/getting-started/installation/),
then choose one installation method:

For editable development, run from this repository's source checkout:

```bash
uv sync
source .venv/bin/activate
```

As a dependency in another uv project:

```bash
uv add openarm-mujoco
source .venv/bin/activate
```

To try a release in a separate directory:

```bash
uv venv
source .venv/bin/activate
uv pip install openarm-mujoco
```

To test a specific wheel, replace the package name with its `.whl` file path.

Launch the OpenArm v2 Cell simulation:

```bash
openarm-mujoco-launch
```

Without White Sheet:

```bash
openarm-mujoco-launch --no-sheet
```

With Wall Collisions:

```bash
openarm-mujoco-launch --walls
```

For v1 Bimanual Model (from the source checkout):

```bash
openarm-mujoco-launch v1/scene.xml --keyframe home
```

## VT03/VT13 Input for the v1 Right-Arm Simulation

Install the optional VT13 serial and MQTT dependencies in the Python environment used to launch MuJoCo:

```bash
python -m pip install -e '.[vt13,competition]'
```

First check the decoded receiver channels without launching the simulation:

```bash
python v1/vt13_serial.py --port /dev/ttyACM0
```

For a direct serial-frame check, run a virtual serial pair in one terminal and keep it
open:

```bash
socat -d -d pty,raw,echo=0 pty,raw,echo=0
```

`socat` prints two PTY paths. Use the current paths as the master's TX and follower's RX:

```bash
python -u v1/master_follower_sim.py --fps 60 \
  --vt13-port /dev/ttyACM0 \
  --serial-tx /dev/pts/5 --serial-rx /dev/pts/6
```

This direct mode sends referee serial frames from master to follower and skips MQTT.
The master swaps the horizontal and vertical channels within each stick. In N mode,
Channel 1 controls J1, Channel 0 controls J2, Channel 3 controls J3, Channel 2 controls
J4, and the receiver dial controls J5. In C mode, Channel 0 controls J6 with software
sign inversion, and Channel 1 controls J7 without inversion. The trigger closes the gripper while held
and opens it when released. In S mode, or while the pause button is held, all VT13
commands hold their current joint positions. The inputs work as low-speed incremental
joint commands: centered axes hold position, and full deflection moves at 0.35 rad/s by
default. Adjust this with `--vt13-speed`. The joint packet contains all eight measured
joint positions in the official `CustomControl` Protobuf bytes field; the eight float
values and packet marker inside that field are specific to this project. The follower
validates referee frames and applies the received positions. The official robot-side
graph-transmission UART uses 921600 baud, 8 data bits, no parity, one stop bit, and no
flow control; `0x0311` is limited to 75 Hz.

The follower scene includes the supplied six-slot rack with its two long tubes vertical
to the ground, its six-slot front facing the arm, and the assembly moved closer to the
arm. Six separate free bodies occupy the corresponding CAD positions and orientations
in the rack. Each unit is about
95 × 150 × 95 mm and 357 g. Its visual is the imported STL; a compact box collision
proxy and a small support pad keep it in place and allow grasping. For this simulation
only, the gripper stroke is extended from 44 to 48 mm per finger so it can open around
the 95 mm unit. The rack and units appear only in the follower window; drive the paired
master arm to position the follower, then hold the VT13 trigger to close the gripper and
release it to open. This simulation stroke change does not alter the real robot's
LeRobot configuration.

To run the local full-software chain from one terminal, install the required system
tools once:

```bash
sudo apt install mosquitto socat
```

Then run the launcher:

```bash
cd /home/fn/xiaohai/openarm_mujoco
bash v1/run_local_mqtt_sim.sh
```

It starts a local Mosquitto broker on port 1885 (or reuses one already listening
there), creates a virtual serial pair, and starts the paired MuJoCo simulation. VT13
acts as the controller input; the master publishes binary Protobuf to MQTT topic
`CustomControl`; a local bridge subscribes and emits the corresponding `0x0311`
referee serial frame to the simulated follower. Press `Ctrl+C` to stop the simulation
and the helper processes started by the launcher. This exercises the software
interfaces, but the local bridge stands in for the referee gateway and radio; it is
not an over-the-air test.

The default VT13 device path is the stable `/dev/serial/by-id/...` link. If this
device has a different path, pass it through `VT13_PORT`, for example:

```bash
VT13_PORT=/dev/ttyACM0 bash v1/run_local_mqtt_sim.sh
```

For the competition path, connect the custom-terminal computer to the competition
network (the protocol specifies terminal IP `192.168.12.2`). Publish to the official
broker using the assigned robot ID as MQTT client ID, and set `--serial-rx` to the
robot-side serial device that exposes received `0x0311` frames:

```bash
python -u v1/master_follower_sim.py --fps 60 \
  --vt13-port /dev/ttyACM0 \
  --serial-rx /dev/ttyUSB0 \
  --mqtt-host 192.168.12.1 --mqtt-port 3333 \
  --mqtt-client-id YOUR_ROBOT_ID
```

Replace `YOUR_ROBOT_ID` with the assigned robot ID. The default MQTT topic is
`CustomControl`; the client publishes binary Protobuf at QoS 2, matching the official
sample client's delivery setting.

The VT13 USB serial input also uses 921600 baud, 8 data bits, no parity, one stop bit,
and no flow control. On Ubuntu, if the program reports permission denied for
`/dev/ttyACM0`, add
the login user to the `dialout` group and log out and back in before launching it.

## OpenArm MuJoCo Web

[OpenArm MuJoCo Web](web/README.md) runs these models right in the
browser (MuJoCo WASM + three.js, no installation): open
<https://enactic.github.io/openarm_mujoco/>, or serve the repository
root locally (`npm run serve` in `web/`) and open
<http://localhost:8080/>. Drive the arms by keyboard or sliders; see
[web/README.md](web/README.md) for details.

## Collision Visualization
- To view collision meshes, activate `Rendering`>`Model Elements`>`Convex Hull` and `Group Enable`>`Geom groups`>`Geom 3` in the left sidebar
- It may also help to hide the visual meshes by deselecting `Geom 2`

## Related links

- 📚 Read the [documentation](https://docs.openarm.dev/simulation/mujoco)
- 💬 Join the community on [Discord](https://discord.gg/FsZaZ4z3We)
- 📬 Contact us through <openarm@enactic.ai>

## Pull request and preview

You can enable preview on your fork by the following:

1. Enable GitHub Pages on your fork:
   1. Open https://github.com/${YOUR_GITHUB_ACCOUNT}/openarm_mujoco/settings/pages
   2. Select "GitHub Actions" as "Source"
2. Accept publishing GitHub Pages from all branches on your fork:
   1. Open https://github.com/${YOUR_GITHUB_ACCOUNT}/openarm_mujoco/settings/environments
   2. Select the "github-pages" environment
   3. Change the default "Deployment branches and tags" rule:
      1. Press the "Edit" button
      2. Change the "Name pattern" to `*` from `main`

You can preview your changes at https://${YOUR_GITHUB_ACCOUNT}.github.io/openarm_mujoco/ .

## License

Licensed under the Apache License 2.0. See `LICENSE` for details.

Copyright 2025 Enactic, Inc.

## Code of Conduct

All participation in the OpenArm project is governed by our
[Code of Conduct](CODE_OF_CONDUCT.md).
