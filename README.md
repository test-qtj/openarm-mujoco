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

Install the optional serial dependency in the Python environment used to launch MuJoCo:

```bash
python -m pip install -e '.[vt13]'
```

First check the decoded receiver channels without launching the simulation:

```bash
python v1/vt13_serial.py --port /dev/ttyACM0
```

Run the paired right-arm simulation with the receiver connected to the VT03 transmitter:

```bash
python -u v1/master_follower_sim.py --fps 60 --vt13-port /dev/ttyACM0
```

The master swaps the horizontal and vertical channels within each stick. In N mode,
Channel 1 controls J1, Channel 0 controls J2, Channel 3 controls J3, Channel 2 controls
J4, and the receiver dial controls J5. In C mode, Channel 0 controls J6 and Channel 1
controls J7, without software sign inversion. The trigger closes the gripper while held
and opens it when released. In S mode, or while the pause button is held, all VT13
commands hold their current joint positions. The inputs work as low-speed incremental
joint commands: centered axes hold position, and full deflection moves at 0.35 rad/s by
default. Adjust this with `--vt13-speed`. The follower copies all eight master joint
positions over local UDP.

The serial link uses 921600 baud, 8 data bits, no parity, one stop bit, and no flow
control. On Ubuntu, if the program reports permission denied for `/dev/ttyACM0`, add
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
