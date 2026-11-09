# so101-rl

Reinforcement learning for the **SO-101** robot arm: train a reaching policy in
**MuJoCo**, then run the same policy through **ROS 2 Jazzy** on the same arm in
**Gazebo Harmonic**, and compare it against a classical inverse-kinematics baseline.

> **Question this project answers:** does a policy trained in one simulator still work
> in another simulator, and how does it compare with classical IK?
> The MuJoCo -> Gazebo transfer is a rehearsal for the later step to a real arm.

## Status

| Milestone | Content | Status |
|---|---|---|
| M0 | SO-101 model in MuJoCo, URDF/MJCF kinematics verified | done |
| M1 | Gymnasium `SO101Reach` environment | - |
| M2 | SAC training in MuJoCo | - |
| M3 | SO-101 in Gazebo via ROS 2 + `gz_ros2_control`, IK baseline | - |
| M4 | RL policy as a ROS 2 node in Gazebo, RL vs IK, MuJoCo vs Gazebo | - |
| M5 | (stretch) pick-and-place with HER | - |
| M6 | Showcase page and demo video | - |

## Repository layout

```
assets/so101/   SO-101 URDF + MJCF + meshes (TheRobotStudio, Apache-2.0, unmodified)
sim/            MuJoCo code: model inspection, FK export, URDF forward kinematics
tools/          helper scripts (URDF chain -> CSV for Octave)
octave/         Octave FK cross-check
tests/          pytest checks, also run by GitHub Actions
train/          RL training (M2)
ros2_ws/src/    ROS 2 packages (M3)
docs/           milestone notes and findings
```

## Quick start (M0)

Requirements: Ubuntu 24.04, Python 3.10+, MuJoCo 3.x, Octave (optional, for the cross-check).

```bash
pip install -r requirements.txt

python -m sim.inspect_model             # model summary + headless joint tracking test
python -m sim.inspect_model --viewer    # same, then wave each joint in the viewer
python -m sim.export_fk                 # 100 random poses -> outputs/fk_mujoco.csv
python -m tools.urdf_chain_to_csv       # URDF chain -> octave/so101_chain.csv
octave --no-gui octave/fk_check.m       # URDF FK in Octave vs MuJoCo, must be < 1 mm
python -m pytest -q tests               # same check in Python (runs in CI)
```

Findings so far: [docs/m0-notes.md](docs/m0-notes.md).

## Contributing

Commit messages follow Conventional Commits, see [CONTRIBUTING.md](CONTRIBUTING.md).

## Credits and license

Robot model: [SO-ARM100 by TheRobotStudio](https://github.com/TheRobotStudio/SO-ARM100),
Apache-2.0, see [NOTICE](NOTICE) and [assets/so101/SOURCE.md](assets/so101/SOURCE.md).

This repository is licensed under the Apache License 2.0 ([LICENSE](LICENSE)).
