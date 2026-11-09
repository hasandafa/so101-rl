# M0 notes - SO-101 model in MuJoCo

Status: **M0 complete.** Checks done while setting up the model, and the findings that shape M1.

## 1. URDF and MJCF describe the same kinematics

| Check | Samples | Max error | Mean error | Result |
|---|---|---|---|---|
| Python URDF FK (`sim/urdf_fk.py`) vs MuJoCo site `gripperframe` | 100 + 200 (pytest) | 0.0034 mm | 0.0018 mm | PASS |
| Octave URDF FK (`octave/fk_check.m`) vs MuJoCo | 100 | 0.0034 mm | 0.0018 mm | PASS |

Two independent implementations (Python and Octave) give the same error figures.
The remaining ~3 µm comes from the rounded angles (e.g. `3.14159` instead of π) in the
upstream files, not from a modelling difference. The known upstream issue about the
`so101_new_calib.urdf` file does not affect the kinematic chain we use.

## 2. Actuator gains: two different parameter sets upstream

| Source | kp | kv | force limit |
|---|---|---|---|
| `so101_new_calib.xml` default class `sts3215` (what MuJoCo actually loads) | 998.22 | 2.731 | ±2.94 N·m in the class, **overridden to ±3.35 N·m** on every `<position>` actuator |
| upstream standalone `joints_properties.xml` (not copied) | 17.8 | 0 | ±3.35 N·m |

The MJCF comment says its gains assume a servo proportional gain of 16 and come from the
RBE501-RL-arm-project. **Decision needed before M3:** Gazebo must use the same servo model,
otherwise the MuJoCo->Gazebo comparison measures a gain mismatch instead of a simulator gap.

## 3. The arm sags under gravity (torque saturation)

Headless tracking test (`python -m sim.inspect_model`): one slow sine per joint, amplitude
60 % of half-range, other joints at 0.

| Joint | Max error | RMS error | Steps at torque limit |
|---|---|---|---|
| shoulder_pan | 1.3° | 0.15° | - |
| shoulder_lift | **22.3°** | 8.6° | 30 % |
| elbow_flex | **8.5°** | 2.6° | 18 % |
| wrist_flex | 0.6° | 0.10° | - |
| wrist_roll | 4.4° | 0.53° | - |

Identical numbers on MuJoCo 3.17 (Python 3.10) and MuJoCo 4.0.0 (Python 3.12), so the
results do not depend on the MuJoCo major version.

Shoulder and elbow hit the ±3.35 N·m limit when the arm is stretched out. This is
physically plausible for a small hobby servo, so it is a property to design around,
not a bug:

- M1 targets should stay inside the region the arm can actually hold.
- The RL observation must include measured joint positions, not commanded ones.
- **BoM link:** ±2.94 N·m ≈ 30 kg·cm, the rating of the 12 V STS3215 (1:345). The
  ±3.35 N·m override is slightly stronger than that servo. Real-hardware plans should use
  the 12 V variant for the follower arm, and the simulation limit should match it.

## 4. The arm does not collide with the floor

All robot geoms have `contype=0 conaffinity=0`, so the arm passes through the floor.
Random FK samples reach z = −0.159 m. For M1: sample targets above the table only and
either add floor collision or penalise z < 0 in the reward. Upstream also removed the
base collision meshes (see `assets/so101/UPSTREAM_README.md`), which matters for M5
(pick-and-place).

## 5. Gripper

The gripper is a revolute joint here (−10° to 100°); LeRobot maps it to 0-100 linear.
Not relevant until M5.
