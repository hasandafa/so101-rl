"""M0.4 - Export end-effector positions from MuJoCo for random joint configurations.

Usage (repo root, `rl` env):
    python -m sim.export_fk                 # 100 samples -> outputs/fk_mujoco.csv
    python -m sim.export_fk --n 500 --seed 1

Output columns: shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll [rad],
                x, y, z [m]  (position of site `gripperframe` in the world frame)

The CSV is the reference that octave/fk_check.m compares against. MuJoCo reads the
MJCF file while the Octave check reads the URDF, so agreement means the two model
files describe the same kinematics.
"""
import argparse

import mujoco
import numpy as np

from sim.paths import ARM_JOINTS, EE_SITE, OUTPUTS_DIR, SCENE_XML


def sample_configs(model, n, rng):
    """Uniform samples inside each arm joint's range, shape (n, 5)."""
    ranges = []
    for name in ARM_JOINTS:
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        ranges.append(model.jnt_range[jid])
    ranges = np.asarray(ranges)
    return rng.uniform(ranges[:, 0], ranges[:, 1], size=(n, len(ARM_JOINTS)))


def ee_positions(model, data, configs):
    """Forward kinematics only (no dynamics) for each configuration."""
    qadr = [model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, j)]
            for j in ARM_JOINTS]
    site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, EE_SITE)
    out = np.empty((len(configs), 3))
    for i, q in enumerate(configs):
        mujoco.mj_resetData(model, data)      # gripper and everything else at 0
        data.qpos[qadr] = q
        mujoco.mj_kinematics(model, data)
        out[i] = data.site_xpos[site_id]
    return out


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--n", type=int, default=100, help="number of samples")
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--out", default=str(OUTPUTS_DIR / "fk_mujoco.csv"))
    args = parser.parse_args()

    model = mujoco.MjModel.from_xml_path(str(SCENE_XML))
    data = mujoco.MjData(model)
    rng = np.random.default_rng(args.seed)

    q = sample_configs(model, args.n, rng)
    p = ee_positions(model, data, q)

    OUTPUTS_DIR.mkdir(exist_ok=True)
    header = ",".join(ARM_JOINTS + ["x", "y", "z"])
    np.savetxt(args.out, np.hstack([q, p]), delimiter=",", header=header,
               comments="", fmt="%.9f")
    print(f"Wrote {args.n} samples to {args.out}")
    print(f"EE workspace span [m]: x {p[:,0].min():+.3f}..{p[:,0].max():+.3f}, "
          f"y {p[:,1].min():+.3f}..{p[:,1].max():+.3f}, "
          f"z {p[:,2].min():+.3f}..{p[:,2].max():+.3f}")


if __name__ == "__main__":
    main()
