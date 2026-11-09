"""M0.4 - Inspect the SO-101 MuJoCo model and exercise every joint.

Usage (from the repo root, inside the `rl` conda env):
    python -m sim.inspect_model            # print model summary + headless tracking test
    python -m sim.inspect_model --viewer   # same, then open the viewer and wave each joint

The headless test commands a slow sine wave to one arm joint at a time through
its position actuator and reports how closely the joint follows. Large tracking
errors point at bad actuator gains or joint limits before any RL is attempted.
"""
import argparse
import time

import mujoco
import numpy as np

from sim.paths import ARM_JOINTS, EE_SITE, SCENE_XML

SINE_PERIOD_S = 4.0       # one full sine cycle per joint
SINE_AMPLITUDE = 0.6      # fraction of half the joint range


def load_model():
    model = mujoco.MjModel.from_xml_path(str(SCENE_XML))
    data = mujoco.MjData(model)
    return model, data


def print_summary(model):
    print(f"Model file      : {SCENE_XML}")
    print(f"timestep        : {model.opt.timestep} s")
    print(f"nq / nv / nu    : {model.nq} / {model.nv} / {model.nu}")
    print(f"bodies / sites  : {model.nbody} / {model.nsite}")

    print("\nJoints")
    print(f"  {'name':<14}{'range [rad]':<24}{'damping':>9}{'armature':>10}{'friction':>10}")
    for j in range(model.njnt):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, j)
        lo, hi = model.jnt_range[j]
        dof = model.jnt_dofadr[j]
        print(f"  {name:<14}[{lo:+.4f}, {hi:+.4f}]   "
              f"{model.dof_damping[dof]:9.3f}{model.dof_armature[dof]:10.3f}"
              f"{model.dof_frictionloss[dof]:10.3f}")

    print("\nActuators (position servos)")
    print(f"  {'name':<14}{'joint':<14}{'kp':>9}{'kv':>8}  {'forcerange [N m]':<18}")
    for a in range(model.nu):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, a)
        jid = model.actuator_trnid[a, 0]
        jname = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, jid)
        kp = model.actuator_gainprm[a, 0]
        kv = -model.actuator_biasprm[a, 2]
        flo, fhi = model.actuator_forcerange[a]
        print(f"  {name:<14}{jname:<14}{kp:9.2f}{kv:8.3f}  [{flo:+.2f}, {fhi:+.2f}]")

    print("\nSites")
    for s in range(model.nsite):
        print(f"  {mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_SITE, s)}")


def sine_targets(model, joint_name, t):
    """Target position for `joint_name` at time t; other joints stay at 0."""
    jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
    lo, hi = model.jnt_range[jid]
    mid, half = (lo + hi) / 2, (hi - lo) / 2
    return mid + SINE_AMPLITUDE * half * np.sin(2 * np.pi * t / SINE_PERIOD_S)


def actuator_id(model, joint_name):
    return mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, joint_name)


def headless_tracking_test(model, data):
    """Wave each arm joint for one period and report tracking error."""
    print("\nHeadless tracking test (one sine period per joint, others held at 0)")
    print(f"  {'joint':<14}{'max |err| [deg]':>16}{'rms err [deg]':>15}")
    steps = int(SINE_PERIOD_S / model.opt.timestep)
    results = {}
    for jname in ARM_JOINTS:
        mujoco.mj_resetData(model, data)
        mujoco.mj_forward(model, data)
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, jname)
        qadr = model.jnt_qposadr[jid]
        aid = actuator_id(model, jname)
        errs = []
        for _ in range(steps):
            target = sine_targets(model, jname, data.time)
            data.ctrl[:] = 0.0
            data.ctrl[aid] = target
            mujoco.mj_step(model, data)
            errs.append(target - data.qpos[qadr])
        errs = np.degrees(np.asarray(errs))
        results[jname] = (np.max(np.abs(errs)), np.sqrt(np.mean(errs ** 2)))
        print(f"  {jname:<14}{results[jname][0]:16.3f}{results[jname][1]:15.3f}")
    return results


def run_viewer(model, data):
    import mujoco.viewer  # imported lazily: needs a display

    mujoco.mj_resetData(model, data)
    site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, EE_SITE)
    print("\nViewer: waving each joint in turn. Close the window to exit.")
    with mujoco.viewer.launch_passive(model, data) as viewer:
        start = time.time()
        while viewer.is_running():
            t = data.time
            k = int(t // SINE_PERIOD_S) % len(ARM_JOINTS)
            jname = ARM_JOINTS[k]
            data.ctrl[:] = 0.0
            data.ctrl[actuator_id(model, jname)] = sine_targets(model, jname, t)
            mujoco.mj_step(model, data)
            viewer.sync()
            if int(t / model.opt.timestep) % 500 == 0:
                ee = data.site_xpos[site_id]
                print(f"  t={t:6.2f}s  moving {jname:<13} EE = "
                      f"({ee[0]:+.3f}, {ee[1]:+.3f}, {ee[2]:+.3f}) m")
            # keep simulation roughly in real time
            lag = data.time - (time.time() - start)
            if lag > 0:
                time.sleep(lag)


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--viewer", action="store_true", help="open the interactive viewer")
    args = parser.parse_args()

    model, data = load_model()
    print_summary(model)
    headless_tracking_test(model, data)
    if args.viewer:
        run_viewer(model, data)


if __name__ == "__main__":
    main()
