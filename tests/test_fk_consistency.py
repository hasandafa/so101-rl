"""URDF (sim/urdf_fk.py) and MJCF (MuJoCo) must agree on end-effector position.

Runs in CI. The Octave script octave/fk_check.m performs the same comparison with
an independent implementation of the URDF math.
"""
import mujoco
import numpy as np

from sim.export_fk import ee_positions, sample_configs
from sim.paths import ARM_JOINTS, SCENE_XML
from sim.urdf_fk import fk, load_chain

TOL_M = 1e-3  # 1 mm, the M0 acceptance threshold


def _mujoco():
    model = mujoco.MjModel.from_xml_path(str(SCENE_XML))
    return model, mujoco.MjData(model)


def test_chain_matches_arm_joints():
    chain = load_chain()
    assert [j.name for j in chain if j.type == "revolute"] == ARM_JOINTS


def test_zero_configuration():
    model, data = _mujoco()
    q = np.zeros((1, len(ARM_JOINTS)))
    p_mj = ee_positions(model, data, q)[0]
    p_urdf = fk(load_chain(), q[0])[:3, 3]
    assert np.linalg.norm(p_mj - p_urdf) < TOL_M


def test_random_configurations():
    model, data = _mujoco()
    q = sample_configs(model, 200, np.random.default_rng(42))
    p_mj = ee_positions(model, data, q)
    chain = load_chain()
    p_urdf = np.array([fk(chain, qi)[:3, 3] for qi in q])
    err = np.linalg.norm(p_mj - p_urdf, axis=1)
    assert err.max() < TOL_M, f"max URDF-vs-MJCF error {err.max()*1e3:.3f} mm"
