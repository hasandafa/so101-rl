"""Write the URDF kinematic chain (base_link -> gripper_frame_link) as a numeric CSV
for octave/fk_check.m, so the Octave check never has to parse XML.

Usage (repo root): python -m tools.urdf_chain_to_csv
Columns: is_revolute, x, y, z, roll, pitch, yaw, axis_x, axis_y, axis_z
Joint names are written to octave/so101_chain_names.txt in the same order.
"""
import numpy as np

from sim.paths import ARM_JOINTS, REPO_ROOT
from sim.urdf_fk import load_chain

OUT_CSV = REPO_ROOT / "octave" / "so101_chain.csv"
OUT_NAMES = REPO_ROOT / "octave" / "so101_chain_names.txt"


def main():
    chain = load_chain()
    revolute = [j.name for j in chain if j.type == "revolute"]
    if revolute != ARM_JOINTS:
        raise SystemExit(f"Chain order {revolute} does not match ARM_JOINTS {ARM_JOINTS}")
    rows = [[1.0 if j.type == "revolute" else 0.0, *j.xyz, *j.rpy, *j.axis] for j in chain]
    np.savetxt(OUT_CSV, np.asarray(rows), delimiter=",", fmt="%.12g",
               header="is_revolute,x,y,z,roll,pitch,yaw,axis_x,axis_y,axis_z", comments="")
    OUT_NAMES.write_text("\n".join(j.name for j in chain) + "\n")
    print(f"Wrote {len(chain)} joints ({len(revolute)} revolute) to {OUT_CSV}")


if __name__ == "__main__":
    main()
