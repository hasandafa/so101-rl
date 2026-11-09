# Source of the SO-101 model files

| Item | Value |
|---|---|
| Upstream repo | https://github.com/TheRobotStudio/SO-ARM100 |
| Upstream path | `Simulation/SO101/` |
| Commit | `a758567c3978dfeefe282ede0500085a48fe8f78` |
| License | Apache-2.0 (see `/LICENSE` and `/NOTICE`) |

Files copied **unmodified**:

- `so101_new_calib.xml` - MJCF, "new calibration" (joint zero = middle of joint range)
- `so101_new_calib.urdf` - URDF, same calibration
- `scene.xml` - MuJoCo scene (floor, light) that includes the MJCF
- `assets/*.stl` - the 13 meshes referenced by the two files above
- `UPSTREAM_README.md` - upstream README

Not copied: Onshape `.part` files, old-calibration and leader-arm variants,
camera variant, and the standalone `joints_properties.xml`
(its values differ from the ones embedded in the MJCF - see `docs/m0-notes.md`).
