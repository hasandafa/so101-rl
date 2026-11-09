# Contributing

## Commit messages

This repository follows [Conventional Commits](https://www.conventionalcommits.org/),
using the [qoomon cheatsheet](https://gist.github.com/qoomon/5dfcdf8eec66a051ecd85625518cfd13).

```
<type>(<optional scope>): <description>

<optional body>

<optional footer>
```

- **Types:** `feat`, `fix`, `refactor`, `perf`, `style`, `test`, `docs`, `build`, `ops`, `chore`
- **Description:** imperative present tense ("add", not "added"), lowercase first letter, no trailing period
- **Breaking change:** `!` before the colon, plus a `BREAKING CHANGE:` footer
- **Initial commit:** `chore: init`

### Scopes used in this repo

| Scope | Covers |
|---|---|
| `assets` | robot model files in `assets/so101/` |
| `sim` | MuJoCo code in `sim/` and `tools/` |
| `env` | Gymnasium environment (M1) |
| `train` | RL training code and configs (M2) |
| `ros2` | ROS 2 packages in `ros2_ws/` (M3+) |
| `octave` | Octave scripts in `octave/` |
| `ci` | GitHub Actions workflows |
| `deps` | `requirements.txt` and environment files |

Milestones (M0, M1, ...) are not part of the subject line. Mark a finished milestone
with a git tag instead, e.g. `git tag -a m0 -m "M0: model verified"`.

### Examples

```
feat(sim): add headless joint tracking test
test(sim): check URDF and MJCF end-effector agreement
fix(env): clip targets to the reachable workspace
docs: record torque saturation finding
ops(ci): run pytest on every push
```
