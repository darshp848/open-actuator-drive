# Notices

Copyright (c) 2026 Darsh Patel

## mjbots moteus X1

The controller boards in `hardware/wip/` and the archived first spin in
`hardware/archive/x1-first-spin/` are derived from the moteus X1 design by
mjbots Robotic Systems LLC, commit 8f747f4ac448d117c1390b77ca2d80c27e751a56
(<https://github.com/mjbots/moteus>), licensed under Apache-2.0. Each of those
folders keeps the upstream `LICENSE` and a `NOTICE.md` that lists the changes.
The original `moteus_x1` file basenames are kept so hierarchical sheet paths
still resolve.

This project is not an mjbots product and is not endorsed by mjbots.
"mjbots" and "moteus" are trademarks of mjbots Robotic Systems LLC.

## OpenArm (Enactic, Inc.)

`simulation/openarm-mujoco/fetch_assets.py` downloads the OpenArm v2 MuJoCo
model from <https://github.com/enactic/openarm_mujoco> at a pinned commit
(Apache-2.0). Only the OpenArm v2 URDF is redistributed, in
`simulation/openarm-mujoco/reference/`.
