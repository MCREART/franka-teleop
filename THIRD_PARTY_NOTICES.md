# Third-party notices

Original iOS client, Python server, tests, and project documentation are distributed under the root MIT license. The vendored model has its own license.

## Franka Emika Panda model

The files under `server/models/panda/` come from Google DeepMind's MuJoCo Menagerie, except for the local provenance record `SOURCE.txt`. The model is derived from the Franka Emika Panda description in `franka_ros`, as described in the upstream README. The model is distributed under Apache-2.0. Its complete license is retained in `server/models/panda/LICENSE`.

The bundled revision is `98a70c569c4c1b6da467a20012f766ee1f8dc521`. The original MJCF and mesh files are retained. At runtime, `server/simulation.py` adds a tabletop, a free cube, a TCP site and visualization settings, adjusts finger friction, and replaces the upstream keyframe with the application's initial configuration.

The upstream directory also contains additional example scenes that are outside this application's bundled subset. Their names may appear in the preserved upstream README.

[Upstream model and revision](https://github.com/google-deepmind/mujoco_menagerie/tree/98a70c569c4c1b6da467a20012f766ee1f8dc521/franka_emika_panda) · [Model license](server/models/panda/LICENSE) · [Upstream README](server/models/panda/README.md) · [Provenance record](server/models/panda/SOURCE.txt)

## Installed dependencies

Python dependencies are installed separately from public PyPI and pinned in `uv.lock`. Their distributions retain their respective license notices. Apple SDKs and application signing are supplied by the developer's Xcode installation.

[MuJoCo](https://github.com/google-deepmind/mujoco) · [NumPy](https://github.com/numpy/numpy) · [websockets](https://github.com/python-websockets/websockets) · [pytest](https://github.com/pytest-dev/pytest)
