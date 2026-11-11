# CAD

Parametric frame parts (issue #1), generated with CadQuery.
Design inputs and the v0/v1 revision plan: `docs/FRAME_DESIGN.md`.

## Setup and use

```sh
uv venv --python 3.11 cad/.venv
uv pip install --python cad/.venv/bin/python cadquery   # 2.8.0 verified
cad/.venv/bin/python cad/check_dimensions.py   # dimension checks
cad/.venv/bin/python cad/make_parts.py         # STEP/STL into cad/out/
```

`cad/out/`, `*.stl`, and `*.step` are git-ignored; the generator
script is the source of truth. v0 uses ASSUMED MG90S dimensions -
do not print v0 for final assembly; v1 follows the measurements
listed in `docs/FRAME_DESIGN.md`.
