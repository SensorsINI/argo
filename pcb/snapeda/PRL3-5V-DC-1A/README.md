PRL3-5V-DC-1A (SnapEDA) placeholder

This directory is referenced by the `argo-v9-stable-rev2` KiCad project.

Expected file/folder layout (relative to repo root `pcb/` when `ARGO` path var is set to that directory):

- Symbol library (required by `sym-lib-table`):
  - `pcb/snapeda/PRL3-5V-DC-1A/PRL3-5V-DC-1A.kicad_sym`

- Footprint library (referenced in the schematic/PCB as `PRL3-5V-DC-1A:RELAY_PRL3-5V-DC-1A`):
  - `pcb/snapeda/PRL3-5V-DC-1A/PRL3-5V-DC-1A.pretty/RELAY_PRL3-5V-DC-1A.kicad_mod`

- 3D model(s) (optional):
  - `pcb/snapeda/PRL3-5V-DC-1A/PRL3-5V-DC-1A.3dshapes/<model>.step`

KiCad path variable:
- Set `ARGO = <absolute path>/pcb` in KiCad (Preferences → Configure Paths) so `${ARGO}` resolves correctly.

Reference links:
- SnapEDA part page: https://www.snapeda.com/parts/PRL3-5V-DC-1A/Same+Sky/view-part/
- Digi-Key product page: https://www.digikey.ch/en/products/detail/same-sky-formerly-cui-devices/PRL3-5V-DC-1A/26833116

Notes:
- This repo intentionally does not include the vendor symbol/footprint/model files. Drop the files exported from SnapEDA into the paths above and the project will resolve them automatically.

