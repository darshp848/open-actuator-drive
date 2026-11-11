#!/bin/bash
# One command: export the coupled sets on the Inspiron (read-only use of its repo), run MuJoCo, report, then the Simulink bus check.
#   bash mabel_gim/pipeline_coupled.sh [--no-x1]
# Stops with a message if the CAD index still misses entries (rerun later). Safe to rerun: new timestamped result folders.
set -e
SIM="C:/Users/darsh/TAMU/EMPE_Lab/Actuator-controls/simulation"
PY="$SIM/.venv-openarm/Scripts/python.exe"
cd "$SIM/mabel_gim"
scp -q designs/export_lib.py designs/export_coupled.py inspiron:scratch/pj_export/
if ! ssh inspiron 'cd ~/scratch/pj_export && python3 -B export_coupled.py'; then echo "CAD index incomplete: stop, rerun later"; exit 2; fi
rm -rf designs_coupled && mkdir designs_coupled
scp -q inspiron:scratch/pj_export/designs_coupled/* designs_coupled/
OUT=$("$PY" -I -B run_coupled.py designs_coupled 12 | tee /dev/stderr | grep '^done' | sed 's/^done //; s/ [0-9]*s$//')
UNC=$(ls -d results/*_sets_cad | tail -1)
python -I -B report_coupled.py "$OUT" "$UNC" > /dev/null
echo "report in $OUT"
if [ "$1" != "--no-x1" ]; then python -I -B ../x1_variants/x1_coupled.py designs_coupled "$OUT/rule.json"; fi
