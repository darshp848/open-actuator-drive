#!/bin/sh
# Follow-up 10 under the 7:45 AM laptop deadline (2 Nov 2026): T = 10 ms route and compact route only (T = 5 ms
# dropped, see fu10_prereg.md amendment). Stage 1 (started by fu10_run.sh) is awaited; afterwards the reference and
# compact stages 2 and 3 run side by side (four GPU processes). Seeds, generations and tags are unchanged.
set -e
export VEL_HZ=1000 T_REF_S=0.010
PY=../.venv-warp/Scripts/python.exe
CPU=../.venv/Scripts/python.exe
R=results
launch() {   # kind gens warm-tag rand|nominal tag
  SEED0=0 $PY arm_gpu_es.py $1 256 $2 arm_es_$3_a.json $4 4 $5_a > $R/fu10_$5_a.log 2>&1 &
  SEED0=4 $PY arm_gpu_es.py $1 256 $2 arm_es_$3_b.json $4 4 $5_b > $R/fu10_$5_b.log 2>&1 &
}
until [ -f $R/arm_es_reference_fu10_T10_st1_a.json ] && [ -f $R/arm_es_reference_fu10_T10_st1_b.json ]; do sleep 30; done
$CPU arm_merge.py reference fu10_T10_st1
launch reference 300 reference_fu10_T10_st1 nominal fu10_T10_st2
launch compact2 300 reference_fu10_T10_st1 nominal fu10_cp_st2
wait
$CPU arm_merge.py reference fu10_T10_st2
$CPU arm_merge.py compact2 fu10_cp_st2
launch reference 300 reference_fu10_T10_st2 rand fu10_T10_st3
launch compact2 300 compact2_fu10_cp_st2 rand fu10_cp_st3
wait
$CPU arm_merge.py reference fu10_T10_st3
$CPU arm_merge.py compact2 fu10_cp_st3
$CPU arm_deploy.py arm_es_reference_fu10_T10_st3.json arm_es_compact2_fu10_cp_st3.json fu10_T10
echo done
