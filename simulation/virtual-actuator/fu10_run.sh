#!/bin/sh
# Follow-up 10 (fu10_prereg.md): T = 10 ms and 5 ms references, compact2 retrained, all with 1 kHz joint velocity
# (VEL_HZ=1000), seeds 0-7, staged ES, then deployment. Same stage splitting/merging as arm_confirm.sh.
# The compact route is warm-started from the T = 10 ms reference stage 1 (300 nominal + 300 randomised generations).
set -e
export VEL_HZ=1000
PY=../.venv-warp/Scripts/python.exe
CPU=../.venv/Scripts/python.exe
stage() {   # kind gens warm-tag(or -) rand|nominal tag
  wa=-; wb=-
  if [ "$3" != - ]; then wa=arm_es_$3_a.json; wb=arm_es_$3_b.json; fi
  SEED0=0 $PY arm_gpu_es.py $1 256 $2 $wa $4 4 $5_a > results/fu10_$5_a.log 2>&1 &
  pa=$!
  SEED0=4 $PY arm_gpu_es.py $1 256 $2 $wb $4 4 $5_b > results/fu10_$5_b.log 2>&1 &
  pb=$!
  wait $pa && wait $pb || { echo "stage $5 failed"; exit 1; }
  $CPU arm_merge.py $1 $5
}
for T in 10 5; do
  export T_REF_S=$(printf '0.%03d' $T)
  stage reference 300 - nominal fu10_T${T}_st1
  stage reference 300 reference_fu10_T${T}_st1 nominal fu10_T${T}_st2
  stage reference 300 reference_fu10_T${T}_st2 rand fu10_T${T}_st3
  if [ $T = 10 ]; then
    stage compact2 300 reference_fu10_T10_st1 nominal fu10_cp_st2
    stage compact2 300 compact2_fu10_cp_st2 rand fu10_cp_st3
  fi
done
T_REF_S=0.010 $CPU arm_deploy.py arm_es_reference_fu10_T10_st3.json arm_es_compact2_fu10_cp_st3.json fu10_T10
T_REF_S=0.005 $CPU arm_deploy.py arm_es_reference_fu10_T5_st3.json arm_es_compact2_fu10_cp_st3.json fu10_T5
