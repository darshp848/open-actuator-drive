#!/bin/sh
# Task B confirmation (arm_prereg.md): seeds 0-7, staged ES, then deployment.
# Each stage runs as two concurrent 4-seed processes (a single 24,576-world process hit a CUDA launch failure);
# per-seed random streams are independent of this split. Halves are merged by arm_merge.py.
set -e
PY=../.venv-warp/Scripts/python.exe
CPU=../.venv/Scripts/python.exe
stage() {   # kind gens warm-tag(or -) rand|nominal tag
  wa=-; wb=-
  if [ "$3" != - ]; then wa=arm_es_$3_a.json; wb=arm_es_$3_b.json; fi
  SEED0=0 $PY arm_gpu_es.py $1 256 $2 $wa $4 4 $5_a > results/arm_confirm_$5_a.log 2>&1 &
  pa=$!
  SEED0=4 $PY arm_gpu_es.py $1 256 $2 $wb $4 4 $5_b > results/arm_confirm_$5_b.log 2>&1 &
  pb=$!
  wait $pa && wait $pb || { echo "stage $5 failed"; exit 1; }
  $CPU arm_merge.py $1 $5
}
for T in 30 15; do
  export T_REF_S=0.0$T
  stage reference 300 - nominal conf_T${T}_st1
  stage reference 300 reference_conf_T${T}_st1 nominal conf_T${T}_st2
  stage reference 300 reference_conf_T${T}_st2 rand conf_T${T}_st3
  if [ $T = 30 ]; then
    stage compact2 300 reference_conf_T30_st1 nominal conf_st2
    stage compact2 300 compact2_conf_st2 rand conf_st3
  fi
done
T_REF_S=0.030 $CPU arm_deploy.py arm_es_reference_conf_T30_st3.json arm_es_compact2_conf_st3.json conf_T30
T_REF_S=0.015 $CPU arm_deploy.py arm_es_reference_conf_T15_st3.json arm_es_compact2_conf_st3.json conf_T15
