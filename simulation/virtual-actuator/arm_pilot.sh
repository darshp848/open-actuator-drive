#!/bin/sh
# Task B pilot (second, with deployment sensing in training): seeds 100-101, same staged protocol as the pendulum confirmation (Follow-up 7).
set -e
PY=../.venv-warp/Scripts/python.exe
export SEED0=100 T_REF_S=0.030
$PY arm_gpu_es.py reference 256 300 - nominal 2 ref_pilot2_st1
$PY arm_gpu_es.py reference 256 300 arm_es_reference_ref_pilot2_st1.json nominal 2 ref_pilot2_st2
$PY arm_gpu_es.py reference 256 300 arm_es_reference_ref_pilot2_st2.json rand 2 ref_pilot2_st3
$PY arm_gpu_es.py compact2 256 300 arm_es_reference_ref_pilot2_st1.json nominal 2 cp_pilot2_st2
$PY arm_gpu_es.py compact2 256 300 arm_es_compact2_cp_pilot2_st2.json rand 2 cp_pilot2_st3
$PY arm_gpu_es.py compact2 256 300 - nominal 2 cp_pilot2_scratch
