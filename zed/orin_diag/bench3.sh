#!/bin/bash
# Alternating TRT FP32 timing of the ONNX models on the Orin, with the average GPU clock during each run.
#   bash bench3.sh [rounds] [model ...]      (stop the cameras first; engines are cached in ~/diag/eng)
T=/usr/src/tensorrt/bin/trtexec
M=~/zed_ws/src/yolo_model
D=~/diag/eng; mkdir -p $D
R=${1:-2}; shift
MODELS=${@:-v26n_260327_260714 deim_n_person_stage1_zed deim_n_person_stage1_zed_lab}
F=$(ls /sys/devices/platform/*/17000000.gpu/devfreq/*/cur_freq /sys/devices/platform/17000000.gpu/devfreq/*/cur_freq 2>/dev/null | head -1)
for m in $MODELS; do
  [ -f $M/$m.onnx ] || { echo "missing $M/$m.onnx"; continue; }
  [ -f $D/$m.engine ] || { echo "building $m ..."; $T --onnx=$M/$m.onnx --saveEngine=$D/$m.engine --duration=1 > $D/$m.build.txt 2>&1; }
done
for r in $(seq $R); do
  for m in $MODELS; do
    [ -f $D/$m.engine ] || continue
    ( while :; do cat $F; sleep 0.5; done ) > $D/f.txt 2>/dev/null & P=$!
    $T --loadEngine=$D/$m.engine --duration=20 > $D/t.txt 2>&1
    kill $P; wait $P 2>/dev/null
    g=$(grep "GPU Compute Time" $D/t.txt | grep -oE "(min|mean|median) = [0-9.]+ ms" | tr '\n' ' ')
    f=$(awk '{s+=$1;n++}END{if(n)printf "%.0fMHz",s/n/1e6}' $D/f.txt)
    echo "r$r $m: $g gpu~$f"
  done
done
