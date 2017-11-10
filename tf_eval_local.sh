#!/bin/bash
#
# While training locally, also evaluate so we can view in TensorBoard
#  tensorboard --logdir /path/to/datasets/YourDataSet/tflogs/eval/
#
. config.py

cd models/research
mkdir -p "../../$datasetTFevallogs"
export PYTHONPATH="$PYTHONPATH:$(pwd):$(pwd)/slim"
python object_detection/eval.py \
    --checkpoint_dir="../../$datasetTFtrainlogs" \
    --eval_dir="../../$datasetTFevallogs" \
    --pipeline_config_path="../../$datasetTFconfig"
