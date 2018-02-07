#!/bin/bash
#
# While training locally, also evaluate so we can view in TensorBoard
#  tensorboard --logdir /path/to/datasets/YourDataSet/tflogs/eval/
#
. config.py

cd models/research
mkdir -p "../../$datasetFolder/$datasetTFevallogs"
export PYTHONPATH="$PYTHONPATH:$(pwd):$(pwd)/slim"
python object_detection/eval.py \
    --checkpoint_dir="../../$datasetFolder/$datasetTFtrainlogs" \
    --eval_dir="../../$datasetFolder/$datasetTFevallogs" \
    --pipeline_config_path="../../$datasetFolder/$datasetTFconfig"
