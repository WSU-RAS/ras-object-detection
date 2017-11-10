#!/bin/bash
#
# Test training locally
#
. config.py

cd models/research
mkdir -p "../../$datasetTFtrainlogs"
export PYTHONPATH="$PYTHONPATH:$(pwd):$(pwd)/slim"
python object_detection/train.py \
    --train_dir="../../$datasetTFtrainlogs" \
    --pipeline_config_path="../../$datasetTFconfig"
