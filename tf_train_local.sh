#!/bin/bash
#
# Test training locally
#
. config.py

cd models/research
mkdir -p "../../$datasetFolder/$datasetTFtrainlogs"
export PYTHONPATH="$PYTHONPATH:$(pwd):$(pwd)/slim"
python object_detection/train.py \
    --train_dir="../../$datasetFolder/$datasetTFtrainlogs" \
    --pipeline_config_path="../../$datasetFolder/$datasetTFconfig"
