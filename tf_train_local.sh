#!/bin/bash
#
# Test training locally
#
. config.py

TFArch=$1
if [[ -z $TFArch ]]; then
    echo "Specify which model to train."
    exit 1
fi

cd models/research
mkdir -p "../../$datasetFolder/$datasetTFtrainlogs/$TFArch"
export PYTHONPATH="$PYTHONPATH:$(pwd):$(pwd)/slim"
python object_detection/train.py \
    --train_dir="../../$datasetFolder/$datasetTFtrainlogs/$TFArch" \
    --pipeline_config_path="../../$datasetFolder/tf_${TFArch}.config"
