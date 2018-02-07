#!/bin/bash
#
# While training locally, also evaluate so we can view in TensorBoard
#  tensorboard --logdir /path/to/datasets/YourDataSet/tflogs/eval/
#
. config.py

TFArch=$1
if [[ -z $TFArch ]]; then
    echo "Specify which model to train."
    exit 1
fi

cd models/research
mkdir -p "../../$datasetFolder/$datasetTFevallogs/$TFArch"
export PYTHONPATH="$PYTHONPATH:$(pwd):$(pwd)/slim"
python object_detection/eval.py \
    --checkpoint_dir="../../$datasetFolder/$datasetTFtrainlogs/$TFArch" \
    --eval_dir="../../$datasetFolder/$datasetTFevallogs/$TFArch" \
    --pipeline_config_path="../../$datasetFolder/tf_${TFArch}.config"
