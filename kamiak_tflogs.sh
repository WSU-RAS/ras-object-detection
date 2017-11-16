#!/bin/bash
#
# Download the TF logs every once in a while to keep TensorBoard updated
# Then run: tensorboard  --logdir datasets/SmartHome/tflogs/train/
#
. config.py

# Note both have trailing slashes
from="$remotessh:$remotedir"
to="$localdir"

# TensorFlow logs
while true; do
    rsync -Pahuv --exclude="model.ckpt*" "$from/$datasetTFtrainlogs" "$to/$datasetTFtrainlogs"
    rsync -Pahuv --exclude="model.ckpt*" "$from/$datasetTFevallogs" "$to/$datasetTFevallogs"
    sleep 30
done
