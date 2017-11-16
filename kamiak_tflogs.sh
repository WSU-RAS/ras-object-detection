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
    rsync -Pahuv "$from/datasets/$dataset/tf/" "$to/datasets/$dataset/tf/"
    sleep 30
done
