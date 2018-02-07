#!/bin/bash
. config.py

# Note both have trailing slashes
from="$remotessh:$remotedir"
to="$localdir"

# YOLO backup files and weights, SLURM output files
rsync -Pahuv --exclude="old" --exclude="old_v2" --include="*/" \
    --include="backup_100/*_final.weights" --include="*.out*" \
    --include="*.err*" --exclude="*" "$from" "$to"

# TensorFlow checkpoints and logs
rsync -Pahuv --exclude="model.ckpt*" \
    "$from/${datasetFolder}/${datasetTFtrainlogs}/" \
    "$to/${datasetFolder}/${datasetTFtrainlogs}/"
rsync -Pahuv --exclude="model.ckpt*" \
    "$from/${datasetFolder}/${datasetTFevallogs}/" \
    "$to/${datasetFolder}/${datasetTFevallogs}/"

# TensorFlow exported models
rsync -Pahuv --include="*.pb/***" --exclude="*" \
    "$from/$datasetFolder/" "$to/$datasetFolder/"

# YOLO test results
rsync -Pahuv "$from/datasets/$dataset/results/" \
    "$to/datasets/$dataset/results/"
rsync -Pahuv "$from/datasets/$dataset/results_iterations/" \
    "$to/datasets/$dataset/results_iterations/"
