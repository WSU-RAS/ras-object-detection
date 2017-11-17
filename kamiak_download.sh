#!/bin/bash
. config.py

# Note both have trailing slashes
from="$remotessh:$remotedir"
to="$localdir"

# YOLO backup files and weights, SLURM output files
rsync -Pahuv --exclude="old" --exclude="old_v2" --include="*/" \
    --include="*_final.weights" --include="*.backup" --include="*.out*" \
    --include="*.err*" --exclude="*" "$from" "$to"

# TensorFlow checkpoints and logs
rsync -Pahuv "$from/$datasetTFtrainlogs/" "$to/$datasetTFtrainlogs/"
rsync -Pahuv "$from/$datasetTFevallogs/" "$to/$datasetTFevallogs/"

# YOLO test results
rsync -Pahuv "$from/datasets/$dataset/results/" "$to/datasets/$dataset/results/"
rsync -Pahuv "$from/datasets/$dataset/results_iterations/" "$to/datasets/$dataset/results_iterations/"
