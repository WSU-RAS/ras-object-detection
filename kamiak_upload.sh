#!/bin/bash
. config.py

echo "Warning: if there's already .data files on Kamiak, it'll run on any not yet run on"

# Note both have trailing slashes
from="$localdir"
to="$remotessh:$remotedir"

# Update darknet
#git submodule update
#cd darknet
#git pull
#cd ..

# Copy only select files
#
# One directory: https://stackoverflow.com/a/21830454/2698494
# Recursive: https://stackoverflow.com/a/11111793/2698494
rsync -Pahuv --include="./" --include="*.txt" --include="*.data" \
    --include="*.names" --include="*.cfg" --include="*.sh" \
    --include="*.config" --include="*.srun" --include="*.py" \
    --include="datasets" --include="$datasetFolder" \
    --include="$datasetFolder/$datasetCompressed" \
        --include="*model.ckpt.*" \
        --include="$datasetFolder/$datasetTFcompressed" \
        --include="$datasetFolder/$datasetTFtrain" \
        --include="$datasetFolder/$datasetTFvalid" \
        --include="$datasetFolder/$datasetTFtest" \
        --include="$datasetFolder/$datasetTFlabels" \
    --exclude="*" "$from" "$to"

# Copy submodules
rsync -Pahuv "$from/darknet" "$to"
rsync -Pahuv "$from/models" --exclude="__pycache__" "$to"

# Make SLURM log folder
ssh "$remotessh" "mkdir -p $remotedir/slurm_logs"
