#!/bin/bash
echo "Warning: if there's already .data files on Kamiak, it'll run on any not yet run on"

# Note both have trailing slashes
from="/home/garrett/Documents/School/17_Fall/CASAS/RAS/kamiak/grey_table/"
to="kamiak:/data/vcea/matt.taylor/Projects/ras-yolo/grey-table/"

# Update darknet
cd darknet
git pull
cd ..

# Copy only select files
#
# One directory: https://stackoverflow.com/a/21830454/2698494
# Recursive: https://stackoverflow.com/a/11111793/2698494
rsync -Pahuv --include="./" --include="*.txt" --include="*.data" \
    --include="*.names" --include="*.cfg" --include="*.sh" --include="*.py" \
    --exclude="*" "$from" "$to"

# Copy darknet
rsync -Pahuv "$from/darknet" "$to"
