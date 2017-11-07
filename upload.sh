#!/bin/bash
. config.py

echo "Warning: if there's already .data files on Kamiak, it'll run on any not yet run on"

# Note both have trailing slashes
from="$localdir"
to="$remotessh:$remotedir"

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
    --include="datasets" --include="$datasetFolder" --include="$datasetCompressed" \
    --exclude="*" "$from" "$to"

# Copy darknet
rsync -Pahuv "$from/darknet" "$to"
