#!/bin/bash
# Note both have trailing slashes
from="kamiak:/data/vcea/matt.taylor/Projects/ras-yolo/grey-table/"
to="/home/garrett/Documents/School/17_Fall/CASAS/RAS/kamiak/grey_table/"

# Copy darknet
rsync -Pahuv --include="*/" --include="*_final.weights" --include="*.backup" \
    --include="*.out" --include="*.err" \
    --exclude="*" "$from" "$to"
