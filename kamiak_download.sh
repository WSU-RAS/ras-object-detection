#!/bin/bash
. config.py

# Note both have trailing slashes
from="$remotessh:$remotedir"
to="$localdir"

rsync -Pahuv --exclude="old" --exclude="old_v2" --include="*/" \
    --include="*_final.weights" --include="*.backup" --include="*.out*" \
    --include="*.err*" --exclude="*" "$from" "$to"

rsync -Pahuv "$from/datasets/$dataset/results/" "$to/datasets/$dataset/results/"
rsync -Pahuv "$from/datasets/$dataset/results_iterations/" "$to/datasets/$dataset/results_iterations/"
