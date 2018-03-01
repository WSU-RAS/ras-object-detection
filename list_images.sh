#!/bin/bash
# Add every N'th image to the list of images to annotate
# From: http://sloth.readthedocs.io/en/latest/examples.html
. config.py
N=$1
[[ -z $N ]] && N=10
out="$datasetFolder/sloth.json"
echo "Outputting to: $out every $N frames"
[[ ! -e "$out" ]] && find "$datasetFolder/images/" -iname "*.png" -print0 | sort -z | awk -vORS=$'\0' "NR%$N==0" | xargs -0 sloth appendfiles "$out"
