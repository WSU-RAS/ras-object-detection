#!/bin/bash
# Add every N'th image to the list of images to annotate
# From: http://sloth.readthedocs.io/en/latest/examples.html
. config.py
N=10
out="datasets/$dataset/sloth.json"
[[ ! -e "$out" ]] && find "datasets/$dataset/images/" -iname "*.png" | sort | awk "NR%$N==1" | xargs sloth appendfiles "$out"
