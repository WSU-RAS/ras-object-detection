#!/bin/bash
#
# Shows the class [im]balance in your sloth.json file
#
. config.py
echo "Class [im]balance in sloth.json:"
grep 'class": "' "$datasetFolder/sloth.json" | grep -v '"class": "image"' | sort | uniq -c
