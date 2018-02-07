#!/bin/bash
. config.py

# We'd waste time if we transfer all the images since we're not using them all.
cat "$trainingPrefix"_100.txt "$testingFile" "$validateFile" > images.txt
cat images.txt | sed '
s#images#labels#gi
s#png#txt#gi
s#jpg#txt#gi
s#jpeg#txt#gi
' > labels.txt
cat images.txt labels.txt > files.txt
rm images.txt labels.txt

# Copy a few more files
echo "${datasetFolder}/${datasetConfig}" >> files.txt
echo "$datasetFolder/$datasetLabels" >> files.txt

# Compress it
[[ ! -e "$datasetFolder/$datasetCompressed" ]] && tar -cvzf "$datasetFolder/$datasetCompressed" -T files.txt
rm files.txt
