#!/bin/bash
#
# If we oversample, the files are huge
#
# Note: gzip apparently is utterly useless on these .record files with
# duplicate images in them. It saved something like 1% of the 25 GiB.
# That would take a few seconds to upload onto Kamiak and make you also have to
# extract the image.
#
. config.py

[[ ! -e "$datasetFolder/$datasetTFcompressed" ]] && \
    tar -cvzf "$datasetFolder/$datasetTFcompressed" \
    "$datasetFolder/$datasetTFtrain" \
    "$datasetFolder/$datasetTFtest" \
    "$datasetFolder/$datasetTFvalid"
