#!/bin/bash
#
# squeezeDet apparently only works with the KITTI dataset's format, so we have
# to convert to that
#
# Note: probably will have to again parse the JSON file, so will later change
# this to a Python script
#
. config.py

# We need to specify the class labels in the Python files or import from a text
# file, so we'll copy it. We can't get the filename from the config file though
# so copy it to a constant name.
cp "$datasetLabels" "labels.names"

# TODO: convert ...
