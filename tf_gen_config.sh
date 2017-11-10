#!/bin/bash
#
# Copy the sample config and change to use our dataset: the paths to our data
# and the number of classes we have
#
. config.py

cp models/research/object_detection/samples/configs/faster_rcnn_resnet101_pets.config \
    "$datasetTFconfig"

# Count labels
classes="$(grep "item" "$datasetTFlabels" | wc -l)"

# The ../../ is because we'll be running the training script from with
# models/research/
sed -ri "
s#num_classes: [0-9]+#num_classes: $classes#g
s#PATH_TO_BE_CONFIGURED/pet_label_map.pbtxt#../../$datasetTFlabels#g
s#PATH_TO_BE_CONFIGURED/pet_train.record#../../$datasetTFtrain#g
s#PATH_TO_BE_CONFIGURED/pet_val.record#../../$datasetTFvalid#g
s#PATH_TO_BE_CONFIGURED#../../$datasetFolder#g
" "$datasetTFconfig"