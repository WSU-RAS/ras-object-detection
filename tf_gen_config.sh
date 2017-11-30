#!/bin/bash
#
# Copy the sample config and change to use our dataset: the paths to our data
# and the number of classes we have
#
. config.py
maxExamples=2000

cp models/research/object_detection/samples/configs/"$TFArch"_pets.config \
    "$datasetTFconfig"

# Count labels
classes="$(grep "item" "$datasetTFlabels" | wc -l)"

# The ../../ is because we'll be running the training script from with
# models/research/
sed -ri "
s#num_classes: [0-9]+#num_classes: $classes#g
s#num_examples: [0-9]+#num_examples: $maxExamples#g
s#max_evals: [0-9]+#max_evals: $maxTFEvals#g
s#PATH_TO_BE_CONFIGURED/model.ckpt#PATH_TO_BE_CONFIGURED/${TFArch}_model.ckpt#g
s#PATH_TO_BE_CONFIGURED/pet_label_map.pbtxt#../../$datasetTFlabels#g
s#PATH_TO_BE_CONFIGURED/pet_train.record#../../$datasetTFtrain#g
s#PATH_TO_BE_CONFIGURED/pet_val.record#../../$datasetTFvalid#g
s#PATH_TO_BE_CONFIGURED/mscoco_label_map.pbtxt#../../$datasetTFlabels#g
s#PATH_TO_BE_CONFIGURED/mscoco_train.record#../../$datasetTFtrain#g
s#PATH_TO_BE_CONFIGURED/mscoco_val.record#../../$datasetTFvalid#g
s#PATH_TO_BE_CONFIGURED#../../$datasetFolder#g
" "$datasetTFconfig"
