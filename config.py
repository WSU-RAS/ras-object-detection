#
# config file
# 
# This is for both Python and Bash, so... make sure syntax works in both, i.e.
# no spaces before or after equal signs and no variables. Probably a better way
# to do this.
#
dataset="SmartHome"
datasetFolder="datasets/SmartHome"

# Files for TensorFlow
datasetTFconfig="datasets/SmartHome/tf.config"
datasetTFtrain="datasets/SmartHome/tftrain.record"
datasetTFtest="datasets/SmartHome/tftest.record"
datasetTFvalid="datasets/SmartHome/tfvalid.record"
datasetTFlabels="datasets/SmartHome/tf_label_map.pbtxt"
# Log files during training for TensorBoard
datasetTFtrainlogs="datasets/SmartHome/tflogs/train"
datasetTFevallogs="datasets/SmartHome/tflogs/eval"
# Which network to use (config is based on copying this sample)
TFArch="rfcn_resnet101"
#TFArch="ssd_mobilenet_v1"
#TFArch="faster_rcnn_resnet101"

# Files for YOLO
datasetLabels="datasets/SmartHome/labels.names"
datasetConfig="datasets/SmartHome/config.cfg"
datasetCompressed="datasets/SmartHome/files.tar.gz"

dataPrefix="dataset"
trainingPrefix="training"
validateFile="validate.txt"
testingFile="testing.txt"
backupPrefix="backup" # remotedir/datasetFolder/backupPrefix_...
weightsName="darknet19_448.conv.23"
weightsDir="/data/vcea/matt.taylor/Projects/ras-object-detection/"

# Connecting to the remote server
# Note: this is rsync, so make sure all paths have a trailing slash
remotedir="/data/vcea/matt.taylor/Projects/ras-object-detection/"
remotessh="kamiak"
localdir="/home/garrett/Documents/School/17_Fall/CASAS/RAS/ras-object-detection/"
