#
# config file
# 
# This is for both Python and Bash, so... make sure syntax works in both, i.e.
# no spaces before or after equal signs and no variables. Probably a better way
# to do this.
#
dataset="SmartHome2"
datasetFolder="datasets/SmartHome2"

# Files for TensorFlow
datasetTFconfig="datasets/SmartHome2/tf.config"
datasetTFtrain="datasets/SmartHome2/tftrain.record"
datasetTFtest="datasets/SmartHome2/tftest.record"
datasetTFvalid="datasets/SmartHome2/tfvalid.record"
datasetTFlabels="datasets/SmartHome2/tf_label_map.pbtxt"
# Log files during training for TensorBoard
datasetTFtrainlogs="datasets/SmartHome2/tflogs/train"
datasetTFevallogs="datasets/SmartHome2/tflogs/eval"
# For evaluation, 0 means forever (until job ends)
maxTFEvals=0
# Which network to use (config is based on copying this sample), note that whenever
# you change this, you need to run ./tf_gen_config.sh again.
#TFArch="rfcn_resnet101"
#TFArch="ssd_mobilenet_v1"
TFArch="ssd_inception_v2"
#TFArch="faster_rcnn_resnet101"

# Files for YOLO
datasetLabels="datasets/SmartHome2/labels.names"
datasetConfig="datasets/SmartHome2/config.cfg"
datasetCompressed="datasets/SmartHome2/files.tar.gz"

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
localdir="/home/garrett/Documents/School/18_Spring/RAS/ras-object-detection/"
