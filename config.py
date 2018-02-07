#
# config file
# 
# This is for both Python and Bash, so... make sure syntax works in both, i.e.
# no spaces before or after equal signs and no variables. Probably a better way
# to do this.
#
dataset="SmartHome2"
datasetFolder="datasets/SmartHome2"

# Files for TensorFlow - in datasetFolder
datasetTFconfig="tf.config"
datasetTFtrain="tftrain.record"
datasetTFtest="tftest.record"
datasetTFvalid="tfvalid.record"
datasetTFlabels="tf_label_map.pbtxt"
# Log files during training for TensorBoard - in datasetFolder
datasetTFtrainlogs="tflogs/train"
datasetTFevallogs="tflogs/eval"
# For evaluation, 0 means forever (until job ends)
maxTFEvals=0
# Which network to use (config is based on copying this sample), note that whenever
# you change this, you need to run ./tf_gen_config.sh again.
#TFArch="rfcn_resnet101"
#TFArch="ssd_mobilenet_v1"
TFArch="ssd_inception_v2"
#TFArch="faster_rcnn_resnet101"

# Files for YOLO - in datasetFolder
datasetLabels="labels.names"
datasetConfig="config.cfg"
datasetCompressed="files.tar.gz"

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
