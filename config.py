#
# config file
# 
# This is for both Python and Bash, so... make sure syntax works in both, i.e.
# no spaces before or after equal signs and no variables.
#
dataset="SmartHome"
datasetFolder="datasets/SmartHome"
datasetLabels="datasets/SmartHome/labels.names"
datasetConfig="datasets/SmartHome/config.cfg"
datasetCompressed="datasets/SmartHome/files.tar.gz"
datasetTFtrain="datasets/SmartHome/tftrain.record"
datasetTFtest="datasets/SmartHome/tftest.record"
datasetTFvalid="datasets/SmartHome/tfvalid.record"

# For generating the lists of what data to use for training
dataPrefix="dataset"
trainingPrefix="training"
validateFile="validate.txt"
testingFile="testing.txt"
backupPrefix="backup" # remotedir/datasetFolder/backupPrefix_...

# Connecting to the remote server
remotedir="/data/vcea/matt.taylor/Projects/ras-object-detection/"
remotessh="kamiak"
localdir="/home/garrett/Documents/School/17_Fall/CASAS/RAS/ras-object-detection/"

# For training
weightsName="darknet19_448.conv.23"
weightsDir="/data/vcea/matt.taylor/Projects/ras-object-detection/"
