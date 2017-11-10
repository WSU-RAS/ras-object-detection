#!/usr/bin/env python3
"""
Sloth to YOLO

Convert the .json file generated with Sloth to be in the format for YOLO.
"""
import os
import math
import random
import hashlib

import config
from sloth_common import getJson, uniqueClasses, getSize, mapLabel, splitData

# Make this repeatable
random.seed(0)

# From:
# https://stackoverflow.com/a/2507871/2698494
def isNotEmpty(filename):
    """
    Check that a file is not empty
    """
    return os.stat(filename).st_size > 0

# Based on:
# http://pythoncentral.io/finding-duplicate-files-with-python/
def hashfile(filename, blocksize=65536):
    hasher = hashlib.md5()

    with open(filename, 'rb') as f:
        while True:
            buf = f.read(blocksize)
            hasher.update(buf)

            if not buf:
                break

    return hasher.hexdigest()

def createDataFile(outputFile, train_file, valid_file, label_file, backup_dir):
    """
    Example output file:

    classes = 3
    train = training.txt
    valid = validate.txt
    names = datasets/YourDataSet/labels.names
    backup = /data/vcea/matt.taylor/Projects/ras-object-detection/datasets/YourDataSet/backup_10
    eval = wsu
    """

    # Count labels
    with open(label_file) as f:
        label_count = sum(1 for _ in f)

    # Output file
    with open(outputFile, 'w') as f:
        f.write("classes = " + str(label_count) + "\n")
        f.write("train = " + train_file + "\n")
        f.write("valid = " + valid_file + "\n")
        f.write("names = " + label_file + "\n")
        f.write("backup = " + backup_dir + "\n")
        f.write("eval = wsu\n")

def removeDuplicates(data):
    """
    Remove duplicates in the data, either duplicate labels or duplicate images

    Note: turns out there aren't any, so not actually used...
    """
    print("Before removing duplicates:", len(data), "examples")

    duplicates = {}
    cleanData = []

    for img, label in data:
        file_hash = hashfile(img)

        if file_hash in duplicates:
            duplicates[file_hash].append(img)
        else:
            duplicates[file_hash] = [img]

            # Not a duplicate, so use this in the test
            cleanData.append((img, label))

    print("After removing duplicates:", len(cleanData), "examples")

    return cleanData

def removeBlank(data):
    """
    Remove images with blank label files

    I think these blank label files is what caused a bunch of NaN's while training.

        "When the annotation data is not correct, by which I mean there exists
        a training image whose annotation is empty."

        ~ http://guanghan.info/blog/en/my-works/train-yolo/
    """
    print("Before removing blanks:", len(data), "examples")

    cleanData = []

    for img, label in data:
        if isNotEmpty(label):
            cleanData.append((img, label))

    print("After removing blanks:", len(cleanData), "examples")

    return cleanData

def yoloLabels(folder, data, labels, labellist):
    """
    Generate the labels/ directory with one file per image specifying the
    locations of the bounding boxes
    """
    results = []
    
    for image in data:
        filename = os.path.join(folder, image['filename'])
        output = os.path.splitext(filename.replace("images", "labels"))[0] + ".txt"
        x, y = size = getSize(filename)
        
        # Skip if we don't have any labels for this image
        if not len(image['annotations']) > 0:
            continue

        # Make the output directory if it doesn't exist
        if not os.path.exists(os.path.dirname(output)):
            os.makedirs(os.path.dirname(output))
        
        with open(output, 'w') as f:
            for a in image['annotations']:
                # Get the class number
                num = mapLabel(labels, a['class'])
                # Convert to YOLO format, see:
                # https://github.com/PalouseRobosub/vision_dev/blob/master/sloth/annotation_containers/darknet.py
                xpos = (a['width'] / 2 + a['x']) / x
                ypos = (a['height'] / 2 + a['y']) / y
                width = a['width'] / x
                height = a['height'] / y
                f.write("{} {} {} {} {}\n".format(num, xpos, ypos, width, height))

                assert not (xpos < 0 or ypos < 0 or width < 0 or height < 0 or \
                        xpos > 1 or ypos > 1 or width > 1 or height > 1), \
                        "Values must be in range [0,1]"
        
        # Save (image file, label file) pairs
        results.append((filename, output))
            
    # Write out list of labels
    with open(labellist, 'w') as f:
        for l in labels:
            f.write(l + "\n")

    return results

def yoloSplit(data, detectDuplicates=False, detectBlanks=False,
        trainPercent=0.7, validPercent=0.1):
    """
    Generate files for YOLO

    You need to have this data (where YourDataSet is specified in the config):
      datasets/YourDataSet/images
      datasets/YourDataSet/labels

    This script will generate 3 files containing a shuffled subset of the image
    filenames:
        training.txt (70% of the images)
        validate.txt (10% of the images)
        testing.txt  (20% of the images)

    Then it generates dataset_10.txt, dataset_20.txt, etc. for each of the
    percentage amounts of training data for the learning curve.

    Note:
     * darknet replaces
         "images" -> "labels"
         "jpg" -> "txt"
         "png" -> "txt"
       in the filename/path to each image to get the labels, so they must only
       differ by that
     * The testing filename must be testing.txt since it is hard-coded in darknet's
       examples/detector.c validate_detector_recall function.
    """
    # Get all these options from the config file
    label_file      = config.datasetLabels
    data_prefix     = config.dataPrefix
    training_prefix = config.trainingPrefix
    validate_file   = config.validateFile
    testing_file    = config.testingFile
    backup_prefix   = os.path.join(config.remotedir, config.datasetFolder, config.backupPrefix)

    # We shouldn't have duplicates or blank label files, but if in doubt, you
    # can run these
    #
    # Remove duplicates from the testing data
    if detectDuplicates:
        data = removeDuplicates(data)

    # Remove the data that isn't labeled (label files are blank)
    if detectBlanks:
        data = removeBlank(data)

    # Split data
    training_data, validate_data, testing_data = splitData(data, trainPercent, validPercent)

    with open(validate_file, 'w') as f:
        for img, label in validate_data:
            f.write(img + "\n")

    with open(testing_file, 'w') as f:
        for img, label in testing_data:
            f.write(img + "\n")

    # For the learning curve, vary the number of examples in the training data
    # Do 10%, 20%, 30%, ... 100% percent
    amounts = [(i, math.floor(i/100*len(training_data))) for i in range(10,110,10)]

    for (percent, amount) in amounts:
        filename = training_prefix+'_'+str(percent)+'.txt'

        with open(filename, 'w') as f:
            for img, label in training_data[:amount]:
                f.write(img + "\n")

        createDataFile(data_prefix + '_' + str(percent) + '.data',
                filename, validate_file, label_file, backup_prefix + '_' +
                str(percent))

if __name__ == "__main__":
    # Get Sloth file
    dataset = config.dataset
    folder = config.datasetFolder
    data = getJson(os.path.join(folder, "sloth.json"))

    # Get the labels out of the Sloth file
    labels = uniqueClasses(data)

    # Generate label files
    results = yoloLabels(folder, data, labels, config.datasetLabels)

    # Generate lists for training, e.g. the 10% training, 20% training,
    # validation, testing sets, etc.
    yoloSplit(results)
