#!/usr/bin/env python3
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

Then it generates dataset_100.txt, dataset_200.txt, etc. for each of the
amounts of training data for the learning curve.

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
import os
import math
import random
import hashlib

# Make this repeatable
random.seed(0)

def findFiles(folder):
    """
    Find all files recursively in specified folder
    """
    files = []

    for dirname, dirnames, filenames in os.walk(folder):
        for filename in filenames:
            files.append(os.path.join(dirname, filename))

    return files

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

# From:
# https://stackoverflow.com/a/2507871/2698494
def isNotEmpty(filename):
    return os.stat(filename).st_size > 0

def removeDuplicates(data):
    """
    Remove duplicates in the data
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

def labelsToImages(labels):
    """
    Replacements to get image filename from labels. This is so that we only use
    labeled images for training. 
    """
    images = []

    for l in labels:
        # TODO: don't assume it's PNG; it could be .jpg, for instance
        images.append(l.replace("labels", "images").replace("txt", "png"))

    return images

if __name__ == "__main__":
    import config

    folder          = config.datasetFolder
    label_file      = config.datasetLabels
    data_prefix     = config.dataPrefix
    training_prefix = config.trainingPrefix
    validate_file   = config.validateFile
    testing_file    = config.testingFile
    backup_prefix   = os.path.join(config.remotedir, config.datasetFolder, config.backupPrefix)

    # Get lists of files in images/ and labels/ folders
    #images = findFiles(os.path.join(folder, "images"))
    labels = findFiles(os.path.join(folder, "labels"))
    # Get the images we have labels for
    images = labelsToImages(labels)

    # Should go together
    assert len(images) == len(labels), \
        "Must have the same number of images as labels"

    # Combine so we can shuffle
    combined = list(zip(images, labels))

    # Remove duplicates from the testing data
    # Note: skip since it turns out there are none (see with `jdupes .`)
    #print("Removing duplicates")
    #combined = removeDuplicates(combined)

    # Remove the data that isn't labeled (label files are blank)
    combined = removeBlank(combined)

    # Shuffle
    random.shuffle(combined)

    # If we need to split it back into images and labels...
    #images, labels = zip(*combined)

    # Split the data
    #
    # 70% training, 10% validation, 20% testing
    training_end = math.floor(0.7*len(combined))
    validate_end = training_end + math.floor(0.1*len(combined))
    #testing_end  = remaining amount

    # Turns out we don't care about the labels?
    training_data = combined[:training_end]
    validate_data = combined[training_end:validate_end]
    testing_data  = combined[validate_end:]

    #with open(os.path.join(training_prefix + '_all.txt'), 'w') as f:
    #    for img, label in training_data:
    #        f.write(img + "\n")

    # For now don't train on all since we only skip 3 images when doing the
    # learning curve by 1000s
    #createDataFile(data_prefix + '_all.data', training_prefix +
    #        '_all.txt', validate_file, label_file, backup_prefix)

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
