#!/usr/bin/env python3
"""
Sloth to TensorFlow Object Detection

Convert the .json file generated with Sloth to be in the format for TF.
"""
import os
import imghdr
import random
import tensorflow as tf
from math import floor
from models.research.object_detection.utils import dataset_util

import config
from sloth_common import getJson, uniqueClasses, predefinedClasses, getSize, mapLabel, splitData

# Make this repeatable
random.seed(0)

def loadImage(filename):
    """
    TensorFlow needs the encoded data
    """
    with tf.gfile.GFile(filename, 'rb') as f:
        encoded = f.read()

    return encoded

def loadImageSimple(filename):
    with open(filename, 'rb') as f:
        encoded = f.read()

    return encoded

def bounds(x):
    """
    TensorFlow errors if we have a value less than 0 or more than 1. This
    occurs if in Sloth you draw a bounding box slightly out of the image. We'll
    just cut the bounding box at the edges of the images.
    """
    return max(min(x, 1), 0)

def create_tf_example(labels, filename, annotations, debug=False):
    """
    Based on:
    https://github.com/tensorflow/models/blob/master/research/object_detection/g3doc/using_your_own_dataset.md
    """
    if debug:
        print(filename)

    width, height = getSize(filename) # Image width and height
    encoded_image_data = loadImage(filename) # Encoded image bytes

    if imghdr.what(filename) == 'png':
        image_format = b'png' # b'jpeg' or b'png'
    elif imghdr.what(filename) == 'jpeg':
        image_format = b'jpeg'
    else:
        raise RuntimeError("Only supports PNG or JPEG images")

    xmins = []        # List of normalized left x coordinates in bounding box (1 per box)
    xmaxs = []        # List of normalized right x coordinates in bounding box (1 per box)
    ymins = []        # List of normalized top y coordinates in bounding box (1 per box)
    ymaxs = []        # List of normalized bottom y coordinates in bounding box (1 per box)
    classes_text = [] # List of string class name of bounding box (1 per box)
    classes = []      # List of integer class id of bounding box (1 per box)

    for a in annotations:
        # Numeric and text class labels
        classes.append(mapLabel(labels, a['class']))
        classes_text.append(a['class'].encode())

        # Scaled min/maxes
        xmins.append(bounds(a['x']/width))
        ymins.append(bounds(a['y']/height))
        xmaxs.append(bounds((a['x']+a['width'])/width))
        ymaxs.append(bounds((a['y']+a['height'])/height))

        # We got errors: maximum box coordinate value is larger than 1.010000
        valid = lambda x: x >= 0 and x <= 1
        assert valid(xmins[-1]) and valid(ymins[-1]) and valid(xmaxs[-1]) and valid(ymaxs[-1]), \
                "Invalid values for "+filename+": "+ \
                str(xmins[-1])+","+str(ymins[-1])+","+str(xmaxs[-1])+","+str(ymaxs[-1])

    tf_example = tf.train.Example(features=tf.train.Features(feature={
        'image/height': dataset_util.int64_feature(height),
        'image/width': dataset_util.int64_feature(width),
        'image/filename': dataset_util.bytes_feature(filename.encode()),
        'image/source_id': dataset_util.bytes_feature(filename.encode()),
        'image/encoded': dataset_util.bytes_feature(encoded_image_data),
        'image/format': dataset_util.bytes_feature(image_format),
        'image/object/bbox/xmin': dataset_util.float_list_feature(xmins),
        'image/object/bbox/xmax': dataset_util.float_list_feature(xmaxs),
        'image/object/bbox/ymin': dataset_util.float_list_feature(ymins),
        'image/object/bbox/ymax': dataset_util.float_list_feature(ymaxs),
        'image/object/class/text': dataset_util.bytes_list_feature(classes_text),
        'image/object/class/label': dataset_util.int64_list_feature(classes),
    }))
    return tf_example

def splitJsonData(data, trainPercent=0.7, validPercent=0.1):
    """
    Split the JSON data so we can get a training, validation, and testing file

    Returns pairs of (img filename, annotations) for each set
    """
    results = []

    for image in data:
        # Skip if we don't have any labels for this image
        if not len(image['annotations']) > 0:
            continue

        results.append((image['filename'], image['annotations']))

    return splitData(results, trainPercent, validPercent)

def splitJsonDataBalanced(data, trainPercent=0.7, validPercent=0.1, limit=None):
    """
    Split the JSON data so we can get a training, validation, and testing file
    However, due to class imbalances, split for each class. Oversample. Then
    combine.

    Limit is the max number of images for a class

    Returns pairs of (img filename, annotations) for each set
    """
    #
    # Get filenames/annotations from file by class (grouped by individual or
    # ones with multiple go in "other" group)
    #
    dataByClass = {}

    for image in data:
        # Skip if we don't have any labels for this image
        if not len(image['annotations']) > 0:
            continue

        classes = []

        for a in image['annotations']:
            if a['class'] not in classes:
                classes.append(a['class'])

        assert "other" not in classes, \
            "If you have a class named 'other' then you need to change this code."

        # Split all files with multiple classes separately
        #
        # simple case... but COCO human images (the reason I have to do any of
        # this) only have 1 since I only extracted the one class
        #
        # TODO make this properly handle the fact that images have multiple and
        # we really want to balance classes not images
        className = "other"

        # If there's just one
        if len(classes) == 1:
            className = classes[0]

        # If not already in the results, create a new array for images with
        # just this class
        if className not in dataByClass:
            dataByClass[className] = []

        # Save it
        dataByClass[className].append((image['filename'], image['annotations']))

    #
    # Split
    #
    splitByClass = {}

    for className, files in dataByClass.items():
        splitByClass[className] = splitData(files, trainPercent, validPercent, limit)

    #
    # Oversampling for training and validation data
    # And, while doing it, combine all the split test/train/valid datasets
    #
    # We don't need to oversample testing since it outputs testing by class
    # anyway, so we already know if human is good whereas keys is really bad.
    #
    train_data = []
    valid_data = []
    test_data = []

    # Get max train len (valid should be proportional)
    maxTrain = 0
    for className, (training_data, validate_data, testing_data), in splitByClass.items():
        if len(training_data) > maxTrain:
            maxTrain = len(training_data)

    print("Max training length:", maxTrain)

    for className, (training_data, validate_data, testing_data), in splitByClass.items():
        duplicateTimes = floor(maxTrain/len(training_data))

        # Oversample training and validation data
        for i in range(duplicateTimes):
            train_data += training_data
            valid_data += validate_data

        # Don't oversample testing
        test_data += testing_data

    printClassDistribution("Training", train_data)
    printClassDistribution("Validation", valid_data)
    printClassDistribution("Testing", test_data)

    return train_data, valid_data, test_data
    #return [],[],[]

def printClassDistribution(desc, data):
    classCount = {}

    for filename, annotations in data:
        for a in annotations:
            if a['class'] not in classCount:
                classCount[a['class']] = 0
            else:
                classCount[a['class']] += 1

    # Find total
    total = 0
    for className, count in classCount.items():
        total += count

    # Print
    print(desc)

    for className, count in classCount.items():
        print(className, ": ", count, " (", "%.2f"%(count/total*100), "%)", sep="")

    print()

def tfRecord(folder, labels, output, data):
    """
    Output to TF record file
    """
    with tf.python_io.TFRecordWriter(output) as writer:
        for (img, annotations) in data:
            filename = os.path.join(folder, img)
            tf_example = create_tf_example(labels, filename, annotations)
            writer.write(tf_example.SerializeToString())

def tfLabels(labels, output):
    with open(output, 'w') as f:
        for i, label in enumerate(labels):
            f.write('item {\n'+
                    '  id: '+str(i+1)+'\n'+
                    '  name: \''+label+'\'\n'+
                    '}\n')

def main(_):
    # Get JSON data
    dataset = config.dataset
    folder = config.datasetFolder
    data = getJson(os.path.join(folder, "sloth.json"))
    labels = uniqueClasses(data)
    #labels = predefinedClasses()

    # Save labels
    tfLabels(labels, os.path.join(folder, config.datasetTFlabels))

    # Split into 70%, 10%, and 20%
    training_data, validate_data, testing_data = splitJsonDataBalanced(data)
    #training_data, validate_data, testing_data = splitJsonData(data, trainPercent=0, validPercent=0)

    # Save the record files
    print("Saving", config.datasetTFtrain)
    tfRecord(folder, labels, os.path.join(folder, config.datasetTFtrain), training_data)
    print("Saving", config.datasetTFvalid)
    tfRecord(folder, labels, os.path.join(folder, config.datasetTFvalid), validate_data)
    print("Saving", config.datasetTFtest)
    tfRecord(folder, labels, os.path.join(folder, config.datasetTFtest), testing_data)

if __name__ == "__main__":
    tf.app.run()
