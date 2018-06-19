#!/usr/bin/env python3
"""
Sloth to TensorFlow Object Detection

Convert the .json file generated with Sloth to be in the format for TF.
Also includes handling huge class imbalances, e.g. when you include some
classes from the COCO dataset.
"""
import os
import imghdr
import random
import operator
import tensorflow as tf
from math import floor, ceil
from models.research.object_detection.utils import dataset_util

import config
from sloth_common import getJson, uniqueClasses, predefinedClasses, \
    getSize, mapLabel, splitData

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

def splitJsonData(data, trainPercent=0.8, validPercent=0.2, shuffle=True):
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

    return splitData(results, trainPercent, validPercent, shuffle=shuffle)

def splitJsonDataBalanced(data, trainPercent=0.8, validPercent=0.2, limit=None):
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

    printFileDistribution(dataByClass)

    #
    # Split
    #
    splitByClass = {}

    for className, files in dataByClass.items():
        splitByClass[className] = splitData(files, trainPercent, validPercent, limit)

    #
    # Oversampling for training and testing, undersampling for validation
    #
    train_data = []
    valid_data = []
    test_data = []

    totalTrain, totalValid, totalTest = totalClasses(splitByClass)

    # Find class that has the most annotations
    maxTrain = max(totalTrain.items(), key=operator.itemgetter(1))[1]
    maxValid = max(totalValid.items(), key=operator.itemgetter(1))[1]
    maxTest = max(totalTest.items(), key=operator.itemgetter(1))[1]
    #print("maxTrain:", maxTrain)

    for className, (training_data, validate_data, testing_data), in splitByClass.items():
        # We want each image at least once
        train_data += training_data
        test_data += testing_data

        # Oversample (random selection with replacement) till we get the correct number
        if len(training_data) > 0:
            train_data += randomSelect(maxTrain, totalTrain, training_data, className)

        if len(testing_data) > 0:
            test_data += randomSelect(maxTest, totalTest, testing_data, className)

    # Find class that has the minimum annotations
    minTrain = min(totalTrain.items(), key=operator.itemgetter(1))[1]
    minValid = min(totalValid.items(), key=operator.itemgetter(1))[1]
    minTest = min(totalTest.items(), key=operator.itemgetter(1))[1]
    #print("minTrain:", minTrain)

    for className, (training_data, validate_data, testing_data), in splitByClass.items():
        if len(validate_data) > 0:
            valid_data += randomSelect(minValid, {}, validate_data, className)

    printClassDistribution("Training", train_data)
    printClassDistribution("Validation", valid_data)
    printClassDistribution("Testing", test_data)

    # Since we split by class, we shuffled in the classes but not among
    # classes, so this will have all humans, then all plants, then ...  which
    # ends up learning only humans, then only plants then, ... which doesn't
    # really work.
    random.shuffle(train_data)
    random.shuffle(valid_data)
    random.shuffle(test_data)

    return train_data, valid_data, test_data

def totalClasses(splitByClass):
    """
    Get the total number of each class in all the images for each split (train,
    test, valid)
    """
    # Get the max number of classes
    totalTrain = {}
    totalValid = {}
    totalTest = {}

    # Set all to zero initially
    for className, _ in splitByClass.items():
        if className != "other":
            totalTrain[className] = 0
            totalValid[className] = 0
            totalTest[className] = 0

    # Count classes in each image's annotations
    for className, (training_data, validate_data, testing_data) in splitByClass.items():
        for filename, annotations in training_data:
            counts = classCount(annotations)

            for name, amount in counts.items():
                totalTrain[name] += amount

        for filename, annotations in validate_data:
            counts = classCount(annotations)

            for name, amount in counts.items():
                totalValid[name] += amount

        for filename, annotations in testing_data:
            counts = classCount(annotations)

            for name, amount in counts.items():
                totalTest[name] += amount

    return totalTrain, totalValid, totalTest

def randomSelect(desired, totals, data, className):
    """
    Randomly select from the data until we have the desired number of the
    specified class
    """
    results = []

    if className in totals:
        count = totals[className]
    else:
        count = 0

    while count < desired:
        choice = random.choice(data)
        results.append(choice)

        if className == "other":
            # Approximate, won't end up being exact, but close enough
            count += classCount(choice[1], findMax=True)[1]
        else:
            count += classCount(choice[1])[className]

    return results

def classCount(annotations, findMax=False):
    """
    Count how many of each class is in an image

    If findMax is true, return the class that occurs the most frequently and
    the number of times it does
    """
    classes = {}

    for a in annotations:
        if a['class'] not in classes:
            classes[a['class']] = 1
        else:
            classes[a['class']] += 1

    if findMax:
        key = max(classes.items(), key=operator.itemgetter(1))[0]
        return key, classes[key]
    else:
        return classes

def printFileDistribution(dataByClass):
    """
    For debugging class imbalances, print out number of images having only one
    particular class. Images with multiple classes are grouped into "other".
    """
    print("File distribution:")
    for className, files in dataByClass.items():
        print(className, ": ", len(files), sep="")
    print()

def printClassDistribution(desc, data):
    """
    For debugging class imbalances, print out how many are in each class and
    the percentage of the total of that class in the final dataset
    """
    classCount = {}

    for filename, annotations in data:
        for a in annotations:
            if a['class'] not in classCount:
                classCount[a['class']] = 1
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

    # Print total files in each
    print("Total files:", len(data))

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

    # Option to generate a learning curve. Note the learning curve does *not*
    # balance classes.
    learningCurve = True

    if not learningCurve:
        # Split, e.g. 80% training, 20% validation, and 0% testing
        training_data, validate_data, testing_data = splitJsonDataBalanced(data, limit=20000)

        # Save the record files
        print("Saving", config.datasetTFtrain)
        tfRecord(folder, labels, os.path.join(folder, config.datasetTFtrain), training_data)
        print("Saving", config.datasetTFvalid)
        tfRecord(folder, labels, os.path.join(folder, config.datasetTFvalid), validate_data)
        print("Saving", config.datasetTFtest)
        tfRecord(folder, labels, os.path.join(folder, config.datasetTFtest), testing_data)

    else:
        # Shuffle only once. If we set shuffle=True for splitJsonData in the
        # for loop, then we'd end up with some of the validation/testing data
        # in the training data at some point (most likely). This ensures it's
        # shuffled, and then we just take less and less data for the learning
        # curve. The later training sets will not include the
        # validation/testing data since they are selected from the front of
        # this data array and each iteration of smaller size.
        random.shuffle(data)

        # Generate learning curve: 10%, 20%, ..., 90%, 100% of training data
        # of which the training data is 80% of the total data and 20% is validation data.
        #
        # Save the test set the first time, and leave it the same for all the others.
        for p in [1, 0.9, 0.8, 0.7, 0.6, 0.5, 0.4, 0.3, 0.2, 0.1]:
            percent = int(p*100)
            training_data, validate_data, testing_data = splitJsonData(data,
                    trainPercent=0.8*p, validPercent=0.2, shuffle=False)

            # Save the record files
            f = config.datasetTFtrain + "." + str(percent)
            print("Saving", f)
            tfRecord(folder, labels, os.path.join(folder, f), training_data)

            # Only generate one validation and test set
            if p == 1:
                print("Saving", config.datasetTFvalid, "(same for each train set)")
                tfRecord(folder, labels, os.path.join(folder, config.datasetTFvalid), validate_data)

                print("Saving", config.datasetTFtest, "(same for each train set)")
                tfRecord(folder, labels, os.path.join(folder, config.datasetTFtest), testing_data)

    #for f, a in training_data:
    #    print(f)

if __name__ == "__main__":
    tf.app.run()
