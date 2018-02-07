#!/usr/bin/env python3
"""
Sloth to TensorFlow Object Detection

Convert the .json file generated with Sloth to be in the format for TF.
"""
import os
import random
import tensorflow as tf
from models.research.object_detection.utils import dataset_util

import config
from sloth_common import getJson, uniqueClasses, getSize, mapLabel, splitData

# Make this repeatable
random.seed(0)

def loadImage(filename):
    """
    TensorFlow needs the encoded data
    """
    with tf.gfile.GFile(filename, 'rb') as f:
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
    image_format = b'png' # b'jpeg' or b'png'

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

    # Save labels
    tfLabels(labels, os.path.join(folder, config.datasetTFlabels))

    # Split into 70%, 10%, and 20%
    training_data, validate_data, testing_data = splitJsonData(data)

    # Save the record files
    print("Saving", config.datasetTFtrain)
    tfRecord(folder, labels, os.path.join(folder, config.datasetTFtrain), training_data)
    print("Saving", config.datasetTFtrain)
    tfRecord(folder, labels, os.path.join(folder, config.datasetTFvalid), validate_data)
    print("Saving", config.datasetTFtest)
    tfRecord(folder, labels, os.path.join(folder, config.datasetTFtest), testing_data)

if __name__ == "__main__":
    tf.app.run()
