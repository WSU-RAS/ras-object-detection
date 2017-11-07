#!/usr/bin/env python3
"""
Sloth to TensorFlow Object Detection

Convert the .json file generated with Sloth to be in the format for TF.
"""
import os
import config
from sloth_common import getJson, uniqueClasses, getSize, mapLabel

import tensorflow as tf
from models.research.object_detection.utils import dataset_util

def loadImage(filename):
    """
    TensorFlow needs the encoded data
    """
    with tf.gfile.GFile(filename, 'rb') as f:
        encoded = f.read()

    return encoded

def create_tf_example(labels, filename, annotations, debug=True):
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
        xmins.append(a['x']/width)
        ymins.append(a['y']/height)
        xmaxs.append((a['x']+a['width'])/width)
        ymaxs.append((a['y']+a['height'])/height)

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

def main(_):
    # Get JSON data
    dataset = config.dataset
    folder = config.datasetFolder
    data = getJson(os.path.join(folder, "sloth.json"))
    labels = uniqueClasses(data)

    # Output to TF file
    with tf.python_io.TFRecordWriter(config.datasetTF) as writer:
        for image in data:
            # Skip if we don't have any labels for this image
            if not len(image['annotations']) > 0:
                continue

            filename = os.path.join(folder, image['filename'])
            tf_example = create_tf_example(labels, filename, image['annotations'])
            writer.write(tf_example.SerializeToString())

if __name__ == "__main__":
    tf.app.run()
