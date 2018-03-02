"""
Common functions for sloth2tf.py and sloth2yolo.py to get data from the Sloth
JSON file.
"""
import os
import json
import math
import struct
import imghdr
import random

def getJson(file):
    """
    Load the JSON file
    """
    with open(file, 'r') as f:
        data = json.load(f)
    
    return data

def findFiles(folder):
    """
    Find all files recursively in specified folder
    """
    files = []

    for dirname, dirnames, filenames in os.walk(folder):
        for filename in filenames:
            files.append(os.path.join(dirname, filename))

    return files

def uniqueClasses(data):
    """
    Get all the labels in the file and alphabetically sort them
    """
    labels = []
    
    for f in data:
        for a in f['annotations']:
            if a['class'] not in labels:
                labels.append(a['class'])
    
    # Make it not depend on order of labels in the annotations
    labels.sort()
    
    return labels

def predefinedClasses():
    """
    Return a fixed set of classes for training on one dataset and testing on
    some subset of classes in a different dataset
    """
    return [ 'dog', 'food', 'glass', 'keys', 'leash', 'pillbottle', 'plant', 'umbrella', 'watercan' ]

def mapLabel(labels, label, zeroIndexed=False):
    """
    Convert the given label to an integer based on where it is in the labels array

    Note: YOLO needs zero-indexed. TensorFlow needs one-indexed.
    
    E.g.: mapLabel(['a', 'b', 'c', 'd'], 'c') # returns 2+1 = 3
    """
    assert label in labels, "Label must be in the list of labels"

    if zeroIndexed:
        return labels.index(label)
    else:
        return labels.index(label)+1

def getSize(filename):
    """
    Get the image size from an image file, see:
    https://stackoverflow.com/a/20380514/2698494
    """
    with open(filename, 'rb') as f:
        head = f.read(24)
        if len(head) != 24:
            return
        if imghdr.what(filename) == 'png':
            check = struct.unpack('>i', head[4:8])[0]
            if check != 0x0d0a1a0a:
                return
            width, height = struct.unpack('>ii', head[16:24])
        elif imghdr.what(filename) == 'gif':
            width, height = struct.unpack('<HH', head[6:10])
        elif imghdr.what(filename) == 'jpeg':
            try:
                f.seek(0) # Read 0xff next
                size = 2
                ftype = 0
                while not 0xc0 <= ftype <= 0xcf:
                    f.seek(size, 1)
                    byte = f.read(1)
                    while ord(byte) == 0xff:
                        byte = f.read(1)
                    ftype = ord(byte)
                    size = struct.unpack('>H', f.read(2))[0] - 2
                # We are at a SOFn block
                f.seek(1, 1)  # Skip `precision' byte.
                height, width = struct.unpack('>HH', f.read(4))
            except Exception: #IGNORE:W0703
                return
        else:
            return
        return width, height

def splitData(data, trainPercent, validPercent, limit=None):
    """
    Shuffle and then split the data into, e.g. 70% training, 10% validation,
    20% (remaining) testing.
    """
    # Shuffle
    random.shuffle(data)

    if limit:
        maxLen = min(limit, len(data))
    else:
        maxLen = len(data)

    # Calculate indices
    training_end = math.ceil(trainPercent*maxLen)
    validate_end = training_end + math.ceil(validPercent*maxLen)
    #testing_end  = remaining amount

    # Split
    training_data = data[:training_end]
    validate_data = data[training_end:validate_end]
    testing_data  = data[validate_end:maxLen]

    return training_data, validate_data, testing_data
