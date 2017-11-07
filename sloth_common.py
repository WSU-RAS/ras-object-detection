"""
Common functions for sloth2tf.py and sloth2yolo.py to get data from the Sloth
JSON file.
"""
import json
import struct
import imghdr

def getJson(file):
    """
    Load the JSON file
    """
    with open(file, 'r') as f:
        data = json.load(f)
    
    return data

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

def mapLabel(labels, label):
    """
    Convert the given label to an integer based on where it is in the labels array
    
    E.g.: mapLabel(['a', 'b', 'c', 'd'], 'c') # returns 2
    """
    assert label in labels, "Label must be in the list of labels"
    return labels.index(label)

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
