#!/usr/bin/env python3
"""
Sloth to YOLO

Convert the .json file generated with Sloth to be in the format for YOLO.
"""
import os
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

def saveYOLO(folder, data, labels, labellist):
    #images = []
    
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
        
        # Since we found annotations for this image, save it to train with
        images.append(filename)
    
    # Write out list of images
    #with open(imagelist, 'w') as f:
    #    for i in images:
    #        f.write(i + "\n")
            
    # Write out list of labels
    with open(labellist, 'w') as f:
        for l in labels:
            f.write(l + "\n")

if __name__ == "__main__":
    import config
    dataset = config.dataset
    folder = config.datasetFolder
    data = getJson(os.path.join(folder, "sloth.json"))
    labels = uniqueClasses(data)
    saveYOLO(folder, data, labels, config.datasetLabels)
