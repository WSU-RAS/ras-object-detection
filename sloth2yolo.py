#!/usr/bin/env python3
"""
Sloth to YOLO

Convert the .json file generated with Sloth to be in the format for YOLO.
"""
import os
from sloth_common import getJson, uniqueClasses, getSize, mapLabel

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
