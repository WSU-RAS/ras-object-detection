#!/usr/bin/env python3
"""
Convert COCO human bounding boxes to Sloth JSON format
"""
import os
import sys
import json
from sloth_common import getJson

def getNames(data, classMap={}):
    """
    Returns dictionary { 1: "person", 2: "bicycle", ... }
    Swaps names to what's in classMap, if provided
    """
    names = {}

    for c in data["categories"]:
        if c["name"] in classMap:
            names[c["id"]] = classMap[c["name"]]
        else:
            names[c["id"]] = c["name"]

    return names

def getFilenames(data, imagePath):
    """
    Returns dictionary { "img id 1": "imagePath/filename 1", ... }
    """
    filenames = {}

    for img in data["images"]:
        filenames[img["id"]] = os.path.join(imagePath, img["file_name"])

    return filenames

def getBoundingBoxes(data):
    """
    Returns dictionary { "img id 1": [ (x,y,w,h), (x,y,w,y), ... ], ...}
    """
    boundingBoxes = {}

    for a in data["annotations"]:
        imageId = a["image_id"]
        categoryId = a["category_id"]
        x, y, w, h = a["bbox"]

        # There can be more than one bounding box per image
        if imageId not in boundingBoxes:
            boundingBoxes[imageId] = []

        boundingBoxes[imageId].append((categoryId, x, y, w, h))

    return boundingBoxes

def generateSloth(filenames, boundingBoxes, classNames, includeClasses):
    slothData = []

    for imageId, imgBoxes in boundingBoxes.items():
        # Generate each bounding box
        annotations = []

        for categoryId, x, y, w, h in imgBoxes:
            className = classNames[categoryId]

            if not includeClasses or className.lower() in includeClasses:
                annotations.append({
                        "class": className,
                        "height": h,
                        "width": w,
                        "x": x,
                        "y": y
                    })

        assert imageId in filenames, "Image ID not found in list of filenames!"

        # Generate annotation for this image, but only if there are some
        # bounding boxes for this image
        if annotations:
            slothData.append({
                    "annotations": annotations,
                    "class": "image",
                    "filename": filenames[imageId]
                })

    return slothData

def outputJson(data):
    print(json.dumps(data, indent=4))

if __name__ == "__main__":
    # Get JSON file
    if len(sys.argv) != 3 and len(sys.argv) != 4:
        #raise RuntimeError("coco2sloth.py person_keypoints_train2017.json path/to/images")
        raise RuntimeError("coco2sloth.py instances_train2017.json path/to/images [human,bicycle,...]")

    filename = sys.argv[1]
    imagePath = sys.argv[2]

    if len(sys.argv) == 4:
        includeClasses = sys.argv[3].lower().split(",")
    else:
        includeClasses = [ "human" ]

    # Load JSON file
    data = getJson(filename)

    # We've changed the names somewhat
    classMap = {
        "person": "human",
        "cup": "glass"
    } # bowl, chair

    # Get dictionary mapping category id to class name
    classNames = getNames(data, classMap)

    # Get dictionary of image filenames by id, so we can easily find the
    # filename
    filenames = getFilenames(data, imagePath)

    # Get bounding boxes for each image
    boundingBoxes = getBoundingBoxes(data)

    # Create Sloth JSON file
    slothData = generateSloth(filenames, boundingBoxes, classNames, includeClasses)

    # Output to stdout
    outputJson(slothData)
