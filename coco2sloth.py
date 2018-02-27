#!/usr/bin/env python3
"""
Convert COCO human bounding boxes to Sloth JSON format
"""
import os
import sys
import json
from sloth_common import getJson

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

        assert categoryId == 1, "Category ID is not 1, i.e. is not person!"

        # There can be more than one bounding box per image
        if imageId not in boundingBoxes:
            boundingBoxes[imageId] = []

        boundingBoxes[imageId].append((x, y, w, h))

    return boundingBoxes

def generateSloth(filenames, boundingBoxes):
    slothData = []
    className = "human" # That's what this COCO JSON file is for...

    for imageId, imgBoxes in boundingBoxes.items():
        # Generate each bounding box
        annotations = []

        for x, y, w, h in imgBoxes:
            annotations.append({
                    "class": className,
                    "height": h,
                    "width": w,
                    "x": x,
                    "y": y
                })

        assert imageId in filenames, "Image ID not found in list of filenames!"

        # Generate annotation for this image
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
    if len(sys.argv) != 3:
        raise RuntimeError("coco2sloth.py person_keypoints_train2017.json path/to/images")

    filename = sys.argv[1]
    imagePath = sys.argv[2]

    # Load JSON file
    data = getJson(filename)

    # Get dictionary of image filenames by id, so we can easily find the
    # filename
    filenames = getFilenames(data, imagePath)

    # Get bounding boxes for each image
    boundingBoxes = getBoundingBoxes(data)

    # Create Sloth JSON file
    slothData = generateSloth(filenames, boundingBoxes)

    # Output to stdout
    outputJson(slothData)
