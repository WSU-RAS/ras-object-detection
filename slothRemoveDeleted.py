"""
Creating a new dataset from part of the old dataset, I only copied some of the
images, and I'd rather not re-label them all. This removes all the deleted
images from the Sloth JSON file. (Or, rather, just doesn't print them out.)

It can also remove a set of classes if desired.
"""
import os
import sys
import json
import config

def getJson(file):
    """
    Load the JSON file
    """
    with open(file, 'r') as f:
        data = json.load(f)
    
    return data

def removeDeleted(data, ignoreClasses=[]):
    """
    If the image doesn't exist, then remove it from the JSON file
    """
    newData = []

    for d in data:
        if os.path.exists(os.path.join(config.datasetFolder, d["filename"])):
            filename = d["filename"]
            className = d["class"]
            annotations = []

            for a in d["annotations"]:
                if a["class"] not in ignoreClasses:
                    annotations.append(a)

            # If we still have some annotations...
            if annotations:
                newData.append({
                        "annotations": annotations,
                        "class": className,
                        "filename": filename
                    })
            else:
                # Removed all annotations due to the skipped ones, so we don't
                # really need this file. Alert user.
                print("rm", d["filename"], file=sys.stderr)

    return newData

def outputJson(data):
    print(json.dumps(data, indent=4))

if __name__ == "__main__":
    data = getJson(os.path.join(config.datasetFolder, "sloth.json"))
    data = removeDeleted(data, ignoreClasses=["food", "watercan"])
    outputJson(data)
