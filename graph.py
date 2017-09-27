#!/usr/bin/env python3
"""
Learning Curve

The graph of the learning curve from the Darknet testing results in results/.
"""

import os
import re
import sys
import random
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import seaborn as sns
from io import StringIO
from matplotlib.ticker import MaxNLocator

# From: https://stackoverflow.com/a/18603065/2698494
def getLastLine(filename):
    with open(filename, "rb") as f:
        first = f.readline()        # Read the first line.
        f.seek(-2, os.SEEK_END)     # Jump to the second last byte.
        while f.read(1) != b"\n":   # Until EOL is found...
            f.seek(-2, os.SEEK_CUR) # ...jump back the read byte plus one more.
        last = f.readline()         # Read last line.

    return last.decode("utf-8")

def percentageToFloat(x):
    return float(x.strip('%'))/100

def getResults(folder="results"):
    results = pd.DataFrame()
    amount_match = re.compile("[^0-9]*([0-9]+).txt")

    # Find all files recursively in specified folder
    for dirname, dirnames, filenames in os.walk(folder):
        for filename in filenames:
            # Get the number out of the filename indiciating number of training examples
            m = amount_match.match(filename)
            amount = int(m.groups()[0])

            # Get the results in the last line of the text file
            line = getLastLine(os.path.join(dirname, filename)).split()

            i = int(line[0])
            correct = int(line[1])
            total = int(line[2])
            proposals = float(line[4])
            iou = percentageToFloat(line[6])
            recall = percentageToFloat(line[7].split(":")[1])

            results = results.append(pd.DataFrame([[amount, i, correct, total, proposals, iou, recall]],
                columns=["Amount", "Images", "Correct", "Total", "Proposals", "Average IOU", "Recall"]))

    return results.sort_values("Amount")

def plotLearningCurve(title, y, x, curves, filename, loc=5):
    """
    Title
    Axis labels - y vs. x
    Curves - data to plot
    loc - place for legend
    """
    fig, ax = plt.subplots(1,1,figsize=(12, 6)) #,dpi=200)
    ax.xaxis.set_major_locator(MaxNLocator(integer=True))
    for i, (name, data) in enumerate(curves):
        plt.plot(data[:,0], data[:,1], color=plotColors[i],
                    marker=plotMarkers[i], label=name, linestyle=plotLines[i])
    plt.title(title)
    plt.ylabel(y)
    plt.xlabel(x)
    plt.legend(loc=loc)

    plt.savefig(filename+".png", bbox_inches='tight', pad_inches=0)

if __name__ == "__main__":
    # Make them look prettier
    plt.style.use('ggplot')
    #sns.set(style="ticks")
    sns.set_style("whitegrid")

    # For reproducibility
    random.seed(0)
    np.random.seed(0)

    # For graphing
    plotColors = ["r", "b", "g", "m", "y", "k", "c"]
    plotMarkers = ["s", "*", "x", "d", ".", "o", "v", "^", "<", ">", "1", "2", "3", "4", ]
    plotLines = ['-', '--', '-.', ':']*2

    # Get results
    results = getResults()

    # Plot and save
    plotLearningCurve("Learning Curve",
                      "Metric (between 0 and 1)", "Number of Training Examples",
                     [("Average IOU", results[['Amount','Average IOU']].values),
                     ("Recall", results[['Amount','Recall']].values)],
                      "LearningCurve")
