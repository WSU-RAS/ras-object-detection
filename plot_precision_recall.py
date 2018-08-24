#!/usr/bin/env python3
import os
import pickle
import seaborn as sns
import matplotlib.pyplot as plt
from matplotlib.ticker import MaxNLocator

import config

def loadPickle(filename):
    """
    Load some data from a pickle file

    Usage:
        if os.path.exists("data.pickle"):
            data1, data2 = loadPickle("data.pickle")
    """
    with open(filename, 'rb') as f:
        data = pickle.load(f)

    return data

def plotCurve(title, ylabel, xlabel, curves, filename, loc='lower left'):
    """
    Title
    Axis labels - y vs. x
    Curves - data to plot
    loc - place for legend
    """
    fig, ax = plt.subplots(1,1,figsize=(8, 5),dpi=200)
    #ax.xaxis.set_major_locator(MaxNLocator(integer=True))
    for i, (name, x, y) in enumerate(curves):
        plt.plot(x, y, color=plotColors[i], label=name)
                    #marker=plotMarkers[i], linestyle=plotLines[i])
    plt.title(title)
    plt.ylabel(ylabel)
    plt.xlabel(xlabel)
    plt.legend(loc=loc)

    plt.savefig(os.path.join(config.datasetFolder, filename+".png"),
            bbox_inches='tight', pad_inches=0, transparent=True)

if __name__ == "__main__":
    filename = "precision-recall.pickle"
    classes = ['dog', 'food', 'glass', 'keys', 'leash', 'pillbottle', 'plant', 'umbrella', 'watercan']

    categories, average_precision_per_class, mean_ap, precisions_per_class, recalls_per_class, corloc_per_class, mean_corloc = loadPickle(filename)
    assert classes==categories, "mismatching/unexpected classes: "+str(categories)

    curves = []
    for i, classname in enumerate(classes):
        curves += [(classname, recalls_per_class[i], precisions_per_class[i])]

    plotColors = sns.color_palette()
    #plotMarkers = ["s", "*", "x", "d", ".", "o", "v", "^", "<", ">", "1", "2", "3", "4" ]*5
    #plotLines = ['-', '--', '-.', ':']*10

    plotCurve("Precision-Recall Curves by Class", "Precision", "Recall", curves, "precision-recall")

    print("Average precision per class (sanity check):")
    for i, classname in enumerate(classes):
        print(classname, "-", average_precision_per_class[i])
