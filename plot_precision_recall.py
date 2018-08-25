#!/usr/bin/env python3
import os
import re
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

def getResults(folder):
    pickle_match = re.compile("(.*).pickle")
    results = {}
    categories_first = None

    # Find all files recursively in specified folder
    for dirname, dirnames, filenames in os.walk(folder):
        for filename in filenames:
            # Get the number out of the filename indiciating number of training examples
            m = pickle_match.match(filename)

            if m is not None:
                name = m.groups()[0]
                data = loadPickle(os.path.join(dirname, filename))
                results[name] = {
                        'categories': data[0],
                        'average_precision_per_class': data[1],
                        'mean_ap': data[2],
                        'precisions_per_class': data[3],
                        'recalls_per_class': data[4],
                        'corloc_per_class': data[5],
                        'mean_corloc': data[6],
                    }

                # All should have the same categories
                if categories_first is None:
                    categories_first = data[0]
                else:
                    assert categories_first == data[0]

    return results

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
            bbox_inches='tight', pad_inches=0)#, transparent=True)

if __name__ == "__main__":
    results = getResults(config.datasetFolder)

    for key, values in results.items():
        print("Architecture:", key)
        curves = []
        for i, cat in enumerate(values['categories']):
            curves += [(cat['name'], values['recalls_per_class'][i], values['precisions_per_class'][i])]

        plotColors = sns.color_palette()
        #plotMarkers = ["s", "*", "x", "d", ".", "o", "v", "^", "<", ">", "1", "2", "3", "4" ]*5
        #plotLines = ['-', '--', '-.', ':']*10

        plotCurve("Precision-Recall Curves by Class with "+key, "Precision", "Recall", curves, "precision_recall_"+key)

        print("Average precision per class (sanity check):")
        for i, cat in enumerate(values['categories']):
            print(cat['name'], "-", values['average_precision_per_class'][i]*100, "%")
        print()
