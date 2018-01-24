#!/usr/bin/python
import csv
import os
import sys

import numpy as np
import matplotlib.pyplot as plt


data = {}
names = []
titlename = ['Speedup in Time', 'Reduction in Number of Expansions']
ylabel = ['Ratio of Computation Time', 'Ratio of Number of Expansions']

fs = 15 # fontsize 

def plot(num):
    plt.figure(num, figsize=(9, 6))
    index = num*2+1
    plot_data = []
    labels = []
    for density in data[names[index]]:
        labels.append(density)
    labels = np.sort(labels)

    length = len(data[names[index]])
    means = np.zeros(length)
    plot_data = np.ndarray(shape=(length,), dtype="object")
    j = 0
    for density in data[names[index]]:
        t0 = np.array(data[names[index]][density])
        t1 = np.array(data[names[index+1]][density])
        t = t1 / t0;
        j = np.argwhere(labels == density)
        plot_data[j[0][0]] = t
        means[j[0][0]] = np.mean(t)

    boxprops = dict(linestyle='-', linewidth=1.5)
    meanlineprops = dict(linestyle='-', linewidth=2.5, color='green')
    meanpointprops = dict(marker='o', markeredgecolor='green',
                          markersize=12, markerfacecolor='none', linewidth=2.5, linestyle='-')
    medianprops = dict(linestyle='-', linewidth=1.5, color='red')
    plt.boxplot(plot_data, whis='range',
            labels = labels, showmeans=True, meanline=True, 
            meanprops=meanlineprops, boxprops=boxprops, medianprops=medianprops)
    plt.plot(np.arange(length) + 1, means, c="g", lw=2, marker='o')
    plt.plot(np.arange(length+2), np.ones(length+2), lw=2, linestyle='--', color='black')
    plt.title(titlename[num]+"("+title+")", fontsize=fs)
    plt.xlabel(names[0] + "(%)", fontsize=fs)
    plt.ylabel(ylabel[num], fontsize=fs)
    plt.xticks(fontsize=12) 
    plt.yticks(fontsize=12) 
    plt.savefig(title+"-"+str(num)+".eps", bbox_inches='tight')


def read_file(filename) :
    with open(filename, 'rb') as  csvfile:
        reader = csv.reader(csvfile, delimiter=',')
        i = 0
        for row in reader:
            if i == 0:
                j = 0
                for element in row:
                    names.append(element)
                    if j > 0:
                        data[element] = {}
                    j += 1
            else:
                j = 0
                density = 0
                for element in row:
                    if j > 0:
                        if density not in data[names[j]]:
                            data[names[j]][density] = [float(element)]
                        else:
                            data[names[j]][density].append(float(element))
                    else:
                        density = float(element) * 100
                    j += 1

            i += 1

        plot(0)
        plot(1)
        plt.show()

if __name__=="__main__":
    if(len(sys.argv) < 2):
        print "Must set input file!"
    else:
        for csvfile in sys.argv:
            path = csvfile
        filename = os.path.basename(path)
        filename = filename.split('.')
        title = filename[0]
 
        read_file(path)

