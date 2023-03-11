import matplotlib as mpl
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap, colorConverter, LinearSegmentedColormap
import numpy as np

from sklearn.decomposition import PCA
from cycler import cycler

cm_cycle = ListedColormap(['#0000aa', '#ff5050', '#50ff50', '#9040a0', '#fff000'])
cm3 = ListedColormap(['#0000aa', '#ff2020', '#50ff50'])
cm2 = ListedColormap(['#0000aa', '#ff2020'])

cdict = {'red': [(0.0, 0.0, cm2(0)[0]),
                 (1.0, cm2(1)[0], 1.0)],

         'green': [(0.0, 0.0, cm2(0)[1]),
                   (1.0, cm2(1)[1], 1.0)],

         'blue': [(0.0, 0.0, cm2(0)[2]),
                  (1.0, cm2(1)[2], 1.0)]}

ReBl = LinearSegmentedColormap("ReBl", cdict)
plt.rc('axes', prop_cycle=(
    cycler('color', cm_cycle.colors) +
    cycler('linestyle', ['-', '-', "--", (0, (3, 3)), (0, (1.5, 1.5))])))


def discrete_scatter(x1, x2, y=None, markers=None, s=10, ax=None,
                     labels=None, padding=.2, alpha=1, c=None, markeredgewidth=None):
    """Adaption of matplotlib.pyplot.scatter to plot classes or clusters.

    Parameters
    ----------

    x1 : nd-array
        input data, first axis

    x2 : nd-array
        input data, second axis

    y : nd-array
        input data, discrete labels

    markers : list of string
        List of markers to use, or None (which defaults to 'o').

    s : int or float
        Size of the marker
    
    cmap : colormap
        Colormap to use.

    labels : nd-array
        input data, labels names

    ax : existing matplotlib axis, or None
    
    padding : float
        Fraction of the dataset range to use for padding the axes.

    alpha : float
        Alpha value for all points.
    """
    if ax is None:
        ax = plt.gca()

    if y is None:
        y = np.zeros(len(x1))

    unique_y = np.unique(y)

    if markers is None:
        markers = ['o', '^', 'v', 'D', 's', '*', 'p', 'h', 'H', '8', '<', '>'] * 10

    if len(markers) == 1:
        markers = markers * len(unique_y)

    if labels is None:
        labels = unique_y

    # lines in the matplotlib sense, not actual lines
    lines = []

    current_cycler = mpl.rcParams['axes.prop_cycle']

    for i, (yy, cycle) in enumerate(zip(unique_y, current_cycler())):
        mask = y == yy
        # if c is none, use color cycle
        if c is None:
            color = cycle['color']
        elif len(c) > 1:
            color = c[i]
        else:
            color = c
        # use light edge for dark markers
        if np.mean(colorConverter.to_rgb(color)) < .4:
            markeredgecolor = "grey"
        else:
            markeredgecolor = "black"

        lines.append(ax.plot(x1[mask], x2[mask], markers[i], markersize=s,
                             label=labels[i], alpha=alpha, c=color,
                             markeredgewidth=markeredgewidth,
                             markeredgecolor=markeredgecolor)[0]
                    )

    ax.legend(labels, loc="best")
    
    if padding != 0:
        pad1 = x1.std() * padding
        pad2 = x2.std() * padding
        xlim = ax.get_xlim()
        ylim = ax.get_ylim()
        ax.set_xlim(min(x1.min() - pad1, xlim[0]), max(x1.max() + pad1, xlim[1]))
        ax.set_ylim(min(x2.min() - pad2, ylim[0]), max(x2.max() + pad2, ylim[1]))

    return lines

def axis_labels(x, y, axlabelfontsize=None, axlabelfontname=None):
    plt.xlabel(x, fontsize=axlabelfontsize, fontname=axlabelfontname)
    plt.ylabel(y, fontsize=axlabelfontsize, fontname=axlabelfontname)

def screeplot(obj="pcascree", axlabelfontsize=9, axlabelfontname="sans-serif",
              axxlabel=None, axylabel=None, dim=(12, 12)):
    ''' Plot a bar plot '''
    y = [x * 100 for x in obj[1]]
    
    plt.subplots(figsize=dim)
    plt.bar(obj[0], y)
    
    if axxlabel:
        xlab = axxlabel
    else:
        xlab='PCs'
    if axylabel:
        ylab = axylabel
    else:
        ylab='Proportion of variance (%)'
    
    plt.xticks(fontsize=7, rotation=70)
    axis_labels(xlab, ylab, axlabelfontsize, axlabelfontname)
    plt.show()

      
def discrete_biplot(cscore, loadings, explained_variance_ratio_, displaydata = True,
                    y=None, ylabels=None,
                    markers=None, s=10, ax=None,
                    arrowcolor='#87ceeb', alphaarrow=1, ls='-', lw=0.5,
               loadlabels=None, padding=.2, alpha=1, c=None, markeredgewidth=None,
               axlabelfontsize=9, axlabelfontname="DejaVu Sans"):
    ''' Adapting biplot from bio... library'''
    if ax is None:
        ax = plt.gca()

    xscale = 1.0 / (cscore[:, 0].max() - cscore[:, 0].min())
    yscale = 1.0 / (cscore[:, 1].max() - cscore[:, 1].min())
    
    var1=round(explained_variance_ratio_[0]*100, 2)
    var2=round(explained_variance_ratio_[1]*100, 2)
    axis_labels("PC1 ({}%)".format(var1), "PC2 ({}%)".format(var2), axlabelfontsize, axlabelfontname)

    

    if displaydata:
        discrete_scatter(cscore[:,0]*xscale, cscore[:,1]*yscale, y=y, labels = ylabels, ax=ax, c=c)
    
    for i in range(len(loadings[0])):
        ax.arrow(0, 0, loadings[0][i], loadings[1][i], color=arrowcolor, alpha=alphaarrow, ls=ls, lw=lw)
        ax.text(loadings[0][i], loadings[1][i], loadlabels[i])
        xlim = ax.get_xlim()
        ylim = ax.get_ylim()
        ax.set_xlim(min(loadings[0][i], xlim[0]), max(loadings[0][i], xlim[1]))
        ax.set_ylim(min(loadings[1][i], ylim[0]), max(loadings[1][i], ylim[1]))

    plt.show()

 
