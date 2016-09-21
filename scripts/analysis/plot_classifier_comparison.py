#!/usr/bin/python
# -*- coding: utf-8 -*-

"""
=====================
Classifier comparison
=====================

A comparison of a several classifiers in scikit-learn on synthetic datasets.
The point of this example is to illustrate the nature of decision boundaries
of different classifiers.
This should be taken with a grain of salt, as the intuition conveyed by
these examples does not necessarily carry over to real datasets.

Particularly in high-dimensional spaces, data can more easily be separated
linearly and the simplicity of classifiers such as naive Bayes and linear SVMs
might lead to better generalization than is achieved by other classifiers.

The plots show training points in solid colors and testing points
semi-transparent. The lower right shows the classification accuracy on the test
set.
"""
print(__doc__)


# Code source: Gaël Varoquaux
#              Andreas Müller
# Modified for documentation by Jaques Grobler
# License: BSD 3 clause

import numpy as np
import matplotlib.gridspec as gridspec
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
import pandas as pd
from sklearn.cross_validation import train_test_split
from sklearn.preprocessing import StandardScaler
from sklearn.datasets import make_moons, make_circles, make_classification
from sklearn.neighbors import KNeighborsClassifier
from sklearn.svm import SVC
from sklearn.tree import DecisionTreeClassifier
from sklearn.ensemble import RandomForestClassifier, AdaBoostClassifier
from sklearn.naive_bayes import GaussianNB
from sklearn.discriminant_analysis import LinearDiscriminantAnalysis
from sklearn.discriminant_analysis import QuadraticDiscriminantAnalysis

h = .02  # step size in the mesh

title = 'Classifier Comparison: Minimax Distance'
input_file = 'minimax_distance.csv'

feature1 = 'MIN_DISTANCE_TO_MEDIAN'
feature2 = 'MAX_DISTANCE_TO_MEDIAN'

xaxis_label = 'Minimum Distance to Median'
yaxis_label = 'Maximum Distance to Median'

names = ["Nearest Neighbors", "Linear SVM", "RBF SVM", "Decision Tree",
         "Random Forest", "AdaBoost", "Naive Bayes", "Linear Discriminant Analysis",
         "Quadratic Discriminant Analysis"]
classifiers = [
    KNeighborsClassifier(3),
    SVC(kernel="linear", C=0.025),
    SVC(gamma=2, C=1),
    DecisionTreeClassifier(max_depth=5),
    RandomForestClassifier(max_depth=5, n_estimators=10, max_features=1),
    AdaBoostClassifier(),
    GaussianNB(),
    LinearDiscriminantAnalysis(),
    QuadraticDiscriminantAnalysis()]

X, y = make_classification(n_features=2, n_redundant=0, n_informative=2,
                           random_state=1, n_clusters_per_class=1)
rng = np.random.RandomState(2)
X += 2 * rng.uniform(size=X.shape)
linearly_separable = (X, y)

mm_frame = pd.read_csv(input_file)

#mm_x = mm_frame.ix[:, ['MEDIAN_SPREAD', 'TEAM_DIAMETER']].values
# mm_x = mm_frame.ix[:,['TOTAL_DISTANCE_TO_MEDIANS', 'MEDIAN_SPREAD']].values

# mm_x = mm_frame.ix[:,['MEDIAN_SPREAD', 'TOTAL_DISTANCE_TO_MEDIANS']].values

mm_x = mm_frame.ix[:, [feature1, feature2]].values

mm_y = mm_frame.ix[:, -1:].values.flatten()

for i in range(len(mm_y)):
    if mm_y[i] == 'PSI':
        mm_y[i] = 0
    else:
        mm_y[i] = 1

mm_y = mm_y.astype(int)

# datasets = [make_moons(noise=0.3, random_state=0),
#             make_circles(noise=0.2, factor=0.5, random_state=1),
#             linearly_separable
#             ]

datasets = [(mm_x, mm_y)]

#figure = plt.figure(figsize=(27, 3))
figure = plt.figure(figsize=(24, 12))

gs = gridspec.GridSpec(3, 6)

plt.title(title)

# i = 1
# iterate over datasets
for ds in datasets:
    # preprocess dataset, split into training and test part
    X, y = ds
    X = StandardScaler().fit_transform(X)
    X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=.35)

    x_min, x_max = X[:, 0].min() - .5, X[:, 0].max() + .5
    y_min, y_max = X[:, 1].min() - .5, X[:, 1].max() + .5
    xx, yy = np.meshgrid(np.arange(x_min, x_max, h),
                         np.arange(y_min, y_max, h))

    # just plot the dataset first
    cm = plt.cm.RdBu
    cm_bright = ListedColormap(['#FF0000', '#0000FF'])
#    ax = plt.subplot(len(datasets), len(classifiers) + 1, i)

    ax = plt.subplot(gs[0:3, 0:3])

    # Plot the training points
    l1 = ax.scatter(X_train[:, 0], X_train[:, 1], s=80, c=y_train, cmap=cm_bright, label='Test Set')
    # and testing points
    l2 = ax.scatter(X_test[:, 0], X_test[:, 1], s=80, c=y_test, cmap=cm_bright, alpha=0.4, label='Training Set')
    ax.set_xlim(xx.min(), xx.max())
    ax.set_ylim(yy.min(), yy.max())
    ax.set_xticks(())
    ax.set_yticks(())

    ax.set_xlabel(xaxis_label)
    ax.set_ylabel(yaxis_label)

    # ax.legend(('one', 'two'), scatterpoints=1, loc='lower right')

#    i += 1

    row = 0
    col = 3

    # iterate over classifiers
    for name, clf in zip(names, classifiers):
#        ax = plt.subplot(len(datasets), len(classifiers) + 1, i)
        ax = plt.subplot(gs[row, col])
        clf.fit(X_train, y_train)
        score = clf.score(X_test, y_test)

        # Plot the decision boundary. For that, we will assign a color to each
        # point in the mesh [x_min, m_max]x[y_min, y_max].
        if hasattr(clf, "decision_function"):
            Z = clf.decision_function(np.c_[xx.ravel(), yy.ravel()])
        else:
            Z = clf.predict_proba(np.c_[xx.ravel(), yy.ravel()])[:, 1]

        # Put the result into a color plot
        Z = Z.reshape(xx.shape)
        ax.contourf(xx, yy, Z, cmap=cm, alpha=.8)

        # Plot also the training points
        ax.scatter(X_train[:, 0], X_train[:, 1], c=y_train, cmap=cm_bright)
        # and testing points
        ax.scatter(X_test[:, 0], X_test[:, 1], c=y_test, cmap=cm_bright,
                   alpha=0.6)

        ax.set_xlim(xx.min(), xx.max())
        ax.set_ylim(yy.min(), yy.max())
        ax.set_xticks(())
        ax.set_yticks(())
        ax.set_title(name)
        ax.text(xx.max() - .3, yy.min() + .3, ('%.2f' % score).lstrip('0'),
                size=15, horizontalalignment='right')
#        i += 1

        col += 1

        if col > 5:
            row += 1
            col = 3

#figure.subplots_adjust(left=.02, right=.98)

plt.tight_layout()
plt.show()
