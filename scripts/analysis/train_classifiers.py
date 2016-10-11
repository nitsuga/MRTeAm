#!/usr/bin/env python


import argparse
import numpy as np
import pandas as pd
import sys

# Preprocessing and metrics
from sklearn import feature_selection
from sklearn.model_selection import KFold, StratifiedKFold, cross_val_score, train_test_split
from sklearn.preprocessing import StandardScaler

# Classifiers
from sklearn.neighbors import KNeighborsClassifier
from sklearn.svm import SVC
from sklearn.tree import DecisionTreeClassifier
from sklearn.ensemble import RandomForestClassifier, AdaBoostClassifier
from sklearn.naive_bayes import GaussianNB
from sklearn.discriminant_analysis import LinearDiscriminantAnalysis, QuadraticDiscriminantAnalysis

import pickle

names = ["Nearest Neighbors", "Linear SVM", "RBF SVM", "Decision Tree",
         "Random Forest", "AdaBoost", "Naive Bayes"]
         # , "Linear Discriminant Analysis","Quadratic Discriminant Analysis"]
classifiers = [
    KNeighborsClassifier(3),
    SVC(kernel="linear", C=0.025),
    SVC(gamma=2, C=1),
    DecisionTreeClassifier(max_depth=5),
    RandomForestClassifier(max_depth=5, n_estimators=10, max_features=2),
    AdaBoostClassifier(),
    GaussianNB()] #,
    # LinearDiscriminantAnalysis(),
    #QuadraticDiscriminantAnalysis()]

def train_classifiers(in_csv):

    try:
        input_frame = pd.read_csv(in_csv)

        # Assume the label is the last column of the data frame
        X = input_frame.ix[:, :-1].values
        y = input_frame.ix[:, -1:].values.flatten()

        # Scale (to normal) X
        X = StandardScaler().fit_transform(X)

        # K-fold splits
        k_fold = KFold(n_splits=10)
        sk_fold = StratifiedKFold(n_splits=10)

        # Split training and test sets
        # X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=.25)

        # Select features

        # iterate over classifiers
        for name, clf in zip(names, classifiers):
#            clf.fit(X_train, y_train)
#            score = clf.score(X_test, y_test)
            scores = cross_val_score(clf, X, y, cv=sk_fold, n_jobs=-1)

            print("{0: <17}: {1}".format(name, scores.mean()))

    except:
        print(sys.exc_info())
        sys.exit(1)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Train classifiers for MRTA mechanism selection.')

    parser.add_argument('in_csv',
                        help='A .csv training file.')

    args = parser.parse_args()
    in_csv = args.in_csv

    train_classifiers(in_csv)
