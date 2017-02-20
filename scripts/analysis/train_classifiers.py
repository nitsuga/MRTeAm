#!/usr/bin/env python


import argparse
import numpy as np
import pandas as pd
import sys
import traceback

# Preprocessing and metrics
from sklearn import feature_selection
from sklearn.feature_selection import chi2
from sklearn import pipeline
from sklearn.model_selection import KFold, StratifiedKFold, GridSearchCV
from sklearn import preprocessing
import sklearn.metrics.classification
from sklearn.preprocessing import StandardScaler

# Classifiers
from sklearn.neighbors import KNeighborsClassifier
from sklearn.svm import SVC
from sklearn.tree import DecisionTreeClassifier
from sklearn.ensemble import RandomForestClassifier, AdaBoostClassifier
from sklearn.naive_bayes import GaussianNB
from sklearn.discriminant_analysis import LinearDiscriminantAnalysis, QuadraticDiscriminantAnalysis

import pickle

robot_names = ['ROBOT1', 'ROBOT2', 'ROBOT3']

names = ["Nearest Neighbors", "Linear SVM", "RBF SVM", "Decision Tree",
         "Random Forest", "AdaBoost", "Naive Bayes"]
         # , "Linear Discriminant Analysis","Quadratic Discriminant Analysis"]

drop_columns = ['WINNER_DIFFERENCE',
                'MAXIMUM_ROBOT_DISTANCE',
                'EXECUTION_PHASE_TIME',
                'TOTAL_DISTANCE',
                'TOTAL_RUN_TIME']

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

# For K-fold stratification
FOLDS = 10


def train_classifiers(in_csv, output):

    try:
        input_frame = pd.read_csv(in_csv)

        # Remove rows where WINNER_DIFFERENCE is less than one standard deviation above zero
        # input_frame = input_frame[input_frame.WINNER_DIFFERENCE > input_frame.WINNER_DIFFERENCE.std()]

        # Drop the 'WINNER_DIFFERENCE' column
        for column in drop_columns:
            try:
                input_frame = input_frame.drop(column, 1)
            except ValueError:
                # print("Column {0} not found".format(column))
                continue

        for robot_name in robot_names:
            input_frame = input_frame.drop('{0}_DISTANCE_TO_ASSIGNED_MEDIAN'.format(robot_name), 1)
            input_frame = input_frame.drop('{0}_DISTANCE_TO_ALL_MEDIANS'.format(robot_name), 1)
            input_frame = input_frame.drop('{0}_STARTX'.format(robot_name), 1)
            input_frame = input_frame.drop('{0}_STARTY'.format(robot_name), 1)

        # print 'Columns: {0}'.format(input_frame.columns)

        # Assume the label is the last column of the data frame
        # X = input_frame.ix[:, :-1].values
        # y = input_frame['MECHANISM'].values

        input_frame = input_frame.dropna().reset_index(drop=True)

        X = input_frame.ix[:, :-1]
        y = input_frame['MECHANISM']

        # Scale (to normal) X
        # X = StandardScaler().fit_transform(X)

        # K-fold splits
        k_folds = KFold(n_splits=FOLDS)
        sk_folds = StratifiedKFold(n_splits=FOLDS)

        # Split training and test sets
        # X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=.25)

        # Select features

        # # iterate over classifiers
        # for name, clf in zip(names, classifiers):
        #     # clf.fit(X_train, y_train)
        #     # score = clf.score(X_test, y_test)
        #
        #     scores = cross_val_score(clf, X, y, cv=sk_folds, n_jobs=-1)
        #     print("{0: <17}: {1}".format(name, scores.mean()))

        scores = []
        max_score = None
        best_features = None
        best_params = None
        for k, (train, test) in enumerate(sk_folds.split(X, y)):
            X_train, X_test, y_train, y_test = X.ix[train], X.ix[test], y.ix[train], y.ix[test]

            # idx = X_test.apply(lambda ts: any(ts < 0), axis=1)
            # print X_test[idx]

            top_feat = feature_selection.SelectKBest(k=4)
            # top_feat = feature_selection.SelectPercentile(percentile=25)

            pipe = pipeline.Pipeline([
                                      ('scaler', preprocessing.StandardScaler()),
                                      ('feat', top_feat),
                                      # ('clf', SVC())])
                                      ('clf', RandomForestClassifier())])
                                      # ('clf', KNeighborsClassifier())])
                                      # ('clf', DecisionTreeClassifier())])

            # SVC parameters
            # param_grid = {
            #     'clf__C': [1, 10, 100, 1000],
            #     'clf__gamma': [0.001, 0.0001],
            #     'clf__kernel': ['rbf', 'linear'],
            # }

            # RandomForestClassifier parameters
            param_grid = {
                'clf__n_estimators': [10, 20, 50],
                # 'clf__min_samples_split': [2, 3, 4, 5, 10],
                'clf__max_depth': [3, None],
                # 'clf__max_features': [1, 3, 10],
                # 'clf__min_samples_split': [1, 3, 10],
                'clf__min_samples_split': [2, 3, 10],
                'clf__min_samples_leaf': [1, 3, 10],
                'clf__bootstrap': [True, False],
                'clf__criterion': ['gini', 'entropy']
            }

            # KNeighborsClassifier parameters
            # param_grid = {
            #     'clf__n_neighbors': [1, 2, 3, 4, 5],
            #     'clf__weights': ['uniform', 'distance']
            # }

            # DecisionTreeClassifier parameters
            # param_grid = {
            #     'clf__criterion': ['gini', 'entropy'],
            #     'clf__splitter': ['best', 'random'],
            #     'clf__max_features': [1, 2, 'auto']
            # }

            gs = GridSearchCV(estimator=pipe, param_grid=param_grid, n_jobs=-1)
            gs.fit(X_train, y_train)

            best_score = gs.best_score_
            scores.append(best_score)

            y_predict = gs.predict(X_test)

            report = sklearn.metrics.classification_report(y_test, y_predict)

            print "Fold: {} {:.4f}".format(k + 1, best_score)
            print report
            print "Best params: {0}".format(gs.best_params_)
            # features = pipe.named_steps['feat']
            # print X.columns[features.transform(np.arange(len(X.columns)))]
            final_feature_indices = gs.best_estimator_.named_steps["feat"].get_support(indices=True)
            final_feature_list = [X.columns[i] for i in final_feature_indices]
            print "Best features: {0}".format(final_feature_list)

            if not max_score or best_score > max_score:
                max_score = best_score
                best_features = final_feature_list
                best_params = gs.best_params_

        print "--------------------------------"
        print "Mean score: {0}, std: {1}".format(np.mean(scores), np.std(scores))
        print "Best features: {0}".format(best_features)
        print "Best params: {0}".format(best_params)

        # Remove the leading 'clf__' from each param key
        for key in best_params:
            new_key = key.replace('clf__', '')
            best_params[new_key] = best_params.pop(key)

        # clf = SVC(**best_params)
        clf = RandomForestClassifier(**best_params)
        # clf = KNeighborsClassifier(**best_params)

        clf = clf.fit(X.ix[:, best_features], y)

        with open('{0}.dump'.format(output), 'wb') as clf_out:
            pickle.dump(clf, clf_out)

        with open('{0}.features'.format(output), 'wb') as feat_out:
            for feat in best_features:
                print >> feat_out, feat


    except:
        traceback.print_exc()
        sys.exit(1)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Train classifiers for MRTA mechanism selection.')

    parser.add_argument('in_csv',
                        help='A .csv training file.')

    parser.add_argument('output',
                        help='Base filename to use when saving (pickling) the classifier and features.')

    args = parser.parse_args()
    in_csv = args.in_csv
    output = args.output

    train_classifiers(in_csv, output)
