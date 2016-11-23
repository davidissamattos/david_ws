#!/usr/bin/env python

"""
Machine Learning class for the HRI problem
"""
from sklearn.cluster import KMeans
import numpy as np
import matplotlib.pyplot as plt
import datetime


class learn_hri:
    def __init__(self, X):
        self.X = X
        self.__findClusters(self.X)

    def __findClusters(self, X):
        self.km = KMeans(n_clusters=3, init='k-means++', n_init=10, max_iter=300, tol=1e-04, random_state=0)
        self.y_km = self.km.fit_predict(X)
	self.saveGraphic()

    def lowerDistance(self):
        # mean between centroids lower and best
        mean = (self.km.cluster_centers_[1, 0] + self.km.cluster_centers_[0, 0]) / 2
        return mean

    def bestDistance(self):
        # best centroid
        return self.km.cluster_centers_[1, 0]

    def higherDistance(self):
        # mean between centroids lower and best
        mean = (self.km.cluster_centers_[1, 0] + self.km.cluster_centers_[2, 0]) / 2
        return mean

    def saveGraphic(self):
        # Plotting
        plt.scatter(self.X[(self.y_km == 0), 0], self.X[(self.y_km == 0), 1],
                    s=50,
                    c='lightgreen',
                    marker='o',
                    label='Close distance')
        plt.scatter(self.X[(self.y_km == 1), 0], self.X[(self.y_km == 1), 1],
                    s=50,
                    c='orange',
                    marker='o',
                    label='Good distance')
        plt.scatter(self.X[(self.y_km == 2), 0], self.X[(self.y_km == 2), 1],
                    s=50,
                    c='red',
                    marker='o',
                    label='Far distance')
        # centroids
        plt.scatter(self.km.cluster_centers_[:, 0], self.km.cluster_centers_[:, 1],
                    s=50,
                    c='black',
                    marker='*',
                    label='Centroids')
        plt.axvline(self.lowerDistance())
        plt.text(self.lowerDistance()-0.25, 0, 'Lower', rotation=0)
        plt.axvline(self.higherDistance())
        plt.text(self.higherDistance(), 0, 'Higher', rotation=0)
        plt.axvline(self.bestDistance())
        plt.text(self.bestDistance(), 0, 'Best', rotation=0)
        plt.legend()
        plt.grid()
        plt.savefig('Clusters' + str(datetime.datetime.now()) + '.png')


