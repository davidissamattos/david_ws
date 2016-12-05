#!/usr/bin/env python

"""
Machine Learning class for the HRI problem
"""
from sklearn.cluster import KMeans
import numpy as np
import matplotlib.pyplot as plt
import datetime
import os


class learn_hri:
    def __init__(self, X):
        self.X = X
	interm = self.X[self.X[:,0].argsort()]
	self.X = interm
        self.__findClusters(self.X)
	self.filepath = os.path.abspath(__file__)

    def __findClusters(self, X):
        self.km = KMeans(n_clusters=3, init='k-means++', n_init=10, max_iter=300, tol=1e-04, random_state=0)
        self.y_km = self.km.fit_predict(X)
        #order array by the first collumn (0)
	self.cluster_ordered = self.km.cluster_centers_[self.km.cluster_centers_[:,0].argsort()]
	#self.y_km_ordered = self.y_km[self.y_km[:,0].argsort()]

    def lowerDistance(self):
        # mean between centroids lower and best
        mean = (self.cluster_ordered[1, 0] + self.cluster_ordered[0, 0]) / 2
        return mean

    def bestDistance(self):
        # best centroid
        return self.cluster_ordered[1, 0]

    def higherDistance(self):
        # mean between centroids lower and best
        mean = (self.cluster_ordered[1, 0] + self.cluster_ordered[2, 0]) / 2
        return mean

    def saveGraphic(self, path):
        # Plotting
	print "Saving graphic"
        fig, ax = plt.subplots()
        ax.scatter(self.X[(self.y_km == 0), 0], self.X[(self.y_km == 0), 1],
                    s=50,
                    c='lightgreen',
                    marker='o',
                    #label='Close distance'
		   )
        ax.scatter(self.X[(self.y_km == 1), 0], self.X[(self.y_km == 1), 1],
                    s=50,
                    c='orange',
                    marker='o',
                    #label='Good distance'
 		   )
        ax.scatter(self.X[(self.y_km == 2), 0], self.X[(self.y_km == 2), 1],
                    s=50,
                    c='red',
                    marker='o',
                    #label='Far distance',
		   )
        # centroids
        ax.scatter(self.cluster_ordered[:, 0], self.cluster_ordered[:, 1],
                    s=50,
                    c='black',
                    marker='*',
                    label='Centroids')
        ax.axvline(self.lowerDistance())
        ax.text(self.lowerDistance(), 0, 'Lower', rotation=0)
        ax.axvline(self.higherDistance())
        ax.text(self.higherDistance(), 0, 'Higher', rotation=0)
        #plt.axvline(self.bestDistance())
        #plt.text(self.bestDistance(), 0, 'Best', rotation=0)
        ax.legend()
        ax.grid()
        fig.savefig(path+'Clusters' + str(datetime.datetime.now()) + '.png')
        
	#plt.show()	
	print "Graphic saved"
	


