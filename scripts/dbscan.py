#! /usr/bin/env python3

import numpy as np
from sklearn.cluster import DBSCAN


def get_clusters(points):
    points = np.asarray(points)
    # model = DBSCAN(eps=0.06, min_samples=6).fit(points)
    model = DBSCAN(eps=5, min_samples=2).fit(points) # for FSDS 

    labels = model.labels_
    # print(labels, "Lables")
    clusters = {}
    for i in range(len(points)):
        point = points[i]
        group = labels[i]

        if group != -1:
            # print(clusters.keys())
            if group not in list(clusters.keys()):
                clusters[group] = []
                clusters[group].append(point)
            else:
                clusters[group].append(point)

    centroids = {}
    variances = {}
    for cluster_id in list(clusters.keys()):
        cluster = np.asarray(clusters[cluster_id])
        summation = np.sum(cluster, axis=0)
        number_of_points = len(cluster)
        centroid = summation/number_of_points
        centroids[cluster_id] = centroid
        centroid = np.repeat([centroids[cluster_id]], len(cluster), axis=0)
        deviation = np.subtract(cluster, centroid)
        variance = np.sum(np.square(deviation)) / (number_of_points - 1)
        variances[cluster_id] = variance

    VARIANCE_THRESHOLD = 10
    for cluster_id in list(clusters.keys()):
        variance = variances[cluster_id]
        if variance > VARIANCE_THRESHOLD:
            clusters.pop(cluster_id)

    E1 = 10
    E2 = 10

    clusters_of_clusters = []
    for cluster_id in list(clusters.keys()):
        N = []
        centroid = centroids[cluster_id]
        for id2 in list(clusters.keys()):
            c2 = centroids[id2]
            distance = (centroid[0] - c2[0])**2 + (centroid[1] - c2[1])**2
            if distance < E1:
                N.append(id2)
        clusters_of_clusters.append(N)
    clusters_of_clusters = set(tuple(x) for x in clusters_of_clusters)

    for coc in clusters_of_clusters:
        new_cluster = []
        for cluster_id in coc:
            new_cluster += clusters[cluster_id]
            clusters.pop(cluster_id)
        clusters[coc[0]] = new_cluster

    clusters_of_clusters = []
    for cluster_id in list(clusters.keys()):
        N = []
        centroid = centroids[cluster_id]
        for id2 in list(clusters.keys()):
            c2 = centroids[id2]
            distance = (centroid[0] - c2[0])**2 + (centroid[1] - c2[1])**2
            if distance < E2:
                N.append(id2)

    clusters_of_clusters = set(tuple(x) for x in clusters_of_clusters)

    for coc in clusters_of_clusters:
        new_cluster = []
        for cluster_id in coc:
            clusters.pop(coc[0])
        new_cluster = clusters[cluster_id]
        clusters[coc[0]] = new_cluster

    return clusters