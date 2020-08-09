#!/usr/bin/python3
'''
    Code related to clustering methods for the use of the pioneer3at
    controller in webots.

    Authors:    Josh Clough
                Evan Thomas
'''

import os
import csv
import numpy as np
import math
from sklearn.cluster import DBSCAN
import research.constants as const
from sklearn.metrics.pairwise import euclidean_distances


def cluster_points(array):
    clustering = DBSCAN(eps=0.5, min_samples=10).fit(array)
    xy_of_inliers = []
    features = []

    for cluster in np.unique(clustering.labels_):

        maxval = 0

        if cluster != -1:

            # Split array into two tables x and y
            subX = array[clustering.labels_ == cluster]
            x, y = subX.reshape(-1, 2).T
            x = x.reshape(-1, 1)
            y = y.reshape(-1, 1)

            xy_of_inliers.append(np.hstack((x, y)).tolist())

            maxval = max(map(max, euclidean_distances(xy_of_inliers[cluster],
                                                      xy_of_inliers[cluster])))

            if maxval > 1:
                features.append(xy_of_inliers[cluster])

    return features


def capture_lidar_scene(robot, lidar_device, timestep,
                        path=os.path.join(const.OUTPUT_PATH, 'points.csv')):

    point_list = []

    with open(path, 'w', newline='') as outfile:
        csvwriter = csv.writer(outfile)

        for i in range(5):

            robot.step(timestep)
            cloud = lidar_device.getPointCloud()

        for row in cloud:

            dist = math.sqrt(row.x**2 + row.y**2 + row.z**2)

            if row.y > -0.2 and row.y < 0.8 \
                and row.x != 0 \
                and row.y != 0 \
                    and dist < 7:
                csvwriter.writerow([row.x, row.y, row.z])
                point_list.append((row.x, row.z))

    return np.array(point_list)
