#!/usr/bin/python3
'''
    Code related to clustering methods for the use of the pioneer3at
    controller in webots.

    Authors:    Josh Clough
                Evan Thomas
'''

import os
import sys
import csv
import logging
import numpy as np
import math
import tsp
from sklearn.cluster import DBSCAN
import research.constants as const
import research.navigation as nav
from sklearn.metrics.pairwise import euclidean_distances
import open3d as o3d
from sklearn.neighbors import NearestNeighbors


def get_targets(robot, timestep, lidar, location, point_array):

    # Before beginning survey scan surrounds, cluster the scene,
    # return clusters for feature mapping

    bboxList = []
    mappingDists = []
    bearingList = []
    featureList = []
    targets = []
    clusters = []

    # Detect features/clusters within lidar scene
    logging.debug("Clustering LiDAR scan ...")
    features = cluster_points(point_array)

    # Obtain the bbox of each cluster
    # Due to differences between the world and the point cloud x=z and z=x
    for feature in features:
        xmax = max(feature, key=lambda x: x[0])[0]
        xmin = min(feature, key=lambda x: x[0])[0]
        ymax = max(feature, key=lambda x: x[1])[1]
        # ymin = min(feature, key=lambda x: x[1])[1]
        zmax = max(feature, key=lambda x: x[2])[2]
        zmin = min(feature, key=lambda x: x[2])[2]

        bboxList.append([xmin, xmax, zmin, zmax])

        mappingDist = (ymax+const.SCANNER_HEIGHT)/ \
            math.tan(math.radians(const.VERTICAL_VOF))

        if (xmax - xmin) > (zmax - zmin) and zmax > const.HOME_LOCATION[2]:
            bearingList.append(90)
            featureList.append([xmin+(xmax-xmin)/2, zmin-mappingDist])
            mappingDists.append(mappingDist)
        elif (xmax - xmin) > (zmax - zmin) and zmax < const.HOME_LOCATION[2]:
            bearingList.append(270)
            featureList.append([xmin+(xmax-xmin)/2, zmax+mappingDist])
            mappingDists.append(mappingDist)
        elif (zmax - zmin) > (xmax - xmin) and xmax > const.HOME_LOCATION[0]:
            bearingList.append(0)
            featureList.append([xmin-mappingDist, zmin+(zmax-zmin)/2])
            mappingDists.append(mappingDist)
        else:
            bearingList.append(180)
            featureList.append([xmax+mappingDist, zmin+(zmax-zmin)/2])
            mappingDists.append(mappingDist)

    # Add 2D home coordinates as a feature and calculate tsp route
    # Disable tsp stdout
    featureList.insert(0, [const.HOME_LOCATION[0],
                           const.HOME_LOCATION[2]])
    devnull = open(os.devnull, 'w')
    oldstdout_fno = os.dup(sys.stdout.fileno())
    os.dup2(devnull.fileno(), 1)
    t = tsp.tsp(featureList)
    os.dup2(oldstdout_fno, 1)

    # Re-order features based on tsp, add Y coordinate,
    # and add home as last feature
    bboxList.insert(0,0)
    bearingList.insert(0,0)
    mappingDists.insert(0,0)
    targets.append([const.HOME_LOCATION, 0])
    features.insert(0,0)

    for ind, val in enumerate(t[1]):
        featureList[val].insert(1,0)
        clusters.insert(ind,features[val])
        targets.insert(ind, [featureList[val], bearingList[val], \
            mappingDists[val], bboxList[val]])

    # Save feature locations to file
    with open(os.path.join(const.OUTPUT_PATH,'features.xyz'),
              'w', newline='') as outfile:
        csvwriter = csv.writer(outfile)
        for i in targets[1:]:
            csvwriter.writerow(i[0])

    return clusters[1:], targets[1:]

def capture_lidar_scene(robot, timestep, lidar_device, location, bearing,
                        method='w', scan='full', threshold=20):

    point_list = []

    # Capture lidar data
    robot.step(timestep)
    cloud = lidar_device.getPointCloud()

    # Filter the lidar point cloud
    for row in cloud:

        distance = math.sqrt(row.x**2 + row.y**2 + row.z**2)

        if scan == 'feature':
            if row.y > -0.5 and row.y < 5 \
                and row.x < 0 and row.z < 5 and row.z > -5 \
                and distance < threshold + 5:
                point_list.append((row.x, row.y, row.z))
        else:
            if row.y > -0.5 and row.y < 5 and distance < threshold + 5:
                point_list.append((row.x, row.y, row.z))

    logging.info("Captured {} points in LiDAR scene".format(len(point_list)))

    return convert_to_o3d(point_list)

def convert_to_o3d(point_list):

    # Convert points to open 3D point cloud
    xyz = filter_points(np.array(point_list))

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(xyz)

    return pcd

def write_lidar_scene(pcd, method='w',
                    path=os.path.join(const.OUTPUT_PATH, 'points.xyz')):

    # Write pointcloud to file
    with open(path, method, newline='') as outfile:
        np.savetxt(outfile, pcd.points, delimiter=",")

def cluster_points(array):

    clusters = []
    features = []

    # Use to DBSCAN to cluster points
    clustering = DBSCAN(eps=2, min_samples=100).fit(array)

    # For each detected cluster calculate the max dimension of the feature
    # if greater than specified distance, mark the cluster as a feature
    for cluster in np.unique(clustering.labels_):

        if cluster != -1:
            subX = array[clustering.labels_ == cluster]
            x, y, z = subX.reshape(-1,3).T
            x = x.reshape(-1,1)
            y = y.reshape(-1,1)
            z = z.reshape(-1,1)
            clusters.append(np.hstack((x, y, z)).tolist())

            xdist = max(clusters[cluster][0]) - min(clusters[cluster][0])
            zdist = max(clusters[cluster][2]) - min(clusters[cluster][2])

            if xdist > const.FEATURE_THRESHOLD \
                or zdist > const.FEATURE_THRESHOLD:
                features.append(clusters[cluster])

    return features

def filter_points(df, k=50, threshold=1):

    # Nearest neighbours outlier setup
    knn = NearestNeighbors(n_neighbors=k)
    knn.fit(df)
    neighbors_and_distances = knn.kneighbors(df)
    knn_distances = neighbors_and_distances[0]
    neighbors = neighbors_and_distances[1]
    kth_distance = [x[-1] for x in knn_distances]
    tnn_distance = np.mean(knn_distances, axis=1)


    return df[abs(tnn_distance - np.mean(tnn_distance)) < threshold * np.std(tnn_distance)]
