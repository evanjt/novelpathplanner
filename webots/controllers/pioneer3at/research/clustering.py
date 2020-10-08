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
import pandas as pd
import scipy
from sklearn.cluster import DBSCAN
from sklearn.metrics.pairwise import euclidean_distances
import open3d as o3d
from sklearn.neighbors import NearestNeighbors

# Project specific functions
import research.constants as const
import research.navigation as nav

def get_targets(robot, timestep, lidar, focalLength, location, point_array):

    bboxList = []
    mappingDists = []
    bearingList = []
    featureList = []
    targets = []
    clusters = []

    logging.debug("Clustering LiDAR scan ...")

    # Detect features/clusters within lidar scene
    features = cluster_points(point_array)

    # Test if the scan threshold is small enough to allow image overlap
    cameraMappingWidth = ((const.SCAN_THRESHOLD)/ \
            math.tan(math.radians(const.CAMERA_HORIZONTAL_FOV/2)))*2
    if cameraMappingWidth < const.SCAN_THRESHOLD and const.DEVICE == 'camera':
        logging.info("Warning: The scanning distance is too large to allow image overlap map")
        robot.warning = "scandist"
    # Identify the mapping distance limit for quality data capture
    verticalDensityMappingDist = (1/math.sqrt(const.POINT_DENSITY))/ \
        math.tan(math.radians(const.LIDAR_VERTICAL_VOF/const.LIDAR_VERTICAL_RESOLUTION))
    horizontalDensityMappingDist = (1/math.sqrt(const.POINT_DENSITY))/ \
        math.tan(math.radians(const.LIDAR_HORIZONTAL_FOV/const.LIDAR_HORIZONTAL_RESOLUTION))
    # pixelResolutionMappingDist = const.PIXEL_RESOLUTION * (focalLength * const.PIXEL_SIZE)

    dataQualityLimit = min([verticalDensityMappingDist, horizontalDensityMappingDist]) # pixelResolutionMappingDist])

    # Obtain the bbox of each cluster
    # Calculate the optimal mapping distance based on the cluster size and device parameters
    for feature in features:
        xmax = max(feature, key=lambda x: x[0])[0]
        xmin = min(feature, key=lambda x: x[0])[0]
        ymax = max(feature, key=lambda x: x[1])[1]
        zmax = max(feature, key=lambda x: x[2])[2]
        zmin = min(feature, key=lambda x: x[2])[2]

        bboxList.append([xmin, xmax, zmin, zmax])

        bottomLidarMappingDist = (const.SCANNER_HEIGHT)/ \
            math.tan(math.radians(const.LIDAR_VERTICAL_VOF/2))
        topLidarMappingDist = (ymax)/ \
            math.tan(math.radians(const.LIDAR_VERTICAL_VOF/2))
        bottomCameraMappingDist = (const.CAMERA_HEIGHT)/ \
            math.tan(math.radians(const.CAMERA_VERTICAL_VOF/2)) + const.CAMERA_OFFSET
        topCameraMappingDist = (ymax + (const.SCANNER_HEIGHT - const.CAMERA_HEIGHT))/ \
            math.tan(math.radians(const.CAMERA_VERTICAL_VOF/2)) + const.CAMERA_OFFSET

        lidarMappingDist = max([bottomLidarMappingDist, topLidarMappingDist])
        cameraMappingDist = max([bottomCameraMappingDist, topCameraMappingDist])
        mappingDist = max([lidarMappingDist, cameraMappingDist])

        if mappingDist > dataQualityLimit:
            mappingDist = const.MAPPINGDISTANCE_WINDOW
            logging.info("Warning: A feature(s) is too tall to map completely at the specified quality")
            robot.warning = "tallfeature"
        elif mappingDist < 3 and dataQualityLimit > 3:
            mappingDist = const.MAPPINGDISTANCE_WINDOW

        if const.DEVICE == "lidar":
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
        if const.DEVICE == "camera":
            if (xmax - xmin) > (zmax - zmin) and zmax > const.HOME_LOCATION[2]:
                bearingList.append(0)
                featureList.append([xmax, zmin-mappingDist])
                mappingDists.append(mappingDist)
            elif (xmax - xmin) > (zmax - zmin) and zmax < const.HOME_LOCATION[2]:
                bearingList.append(180)
                featureList.append([xmin, zmax+mappingDist])
                mappingDists.append(mappingDist)
            elif (zmax - zmin) > (xmax - xmin) and xmax > const.HOME_LOCATION[0]:
                bearingList.append(270)
                featureList.append([xmin-mappingDist, zmin])
                mappingDists.append(mappingDist)
            else:
                bearingList.append(90)
                featureList.append([xmax+mappingDist, zmax])
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

def capture_lidar_scene(robot, method='w', scan='full', threshold=20, seeing_buffer = 4):

    point_list = []

    # Capture lidar data
    robot.robot.step(robot.timestep)
    cloud = robot.lidar.getPointCloud()

    # Filter the lidar point cloud
    for row in cloud:
        distance = math.sqrt(row.x**2 + row.y**2 + row.z**2)

        if scan == 'feature' and const.DEVICE == 'lidar':
            if row.y > -0.4 and row.y < 4 \
                and row.x < 0 and row.z < 4 and row.z > -4 \
                and distance < threshold + seeing_buffer:
                point_list.append((row.x, row.y, row.z))
        elif scan == 'feature' and const.DEVICE == 'camera':
            if row.y > -0.4 and row.y < 4 \
                and row.z < 0 and row.x < 4 and row.x > -4 \
                and distance < threshold + seeing_buffer:
                point_list.append((row.x, row.y, row.z))
        else:
            if row.y > -0.4 and row.y < 4 and distance < threshold + seeing_buffer:
                point_list.append((row.x, row.y, row.z))

    logging.info("Captured {} points in LiDAR scene".format(len(point_list)))

    # Filter outlier points
    xyz = filter_points(np.array(point_list))

    o3dpoints = convert_to_o3d(xyz)

    ''' Gets the quality of the scanned lidar, sends it to a variable in the robot's
        class, and if this variable changes, the logger detects it and writes to the
        coordinates file then resets it - How kind of it :)
    '''
    if scan == 'feature':
        try:
            get_lidar_quality(o3dpoints, robot)
        except:
            logging.info("skipped a problematic scan")
            pass

    return o3dpoints

def convert_to_o3d(point_list):

    # Convert points to open 3D point cloud

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(point_list)

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

def get_lidar_quality(pcd, robot):
    ''' Uses the input scan (which happens on a subset of lidar when next to a feature)
        to determine some quality measurements to then append to the output coordinates
        logger file.

        In: Point cloud in o3d format
            Robot's class

        Returns:
            Nothing -- Writes the attributes to the class for the logger to pick up
    '''
    plane_model, inliers = pcd.segment_plane(distance_threshold=0.01,
                                             ransac_n=3,
                                             num_iterations=1000)
    inlier_cloud = pcd.select_by_index(inliers)
    np_inlier = np.array(inlier_cloud.points)
    clustering = DBSCAN(eps=2, min_samples=50).fit(np_inlier)
    total_area = 0
    for cluster in np.unique(clustering.labels_):
        subX = np_inlier[clustering.labels_ == cluster]
        convex_hull = scipy.spatial.ConvexHull(subX)
        total_area += convex_hull.area
        logging.info("[Cluster {:2d}] Area: {:.2f}m^2 | Points: {:5d} | Point density: {:.2f} | Meets threshold: {}".format(
            cluster, convex_hull.area, subX.shape[0],
            subX.shape[0]/convex_hull.area, subX.shape[0]/convex_hull.area > const.POINT_DENSITY))
    average_density = np_inlier.shape[0]/total_area
    logging.info("             Avg density: {:.2f}m^2".format(average_density))
    robot.average_density = average_density
    robot.lidar_num_clusters = len(np.unique(clustering.labels_))
