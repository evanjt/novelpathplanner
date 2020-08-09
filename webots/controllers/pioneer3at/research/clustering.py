import os
import sys
import csv
import numpy as np
import math
import tsp
from sklearn.cluster import DBSCAN
from research.constants import *
from sklearn.metrics.pairwise import euclidean_distances

def get_targets(robot, timestep, lidar):

    # Before beginning survey scan surrounds, cluster the scene, return the xy of clusters for feature mapping
    featureList = []
    targets = []

    # Capture lidar scene and identify features
    point_array = capture_lidar_scene(robot, lidar, timestep)
    features = cluster_points(point_array)
    
    # Obtain the centroid of each cluster
    for feature in features:
        featureList.append(np.ndarray.tolist(np.average(feature, axis=0)))

    # For each cluster check if two clusters are close by and on the same bearing (i.e. the same feature)

    # Add 2D home coordinates as a feature and calculate tsp route                    
    featureList.insert(0, [HOME_LOCATION[0], HOME_LOCATION[2]])
    devnull = open(os.devnull, 'w')
    oldstdout_fno = os.dup(sys.stdout.fileno())
    os.dup2(devnull.fileno(), 1)
    t = tsp.tsp(featureList)
    os.dup2(oldstdout_fno, 1)
    
    # Re-order features based on tsp, add Y coordinate, and add home as last feature  
    targets.append(HOME_LOCATION)
    for ind, val in enumerate(t[1]):
        featureList[val].insert(1,0)
        targets.insert(ind, featureList[val])

    # Save feature locations to file
    with open(os.path.join(OUTPUT_PATH,'features.csv'), 'w', newline='') as outfile:
        csvwriter = csv.writer(outfile)        
        for i in targets[1:]:
            csvwriter.writerow(i)

    return targets[1:]

def capture_lidar_scene(robot, lidar_device, timestep, path=os.path.join(OUTPUT_PATH, 'points.csv')):

    point_list = []

    # Capture lidar points in a csv
    with open(path, 'w', newline='') as outfile:
        csvwriter = csv.writer(outfile)

        # Due to Velodyne intricacies the every fifth step yields a full scan
        for i in range(5):
            robot.step(timestep)
            cloud = lidar_device.getPointCloud()
        
        #  Filter the lidar point cloud
        for row in cloud:
            distance = math.sqrt(row.x**2 + row.y**2 + row.z**2)
            if row.y > -0.2 and row.y < 0.8 \
            and row.x != 0 and row.y != 0 \
            and distance < 7:
                csvwriter.writerow([row.x, row.y, row.z])
                point_list.append((row.x, row.z))
    
    return np.array(point_list)

def cluster_points(array):

    clusters = []
    features = []

    # Use to DBSCAN to cluster points    
    clustering = DBSCAN(eps=0.5, min_samples=10).fit(array) # need to refine parameters, including too many outliers

    # For each detected cluster calculate the max dimension of the feature
    # if greater than 1 meter mark the cluster as a feature 
    for cluster in np.unique(clustering.labels_):        
    
        if cluster != -1:
            subX = array[clustering.labels_ == cluster]
            x, y = subX.reshape(-1,2).T
            x = x.reshape(-1,1)
            y = y.reshape(-1,1)
            clusters.append(np.hstack((x, y)).tolist())
        
            maxval = max(map(max, euclidean_distances(clusters[cluster], clusters[cluster]))) # need a better hueristic for determining what clusters are features

            if maxval > 1:
                features.append(clusters[cluster])
                
    return features