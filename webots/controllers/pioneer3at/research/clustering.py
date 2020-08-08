import os
import csv
import numpy as np
import math
from sklearn.cluster import DBSCAN
from sklearn import linear_model
from research.constants import *

def cluster_points(array, ransac_threshold=0.5):

    clustering = DBSCAN(eps=0.5, min_samples=10).fit(array)
    count_high_ransac = 0
    xy_of_inliers = []

    for cluster in np.unique(clustering.labels_):

        if cluster >= 0:
            # Split array into two tables x and y
            subX = array[clustering.labels_ == cluster]
            x, y = subX.reshape(-1,2).T
            x = x.reshape(-1,1)
            y = y.reshape(-1,1)

            # Fit a line quickly with RANSAC over the 2D data
            # Setup RANSAC with min_samples on total number of points
            ransac = linear_model.RANSACRegressor()
            ransac.fit(x, y)
            inlier_mask = ransac.inlier_mask_
            outlier_mask = np.logical_not(inlier_mask)

            # Determine fit of line with the same points (not ideal, would be
            # better to split 80/20% dataset and test data with independent data
            ransac_score = ransac.score(x,y)

            if ransac_score > ransac_threshold:
                count_high_ransac += 1

                # Combine x and y together of the inliers of the cluster, form a list, append
                xy_of_inliers.append(np.hstack((x[inlier_mask], y[inlier_mask])).tolist())

    print("Suitable RANSAC clusters: {}".format(count_high_ransac))

    return xy_of_inliers

def capture_lidar_scene(robot, lidar_device, timestep, path=os.path.join(OUTPUT_PATH, 'points.csv')):

    point_list = []

    with open(path, 'w', newline='') as outfile:
        csvwriter = csv.writer(outfile)

        for i in range(5):

            robot.step(timestep)
            cloud = lidar_device.getPointCloud()

        for row in cloud:

            dist = math.sqrt(row.x**2 + row.y**2 + row.z**2)

            if row.y > -0.2 and row.y < 0.8 \
            and row.x != 0 and row.y != 0 \
            and dist < 7:
                csvwriter.writerow([row.x, row.y, row.z])
                point_list.append((row.x, row.z))

    return np.array(point_list)
