#!/usr/bin/python3

'''
    Presents a series of summary information for a robot run
    based on the coordinates.json file recorded during a session

    Authors:    Josh Clough
                Evan Thomas
'''

import glob
import pandas as pd
import json
import os
from shapely.geometry import Point, LineString

B_PER_SECOND = 409600 # bytes

def execute_summary(folder):
    filename = os.path.join(folder, "coordinates.json")

    summary_filename = os.path.join(folder, "summarystats.txt")
    outstring = ""

    with open(filename, 'r') as f:
        coords = list(f.readlines())
    trajectory_list = []

    # Create a list of coordinates and the dict of their properties
    coordproperties = []
    for coord in coords:
        # Convert line to json
        try:  # Ignore newlines passing
            coord_json = json.loads(coord)
            coordproperties.append(coord_json['properties'])
            trajectory_list.append(Point(
                coord_json['geometry']['coordinates'][0],
                coord_json['geometry']['coordinates'][1]))
        except:
            pass
    trajectory = LineString(trajectory_list)

    outstring += "Total time: {} seconds\n".format(coordproperties[-1]['time'])
    outstring += "Total length: {:.2f} m\n".format(trajectory.length)
    outstring += "Average speed: {:.2f} km/h\n".format(
        (trajectory.length / coordproperties[-1]['time']) * 3.6)


    features = []
    mode = None
    at_feature = None
    feature_runtime = {}
    for coord in coordproperties:
        if coord['type'] == "scan_camera" or coord['type'] == "scan_lidar":
            mode = coord['type']
            features.append([coord['currentTarget'], coord['currentScan'],
                             coord['currentEdge'], coord['MeetsPointThshld'],
                             coord['NumClusters']])
            if at_feature is None:
                at_feature = coord['currentTarget']
                feature_start_time = coord['time']
            elif at_feature is not None and at_feature != coord['currentTarget']:
                feature_runtime[at_feature] = last_time - feature_start_time
                at_feature = coord['currentTarget']
                feature_start_time = coord['time']

            last_time = coord['time']

    # Summary of features
    try:
        total_bytes_feature_always_on = 0
        total_bytes_optimal = 0
        df = pd.DataFrame(features)
        feature_count = len(df[0].unique())
        outstring += "Mode: {}\n".format(mode)
        outstring += "Features: {}\n".format(feature_count-1) # Last feature is start point
        outstring += "------------\n"
        outstring += "Feature|Scans| Below Threshold |"
        outstring += "Occlus./| Total |  Data captured (MB)\n"
        outstring += "   #   |  #  |    #   |    %   |"
        outstring += "Concav. |Time(s)|Feature|Scan|always-on\n"

        outstring += "-------|-----|--------|--------|"
        outstring += "--------|-------|-------|----|---------\n"

        scan_size_bytes = {}

        # Get total file sizes of run
        for i in range(1, feature_count):
            for filename in glob.iglob(os.path.join(folder,'lidar_feature{}scan*.xyz'.format(i))):
                if scan_size_bytes.get(i) is None:
                    scan_size_bytes[i] = os.path.getsize(filename)
                else:
                    scan_size_bytes[i] += os.path.getsize(filename)

        for i in range(feature_count-1):
            total_bytes_optimal += scan_size_bytes[i+1]
            total_bytes_feature_always_on += feature_runtime[i] * B_PER_SECOND
            scan_count = 0
            threshold_failure = 0
            occlusions = 0
            for feature in features:
                if feature[0] == i:
                    scan_count += 1
                    if feature[3] == 0:
                        threshold_failure += 1
                    if feature[4] > 1:
                        occlusions += 1
            outstring += "{:7}|{:5}" \
                "|{:8}|{:8.2f}" \
                "|{:8}" \
                "|{:7.2f}" \
                "|{:7.2f}|{:.2f}|{:9.2f}\n".format(
                    i, scan_count, threshold_failure,
                    threshold_failure/scan_count*100, occlusions,
                    feature_runtime[i], total_bytes_optimal/1048576, scan_size_bytes[i+1]/1048576/scan_count,
                    total_bytes_feature_always_on/1024/1024)
    except:
         outstring += "No features scanned\n"
    outstring += "------------\n"
    outstring += "Total data (optimal method): {:.2f} MB\n".format(total_bytes_optimal/1024/1024)
    outstring += "Theoretical feature-on LiDAR capture size @ {} MB/s: {:.2f} MB\n".format(MB_PER_SECOND, total_bytes_feature_always_on/1024/1024)
    outstring += "Theoretical always-on LiDAR capture size @ {} MB/s: {:.2f} MB\n\n".format(MB_PER_SECOND,
                                                                                            float(coordproperties[-1]['time'])*B_PER_SECOND/1024/1024)
    outstring += "Percentage of data (optimal method) compared to LiDAR feature-on/always-on: {:.2f}%/{:.2f}%".format(total_bytes_optimal/total_bytes_feature_always_on*100,
                                                                                       total_bytes_optimal/(float(coordproperties[-1]['time'])*B_PER_SECOND)*100)

    print(outstring)  # Print to screen
    with open(summary_filename, 'w') as f:  # Log to file
        f.write(outstring)
    return (feature_count-1,
            float(coordproperties[-1]['time']),
            total_bytes_optimal/1024/1024,
            total_bytes_feature_always_on/1024/1024,
            float(coordproperties[-1]['time'])*B_PER_SECOND/1024/1024)

def main():
    # Run for all folders in output directory
    # execute_summary('/home/evan/research/simulation-git/output/2020-10-09T00.51.17.203900/')
    summaries = []
    for folder in glob.iglob("/home/evan/research/simulation-git/output/**"):
        try:
            summaries.append([folder.split("/")[-1], execute_summary(folder)])
        except:
            pass
