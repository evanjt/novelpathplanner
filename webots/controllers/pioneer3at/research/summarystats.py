#!/usr/bin/python3

'''
    Presents a series of summary information for a robot run
    based on the coordinates.json file recorded during a session

    Authors:    Josh Clough
                Evan Thomas
'''

import pandas as pd
import json
import os
import sys
import research.constants as const
from shapely.geometry import Point, LineString

def execute_summary(folder):
    filename = os.path.join(folder, "coordinates.json")
    summary_filename = os.path.join(folder, "summarystats.txt")
    outfile = open(summary_filename, 'a')
    sys.stdout = outfile # Write all print statements to file
    with open(filename, 'r') as f:
        coords = list(f.readlines())
    trajectory_list = []
    # Create a list of coordinates and the dict of their properties
    coordproperties = []
    for coord in coords:
        # Convert line to json
        try: # Ignore newlines passing
            coord_json = json.loads(coord)
            coordproperties.append(coord_json['properties'])
            trajectory_list.append(Point(coord_json['geometry']['coordinates'][0], coord_json['geometry']['coordinates'][1]))
        except:
            pass

    trajectory = LineString(trajectory_list)
    # Find total run time
    print("Total time:", coordproperties[-1]['time'], "seconds")
    print("Total length: {:.2f} m".format(trajectory.length))
    print("Average speed: {:.2f} km/h".format((trajectory.length / coordproperties[-1]['time']) * 3.6))
    print("")
    features = []
    for coord in coordproperties:
        if coord['type'] == "scan_camera" or coord['type'] == "scan_lidar":
            mode = coord['type']
            features.append([coord['currentTarget'], coord['currentScan'],
                        coord['currentEdge'], coord['MeetsPointThshld'], coord['NumClusters']])
    print("Mode:", mode)
    df = pd.DataFrame(features)
    feature_count = len(df[0].unique())
    print("Features: {}".format(feature_count))
    print("------------")
    for i in range(len(df[0].unique())):
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
        print("[{}] Scans: {:2} | Below Threshold: {:2} ({:6.2f}%)| Occlusions/concavities: {}".format(i, scan_count, threshold_failure, threshold_failure/scan_count*100, occlusions))
    outfile.close()
