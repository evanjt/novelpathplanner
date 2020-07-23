#!/usr/bin/python3

from math import sqrt

def euclidean_dist(x2, y2, z2):
    x1, y1, z1 = (0, 0, 0)
    dist = sqrt((x1-x2)**2 + (y1-y2)**2 + (z1-z2)**2)
    return dist
