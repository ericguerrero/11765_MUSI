#!/usr/bin/env python3
# -*- coding: utf-8 -*-
'''

'''

import numpy as np
import matplotlib.pyplot as plt


class Reader():
    def __init__(self, dataset, robot, end_frame, plot=True, metrics=True):
        self.load_data(dataset, robot, end_frame)

    def load_data(self, dataset, robot, end_frame):
        # Loading dataset
        # Barcodes: [Subject#, Barcode#]
        self.barcodes_data = np.loadtxt(dataset + "/Barcodes.dat")
        # Ground truth: [Time[s], x[m], y[m], orientation[rad]]
        self.groundtruth_data = np.loadtxt(dataset + "/" + robot +"_Groundtruth.dat")
        # Landmark ground truth: [Subject#, x[m], y[m]]
        self.landmark_groundtruth_data = np.loadtxt(dataset + "/Landmark_Groundtruth.dat")
        # Measurement: [Time[s], Subject#, range[m], bearing[rad]]
        self.measurement_data = np.loadtxt(dataset + "/" + robot +"_Measurement.dat")
        # Odometry: [Time[s], Subject#, forward_V[m/s], angular _v[rad/s]]
        self.odometry_data = np.loadtxt(dataset + "/" + robot +"_Odometry.dat")

        # Collect all input data and sort by timestamp
        # Add subject "odom" = -1 for odometry data
        odom_data = np.insert(self.odometry_data, 1, -1, axis = 1)
        self.data = np.concatenate((odom_data, self.measurement_data), axis = 0)
        self.data = self.data[np.argsort(self.data[:, 0])]

        # Remove all data before the fisrt timestamp of groundtruth
        # Use first groundtruth data as the initial location of the robot
        for i in range(len(self.data)):
            if (self.data[i][0] > self.groundtruth_data[0][0]):
                break
        self.data = self.data[i:]

        # Remove all data after the specified number of frames
        self.data = self.data[:end_frame]
        cut_timestamp = self.data[end_frame - 1][0]
        # Remove all groundtruth after the corresponding timestamp
        for i in range(len(self.groundtruth_data)):
            if (self.groundtruth_data[i][0] >= cut_timestamp):
                break
        self.groundtruth_data = self.groundtruth_data[:i]

        # Combine barcode Subject# with landmark Subject# to create lookup-table
        # [x[m], y[m], x std-dev[m], y std-dev[m]]
        self.landmark_locations = {}
        for i in range(5, len(self.barcodes_data), 1):
            self.landmark_locations[self.barcodes_data[i][1]] = self.landmark_groundtruth_data[i - 5][1:]

        # Lookup table to map barcode Subjec# to landmark Subject#
        # Barcode 6 is the first landmark (1 - 15 for 6 - 20)
        self.landmark_indexes = {}
        for i in range(5, len(self.barcodes_data), 1):
            self.landmark_indexes[self.barcodes_data[i][1]] = i - 4
            

if __name__ == "__main__":
    # Dataset 1
    dataset = "data/MRCLAM_Dataset1"
    end_frame = 3200
    robot = 'Robot1'
    #
    r = Reader(dataset, robot, end_frame)
