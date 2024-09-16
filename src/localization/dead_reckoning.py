#!/usr/bin/env python3
# -*- coding: utf-8 -*-
'''

'''

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


class DeadReckoning():
    def __init__(self, dataset, robot, end_frame, plot=True):
        self.load_data(dataset, robot, end_frame)
        self.plot = plot

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
            if (self.data[i][1] == -1):
                if (self.data[i][0] > self.groundtruth_data[0][0]):
                    break
        self.data = self.data[i:]
        
        for i in range(len(self.groundtruth_data)):
            if (self.groundtruth_data[i][0] > self.data[0][0]):
                break
        self.groundtruth_data = self.groundtruth_data[i:]
        for i in range(len(self.data)):
            if (self.data[i][1] == -1):
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
           
    def run(self): 
        self.initialization()
        for data in self.data:
            if (data[1] == -1):
                self.motion_update(data)
        if self.plot: self.plot_data()
            
    def initialization(self):
        # Initial state
        self.states = np.array([self.groundtruth_data[0]])
        self.last_timestamp = self.states[-1][0]

    def motion_update(self, control):
        # State: [x, y, θ]
        # Control: [v, w]
        # State-transition function (simplified):
        # [x_t, y_t, θ_t] = g(u_t, x_t-1)
        #   x_t  =  x_t-1 + v * cosθ_t-1 * delta_t
        #   y_t  =  y_t-1 + v * sinθ_t-1 * delta_t
        #   θ_t  =  θ_t-1 + w * delta_t
        
        # Skip motion update if two odometry data are too close
        delta_t = control[0] - self.last_timestamp
        if (delta_t < 0.001):
            return
        # Compute updated [x, y, theta]
        x_t = self.states[-1][1] + control[2] * np.cos(self.states[-1][3]) * delta_t
        y_t = self.states[-1][2] + control[2] * np.sin(self.states[-1][3]) * delta_t
        theta_t = self.states[-1][3] + control[3] * delta_t
        # Limit θ within [-pi, pi]
        if (theta_t > np.pi):
            theta_t -= 2 * np.pi
        elif (theta_t < -np.pi):
            theta_t += 2 * np.pi
        self.last_timestamp = control[0]
        self.states = np.append(self.states, np.array([[control[0], x_t, y_t, theta_t]]), axis = 0)
        
    def plot_data(self):
        # Ground truth data
        plt.plot(self.groundtruth_data[:, 1], self.groundtruth_data[:, 2], 'b', label="Robot State Ground truth")

        # States
        plt.plot(self.states[:, 1], self.states[:, 2], 'r', label="Robot State Estimate")

        # Start and end points
        plt.plot(self.groundtruth_data[0, 1], self.groundtruth_data[0, 2], 'go', label="Start point")
        plt.plot(self.groundtruth_data[-1, 1], self.groundtruth_data[-1, 2], 'yo', label="End point")

        # Landmark ground truth locations and indexes
        landmark_xs = []
        landmark_ys = []
        for location in self.landmark_locations:
            landmark_xs.append(self.landmark_locations[location][0])
            landmark_ys.append(self.landmark_locations[location][1])
            index = self.landmark_indexes[location] + 5
            plt.text(landmark_xs[-1], landmark_ys[-1], str(index), alpha=0.5, fontsize=10)
        plt.scatter(landmark_xs, landmark_ys, s=200, c='k', alpha=0.2, marker='*', label='Landmark Locations')

        # plt.title("Localization with only odometry data")
        plt.title("Dead Reckoning")
        plt.legend()
        plt.show()
        
    def represent_dataset(self):
        # Ground truth data
        plt.plot(self.groundtruth_data[:, 1], self.groundtruth_data[:, 2], 'b', label="Robot State Ground truth")

        # Start and end points
        plt.plot(self.groundtruth_data[0, 1], self.groundtruth_data[0, 2], 'gx', label="Start point")
        plt.plot(self.groundtruth_data[-1, 1], self.groundtruth_data[-1, 2], 'rx', label="End point")

        # Landmark ground truth locations and indexes
        landmark_xs = []
        landmark_ys = []
        for location in self.landmark_locations:
            landmark_xs.append(self.landmark_locations[location][0])
            landmark_ys.append(self.landmark_locations[location][1])
            index = self.landmark_indexes[location] + 5
            plt.text(landmark_xs[-1], landmark_ys[-1], str(index), alpha=0.5, fontsize=10)
        plt.scatter(landmark_xs, landmark_ys, s=200, c='k', alpha=0.2, marker='*', label='Landmark Locations')

        plt.title("Robot Groundtruth and Map")
        plt.legend(bbox_to_anchor=(1.05, 1), loc=2, borderaxespad=0.)
        plt.show()
            
    def build_dataframes(self):
        self.gt = build_timeseries(self.groundtruth_data, cols=['stamp','x','y','theta'])
        self.states = build_timeseries(self.states, cols=['stamp','x','y','theta'])
        self.measurements = build_timeseries(self.data, cols=['stamp','type','range_l','bearing_l'])
        self.motion = self.measurements[self.measurements.type == -1].rename(columns={'range_l': 'v', 'bearing_l': 'omega'})
        landmarks = self.measurements[self.measurements.type != -1]
        self.sensor = filter_static_landmarks(landmarks, self.barcodes_data)
        
        
    def transform_landmarks(self):
        self.sensor_gt = self.sensor.join(self.gt).dropna()
        range_l = self.sensor_gt.range_l
        bearing_l = self.sensor_gt.bearing_l
        x_t = self.sensor_gt.x
        y_t =  self.sensor_gt.y
        theta_t = self.sensor_gt.theta

        x = range_l*np.cos(bearing_l)
        y = range_l*np.sin(bearing_l)

        self.sensor_gt['x_l'] = x_t + x*np.cos(theta_t) - y*np.sin(theta_t)
        self.sensor_gt['y_l'] = y_t + x*np.sin(theta_t) + y*np.cos(theta_t)

        
def build_timeseries(data,cols):
    timeseries = pd.DataFrame(data, columns=cols)
    timeseries['stamp'] = pd.to_datetime(timeseries['stamp'], unit='s')
    timeseries = timeseries.set_index('stamp')
    return timeseries

def filter_static_landmarks(lm, barcodes):
    for L,l in dict(barcodes).items(): # Translate barcode num to landmark num
        lm[lm==l]=L
    lm = lm[lm.type > 5] # Keep only static landmarks 
    return lm 

if __name__ == "__main__":
    # Dataset 1
    dataset = "data/MRCLAM_Dataset1"
    end_frame = 3200
    robot = 'Robot1'
    #
    r = Reader(dataset, robot, end_frame)
