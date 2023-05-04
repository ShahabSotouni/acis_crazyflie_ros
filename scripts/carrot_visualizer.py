# !/usr/local/bin/python
import math
import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import time


class CarrotVisualizer:
    def __init__(self):
        self.plt = plt
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection='3d')
    def update_plot(self, carrot, follower):
        self.ax.clear()
        self.ax.scatter(*carrot, c='r', marker='o', label='Carrot')
        self.ax.scatter(*follower, c='b', marker='^', label='Follower')
        # Draw a line between the two points
        self.ax.plot([carrot[0], follower[0]], [carrot[1], follower[1]], [carrot[2], follower[2]], 'g:', label='Line')
        self.ax.legend()
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_zlabel('Z')
        self.ax.set_xlim(-2, 2)
        self.ax.set_ylim(-2, 2)
        self.ax.set_zlim(-2, 2)
        self.plt.pause(0.001)
