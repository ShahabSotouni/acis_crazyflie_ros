#!/usr/bin/env python

import math
import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import time
import sys
import os
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

import carrot_visualizer

p_gain = 2
i_gain = 0.2
d_gain = 0.01


class CarrotGenerator:
    """Generates a carrot for the Crazyflie to follow"""

    time_datum = 0.0
    period_horizontal = 10.0
    period_vertical = 60.0
    radius = 1.0
    height = 1.0
    """ Constructor"""

    def __init__(self, time_datum, period_horizontal, period_vertical, radius, height):
        """
        Args:
            time_datum: time at which the carrot is at the origin
            period_horizontal: time for one revolution in the horizontal plane
            period_vertical: time for one revolution in the vertical plane
            radius: radius of the horizontal circle
            height: height of the vertical circle"""
        self.time_datum = time_datum
        self.period_horizontal = period_horizontal
        self.period_vertical = period_vertical
        self.radius = radius
        self.height = height

    def generate_carrot(self):
        local_time = rospy.get_time() - self.time_datum
        thetaH = math.pi * 2 * local_time / self.period_horizontal
        thetaV = math.pi * 2 * local_time / self.period_vertical
        x_carrot = self.radius * math.cos(thetaH)
        y_carrot = self.radius * math.sin(thetaH)
        z_carrot = self.height * math.sin(thetaV)
        return [x_carrot, y_carrot, z_carrot]

def reader():
    # Configure ROS node
    pub = rospy.Publisher("crazyflie", PoseStamped, queue_size=100)
    carrot_pub = rospy.Publisher("crazyflie_carrot", PoseStamped, queue_size=100)

    carrot_plot = carrot_visualizer.CarrotVisualizer()
    
    rospy.init_node("cf_data_publisher", anonymous=True)
    rate = rospy.Rate(20)  # 20hz
    cg = CarrotGenerator(rospy.get_time(), 15.0, 60.0, 1.0, 0.5)
    drone_pos = np.random.normal([0.0, 0.0, 0.0], [1, 1, 0.5], 3)
    drone_vel = np.zeros(3)
    drone_acc = np.zeros(3)
    integrator = np.zeros(3)
    print(drone_pos)
    err0 = np.zeros(3)
    while not rospy.is_shutdown():
        carrot = cg.generate_carrot()
        carrot[2] = carrot[2] + 1.0
        err = carrot - drone_pos
        errdiff = (err - err0) * (20.0)
        err0 = err
        dist = np.linalg.norm(err)
        #dist = max(dist, 0.05)
        integrator = integrator + err * (1 / 20.0)
        #drone_acc = dist * err * p_gain + integrator * i_gain + errdiff * d_gain
        vel_cmd = dist * err * p_gain + integrator * i_gain + errdiff * d_gain
        drone_vel = drone_vel*0.5 + vel_cmd *0.5
        drone_pos = drone_pos + drone_vel * (1 / 20.0)
        msg = PoseStamped()
        msg.header.stamp.secs = rospy.Time.now().secs
        msg.header.stamp.nsecs = rospy.Time.now().nsecs
        msg.header.frame_id = "parent_crazyflie_frame"
        msg.pose.position.x = drone_pos[0]
        msg.pose.position.y = drone_pos[1]
        msg.pose.position.z = drone_pos[2]
        msg.pose.orientation.w = 1.0
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = 0.0
        msg_carrot = PoseStamped()
        msg_carrot.header.stamp.secs = rospy.Time.now().secs
        msg_carrot.header.stamp.nsecs = rospy.Time.now().nsecs
        msg_carrot.header.frame_id = "parent_crazyflie_frame"
        msg_carrot.pose.position.x = carrot[0]
        msg_carrot.pose.position.y = carrot[1]
        msg_carrot.pose.position.z = carrot[2]
        msg_carrot.pose.orientation.w = 1.0
        msg_carrot.pose.orientation.x = 0.0
        msg_carrot.pose.orientation.y = 0.0
        msg_carrot.pose.orientation.z = 0.0
        carrot_pub.publish(msg_carrot)
        # print(msg)
        print(dist)
        pub.publish(msg)
        
        carrot_plot.update_plot(carrot, drone_pos)
        
        rate.sleep()


if __name__ == "__main__":
    try:
        reader()
    except rospy.ROSInterruptException:
        pass
