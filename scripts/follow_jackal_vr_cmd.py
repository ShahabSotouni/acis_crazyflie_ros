#!/usr/bin/env python3
import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry


# Import crazyflie lib
import time
from threading import Timer

import cflib.crtp  # noqa
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.position_hl_commander import PositionHlCommander
from cflib.utils import uri_helper
from pynput.keyboard import Key, Listener, KeyCode
from enum import Enum

# Importing math Library
import math

# Importing carrot_visualizer
import carrot_visualizer


debugFlag = True

# URI to the Crazyflie to connect to
uri = uri_helper.uri_from_env(default="radio://0/80/2M/E7E7E7E7E7")


class FlightMode(Enum):
    FOLLOW_JACKAL = 1
    FOLLOW_CARROT = 2
    LAND = 3
    IDLE = 4


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


class CFLogger:
    """
    Simple logging class that logs the Data from a supplied
    link uri.
    """

    # Store last pose as a class variable
    lastPose = PoseStamped()
    p_horizontal = 0.5
    p_vertical = 0.5
    max_vel_horizontal = 0.3
    max_vel_vertical = 0.2
    safetyTimer = None
    mode = FlightMode.IDLE
    jackal_pose = [0.0, 0.0, 0.0]
    # Get Publisher handle from ROS

    def __init__(self, link_uri, pub, carrot_pub, plot):
        """Initialize and run the example with the specified link_uri"""
        self.carrot_plot = plot
        self.pub = pub
        self.carrot_pub = carrot_pub
        self._cf = Crazyflie(rw_cache="./cache")
        self.scf = SyncCrazyflie(link_uri, self._cf)
        # create carrot generator instance
        self.cg = CarrotGenerator(rospy.get_time(), 15.0, 60.0, 1.0, 0.5)
        # Connect some callbacks from the Crazyflie API
        self._cf.connected.add_callback(self._connected)
        self._cf.disconnected.add_callback(self._disconnected)
        self._cf.connection_failed.add_callback(self._connection_failed)
        self._cf.connection_lost.add_callback(self._connection_lost)

        # Print keyboard commands
        print("Keyboard Commands:")
        print("i: Idle Mode")
        print("c: Carrot Mode")
        print("j: Jackal Mode")
        print("space: Land")

        self.listener = Listener(on_press=self.on_press, on_release=self.on_release)
        self.listener.start()

        rospy.loginfo("Connecting to %s" % link_uri)

        # Try to connect to the Crazyflie
        self._cf.open_link(link_uri)

        # Position Commander
        self.pc = PositionHlCommander(
            self.scf,
            x=0.0,
            y=0.0,
            z=0.0,
            default_velocity=0.3,
            default_height=0.5,
            controller=PositionHlCommander.CONTROLLER_PID,
        )
        rospy.loginfo("waiting for 10 seconds before starting the motion")
        rospy.sleep(10)
        self.pc.take_off(0.5, 0.2)
        rospy.loginfo("TakeOff!")
        
        # Variable used to keep main loop occupied until disconnect
        self.is_connected = True

    # Destructor
    def __del__(self):
        # body of destructor
        rospy.loginfo("Destructor Called! Landing Now!")
        print("Destructor Called! Landing Now!")
        self.pc.land()

    def _connected(self, link_uri):
        """This callback is called form the Crazyflie API when a Crazyflie
        has been connected and the TOCs have been downloaded."""

        rospy.loginfo("Connected to %s" % link_uri)

        # The definition of the logconfig can be made before connecting
        self._lg_stab = LogConfig(name="Stabilizer", period_in_ms=50)
        self._lg_stab.add_variable("stateEstimate.x", "float")
        self._lg_stab.add_variable("stateEstimate.y", "float")
        self._lg_stab.add_variable("stateEstimate.z", "float")
        self._lg_stab.add_variable("stabilizer.roll", "float")
        self._lg_stab.add_variable("stabilizer.pitch", "float")
        self._lg_stab.add_variable("stabilizer.yaw", "float")
        # The fetch-as argument can be set to FP16 to save space in the log packet
        self._lg_stab.add_variable("pm.vbat", "FP16")

        # Adding the configuration cannot be done until a Crazyflie is
        # connected, since we need to check that the variables we
        # would like to log are in the TOC.
        try:
            self._cf.log.add_config(self._lg_stab)
            # This callback will receive the data
            self._lg_stab.data_received_cb.add_callback(self._stab_log_data)
            rospy.loginfo("Added Stabilizer data received callback")
            # This callback will be called on errors
            self._lg_stab.error_cb.add_callback(self._stab_log_error)
            # Start the logging
            self._lg_stab.start()
        except KeyError as e:
            rospy.loginfo(
                "Could not start log configuration,"
                "{} not found in TOC".format(str(e))
            )
        except AttributeError:
            rospy.loginfo("Could not add Stabilizer log config, bad configuration.")

    def _stab_log_error(self, logconf, msg):
        """Callback from the log API when an error occurs"""
        rospy.loginfo("Error when logging %s: %s" % (logconf.name, msg))

    def _stab_log_data(self, timestamp, data, logconf):
        """Callback from a the log API when data arrives"""
        if debugFlag:
            print("Data Arrived Callback")
        # print(self.safetyTimer)
        if self.safetyTimer is not None:
            self.safetyTimer.cancel()

        # check if rospy is shutdown
        # if rospy.is_shutdown:
        #     print("ROS is shutdown, removing callback and landing")
        #     self._lg_stab.data_received_cb.remove_callback(self._stab_log_data)
        #     self.pc.land()

        # print("Data Arrived Callback2")
        # rospy.loginfo(f'[{timestamp}][{logconf.name}]: ')
        msg = PoseStamped()
        # print()
        msg.header.stamp.secs = rospy.Time.now().secs
        msg.header.stamp.nsecs = rospy.Time.now().nsecs
        msg.header.frame_id = "parent_crazyflie_frame"
        x = msg.pose.position.x = data["stateEstimate.x"]
        y = msg.pose.position.y = data["stateEstimate.y"]
        z = msg.pose.position.z = data["stateEstimate.z"]
        if debugFlag:
            print(f"x: ", x, "y: ", y, "z: ", z)
        msg.pose.orientation.w = 1.0
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = 0.0
        self.pub.publish(msg)
        carrot = self.cg.generate_carrot()
        carrot[2] = carrot[2] + 1.0
        msg_carrot = PoseStamped()
        msg_carrot.header.stamp.secs = rospy.Time.now().secs
        msg_carrot.header.stamp.nsecs = rospy.Time.now().nsecs
        msg_carrot.header.frame_id = "parent_crazyflie_frame"
        if debugFlag:
            print(f"mode: {self.mode.name}")
        if self.mode == FlightMode.FOLLOW_CARROT:
            xc = carrot[0]
            yc = carrot[1]
            zc = carrot[2]
        elif self.mode == FlightMode.FOLLOW_JACKAL:
            xc = self.jackal_pose[0]
            yc = self.jackal_pose[1]
            zc = 1.0
        else:
            xc = 0.0
            yc = 0.0
            zc = 0.5

        saturate = lambda x, a, b: min(max(x, a), b)
        xc = saturate(xc, -1.4, 1.4)
        yc = saturate(yc, -1.4, 1.4)
        zc = saturate(zc, 0.5, 1.5)

        msg_carrot.pose.position.x = xc
        msg_carrot.pose.position.y = yc
        msg_carrot.pose.position.z = zc
        msg_carrot.pose.orientation.w = 1.0
        msg_carrot.pose.orientation.x = 0.0
        msg_carrot.pose.orientation.y = 0.0
        msg_carrot.pose.orientation.z = 0.0
        self.carrot_pub.publish(msg_carrot)

        if debugFlag:
            print(f"xc: {xc}, yc: {yc}, zc: {zc}")
        plot_c = [xc, yc, zc]
        plot_d = [x, y, z]
        if debugFlag:
            print("plot_c: ", plot_c, "plot_d: ", plot_d)
        # self.carrot_plot.update_plot(plot_c, plot_d)
        # if debugFlag:
        #    print("carrot plot updated")
        if self.mode == FlightMode.LAND:
            self.pc.land()
        else:
            self.pc.go_to(xc, yc, zc, 1)
        # Link Safety Timer
        self.safetyTimer = Timer(1, self.safetyTimerCallback)
        self.safetyTimer.start()

    def _connection_failed(self, link_uri, msg):
        """Callback when connection initial connection fails (i.e no Crazyflie
        at the specified address)"""
        rospy.loginfo("Connection to %s failed: %s" % (link_uri, msg))
        self.is_connected = False

    def _connection_lost(self, link_uri, msg):
        """Callback when disconnected after a connection has been made (i.e
        Crazyflie moves out of range)"""
        rospy.loginfo("Connection to %s lost: %s" % (link_uri, msg))

    def _disconnected(self, link_uri):
        """Callback when the Crazyflie is disconnected (called in all cases)"""
        rospy.loginfo("Disconnected from %s" % link_uri)
        self.is_connected = False

    def safetyTimerCallback(self):
        rospy.loginfo("Safety Timer Timeout! Landing Now!")
        self.mode = FlightMode.LAND
        self.pc.land()

    def on_press(self, key):
        print("{0} pressed".format(key))
        print("key: ", key)
        if key == KeyCode.from_char("i") or key == KeyCode.from_char("I"):
            # Idle Mode
            print("Idle Mode")
            self.mode = FlightMode.IDLE
        elif key == KeyCode.from_char("c") or key == KeyCode.from_char("C"):
            # Carrot Mode
            print("Follow Carrot Mode")
            self.mode = FlightMode.FOLLOW_CARROT
        elif key == KeyCode.from_char("j") or key == KeyCode.from_char("J"):
            # Jackal Mode
            print("Follow Jackal Mode")
            self.mode = FlightMode.FOLLOW_JACKAL
        elif key == Key.space:
            # Land
            self.mode = FlightMode.LAND
            print("Land")
            self.pc.land()

    def on_release(self, key):
        print("{0} release".format(key))
        if key == Key.esc:
            # Stop listener
            return False

    def jackal_callback(self, data):
        self.jackal_pose[0] = 2.50 - data.pose.pose.position.x
        self.jackal_pose[1] = -data.pose.pose.position.y
        if debugFlag:
            print("jackal_pose: ", self.jackal_pose)
    def jackal_callback_tf(self, data):
        self.jackal_pose[0] = 2.50 - data.transform.translation.x
        self.jackal_pose[1] = -data.transform.translation.x
        if debugFlag:
            print("jackal_pose_tf: ", self.jackal_pose)
    def vr_callback(self, data):
        self.jackal_pose[0] = 2.50 - data.transform.translation.x
        self.jackal_pose[1] = -data.transform.translation.x
        if debugFlag:
            print("jackal_pose_tf: ", self.jackal_pose)


def reader():
    # Configure ROS node
    pub = rospy.Publisher("crazyflie", PoseStamped, queue_size=100)
    carrot_pub = rospy.Publisher("crazyflie_carrot", PoseStamped, queue_size=100)

    rospy.init_node("cf_data_publisher", anonymous=True)

    # set rate
    rate = rospy.Rate(10)  # 10hz

    # Initialize the low-level drivers
    rospy.loginfo("Initializing drivers")
    cflib.crtp.init_drivers()

    carrot_plot = carrot_visualizer.CarrotVisualizer()

    # Create instance of the logging class
    le = CFLogger(uri, pub, carrot_pub, carrot_plot)

    jackal_sub = rospy.Subscriber("/odometry/filtered", Odometry, le.jackal_callback)
    # jackal_sub_tf = rospy.Subscriber("/tf_echo", TransformStamped, le.jackal_callback_tf)
    
    vr_sub = rospy.Subscriber("/odometry/filtered", Odometry, le.jackal_callback)

    rospy.spin()

    rospy.loginfo("waiting for connection to be closed...")
    le._cf.close_link()
    while le.is_connected:
        time.sleep(1)
    le.listener.stop()


if __name__ == "__main__":
    try:
        reader()
    except rospy.ROSInterruptException:
        pass
