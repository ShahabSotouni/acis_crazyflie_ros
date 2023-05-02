#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped

# Import crazyflie lib
import logging
import time
from threading import Timer

import cflib.crtp  # noqa
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.utils import uri_helper


# URI to the Crazyflie to connect to
uri = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')

class CFLogger:
    """
    Simple logging class that logs the Data from a supplied
    link uri.
    """
    # Store last pose as a class variable
    lastPose = PoseStamped()
    
    # Get Publisher handle from ROS
    
    def __init__(self, link_uri, pub):
        """ Initialize and run the example with the specified link_uri """
        self.pub = pub
        self._cf = Crazyflie(rw_cache='./cache')

        # Connect some callbacks from the Crazyflie API
        self._cf.connected.add_callback(self._connected)
        self._cf.disconnected.add_callback(self._disconnected)
        self._cf.connection_failed.add_callback(self._connection_failed)
        self._cf.connection_lost.add_callback(self._connection_lost)

        rospy.loginfo('Connecting to %s' % link_uri)

        # Try to connect to the Crazyflie
        self._cf.open_link(link_uri)

        # Variable used to keep main loop occupied until disconnect
        self.is_connected = True

    def _connected(self, link_uri):
        """ This callback is called form the Crazyflie API when a Crazyflie
        has been connected and the TOCs have been downloaded."""
        rospy.loginfo('Connected to %s' % link_uri)

        # The definition of the logconfig can be made before connecting
        self._lg_stab = LogConfig(name='Stabilizer', period_in_ms=50)
        self._lg_stab.add_variable('stateEstimate.x', 'float')
        self._lg_stab.add_variable('stateEstimate.y', 'float')
        self._lg_stab.add_variable('stateEstimate.z', 'float')
        self._lg_stab.add_variable('stabilizer.roll', 'float')
        self._lg_stab.add_variable('stabilizer.pitch', 'float')
        self._lg_stab.add_variable('stabilizer.yaw', 'float')
        # The fetch-as argument can be set to FP16 to save space in the log packet
        self._lg_stab.add_variable('pm.vbat', 'FP16')

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
            rospy.loginfo('Could not start log configuration,'
                  '{} not found in TOC'.format(str(e)))
        except AttributeError:
            rospy.loginfo('Could not add Stabilizer log config, bad configuration.')

        # Start a timer to disconnect in 10s
        #t = Timer(5, self._cf.close_link)
        #t.start()

    def _stab_log_error(self, logconf, msg):
        """Callback from the log API when an error occurs"""
        rospy.loginfo('Error when logging %s: %s' % (logconf.name, msg))

    def _stab_log_data(self, timestamp, data, logconf):
        """Callback from a the log API when data arrives"""
        print("Check point 1")
        # rospy.loginfo(f'[{timestamp}][{logconf.name}]: ')
        msg = PoseStamped()
        # print()
        msg.header.stamp.secs = rospy.Time.now().secs
        msg.header.stamp.nsecs = rospy.Time.now().nsecs
        msg.header.frame_id = 'map'
        msg.pose.position.x = data["stateEstimate.x"]
        msg.pose.position.y = data["stateEstimate.y"]
        msg.pose.position.z = data["stateEstimate.z"]
        msg.pose.orientation.w = 1.0
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = 0.0
        self.pub.publish(msg)
        print(msg)
        # for name, value in data.items():
        #     rospy.loginfo(f'{name}: {value:3.3f} ')
            # print("data: ", data)

    def _connection_failed(self, link_uri, msg):
        """Callback when connection initial connection fails (i.e no Crazyflie
        at the specified address)"""
        rospy.loginfo('Connection to %s failed: %s' % (link_uri, msg))
        self.is_connected = False

    def _connection_lost(self, link_uri, msg):
        """Callback when disconnected after a connection has been made (i.e
        Crazyflie moves out of range)"""
        rospy.loginfo('Connection to %s lost: %s' % (link_uri, msg))

    def _disconnected(self, link_uri):
        """Callback when the Crazyflie is disconnected (called in all cases)"""
        rospy.loginfo('Disconnected from %s' % link_uri)
        self.is_connected = False



def reader():
    # Configure ROS node
    pub = rospy.Publisher('crazyflie', PoseStamped, queue_size=100)
    rospy.init_node('cf_data_publisher', anonymous=True)
   
    # rate = rospy.Rate(100) # 100hz
    
    # Initialize the low-level drivers
    rospy.loginfo("Initializing drivers")
    cflib.crtp.init_drivers()
    
    # Create instance of the logging class
    le = CFLogger(uri,pub)
    
    
    # while not rospy.is_shutdown():
    
    # msg = PoseStamped()
    
    # msg.header.stamp.secs = rospy.Time.now().secs
    # msg.header.stamp.nsecs = rospy.Time.now().nsecs
    # msg.header.frame_id = "cf1/lh"
    # msg.pose.position.x = 0.0
    # msg.pose.position.y = 0.0
    # msg.pose.position.z = 0.0
    # msg.pose.orientation.w = 1.0
    # msg.pose.orientation.x = 0.0
    # msg.pose.orientation.y = 0.0
    # msg.pose.orientation.z = 0.0
    
    # print(msg)   
        
    # #     info_str = "cf2: %s" % rospy.get_time()
    # #     rospy.loginfo(info_str)
    # pub.publish(msg)
    # #     rate.sleep()
    
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
    rospy.loginfo("waiting for connection to be closed...")
    le._cf.close_link()
    while le.is_connected:
        time.sleep(1)

if __name__ == '__main__':
    try:
        reader()
    except rospy.ROSInterruptException:
        pass
