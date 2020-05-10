#!/usr/bin/env python
import roslib; roslib.load_manifest('roverbridge')
import rospy
import argparse

import tf.transformations
from geometry_msgs.msg import Twist

DEFAULT_ROVER_ID = 'rover'
DEFAULT_ROVER_PORT = '/dev/serial0'
DEFAULT_ROVER_BAUDRATE = 115200

from rover import RoverClient
import time

class RoverBridge:
    def __init__(self, port, baudrate, vehicle_id, log_level=rospy.INFO):
        rospy.init_node('roverbridge', log_level=log_level)
        rospy.loginfo("Connecting to rover.")
        self.rover = RoverClient(port,baudrate)
        self.rover.connect()
        
        # Suscribers
        self.vehicle_id = vehicle_id
        self.cmd_vel_topic = self.vehicle_id+"/"+"cmd_vel"
        rospy.loginfo("Suscribed to %s" % self.cmd_vel_topic)
        rospy.Subscriber(self.cmd_vel_topic, Twist, self.cmd_vel_callback)

        # Publishers
        # TODO General TMY
        # TODO IMU
        # TODO GPS
        
    def run(self):
        while not rospy.is_shutdown():
            rospy.spin()
        rospy.loginfo("Disconnecting from rover.")
        self.rover.disconnect()

    # Suscribers

    # Suscribers :: Motor Control
    def cmd_vel_callback(self,msg):        
        rospy.loginfo("/cmd_vel received")
        linear = msg.linear.x
        angular = msg.angular.z
        v_l = (linear - angular) * 255.0
        v_r = (linear + angular) * 255.0
        self.rover.update_motor_speeds( [int(v_l),int(v_r)], RoverClient.MOTOR_A | RoverClient.MOTOR_B )  

    # Publishers :: General TMY
    # Publishers :: IMU
    # Publishers :: GPS

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Interface with Rover')
    parser.add_argument('--vehicle_id', type=str, default=DEFAULT_ROVER_ID, help='rover identifier')
    parser.add_argument('--port', type=str, default=DEFAULT_ROVER_PORT, help='port')
    parser.add_argument('--baudrate', type=int, default=DEFAULT_ROVER_BAUDRATE, help='baudrate')
    parser.add_argument('--loglevel', type=int, default=rospy.DEBUG, help='loglevel (0=trace, 6=critical)' )
    args = parser.parse_args()
    rover_ctl = RoverBridge(args.port, args.baudrate, args.vehicle_id, args.loglevel)
    rover_ctl.run()
