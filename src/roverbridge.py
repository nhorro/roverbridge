#!/usr/bin/env python
import roslib; roslib.load_manifest('roverbridge')
import rospy
import tf
import argparse
import struct
from math import sin, cos, pi

from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from nav_msgs.msg import Odometry

DEFAULT_ROVER_ID = 'rover'
DEFAULT_ROVER_PORT = '/dev/serial0'
DEFAULT_ROVER_BAUDRATE = 115200

from rover import RoverClient

class RoverBridge:
    def __init__( self, port, baudrate, node_name, vehicle_id, disable_motors, log_level=rospy.INFO):
        rospy.init_node(node_name, log_level=log_level)
        rospy.loginfo("Connecting to rover.")

        self.disable_motors=disable_motors
        self.simulate_odometry = True
        self.simulate_imu = True

        # Initial parameters
        self.last_time = rospy.Time.now()
        # Position
        self.x = 0.0          
        self.y = 0.0         
        self.theta = 0.0       
        # Velocity
        self.vx = 0.0
        self.vy = 0.0
        self.v = 0.0           # Magnitud of input speed vector (-1.0,1.0)                
        self.vtheta = 0.0      # Orientation of speed vector in radians (0=Forward, <0=left, >0 right)        

        # Unicycle_modek
        #self.v_l = 0.0         # Speed of left wheel (RPM)
        #self.v_r = 0.0         # Speed of right wheel (RPM)
        #self.wheel_distance = 0.12
        #self.wheel_radius = 0.03            

        self.report_handlers = [
            self.handle_general_tmy_report,
            self.handle_command_execution_status_report,
            self.handle_ahrs_state_report
        ]
        self.rover = RoverClient(port,baudrate, self.report_handlers)
        self.rover.connect()
        
        # Suscribers
        self.vehicle_id = vehicle_id
        self.cmd_vel_topic = self.vehicle_id+"/"+"cmd_vel"
        rospy.loginfo("Suscribed to %s" % self.cmd_vel_topic)
        rospy.Subscriber(self.cmd_vel_topic, Twist, self.cmd_vel_callback)

        # Publishers

        # Odometry
        self.odom_msg = Odometry()
        self.odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)
        self.odom_broadcaster = tf.TransformBroadcaster()
        
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
        rospy.logdebug("/cmd_vel received")

        self.v = msg.linear.x        
        self.vtheta = msg.angular.z

        # Update motor speeds (actually applied currents)
        if not self.disable_motors:
            v_a = (self.v - self.vtheta)
            v_b = (self.v + self.vtheta)
            self.rover.update_motor_speeds( 
                [
                    int(v_a * 255.0),
                    int(v_b * 255.0)
                ], 
                RoverClient.MOTOR_A | RoverClient.MOTOR_B 
            )  

        current_time = rospy.Time.now()
        dt = (current_time - self.last_time).to_sec()

        # Update simulated odometry
        if self.simulate_odometry:

            # TODO: replace for unicycle model
            self.vx = self.v * cos(self.vtheta)
            self.vy = self.v * sin(self.vtheta)
            self.theta += self.vtheta*dt
            self.x += self.vx*dt
            self.y += self.vy*dt

            odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.theta)
            self.odom_broadcaster.sendTransform(
                (self.x, self.y, 0.),
                odom_quat,
                current_time,
                "base_link",
                "odom"
            )

            # next, we'll publish the odometry message over ROS    
            self.odom_msg.header.stamp = current_time
            self.odom_msg.header.frame_id = "odom"

            # set the position
            self.odom_msg.pose.pose = Pose(
                Point(self.x, self.y, 0.), Quaternion(*odom_quat)
            )

            # set the velocity
            self.odom_msg.child_frame_id = "base_link"
            self.odom_msg.twist.twist = Twist(
                Vector3(self.vx, self.vy, 0.), 
                Vector3(0, 0, self.vtheta)
            )

            # publish the message
            self.odom_pub.publish(self.odom_msg)
    
        if self.simulate_imu:
            pass

        self.last_time = current_time

    # Publishers :: General TMY
    def handle_general_tmy_report(self, payload):
        rospy.logdebug("General telemetry report received")
        pass

    def handle_command_execution_status_report(self, payload):
        rospy.logdebug("Command execution status report received")
        pass

    def handle_ahrs_state_report(self, payload):
        rospy.logdebug("AHRS report received")

        imu_status = payload[1]
        imu_values = struct.unpack("<10f", payload[4:])
        
        """           
        print("IMU Status:", imu_status)
        for i in imu_values:
            print(i)
                 
        tmp = "Payload (" + str(len(payload)) + " bytes) :"
        for x in payload:
            tmp+= "%02X " % x
        print(tmp)
        """
        

    # Publishers :: IMU
    # Publishers :: GPS

def str2bool(v):
    if isinstance(v, bool):
       return v
    if v.lower() in ('yes', 'true', 't', 'y', '1'):
        return True
    elif v.lower() in ('no', 'false', 'f', 'n', '0'):
        return False
    else:
        raise argparse.ArgumentTypeError('Boolean value expected.')    

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Interface with Rover')
  
    parser.add_argument('--port', type=str, default=DEFAULT_ROVER_PORT, help='port')
    parser.add_argument('--baudrate', type=int, default=DEFAULT_ROVER_BAUDRATE, help='baudrate')

    parser.add_argument('--name', type=str, default="roverbridge", help='node name')
    parser.add_argument('--vehicle_id', type=str, default=DEFAULT_ROVER_ID, help='rover identifier prefix for pub/sub.')
    
    parser.add_argument('--disable_motors', type=str2bool, default=False, help='disable motors (for testing odometry, imu, etc.)' )
    parser.add_argument('--simulate_odometry', type=str2bool, default=True, help='publish simulated odometry' )
    parser.add_argument('--simulate_imu', type=str2bool, default=True, help='publish simulated IMU' )

    parser.add_argument('--loglevel', type=int, default=rospy.DEBUG, help='loglevel (0=trace, 6=critical)' )

    args, unknown = parser.parse_known_args()

    print(args.name)

    rover_ctl = RoverBridge(    port=args.port, 
                                baudrate=args.baudrate, 
                                node_name=args.name, 
                                vehicle_id=args.vehicle_id, 
                                disable_motors=args.disable_motors,
                                log_level=args.loglevel,
                             )
    rover_ctl.run()
