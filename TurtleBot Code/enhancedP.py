# -*- coding: utf-8 -*-
"""───────────────────────────────────────────────────────────────────────
                     ┬─┐┌─┐┌┐ ┌─┐┌┬┐  ┬  ┌─┐┌┐                           
                     ├┬┘│ │├┴┐│ │ │   │  ├─┤├┴┐                          
                     ┴└─└─┘└─┘└─┘ ┴   ┴─┘┴ ┴└─┘                          
 ────────────────────────────────────────────────────────────────────────"""
"""
    Project Title: TurtleBot Implementation
    Author: Juan Lopez Muro, Cecilia Diaz, Shreya Srikanth
    Date: 01-31-2025
"""
"""                    README
Task0: The controller and the main section are the only methods that need to be modified. Both are at the end of this script.
Task1: Record a manual trajectory.
Task2: Play back the recorded manual trajectory in an open loop.
Task3: Design and code the feedback controller.
Task4: Follow the recorded manual trajectory in a closed loop using your controller.
Task5: Follow the proposed trajectory in a closed loop using your controller.
"""
import sys                      # Importing the sys module to access system-specific parameters and functions
import math                     # Importing the math module for math computations
import time                     # Importing the time module for time-related functions
import rospy                    # Importing rospy, the client library for ROS in Python
#import threading                # Importing the threading module for multi-threading capabilities
from datetime import datetime   # Importing the Twist message type from geometry_msgs
#from sshkeyboard import listen_keyboard                                     # Importing the function listen_keyboard from the sshkeyboard module
from geometry_msgs.msg import Twist                                  # Importing the Twist message type from geometry_msgs for turtlebot
from geometry_msgs.msg import TwistStamped, PoseStamped              # Importing the Twist message type from geometry_msgs for VICON
from sensor_msgs.msg   import Imu              # Importing the Twist message type from geometry_msgs
from tf.transformations import euler_from_quaternion, quaternion_from_euler # Importing the Twist message type from geometry_msgs


class Turtlebot:
    def __init__(self):
        """ -------- Start of Controller Gains --------- """
        self.vx_traj = 0.0
        self.vy_traj = 0.0
        self.omg_traj = 0.0

        self.x_r = 0.025  # Define your geometric parameters
        self.y_r = 0.025  # Define your geometric parameters
        
        self.k_x   = 1.0  # control proportional gain
        self.k_y   = 1.0  # control proportional gain
        self.k_yaw = 0.05 # control proportional gain
        
        self.k_vx  = 0.0
        self.k_vy  = 0.0
        self.k_omg = 0.0

        self.f_v = 1.0
        self.f_omg = 1.0
        
        self.linear_speed_max = 0.22
        self.angular_speed_max = 2.84
        """ -------- End of Controller Gains --------- """

        # Trajectory attributes
        self.linear_speed_traj  = 0.0
        self.angular_speed_traj = 0.0

        self.x_traj   = 0.0
        self.y_traj   = 0.0
        self.yaw_traj = 0.0

        self.vx_traj  = 0.0
        self.vy_traj  = 0.0
        self.omg_traj = 0.0
        
        # ROS Node Setup
        rospy.init_node('my_node')      # Initialize ROS node
        rospy.loginfo("Initializing...")    
        self.rate = None
        self.exit_requested = False     # Flag to indicate program termination
        self.debug_flag = False
        
        # Initializing variables to store linear and angular speeds
        self.linear_speed = 0.0
        self.angular_speed = 0.0

        # Variable to store log data for post-analysis
        self.start_time = None              # Variable to store the start time
        self.current_idx=0
        self.log_data_buffer = []
        self.log_lines = []                 # List to store lines from the log file
        log_string = f"log_0.txt"           # Default name of the log file
        self.log_file_name = log_string
        read_string = f"log_0.txt"          # Default name of the log file
        self.read_file_name = read_string

        # VICON Pose and twist attributes
        self.position           = (0.0, 0.0, 0.0)       # Initializing position as a tuple (x, y, z)
        self.orientation        = (0.0, 0.0, 0.0, 0.0)  # Initializing orientation as a tuple (x, y, z, w)        
        self.linear_velocity    = (0.0, 0.0, 0.0)       # Initializing linear velocity as a tuple (dx, dy, dz)
        self.angular_velocity   = (0.0, 0.0, 0.0)       # Initializing angular velocity as a tuple (wx, wy, wz)

        # VICON Computed psi and omega attributes
        self.roll   = 0.0
        self.pitch  = 0.0
        self.yaw    = 0.0
        self.omega  = 0.0 

        # Create a Twist message to control the Turtlebot's linear and angular speeds
        self.msg = Twist()

        # Publish the Twist message to the '/cmd_vel' topic using ROS Publisher
        self.speed_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
      
        # Flag to control subscription to the VICON node for PoseStamped and TwistStamped messages
        self.is_pose_subscribed = False
        self.is_twist_subscribed = False
        # Subscription to the VICON
        self.pose_subscriber = None
        self.twist_subscriber = None
        
        # # Flag to control subscription to the own twist
        
        self.is_own_imu_subscribed   = False
        self.is_own_twist_subscribed = False
        # Subscription to the own twist
        self.own_twist_subscriber = None
        self.current_linear_x  = 0
        self.current_angular_z = 0
        self.own_imu_subscriber = None
        self.imu_angular_velocity = (0.0, 0.0, 0.0)       # Initializing angular velocity as a tuple (wx, wy, wz)

    def subscribe_own_twist(self):
        # Subscribe to the TwistStamped topic if not already subscribed
        if not self.is_own_twist_subscribed:
            self.own_twist_subscriber = rospy.Subscriber('/cmd_vel', Twist, self.own_twist_callback)
            self.is_own_twist_subscribed = True
            
    def own_twist_callback(self, twist_msg):
        # Callback function to handle Twist messages
        self.linear_speed  = twist_msg.linear.x
        self.angular_speed = twist_msg.angular.z

    def subscribe_own_imu(self):
        # Subscribe to the TwistStamped topic if not already subscribed
        if not self.is_own_imu_subscribed:
            self.own_imu_subscriber = rospy.Subscriber('/imu', Imu, self.own_imu_callback)
            self.is_own_imu_subscribed = True
            
    def own_imu_callback(self, imu_msg):
        # Callback function to handle Twist messages
        self.imu_angular_velocity  = (imu_msg.angular_velocity.x,
                                      imu_msg.angular_velocity.y,
                                      imu_msg.angular_velocity.z)
                                 
    def publish_speeds(self):       
        # Create a Twist message to control the Turtlebot's linear and angular speeds
        self.msg.linear.x  = self.linear_speed   # Set linear speed
        self.msg.angular.z = self.angular_speed  # Set angular speed
        # Publish the Twist message to the '/cmd_vel' topic using ROS Publisher
        self.speed_publisher.publish(self.msg)

    def log_to_buffer(self):
        if self.start_time is None:
            self.start_time = rospy.get_time()

        current_time = rospy.get_time() - self.start_time
        if self.debug_flag is False:
            log_data = f"{current_time:.4f}\t{self.linear_speed:.4f}\t{self.angular_speed:.4f}\t" \
                       f"{self.position[0]:.4f}\t{self.position[1]:.4f}\t" \
                       f"{self.yaw:.4f}\t" \
                       f"{self.linear_velocity[0]:.4f}\t{self.linear_velocity[1]:.4f}\t" \
                       f"{math.sqrt(math.pow(self.linear_velocity[0], 2) + math.pow(self.linear_velocity[1], 2)):.4f}\t" \
                       f"{self.imu_angular_velocity[2]:.4f}\n"
        else:   
            log_data = f"{current_time:.4f}\t{self.linear_speed:.4f}\t{self.angular_speed:.4f}\t" \
                       f"{self.position[0]:.4f}\t{self.position[1]:.4f}\t{self.position[2]:.4f}\t" \
                       f"{self.orientation[0]:.4f}\t{self.orientation[1]:.4f}\t{self.orientation[2]:.4f}\t{self.orientation[3]:.4f}\t" \
                       f"{self.roll:.4f}\t{self.pitch:.4f}\t{self.yaw:.4f}\t" \
                       f"{self.linear_velocity[0]:.4f}\t{self.linear_velocity[1]:.4f}\t{self.linear_velocity[2]:.4f}\t" \
                       f"{self.angular_velocity[0]:.4f}\t{self.angular_velocity[1]:.4f}\t{self.angular_velocity[2]:.4f}\n"
        # Append log data to buffer for post-analysis
        self.log_data_buffer.append(log_data)

    def write_log_data_to_file(self, log_file_name=None):
        if log_file_name is None:
            log_file_name = self.log_file_name
        else:
            self.log_file_name = log_file_name
        # Write accumulated log data to a file at the end of the program
        with open(log_file_name, 'w') as file:
            if self.debug_flag is False:
                header = "T (s)\tLin\tAng\tX\tY\tyaw\tdx\tdy\tv\twz\n"
            else:
                header = "T (s)\tLin\tAng\tX\tY\tZ\tq0\tq1\tq2\tq3\troll\tpitch\tyaw\tdx\tdy\tdz\twx\twy\twz\n"
            file.write(header)
            file.writelines(self.log_data_buffer)

    def read_from_file(self, read_file_name=None):        
        if read_file_name is None:
            self.read_file_name = self.log_file_name
        else:
            self.read_file_name = read_file_name
            
        # Open the log file in read mode
        with open(self.read_file_name, 'r') as file:
            # Skip the first line (header) in the log file
            file.readline()
            # Read each line in the log file, split by tabs, and store them in log_lines list
            for line in file:
                self.log_lines.append(line.split('\t'))

    def update_speeds(self):
        start_time = time.time()

        if self.log_lines:
            # Start updating speeds from the first entry
            if self.current_idx < len(self.log_lines) - 1:
                # current_time = float(self.log_lines[self.current_idx][0])
                # next_time = float(self.log_lines[self.current_idx + 1][0])
                self.linear_speed = float(self.log_lines[self.current_idx][1])
                self.angular_speed = float(self.log_lines[self.current_idx][2])
                self.current_idx += 1
            else:
                self.linear_speed  = 0.0
                self.angular_speed = 0.0
                self.exit_requested = True
                
    def update_traj(self):
        if self.log_lines:
            # Start updating speeds from the first entry
#            current_time = float(self.log_lines[self.current_idx][0])
#            next_time = float(self.log_lines[self.current_idx + 1][0])            
            if self.current_idx < len(self.log_lines) - 1:
                if self.debug_flag is False:
                    self.linear_speed_traj  = float(self.log_lines[self.current_idx][1])
                    self.angular_speed_traj = float(self.log_lines[self.current_idx][2])

                    self.x_traj   = float(self.log_lines[self.current_idx][3])
                    self.y_traj   = float(self.log_lines[self.current_idx][4])
                    self.yaw_traj = float(self.log_lines[self.current_idx][5])

                    self.vx_traj  = float(self.log_lines[self.current_idx][6])
                    self.vy_traj  = float(self.log_lines[self.current_idx][7])
                    self.omg_traj = float(self.log_lines[self.current_idx][9])
                else:
                    self.linear_speed_traj  = float(self.log_lines[self.current_idx][1])
                    self.angular_speed_traj = float(self.log_lines[self.current_idx][2])

                    self.x_traj   = float(self.log_lines[self.current_idx][3])
                    self.y_traj   = float(self.log_lines[self.current_idx][4])
                    self.yaw_traj = float(self.log_lines[self.current_idx][12])

                    self.vx_traj  = float(self.log_lines[self.current_idx][13])
                    self.vy_traj  = float(self.log_lines[self.current_idx][14])
                    self.omg_traj = float(self.log_lines[self.current_idx][18])                
                self.current_idx += 1
                
            elif self.current_idx == len(self.log_lines) - 1:
                self.linear_speed_traj  = 0.0
                self.angular_speed_traj = 0.0

                self.x_traj   = float(self.log_lines[self.current_idx][3])
                self.y_traj   = float(self.log_lines[self.current_idx][4])
                self.yaw_traj = float(self.log_lines[self.current_idx][5])

                self.vx_traj  = 0.0
                self.vy_traj  = 0.0
                self.omg_traj = 0.0

    def subscribe_vicon(self):
        # Subscribe to the PoseStamped topic if not already subscribed
        if not self.is_pose_subscribed:
            self.pose_subscriber = rospy.Subscriber(
                '/vrpn_client_node/turtlebot_1/pose',
                PoseStamped,
                self.pose_callback
            )
            self.is_pose_subscribed = True

        # Subscribe to the TwistStamped topic if not already subscribed
        if not self.is_twist_subscribed:
            self.twist_subscriber = rospy.Subscriber(
                '/vrpn_client_node/turtlebot_1/twist',  # Replace with your TwistStamped topic
                TwistStamped,
                self.twist_callback
            )
            self.is_twist_subscribed = True

    def unsubscribe_vicon(self):
        # Unsubscribe from the PoseStamped topic if already subscribed
        if self.is_pose_subscribed:
            if self.pose_subscriber is not None:
                self.pose_subscriber.unregister()
                self.pose_subscriber = None
            self.is_pose_subscribed = False

        # Unsubscribe from the TwistStamped topic if already subscribed
        if self.is_twist_subscribed:
            if self.twist_subscriber is not None:
                self.twist_subscriber.unregister()
                self.twist_subscriber = None
            self.is_twist_subscribed = False

    def pose_callback(self, data):
        # Callback function to handle PoseStamped messages
        self.position = (data.pose.position.x,
                         data.pose.position.y, data.pose.position.z)
        self.orientation = (
            data.pose.orientation.x,
            data.pose.orientation.y,
            data.pose.orientation.z,
            data.pose.orientation.w
        )
        # Update other attributes as needed
        (self.roll, self.pitch, self.yaw) = euler_from_quaternion (self.orientation)
        

    def twist_callback(self, data):
        # Callback function to handle TwistStamped messages
        # Access linear and angular velocities from TwistStamped message
        self.linear_velocity = (data.twist.linear.x,
                                data.twist.linear.y, data.twist.linear.z)
        self.angular_velocity = (data.twist.angular.x,
                                 data.twist.angular.y, data.twist.angular.z)

        # Handle TwistStamped data here
        # Update other attributes or perform any necessary computations

        self.omega = 0.0  # Initializing yaw angle rate

    def controller(self):                          # CONTROL ALGORITHM - WORK HERE!!!!!!!
        
        # Sensor current position and orientation
        x_sensor = self.position[0]
        y_sensor = self.position[1]
        psi_sensor = self.yaw
        
        '''Controller'''            
        # WRITE CONTROLLER HERE

        # Desired trajectory attributes - TUNABLE
        x_d = self.x_traj
        y_d = self.y_traj
        
        # Proportional gain values - TUNABLE
        k_x_p = 1
        k_y_p = 1
        k_psi_p = 4

        # Error calculation
        x_error = x_d - x_sensor
        y_error = y_d - y_sensor
        
        #psi_error = math.atan2(math.sin(psi_error), math.cos(psi_error))

        # Velocity components
        vx = k_x_p*x_error
        vy = k_y_p * y_error
        psi_d = math.atan2(vy,vx)
        psi_error = psi_d - psi_sensor

        # Feedback linear velocity
        v_fb = math.hypot(vx, vy)
        # v_fb = vx for x controller test only
        # v_fb = vy for y controller test only
        # v_fb = 0  for yaw controler test only

        omg_fb = k_psi_p * psi_error # gain times control again

        self.linear_speed = v_fb
        self.angular_speed = omg_fb
        
        # Step S.0 and S.1: Saturation: Saturate self.linear_speed and self.angular_speed
        # If needed you can use the max(x,y) and min(x,y) to compute the min and max between two values.
        linear_speed_max = +self.linear_speed_max
        linear_speed_min = -self.linear_speed_max
        angular_speed_max = +self.angular_speed_max
        angular_speed_min = -self.angular_speed_max
        if self.current_idx >= len(self.log_lines):
            self.linear_speed = 0
            self.angular_speed = 0  
        
        self.linear_speed = max(-self.linear_speed_max, min(self.linear_speed, self.linear_speed_max))
        self.angular_speed = max(-self.angular_speed_max, min(self.angular_speed, self.angular_speed_max))       

    def cleanup(self):      
        # Shutdown ROS node
        rospy.signal_shutdown("Cleanup and exit")
        
        # Wait for rospy to finish shutting down
        while not rospy.is_shutdown():
            pass
        rospy.loginfo("KeyboardInterrupt received. Exiting gracefully...")
        # Exit Python script
        sys.exit(0)

    def record_traj(self, log_file_name=None):    
        # Subscribe to the topics
        self.subscribe_own_twist()
        self.subscribe_own_imu()
        self.subscribe_vicon()  
        try:
            rospy.loginfo("Running...")        
            self.rate = rospy.Rate(100) # 10hz
            while not rospy.is_shutdown() and not self.exit_requested:
                self.log_to_buffer()
                self.rate.sleep()
        except KeyboardInterrupt:
            pass
        finally:
            print("")
            self.exit_requested = True      # Set exit flag to True
            self.write_log_data_to_file(log_file_name)
            self.cleanup()
            
    def play_traj(self, read_file_name=None, log_file_name=None):      
        # Open the log file in read mode
        self.read_from_file(read_file_name) 
        # Subscribe to the topics
        self.subscribe_own_twist()
        self.subscribe_own_imu()
        self.subscribe_vicon()
        try:
            self.rate = rospy.Rate(100) # 10hz
            while not rospy.is_shutdown() and not self.exit_requested:
                # Update speeds from the first entry
                self.update_speeds()
                # Start publishing twist messages for robot movement
                self.publish_speeds()
                # Start logging speeds if the log flag is set
                self.log_to_buffer()
                self.rate.sleep()
        except KeyboardInterrupt:
            pass
        finally:
            print("")
            self.exit_requested = True      # Set exit flag to True
            self.write_log_data_to_file(log_file_name)
            self.cleanup()
                
    def control_traj(self, read_file_name=None, log_file_name=None):  
        # Open the log file in read mode
        self.read_from_file(read_file_name) 
        # Subscribe to the topics
        self.subscribe_own_twist()
        self.subscribe_own_imu()
        self.subscribe_vicon()
        try:
            self.rate = rospy.Rate(100) # 10hz
            while not rospy.is_shutdown() and not self.exit_requested:
                self.update_traj() # Update speeds from the first entry
                self.controller()
#                # Update speeds from the first entry
 #               self.update_speeds()
                # Start publishing twist messages for robot movement
                self.publish_speeds()
                # Start logging speeds if the log flag is set
                self.log_to_buffer()
                self.rate.sleep()
        except KeyboardInterrupt:
            pass
        finally:
            print("")
            self.exit_requested = True      # Set exit flag to True
            self.write_log_data_to_file(log_file_name)
            self.cleanup()
     
if __name__ == '__main__':
    obj = Turtlebot()   # Create an instance of the Turtlebot class

        # Uncomment only one line of code to execute:   
    
    # TASK1
    #obj.record_traj("log_GSET_Robot.txt")      # Execute the record_traj  method to record            the turtlebot manual trajectory
    
    # TASK2
    #obj.play_traj("log_GSET_Robot.txt", "log_plybck_GSET_Robot.txt")        # Execute the record_traj  method to play back         the turtlebot manual trajectory

    # TASK3 and TASK 4
    obj.control_traj("log_GSET_Robot.txt", "log_ctrl_GSET_Robot.txt")
    
    # TASK5
    #obj.control_traj("trajectoryRU_SOL.txt", "log_ctrl_RU_GSET_Robot.txt")               # Execute the control_traj method to feed back control the turtlebot        trajectory