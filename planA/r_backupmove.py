import math
import rospy
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
import geometry_msgs.msg
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_msgs.msg import Int16
from std_msgs.msg import Bool
import numpy as np
import math
from math import atan2
import pickle
import cmath
from nav_msgs.msg import Odometry
import paho.mqtt.client as mqtt
import time
MQTT_BROKER = '192.168.1.102'
# import tf2_ros
# from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
# import RPI.GPIO as GPIO
import paho.mqtt.client as mqtt
with open("waypoints_sim.pickle","rb") as handle:
    waypoints = pickle.load(handle)


mapfile = 'map.txt'
speedchange = 0.05
angle_error = 2
# print("in in in ")
count = 0
rotatechange = 0.1
table = 0
can = bool

# quad_1 = range(0, 0.5 * pi)
# quad_2 = range (0.5 * pi, pi)
# quad_3 = range(pi, -0.5 * pi)
# quad_4 = range(-0.5 * pi, -pi)

# waypoints = {1:[-2.5769601779175844, -1.23103603909631],2:[-2.1849758576303553, -0.5775387034088549]}

def euler_from_quaternion(x, y, z, w):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return  yaw_z # in radians

def move_forward(distance):
        rospy.init_node('move_forward_node', anonymous=True)
        velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        vel_msg = Twist()
        # Set the linear velocity to move the Turtlebot forward
        vel_msg.linear.x = 0.1  # Adjust this value as needed to achieve the desired speed
        # Calculate the time required to move the Turtlebot forward by 69 inches
        speed = vel_msg.linear.x  # inches/second
        time = distance / speed  # seconds
        # Move the Turtlebot forward for the calculated time
        start_time = rospy.Time.now().to_sec()
        while (rospy.Time.now().to_sec() - start_time) < time:
            velocity_publisher.publish(vel_msg)
        # Stop the Turtlebot
        vel_msg.linear.x = 0.0
        velocity_publisher.publish(vel_msg)
        rospy.spin()

def turn_180():
    rospy.init_node('turn_180_degrees_node', anonymous=True)
    a_velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    a_vel_msg = Twist()
    a_vel_msg.angular.z = 0.5  
    angle = 180.0
    a_speed = a_vel_msg.angular.z  
    time = (angle * 3.14159265359 / 180.0) / a_speed
    start_time = rospy.Time.now().to_sec()
    while (rospy.Time.now().to_sec() - start_time) < time:
        a_velocity_publisher.publish(a_vel_msg)
    a_vel_msg.angular.z = 0.0
    a_velocity_publisher.publish(a_vel_msg)
    rospy.spin()

def turn_left():
    rospy.init_node('turn_left_node', anonymous=True)
    a_velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    a_vel_msg = Twist()
    a_vel_msg.angular.z = 0.5  
    angle = 90.0
    a_speed = a_vel_msg.angular.z  
    time = (angle * 3.14159265359 / 90.0) / a_speed
    start_time = rospy.Time.now().to_sec()
    while (rospy.Time.now().to_sec() - start_time) < time:
        a_velocity_publisher.publish(a_vel_msg)
    a_vel_msg.angular.z = 0.0
    a_velocity_publisher.publish(a_vel_msg)
    rospy.spin()

def turn_right():
    rospy.init_node('turn_left_node', anonymous=True)
    a_velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    a_vel_msg = Twist()
    a_vel_msg.angular.z = -0.5  
    angle = 90.0
    a_speed = a_vel_msg.angular.z  
    time = (angle * 3.14159265359 / 90.0) / a_speed
    start_time = rospy.Time.now().to_sec()
    while (rospy.Time.now().to_sec() - start_time) < time:
        a_velocity_publisher.publish(a_vel_msg)
    a_vel_msg.angular.z = 0.0
    a_velocity_publisher.publish(a_vel_msg)
    rospy.spin()

class Auto_Mover(Node):
    docked = ''
    dir = 0.0
    rot_q = 0.0
    orien = 5.0
    count = 0
    front = 5.0
    yaw = 0.0
    irdata = [-1,-1]
    # range = np.array([])

    def __init__(self) -> None:
        self.x = -1
        self.y = -1
        super().__init__('auto_mover')
        self.publisher_ = self.create_publisher(geometry_msgs.msg.Twist, 'cmd_vel',10)
        self.dock_publisher = self.create_publisher(Bool,'docking_status',5)
        # self.user_subscription = self.create_subscription(String,
                                                        #   'user',self.user_sub,10)
        # self.odom_subsription = self.create_subscription(Odometry,
        #     'odom',
        #     self.odom_callback,
        #     10)
        self.sim_can_subscription = self.create_subscription(Bool,'can',self.can_callback,10)
        # self.sim_dock_subscription = self.create_subscription(String,'dock',self.can_sub,10)
        # self.occ_subscription = self.create_subscription(
        #     OccupancyGrid,
        #     'map',
        #     self.odom_callback,
        #     qos_profile_sensor_data)
        # self.occ_subscription  # prevent unused variable warning
        # self.occdata = np.array([])

        self.map2base_subscription = self.create_subscription(Pose,'/map2base',self.odom_callback,10)
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            qos_profile_sensor_data)  
        self.IRLeft_subscriber = self.create_subscription(Int16,'IRLeft',self.ir_callbackL,3)
        self.IRRight_subscriber = self.create_subscription(Int16,'IRRight',self.ir_callbackR,3)
    def ir_callbackL(self, msg):
        self.irdata[0] = msg.data
    def ir_callbackR (self,msg):
        self.irdata[1] = msg.data
    def can_callback(self,msg):
        self.can = msg.data
    def odom_callback(self, msg):
        # # print("callback")
        #For map2base

        self.x = msg.position.x
        self.y = msg.position.y
        self.rot_q = msg.orientation
        # print("In callback")
        # print("self.orien",self.orien)
        #for odom

        # self.rot_q = msg.pose.pose.orientation
        # self.x = msg.pose.pose.position.x
        # self.y = msg.pose.pose.position.y

        self.orien = euler_from_quaternion(self.rot_q.x,self.rot_q.y,self.rot_q.z,self.rot_q.w)
        
        # print(self.rot_q)
        
    def scan_callback(self, msg):
        # create numpy array
        # print("in scan callback")
        laser_range = np.array(msg.ranges)
        # print("break here:1")
        positive_range = laser_range[-30:-1]
        p = laser_range[-10:-1]
        n = laser_range[0:10]
        check_range = np.append(n,p)

        # print(laser_range[0])
        # taken_range = np.add(taken_range, laser_range[0:16])
        other_range = (laser_range[0:30])
        taken_range = np.append(other_range , positive_range)
        taken_range[taken_range==0] = np.nan
        # print(laser_range)
        # taken_range = list(filter(lambda x: x == 0.0,taken_range))
        # find index with minimum value
        # self.front = laser_range[0]
        # print("break here:2")
        if np.isnan(check_range).all() == True:
            self.front = 3.0
            # print("all NAN")
        else:
            lr2i = np.nanargmin(check_range)
            self.front = check_range[lr2i]
            # print("updated dist", self.front)
        if taken_range.size != 0:
            # print(taken_range)
            # use nanargmax as there are nan's in laser_range added to replace 0's
            self.dir = np.nanargmin(taken_range)
            # self.get_logger().info('Picked direction: %d %f m' % (self.dir,taken_range[self.dir]))
            return
        else:
            self.dir = 0
            self.get_logger().info('No data!')
            # print("laser range",taken_range)
        
        
        # self.angle_go = math.radians(lr2i)
        # log the info
        # self.get_logger().info('Shortest distance at %i degrees' % lr2i)    
    def stopbot(self):
        twist = geometry_msgs.msg.Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.publisher_.publish(twist)

    def pick_direction(self):
        try:
            rclpy.spin_once(self)
            
            twist = geometry_msgs.msg.Twist()
            # self.get_logger().info('In pick_direction')
            print("self.dir",self.dir)
            angle = self.dir
            degree_to_turn = angle - math.degrees(self.orien ) 
            print("degree to turn",(degree_to_turn))
            print("dtt ", angle)
            print("orien",math.degrees(self.orien))
            print("front",self.front)
            while self.front > 0.25:
                print("in loop")
                # self.rotatebot(degree_to_turn)
                rclpy.spin_once(self)
                if abs(degree_to_turn) > angle_error:
                    degree_to_turn = angle - math.degrees(self.orien ) 
                    print("degree to turn",(degree_to_turn))
                    print("dtt ", angle)
                    print("orien",math.degrees(self.orien))
                    # print(self.dir)
                    twist.linear.x = 0.0
                    twist.angular.z = 0.5
                
               
                else:      
                    print("heading to table 6")
                    print(self.front)
                    twist.linear.x = 0.1
                    twist.angular.z = 0.0                 
                                    
                self.publisher_.publish(twist)
        finally:
            twist.linear.x = 0.0
            twist.angular.z = 0.0                 
                                    
            self.publisher_.publish(twist)
    

    def moveTable1():
        move_forward(69)
        turn_180()

    def returnFrom1():
        move_forward(69)
        turn_180()

    def moveTable2():
        move_forward(69)
        turn_right()
        move_forward(30)
        turn_right()

    def returnFrom2():
        turn_right()
        move_forward(30)
        turn_left()
        move_forward(69)
        turn_180()

    def moveTable3():
        move_forward(15)
        turn_right()
        move_forward(30)
        turn_right()

    def returnFrom3():
        turn_right
        move_forward(30)
        turn_left()
        move_forward(30)
        turn_180()
    
    def moveTable4():
        move_forward(15)
        turn_right()
        move_forward(60)
        turn_left()
    
    def returnFrom4():
        turn_right
        move_forward(60)
        turn_left()
        move_forward(15)
        turn_180

    def moveTable5():
        turn_right
        move_forward(90)
        turn_left
        move_forward(22)
        turn_180()

    def returnFrom5():
        move_forward(22)
        turn_right
        move_forward(90)
        turn_right()
    
    def moveTable6(self):
        move_forward(69)
        turn_right()
        move_forward(60)
        turn_left
        move_forward(63)
        turn_left()
        self.pick_direction()

    def returnFrom6():
        turn_180()
        #move until wall

        turn_right()
        move_forward(63)
        turn_right()
        move_forward(60)
        turn_left()
        move_forward(69)
        turn_180()


def main(args = None):

    try:
        rclpy.init(args = args)
        auto_move = Auto_Mover()
        
        # auto_move.rotatebot(90) 
        
            # to get ip address of the laptop
        #connect to esp32
        while True:
            client = mqtt.Client("Turtlebot")
            client.on_connect = on_connect
            client.on_message = table_num
            client.username_pw_set("roger", "password")
            client.connect(MQTT_BROKER, 1883)
            client.loop_start()
            client.subscribe("TableNo")
            rclpy.spin_once(auto_move)
            
            if can == True:

                auto_move.path()
            else:
                time.sleep(10)
                auto_move.path()

    except KeyboardInterrupt:
        auto_move.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()