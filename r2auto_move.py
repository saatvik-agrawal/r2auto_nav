import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
# from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose, Twist
from std_msgs.msg import String, Bool
import numpy as np
import pickle
import cmath
# import RPI.GPIO as GPIO
import paho.mqtt.client as mqtt
with open("waypoints.pickle","rb") as handle: #using pickle as waypoint wrapper
    waypoints = pickle.load(handle)

print(waypoints)
# mapfile = 'map.txt'
rotatechange = 0.3
speedchange = 0.1
angle_error = 2
route = {1:[1, 0],2:[2, 0],3:[2, 0],4:[3,0],5:[4,0],6:[5, 6, 10,  0]}
# print("in in in ")
count = 0

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
    orientation = math.atan2(t3, t4)

    return orientation # in radians

class Auto_Mover(Node):
    def __init__(self):
        self.table = 0
        #self.rot_q = 0.0
        self.orien = 0.0
        #count = 0
        self.front = 0.0
        self.yaw = 0.0
        self.x = 0.0
        self.y = 0.0
        self.mapbase = None
        super().__init__('auto_mover')
        self.publisher_ = self.create_publisher(geometry_msgs.msg.Twist, 'cmd_vel',5)
        self.userinput = self.create_subscription(String,'TableNo', self.user_sub, 5)
        self.map2base_sub = self.create_subscription(Pose, 'map2base', self.map2base_callback, 5)
        self.lidarsub = self.create_subscription(LaserScan, 'scan', self.scan_callback, qos_profile_sensor_data)  
        self.cansub = self.create_subscription(Bool, 'can', self.can_callback, 5)

    def map2base_callback(self, msg):
        # self.get_logger().info('In map2basecallback')
        self.mapbase = msg.position
        self.roll, self.pitch, self.yaw = euler_from_quaternion(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)    
        
    def user_sub(self, msg):
        self.table = int(msg.data)
        # print("in subcriber user")    

    def scan_callback(self, msg):
        # create numpy array
        laser_range = np.array(msg.ranges)
        positive_range = laser_range[-20:-1]
        # print(laser_range[0])
        # taken_range = np.add(taken_range, laser_range[0:16])
        other_range = (laser_range[0:20])
        taken_range = np.append(other_range , positive_range)
        laser_range[laser_range==0] = np.nan
        # find index with minimum value
        # self.front = laser_range[0]
        lr2i = np.nanargmin(taken_range)
        self.front = taken_range[lr2i]
 
    def can_callback(self, msg):
        self.can = msg.state
    
    def delta_angle(self, point):
        self.goal_x = waypoints[point][0][0]
        self.goal_y = waypoints[point][0][1]
        theta = (math.atan2(self.goal_y-self.mapbase.y,self.goal_x-self.mapbase.x))
        self.delta = math.degrees(theta - self.yaw)
        return self.delta
    
    def rotatebot(self, rot_angle):
        self.get_logger().info('Rotating')
        sign = np.sign(rot_angle)
        prev = 0
        if prev != rotatechange * sign:
            twist.angular.z = rotatechange * sign
            twist.linear.x = 0.0
            self.publisher_.publish(twist)
            prev = rotatechange * sign
        while abs(int(rot_angle*100)) >= 5:
            rclpy.spin_once(self)
            
        self.stopbot()
    
    def robotforward(self):
        # start moving
        self.get_logger().info('Start moving')
        twist = Twist()
        twist.linear.x = speedchange
        twist.angular.z = 0.0
        # not sure if this is really necessary, but things seem to work more
        # reliably with this
        self.publisher_.publish(twist)
        twist.linear.x = 0.0
        time.sleep(1.5) #need to adjust based on speed
        self.publisher_.publish(twist)
    
    def stopbot(self):
        self.get_logger().info('In stopbot')
        twist = Twist()
        # publish to cmd_vel to move TurtleBot
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        # time.sleep(1)
        self.publisher_.publish(twist)
    
    def move_to_point(self, point):
        print("tavelling to waypoint")
        # points_char = int(input("enter waypoint to travel: "))
        twist = geometry_msgs.msg.Twist()
        # print("qewagdsfnc")
        
        # rclpy.init_node("speed_controller")
        # r = rclpy.Rate(4)
        try:
            while self.front > 2:
                self.rotatebot(self.delta_angle(point))
                self.robotforward()
            self.stopbot()
        finally:
            # stop moving   
            self.stopbot()
    
    def sequential(self,destination):
        for point in destination:
            if point == 0 or point == 5:
                self.move_to_point(point)
            else:
                self.move_to_point(point)
                if tableno == 3:
                    #face behind
                elif tableno == 6:
                    #special case
                else:
                    #face forward



                   
            if tableno == 6:

                self.move_to_point(point)
                                   
def main(args = None):

    try:
        rclpy.init(args = args)
        auto_move = Auto_Mover()
        while True:
            rclpy.spin_once(auto_move)
            tableno = self.table
            if tableno != -1:
                sequential(route[tableno])
                self.
         #print (table_num)
        #  if(table_num != -1):
        #      # send message to esp32 to tell it that the robot has un-docked and is moving to the table
        #      client.publish("esp32/input", "0")
        #      #navigation.moveToTable(table_num)
        #      navigation.dock()
        #      table_num = -1
         
        pass
    except KeyboardInterrupt:
        auto_move.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
