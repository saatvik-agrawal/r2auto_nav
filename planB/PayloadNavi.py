import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Int8, Bool, String
import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
import cv2
import scipy.stats
import numpy as np
import math
import cmath
import time

# constants
rotatechange = 0.1
speedchange = 0.1
occ_bins = [-1, 0, 70, 100]  # Setting threshold between occupied and unoccupied cell
stop_distance = 0.25
scanfile = 'lidar.txt'
mapfile = 'map.txt'
