import RPi.GPIO as GPIO
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import time

L=3
R=2
cansensor = 4
GPIO.setmode(GPIO.BCM)
GPIO.setup(L,GPIO.IN) #GPIO L for Left IR
GPIO.setup(R,GPIO.IN) #GPIO R for Right IR
GPIO.setup(cansensor, GPIO.IN)

rotatechange = 0.1
speedchange = -0.02

class Docker(Node):
  def __init__(self):
    super().__init__('docking')
    self.mover = self.create_publisher(Twist,'cmd_vel',10)
    #self.canstate = self.create_publisher(Bool, 'can', 10)
    self.following = True
    self.onoroff = self.create_subscription(Bool,'docking_status', self.followersub, 5)
  
  def followersub(self, msg):
    self.following = True #msg.data

  def canstatus(self):
    msg = Bool()
    if GPIO.input(cansensor) == 1:
      msg.data = True
    else:
      msg.data = False
    self.canstate.publish(msg)

  def stopbot(self):
    twist = Twist()
    twist.linear.x = 0.0
    twist.angular.z = 0.0
    self.mover.publish(twist)

  def IR_follow(self):
    twist = Twist()
    print("in IR_follow")
    follow = True
    extreme = True
    while (follow == True):
      print(GPIO.input(L))
      print(GPIO.input(R))
      if(GPIO.input(L)==0 and GPIO.input(R)==0): #Front
        twist.linear.x = speedchange
        twist.angular.z = 0.0
        print(twist.linear.x)
        time.sleep(1)
        self.mover.publish(twist)
        print("publishing")
        if (extreme == True and GPIO.input(L)!=0 and GPIO.input(R)!=0): #extreme case where bot is perpendicular, run once
          twist.linear.x = 0.0
          twist.angular.z = 0.0
          time.sleep(1)
          self.mover.publish(twist)
          while (GPIO.input(L)!=0 and GPIO.input(R)!=0):
            twist.linear.x = -0.02
            time.sleep(0.1)
            self.mover.publish(twist)
            twist.linear.x = 0.0
            twist.angular.z = 0.1
            time.sleep(0.1)
            self.mover.publish(twist)
            time.sleep(1)
            twist.angular.z = 0.0
            time.sleep(0.1)
            self.mover.publish(twist)
          extreme = False
      elif (GPIO.input(L)==0 and GPIO.input(R)!=0): #Right (Clockwise)
        twist.linear.x = 0.0
        twist.angular.z = -rotatechange
        self.mover.publish(twist)
        extreme = False
      elif (GPIO.input(L)!=0 and GPIO.input(R)==0): #Left (Counter-Clockwise)
        twist.linear.x = 0.0
        twist.angular.z = rotatechange
        time.sleep(0.1)
        self.mover.publish(twist)
        extreme = False
      else: #Dont move
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.mover.publish(twist)
        follow = False

  def docking(self):
    try:
      print("running")
      while True:
        print("following")
        self.IR_follow() #to be removed
        #rclpy.spin_once(self)
        #print("spin done")
        #if self.following == True:
        #  print("IR Following/Docking")
        #  self.IR_follow()
        #self.canstate.publish()
        #time.sleep(1)

    except Exception as e:
      print(e)
        # Ctrl-c detected
    finally:
      self.stopbot()

def main(args = None):
  rclpy.init(args = args)
  print("program start")
  docker = Docker()
  print("docker assigned")
  docker.docking()

  docker.destroy_node()
  rclpy.shutdown()


if __name__ == '__main__':
    main()
