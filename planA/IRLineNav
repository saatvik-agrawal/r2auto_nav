import RPi.GPIO as IO
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import time

L=5
R=6
cansensor = 4
IO.setup(L,IO.IN) #GPIO L for Left IR
IO.setup(R,IO.IN) #GPIO R for Right IR
IO.setuo(cansensor, IO.IN)

rotatechange = 0.1
speedchange = 0.05

class Docker(Node):
  
  def __init__(self):
    super().__init__('docking')
    self.mover = self.create_publisher(geometry_msgs.msg.Twist, 'cmd_vel',5)
    self.canstate = self.create_publisher(Bool, 'can', 5)
    self.onoroff = self.create_subscription(Bool,'docking?', self.followersub, 5)
  
  def followersub(self, msg):
    self.following = msg.data

  def canstatus(self):
    msg = Bool()
    if IO.input(cansensor) == 1:
      msg.data = True
    else:
       msg.data = False
    self.canstate.publish(msg)

  def IR_follow(self)
    twist = Twist()
    follow = True
    while (follow = True):
      if(IO.input(L)==1 and IO.input(R)==1): #Front
        twist.linear.x = speedchange
        twist.angular.z = 0.0
        self.mover.publish(twist)
        if (extreme == True and IO.input(L)!=1 and IO.input(R)!=1): #extreme case where bot is perpendicular, run once
          twist.linear.x = 0.0
          twist.angular.z = 0.0
          self.mover.publish(twist)
          while (IO.input(L)!=1 and IO.input(R)!=1):
            twist.linear.x = 0.05
            time.sleep(0.1)
            twist.linear.x = 0.0
            twist.angular.z = 0.1
            self.mover.publish(twist)
            time.sleep(0.4)
            twist.angular.z = 0.0
            self.mover.publish(twist)
          extreme = False
      elif (IO.input(L)==1 and IO.input(R)!=1): #Right (Clockwise)
        twist.linear.x = 0.0
        twist.angular.z = -rotatechange
        self.mover.publish(twist)
        extreme = False
      elif (IO.input(L)!=1 and IO.input(R)==1): #Left (Counter-Clockwise)
        twist.linear.x = 0.0
        twist.angular.z = rotatechange
        self.mover.publish(twist)
        extreme = False
      else: #Dont move
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.mover.publish(twist)
        follow = False
  
  def docking(self):
    try:
      sleep(1)
      rclpy.spin_once(self)
      while rclpy.ok():
        if self.following == True:
           self.IR_follow()
        self.canstate.publish()
        for i in range(500):
            rclpy.spin_once(self)

    except Exception as e:
      print(e)
        # Ctrl-c detected
    finally:
      self.stopbot()

  def main(args = None):
    rclpy.init(args = args)
    docker = Docker()
    docker.docking()

    auto_nav.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

                
  
                    
