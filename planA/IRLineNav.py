import RPi.GPIO as GPIO
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Int16
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
    self.canstate = self.create_publisher(Bool, 'can', 10)
    self.leftPub = self.create_publisher(Int16, IRLeft, 3)
    self.rightPub = self.create_publisher(Int16, IRRight, 3)
  
  def canstatus(self):
    msg = Bool()
    if GPIO.input(cansensor) == 1:
      msg.data = True
    else:
      msg.data = False
    self.canstate.publish(msg)

  def IRPublish(self):
    leftmsg = Int16()
    rightmsg = Int16()
    leftmsg = GPIO.input(L)
    rightmsg = GPIO.input(R)
    self.leftPub.publish(leftmsg)
    self.rightPub.publish(rightmsg)
    

  def publisher(self):
    try:
      print("running")
      while True:
        print("publishing")
        self.canstatus()
        self.IRPublish()

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
  docker.publisher()

  docker.destroy_node()
  rclpy.shutdown()


if __name__ == '__main__':
    main()
