#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import Point
from std_msgs.msg import String
from geometry_msgs.msg import Twist

global seconds
global oldSeconds
global control_linear_vel
global control_angular_vel
global target_linear_vel
global target_angular_vel
global eSum
global aSum
global totalTime
totalTime = 0
control_linear_vel = 0
control_angular_vel = 0
target_linear_vel = 0
target_angular_vel = 0
eSum = 0
aSum = 0

def processObject(objectData):
  global target_linear_vel
  global target_angular_vel
  global eSum
  global aSum
  global seconds
  global oldSeconds
  global totalTime
  
  linError = objectData.x - 0.33
  if (objectData.y > 1):
    angError = objectData.y - 6.28318548203
  else:
    angError = objectData.y
  

  eSum += linError * .2
  aSum += angError * .2
  if (eSum > 100):
    eSum = 100
  if (aSum > 100):
    aSum = 100
    
  lin_xDot = (0.2 * linError) + (0.02 * eSum)
  ang_xDot = (0.2 * angError) + (0.02 * aSum)
  
  target_linear_vel += lin_xDot
  target_angular_vel += ang_xDot
  
  if (linError < .07 and linError > -.07):
    target_linear_vel = 0
    eSum = 0
    
  if (angError < .05 and angError > -.05):
    target_angular_vel = 0
    aSum = 0
  
  
#  if target_linear_vel > control_linear_vel:
#    control_linear_vel = min( target_linear_vel, control_linear_vel + (0.01/4.0) )
#  else:
#    control_linear_vel = target_linear_vel

#  if target_angular_vel > control_angular_vel:
#    control_angular_vel = min( target_angular_vel, control_angular_vel + (0.1/4.0) )
#  else:
#    control_angular_vel = target_angular_vel
    
  twist = Twist()
  twist.linear.x = target_linear_vel; twist.linear.y = 0; twist.linear.z = 0
  twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = target_angular_vel
  pub.publish(twist)

def Init():
  
  rospy.init_node('chase_object', anonymous = True)
  
  rospy.Subscriber('objectPoint', Point, processObject)
  
  global seconds
  global oldSeconds
  seconds = rospy.get_time()
  oldSeconds = rospy.get_time()
  
  global pub
  pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 5)

if __name__ == '__main__':
    try:
        Init()
    except rospy.ROSInterruptException:
        pass


rate = rospy.Rate(10)

while not rospy.is_shutdown():

  rospy.spin()
