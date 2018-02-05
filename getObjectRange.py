#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import Point
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan

def processLidar(scanData):
  closestObject = [0, 5]
  for i in range (0, 360):
    num = 0
    distance = 0
    for j in range (-2, 3):
      newDist = scanData.ranges[(i + j) % 360]
      if (newDist < 3.5 and newDist > .12):
        distance += newDist
        num += 1
    if (num == 0):
      continue
    distance = distance / num
    if (distance < closestObject[1]):
      closestObject[1] = distance
      closestObject[0] = i
           
  pub.publish(closestObject[0], closestObject[1], 0)
  print(closestObject[0], closestObject[1], 0)
  

def Init():

  rospy.Subscriber("/scan", LaserScan, processLidar)
  
  rospy.init_node('get_range', anonymous = True)
  
  global pub
  pub = rospy.Publisher('objectPoint', Point, queue_size = 5)
  
if __name__ == '__main__':
    try:
        Init()
    except rospy.ROSInterruptException:
        pass

#rate = rospy.Rate(10)

while not rospy.is_shutdown():

  rospy.spin()
