#!/usr/bin/env python

import rospy
import numpy as np
import math
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

def processOdom(odomData):
  global currentX
  global currentY
  global yaw

  currentX = odomData.pose.pose.position.x
  currentY = odomData.pose.pose.position.y
  w = odomData.pose.pose.orientation.w
  x = odomData.pose.pose.orientation.x
  y = odomData.pose.pose.orientation.y
  z = odomData.pose.pose.orientation.z
  
  t3 = +2.0 * (w * z + x * y)
  t4 = +1.0 - 2.0 * (y * y + z * z)
  yaw = math.degrees(math.atan2(t3, t4))
  
def processObject(objectPoint):
  global currentX
  global currentY
  global yaw
  global objectDist
  global objectAngle
  global objectX
  global objectY
  
  localAngle = objectPoint.x
  objectDist = objectPoint.y
  objectAngle = (yaw + localAngle) % 360
  objectX = currentX + objectDist * math.cos(yaw + objectAngle)
  objectY = currentY + objectDist * math.sin(yaw + objectAngle)
  #print(objectX, objectY, objectAngle, yaw)
  
  

def Init():
  
  rospy.init_node('go_to_goal', anonymous = True)
  
  rospy.Subscriber('/odom', Odometry, processOdom)
  rospy.Subscriber('objectPoint', Point, processObject)
  
  global pub
  pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 5)

if __name__ == '__main__':
  global currentX
  global currentY
  global yaw
  global objectDist
  global objectAngle
  global objectX
  global objectY
  currentX = 0
  currentY = 0
  yaw = 0
  objectDist = 5
  objectAngle = 0
  objectX = 0
  objectY = 0
  
  eSum = 0
  aSum = 0
  linGoalControl = 0
  angGoalControl = 0
  eSumAvoid = 0
  aSumAvoid = 0
  linAvoidControl = 0
  angAvoidControl = 0
  eSumFWC = 0
  aSumFWC = 0
  linFWCControl = 0
  angFWCControl = 0
  eSumFWCC = 0
  aSumFWCC = 0
  linFWCCControl = 0
  angFWCCControl = 0
  tau = 0
  xTau = 0
  controlState = 1


  try:
    Init()
  except rospy.ROSInterruptException:
    pass


  rate = rospy.Rate(10)
  points = []
  wayPoints = open('wayPoints.txt', 'r')
  for line in wayPoints:
    line_words = line.split()
    points.append([float(line_words[0]), float(line_words[1])])
  wayPoints.close()

  objective = 0
  
  while not rospy.is_shutdown():
  
    gotPoint = False
    
    if (objective == len(points)):
      twist = Twist()
      twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
      twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
      pub.publish(twist)
      print("Finished!")
      rospy.spin()
    
    goal = points[objective]
    goalX = goal[0]
    goalY = goal[1]
    
    linError = (((goalX - currentX) ** 2) + ((goalY - currentY) ** 2)) ** 0.5
    #print ("Goal: (",goalX,goalY,") Current: (",currentX, currentY,") Error:",linError)
    theta = yaw
    theta = (theta * 3.14159265359) / 180
    thetaD = math.atan2((goalY - currentY), (goalX - currentX))
    angError = math.atan2(math.sin(thetaD - theta), math.cos(thetaD - theta))
    #print ("Theta:",theta,"ThetaD:",thetaD,"Error:",angError)
    
    ##Go to goal controller##
    eSum += linError * .02
    aSum += angError * .02
    if (eSum > 100):
      eSum = 100
    if (aSum > 100):
      aSum = 100
      
    lin_xDot = (0.2 * linError) + (0.02 * eSum)
    ang_xDot = (0.2 * angError) + (0.02 * aSum)
    
    if(linGoalControl < 1 and linGoalControl > -1):
      linGoalControl += lin_xDot
    if(angGoalControl < 1 and angGoalControl > -1):
      angGoalControl += ang_xDot
    
    if (linError < .01 and linError > -.01):
      linGoalControl = 0
      eSum = 0
      if (gotPoint == False):
        objective += 1
        print("Got it!")
        gotPoint = True
        twist = Twist()
        twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        pub.publish(twist)
        rospy.sleep(2.)
      
    if (angError < .2 and angError > -.2):
      angGoalControl = 0
      aSum = 0
    else:
      linGoalControl = 0
      
      
    ##Avoid obstacle controller##
    linErrorAvoid = (((goalX - currentX) ** 2) + ((goalY - currentY) ** 2)) ** 0.5
    avoidTheta = (objectAngle + 180) % 360
    avoidTheta = (avoidTheta * 3.14159265359) / 180
    angErrorAvoid = math.atan2(math.sin(avoidTheta - theta), math.cos(avoidTheta - theta))
    
    eSumAvoid += linErrorAvoid * .02
    aSumAvoid += angErrorAvoid * .02
    if (eSumAvoid > 100):
      eSumAvoid = 100
    if (aSumAvoid > 100):
      aSumAvoid = 100
      
    lin_xDotAvoid = (0.2 * linErrorAvoid) + (0.02 * eSumAvoid)
    ang_xDotAvoid = (0.2 * angErrorAvoid) + (0.02 * aSumAvoid)
    
    if(linAvoidControl < 1 and linAvoidControl > -1):
      linAvoidControl += lin_xDotAvoid
    if(angAvoidControl < 1 and angAvoidControl > -1):
      angAvoidControl += ang_xDotAvoid
    
    if (linErrorAvoid < .01 and linErrorAvoid > -.01):
      linAvoidControl = 0
      eSumAvoid = 0
      if (gotPoint == False):
        objective += 1
        print("Got it!")
        gotPoint = True
        twist = Twist()
        twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        pub.publish(twist)
        rospy.sleep(2.)
      
    if (angErrorAvoid < .2 and angErrorAvoid > -.2):
      angAvoidControl = 0
      aSumAvoid = 0
    else:
      linAvoidControl = 0
      
    ##Follow Wall Clockwise Controller##
      
    linErrorFWC = (((goalX - currentX) ** 2) + ((goalY - currentY) ** 2)) ** 0.5
    FWCTheta = (objectAngle + 90) % 360
    FWCTheta = (FWCTheta * 3.14159265359) / 180
    angErrorFWC = math.atan2(math.sin(FWCTheta - theta), math.cos(FWCTheta - theta))
    
    eSumFWC += linErrorFWC * .02
    aSumFWC += angErrorFWC * .02
    if (eSumFWC > 100):
      eSumFWC = 100
    if (aSumFWC > 100):
      aSumFWC = 100
      
    lin_xDotFWC = (0.2 * linErrorFWC) + (0.02 * eSumFWC)
    ang_xDotFWC = (0.2 * angErrorFWC) + (0.02 * aSumFWC)
    
    if(linFWCControl < 1 and linFWCControl > -1):
      linFWCControl += lin_xDotFWC
    if(angFWCControl < 1 and angFWCControl > -1):
      angFWCControl += ang_xDotFWC
    
    if (linErrorFWC < .01 and linErrorFWC > -.01):
      linFWCControl = 0
      eSumFWC = 0
      if (gotPoint == False):
        objective += 1
        print("Got it!")
        gotPoint = True
        twist = Twist()
        twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        pub.publish(twist)
        rospy.sleep(2.)
      
    if (angErrorFWC < .2 and angErrorFWC > -.2):
      angFWCControl = 0
      aSumFWC = 0
    else:
      linFWCControl = 0
      
    ##Follow Wall CounterClockwise Controller##
    
    linErrorFWCC = (((goalX - currentX) ** 2) + ((goalY - currentY) ** 2)) ** 0.5
    FWCCTheta = (objectAngle + 270) % 360
    FWCCTheta = (FWCCTheta * 3.14159265359) / 180
    angErrorFWCC = math.atan2(math.sin(FWCCTheta - theta), math.cos(FWCCTheta - theta))
    
    eSumFWCC += linErrorFWCC * .02
    aSumFWCC += angErrorFWCC * .02
    if (eSumFWCC > 100):
      eSumFWCC = 100
    if (aSumFWCC > 100):
      aSumFWCC = 100
      
    lin_xDotFWCC = (0.2 * linErrorFWCC) + (0.02 * eSumFWCC)
    ang_xDotFWCC = (0.2 * angErrorFWCC) + (0.02 * aSumFWCC)
    
    if(linFWCCControl < 1 and linFWCCControl > -1):
      linFWCCControl += lin_xDotFWCC
    if(angFWCCControl < 1 and angFWCCControl > -1):
      angFWCCControl += ang_xDotFWCC
    
    if (linErrorFWCC < .01 and linErrorFWCC > -.01):
      linFWCCControl = 0
      eSumFWCC = 0
      if (gotPoint == False):
        objective += 1
        print("Got it!")
        gotPoint = True
        twist = Twist()
        twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        pub.publish(twist)
        rospy.sleep(2.)
      
    if (angErrorFWCC < .2 and angErrorFWCC > -.2):
      angFWCCControl = 0
      aSumFWCC = 0
    else:
      linFWCCControl = 0
    
    ##Determine which controller to use
    
    if (controlState == 1):
      if (objectDist <= .2 + .1 and math.cos(abs(thetaD - FWCTheta)) > 0):
        controlState = 2
        tau = rospy.get_time()
        xTau = linError
      elif (objectDist <= .2 + .1 and math.cos(abs(thetaD - FWCCTheta)) > 0):
        controlState = 3
        tau = rospy.get_time()
        xTau = linError
    elif (controlState == 2):
      if (math.cos(abs(thetaD - avoidTheta)) > 0 and linError < xTau and objectDist > .2 + .1):
        controlState = 1
      elif (objectDist < .2 - .1):
        controlState = 4
    elif (controlState == 3):
      if (math.cos(abs(thetaD - avoidTheta)) > 0 and linError < xTau and objectDist > .2 + .1):
        controlState = 1
      elif (objectDist < .2 - .1):
        controlState = 4
    elif (controlState == 4):
      if (objectDist >= .2 - .1 and math.cos(abs(thetaD - FWCTheta)) > 0):
        controlState = 2
        tau = rospy.get_time()
        xTau = linError
      elif (objectDist >= .2 - .1 and math.cos(abs(thetaD - FWCCTheta)) > 0):
        controlState = 3
        tau = rospy.get_time()
        xTau = linError
    else:
      controlState = 1
      
    if (controlState == 1):
      target_linear_vel = linGoalControl
      target_angular_vel = angGoalControl
    elif (controlState == 2):
      target_linear_vel = linFWCControl
      target_angular_vel = angFWCControl
    elif (controlState == 3):
      target_linear_vel = linFWCCControl
      target_angular_vel = angFWCCControl
    elif (controlState == 4):
      target_linear_vel = linAvoidControl
      target_angular_vel = angAvoidControl
    else:
      target_linear_vel = 0
      target_angular_vel = 0

    
    twist = Twist()
    twist.linear.x = target_linear_vel; twist.linear.y = 0; twist.linear.z = 0
    twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = target_angular_vel
    pub.publish(twist)
  
  
  
  
  
  
  
  
  
  
