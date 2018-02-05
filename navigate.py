#!/usr/bin/env python

import rospy
import numpy as np
import math
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import LaserScan
import cv2
import sys
import csv
from cv_bridge import CvBridge

bridge = CvBridge()

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
  if (yaw < 0):
    yaw += 360

def processLidar(scanData):
  global wallDist
  
  wallDist = 0
  j = 0
  array = [358, 359, 0, 1, 2]
  for x in array:
    test = scanData.ranges[x]
    if (test < 4.0 and test > .1):
      wallDist += scanData.ranges[x]
      j += 1
  
  if (j > 0):
    wallDist = wallDist / j
  

def processImage(imageData):
  global testReady
  global resultReady
  global knn
  global label
  global imgArray
  
  if (testReady):
    img = bridge.compressed_imgmsg_to_cv2(imageData, "bgr8")
    imgBlur = cv2.GaussianBlur(img, (9, 9), 0)
    hsv = cv2.cvtColor(imgBlur, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, np.array([163, 80, 50], np.uint8), np.array([192, 200, 220], np.uint8))
    x, y, w, h = cv2.boundingRect(mask)
    #print(w * h)
    if ((w * h) > 1000):
      color = "red"
    else:
      mask = cv2.inRange(hsv, np.array([47, 50, 30], np.uint8), np.array([75, 200, 220], np.uint8))
      x, y, w, h = cv2.boundingRect(mask)
      #print(w * h)    
      if ((w * h) > 1000):
        color = "green"
      else:
        mask = cv2.inRange(hsv, np.array([95, 100, 50], np.uint8), np.array([120, 250, 220], np.uint8))
        x, y, w, h = cv2.boundingRect(mask)
        #print(w * h)
        if ((w * h) > 1000):
          color = "blue"
        else:
          x = 75
          y = 75
          w = 100
          h = 75
          color = "wall"
        
    crop_img = img[y:(y + h), x:(x + w)]
    cv2.imshow("image", crop_img)
    k = cv2.waitKey(1)
    test_img = np.array(cv2.resize(crop_img, (33, 25)))
    test_img = test_img.flatten().reshape(1, 33 * 25 * 3)
    test_img = test_img.astype(np.float32)
    label, results, neighbours, dist = knn.findNearest(test_img, 4)
    imgArray[int(label)] += 1


def Init():
  
  rospy.init_node('navigate', anonymous = True)
  
  rospy.Subscriber('/odom', Odometry, processOdom)
  rospy.Subscriber("/scan", LaserScan, processLidar)
  rospy.Subscriber("/raspicam_node/image/compressed", CompressedImage, processImage)
  
  global pub
  pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)

if __name__ == '__main__':

  global currentX
  global currentY
  global yaw
  global wallDist
  global testReady
  global resultReady
  global knn
  global label
  global imgArray
  
  z = 1
  offset = 10
  currentX = 0
  currentY = 0
  yaw = 0
  wallDist = 5
  testReady = False
  resultReady = False
  label = 0
  first = True
  imgArray = [0, 0, 0, 0 ,0, 0]
  
  
  ### Load training images and labels

  with open('imgs/train.txt', 'rb') as f:
    reader = csv.reader(f)
    lines = list(reader)
      
  train = []
  # This section reads in all the images and stores each image as one long vector of values
  for i in range(len(lines)):
    img = cv2.imread("imgs/"+lines[i][0]+".png")
    imgBlur = cv2.GaussianBlur(img, (9, 9), 0)
    hsv = cv2.cvtColor(imgBlur, cv2.COLOR_BGR2HSV)
    #This mask is used to filter out anything that isn't the specific type of red desired
    mask = cv2.inRange(hsv, np.array([163, 80, 9], np.uint8), np.array([192, 200, 220], np.uint8))
    x, y, w, h = cv2.boundingRect(mask)
    #This conditional is checking if the amount of red in the image is significant enough to consider it a red sign
    if ((w * h) > 1000):
      color = "red"
    else:
      mask = cv2.inRange(hsv, np.array([46, 50, 9], np.uint8), np.array([75, 200, 220], np.uint8))
      x, y, w, h = cv2.boundingRect(mask)
      #This conditional is checking if the amount of red in the image is significant enough to consider it a green sign
      if ((w * h) > 1000):
        color = "green"
      else:
        mask = cv2.inRange(hsv, np.array([95, 100, 9], np.uint8), np.array([122, 250, 220], np.uint8))
        x, y, w, h = cv2.boundingRect(mask)
        #This conditional is checking if the amount of red in the image is significant enough to consider it a blue sign
        if ((w * h) > 1000):
          color = "blue"
        #If it is none of these it is probably a wall
        else:
          x = 75
          y = 75
          w = 100
          h = 75
          color = "wall"
    #Crop the image to only include the part that we want to classify
    crop_img = img[y:(y + h), x:(x + w)]
    train.append(np.array(cv2.resize(crop_img, (33, 25))))

  train = np.array(train)

  # here we reshape each image into a long vector and ensure the data type is a float (which is what KNN wants)
  train_data = train.flatten().reshape(len(lines), 33 * 25 * 3)
  train_data = train_data.astype(np.float32)

  # read in training labels
  train_labels = np.array([np.int32(lines[i][1]) for i in range(len(lines))])


  ### Train classifier
  knn = cv2.ml.KNearest_create()
  knn.train(train_data, cv2.ml.ROW_SAMPLE, train_labels)
  
  try:
    Init()
  except rospy.ROSInterruptException:
    pass
  
  rate = rospy.Rate(10)
  rospy.sleep(2.)
  newYaw = yaw
  while not rospy.is_shutdown():
    rate.sleep()
    if (wallDist > 0.44):
#      print("1st loop")
      if (first):
        twist = Twist()
        twist.linear.x = 0.8; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        pub.publish(twist)
        first = False
      elif (abs(yaw - newYaw) > 3):
        twist = Twist()
        twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        pub.publish(twist)
        aSum = 0
        target_angular_vel = 0
        while(1):
          rate.sleep()
          angError = (-(yaw - newYaw) * 3.14159) / 180
          if ((yaw >= 270 and yaw <= 360 and newYaw >= 0 and newYaw <= 90) or (yaw >= 0 and yaw <= 90 and newYaw >= 270 and newYaw <= 360)):
            angError = -angError
          print(yaw, newYaw, angError)
          aSum += angError * .2
          if (aSum > 10):
            aSum = 10
          elif (aSum < -10):
            aSum = -10
            
          ang_xDot = (0.1 * angError) + (0.02 * aSum)
          
          target_angular_vel += ang_xDot
            
          if (angError < .03 and angError > -.03):
            print("exit loop 2")
            twist = Twist()
            twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
            pub.publish(twist)
            first = True
            break
          
          if (target_angular_vel > 0.5):
            target_angular_vel = 0.5
          elif (target_angular_vel < -0.5):
            target_angular_vel = -0.5
            
          twist = Twist()
          twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
          twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = target_angular_vel
          pub.publish(twist)
        
    else:
#      print("2nd loop")
#      seconds = rospy.get_time()
#      while ((rospy.get_time() - seconds) < 3):
      first = True
      twist = Twist()
      twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
      twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
      pub.publish(twist)
      
      imgArray = [0, 0, 0, 0, 0, 0]
      testReady = True
      rospy.sleep(2.)
      testReady = False
      newLabel = 0
      for j in range(0, 6):
        if (imgArray[j] > imgArray[newLabel]):
          newLabel = j
      print(imgArray)
      print("Label is: " + str(newLabel))
      
      if (abs(yaw - newYaw) > 2):
        newYaw = newYaw
      elif (newLabel == 1):
        newYaw = (yaw + 90) % 360
        offset = 10
        z = 1
      elif (newLabel == 2):
        newYaw = (yaw - 90) % 360
        offset = 10
        z = 1
      elif (newLabel == 3 or newLabel == 4):
        newYaw = (yaw + 180) % 360
        offset = 10
        z = 1
      elif (newLabel == 5):
        print("GOAL!")
        rospy.spin()
      else:
        newYaw = (yaw + offset) % 360
        offset = offset + 10 * -z
        z += 1
      
      aSum = 0
      target_angular_vel = 0
      while(1):
        rate.sleep()
        angError = (-(yaw - newYaw) * 3.14159) / 180
        if ((yaw >= 270 and yaw <= 360 and newYaw >= 0 and newYaw <= 90) or (yaw >= 0 and yaw <= 90 and newYaw >= 270 and newYaw <= 360)):
          angError = -angError
        print(yaw, newYaw, angError)
        aSum += angError * .2
        if (aSum > 10):
          aSum = 10
        elif (aSum < -10):
          aSum = -10
          
        ang_xDot = (0.1 * angError) + (0.02 * aSum)
        
        target_angular_vel += ang_xDot
          
        if (angError < .03 and angError > -.03):
          print("exit loop 1")
          twist = Twist()
          twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
          twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
          #rospy.sleep(0.3)
          pub.publish(twist)
          break
        
        if (target_angular_vel > 0.4):
          target_angular_vel = 0.4
        elif (target_angular_vel < -0.4):
          target_angular_vel = -0.4
          
        twist = Twist()
        twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = target_angular_vel
        pub.publish(twist)
      




































