##########################################################################
#                            Trent Rankin                                #
#                    Image Classification Algorithm                      #
#                                                                        #
# This program takes a set of training images to train a classifier      #
# The classifier is used to classify a set of test images and compute    #
# the number of images classified correctly                              #
##########################################################################

#!/usr/bin/env python3

import cv2
import sys
import csv
import numpy as np

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
  mask = cv2.inRange(hsv, np.array([163, 110, 60], np.uint8), np.array([192, 180, 200], np.uint8))
  x, y, w, h = cv2.boundingRect(mask)
  #This conditional is checking if the amount of red in the image is significant enough to consider it a red sign
  if ((w * h) > 1000):
    color = "red"
  else:
    mask = cv2.inRange(hsv, np.array([47, 70, 30], np.uint8), np.array([75, 170, 200], np.uint8))
    x, y, w, h = cv2.boundingRect(mask)
    #This conditional is checking if the amount of red in the image is significant enough to consider it a green sign
    if ((w * h) > 1000):
      color = "green"
    else:
      mask = cv2.inRange(hsv, np.array([95, 150, 65], np.uint8), np.array([120, 220, 200], np.uint8))
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


### Run test images
with open('imgs/test.txt', 'rb') as f:
    reader = csv.reader(f)
    lines = list(reader)

correct = 0.0
confusion_matrix = np.zeros((6,6))

#This loop is the same as before, it is cropping the test images to ensure that they are classified based on the sign portion only
for i in range(len(lines)):
    img = cv2.imread("imgs/"+lines[i][0]+".png")
    imgBlur = cv2.GaussianBlur(img, (9, 9), 0)
    hsv = cv2.cvtColor(imgBlur, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, np.array([163, 110, 60], np.uint8), np.array([192, 180, 200], np.uint8))
    x, y, w, h = cv2.boundingRect(mask)
    if ((w * h) > 1000):
      color = "red"
    else:
      mask = cv2.inRange(hsv, np.array([47, 70, 30], np.uint8), np.array([75, 170, 200], np.uint8))
      x, y, w, h = cv2.boundingRect(mask)
      if ((w * h) > 1000):
        color = "green"
      else:
        mask = cv2.inRange(hsv, np.array([95, 150, 65], np.uint8), np.array([120, 220, 200], np.uint8))
        x, y, w, h = cv2.boundingRect(mask)
        if ((w * h) > 1000):
          color = "blue"
        else:
          x = 75
          y = 75
          w = 100
          h = 75
          color = "wall"
        
    crop_img = img[y:(y + h), x:(x + w)]
    test_img = np.array(cv2.resize(crop_img, (33, 25)))
    test_img = test_img.flatten().reshape(1, 33 * 25 * 3)
    test_img = test_img.astype(np.float32)

    test_label = np.int32(lines[i][1])

    ret, results, neighbours, dist = knn.findNearest(test_img, 3)

    if test_label == ret:
        print(lines[i][0], " Correct, ", ret)
        correct += 1
        confusion_matrix[np.int32(ret)][np.int32(ret)] += 1
    else:
        confusion_matrix[test_label][np.int32(ret)] += 1
        
        print(lines[i][0], " Wrong, ", test_label, " classified as ", ret)
        print "\tneighbours: ", neighbours
        print "\tdistances: ", dist



print("\n\nTotal accuracy: ", correct/len(lines))
print(confusion_matrix)
