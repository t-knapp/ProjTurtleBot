#!/usr/bin/env python
import numpy as np
import cv2

class DetectBlob(object):
  i=0
  def __init__(self, minColor, maxColor):
    self.minColor = minColor
    self.maxColor = maxColor

    self.params = cv2.SimpleBlobDetector_Params()
    self.params.filterByColor = False
    self.params.filterByInertia = False
    self.params.filterByCircularity = False
    self.params.filterByArea = False
    self.params.filterByConvexity = True
    self.params.minConvexity = 0
    self.params.maxConvexity = 1

  def identifyColor(self, img):
    img_blur = img
#    img_blur = cv2.GaussianBlur(img,(5,5),0)
    img_blur = cv2.medianBlur(img_blur, 5)
    img_hsv = cv2.cvtColor(img_blur,cv2.COLOR_BGR2HSV)
    # rospy.loginfo(img_hsv[320][240])
    img_inrange = cv2.inRange(img_hsv, self.minColor, self.maxColor)
    # cv2.imwrite("/tmp/img/hsv_in_range_" + str(DetectBlob.i) + ".jpg", img_inrange)
    img_erode = cv2.erode(img_inrange, None, iterations = 3)
    img_dilate = cv2.dilate(img_erode, None, iterations = 10)
    # cv2.imwrite("/tmp/img/hsv_dilate"+str(DetectBlob.i)+".jpg", img_dilate)
    DetectBlob.i += 1
    # cv2.imshow('detect ball', img_dilate)

    return img_dilate

  def getBlobs(self, img):
    binary = self.identifyColor(img)

    detector = cv2.SimpleBlobDetector(self.params)
    keypoints = detector.detect(binary)

    keypoints.sort(self.compKeypoint)
    keypoints = self.throwOutSmallBlobs(keypoints, 30)
    
    im_with_keypoints = cv2.drawKeypoints(img, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    # cv2.imwrite("/tmp/img/key_"+str(DetectBlob.i)+".jpg", im_with_keypoints)
    DetectBlob.i += 1
    return keypoints

  def compKeypoint(self, x, y):
    if x.size > y.size:
      return -1
    elif x.size < y.size:
      return 1
    else:
      return 0

  def throwOutSmallBlobs(self, keypoints, minSize):
    returnList = []

    for kp in keypoints:
      if kp.size >= minSize:
        returnList.append(kp)

    return returnList

if __name__ == '__main__':
  DetectBlob(np.array([160, 120, 70], np.uint8), np.array([200, 255, 255], np.uint8))
  # im_with_keypoints = cv2.drawKeypoints(im, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
  # cv2.imshow("Keypoints", im_with_keypoints)