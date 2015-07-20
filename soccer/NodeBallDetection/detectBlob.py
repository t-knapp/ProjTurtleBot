# -*- coding: utf-8 -*-


#!/usr/bin/env python
import numpy as np
from FilterOption import *
import cv2

'''
Ur-Version von Team BaloonTerminator

Erweiterung / Optimierung / Anpassung durch Team Böing


Parameter self.detectExtraRed beachten!
   Da rot im HSV Farbraum bei 0° und 360° (in OpenCV bei 180°) liegt,
   sind zwei Erkennungsschritte (cv2.inrange(min, max)) notwendig.
   Dieser Parameter bestimmt ob zum im GUI eingestellten Wert
   noch der Bereich um 0° - 15° erkannt werden soll.
   Beide erkannten Bilder werden verbunden. Führt zu exzellenter
   Ballerkennung für roten Ball!
   In NodeBallDetection auf True
   In NodeGoalDetection auf False
'''
class DetectBlob(object):
  i=0
  def __init__(self, name="detectBlob", position=(0,0), detectExtraRed=False):
    self.minColor = np.array([0, 0, 0], np.uint8)
    self.maxColor = np.array([0, 0, 0], np.uint8)
    
    self.name = name
    self.detectExtraRed = detectExtraRed
    
    ''' Shape-Filter '''
    circularityOpt = FilterOption("circularity")
    inertiaOpt = FilterOption("inertia")
    convexityOpt = FilterOption("convexity")
    self.filterShape = [circularityOpt, inertiaOpt, convexityOpt]

    ''' Blur-Filter '''
    self.filterBlur = 1       

    self.params = cv2.SimpleBlobDetector_Params()
    self.params.filterByColor = False
    self.params.filterByInertia = False
    
    self.params.minInertiaRatio = 0
    self.params.maxInertiaRatio = 1
    
    self.params.filterByCircularity = False
    self.params.minCircularity = 0.8
    self.params.maxCircularity = 1

    self.params.filterByArea = False

    self.params.filterByConvexity = True
    self.params.minConvexity = 0.5
    self.params.maxConvexity = 1

    # OpenCV windows
    self.cv_range = self.name + " :: inrange"
    cv2.namedWindow(self.cv_range, 1)
    cv2.moveWindow(self.cv_range, position[0],position[1])
    
    #cv2.namedWindow("debug", 1)
    #cv2.namedWindow("or", 1)
    
    self.cvWindows = dict()
    self.position = position
    
    cv2.startWindowThread()

  def cvCheckBoxCallback(self, varName, varValue):
      if self.cvWindows[varName]:
        self.cvWindows[varName] = varValue
        cv2.destroyWindow(varName)
      else:
        cv2.namedWindow(varName, 1)
        self.cvWindows[varName] = varValue
        if varName == self.cv_range:
            cv2.moveWindow(self.cv_range, self.position[0], self.position[1])


  # Callback for color adjusting, used by gui
  def setColors(self, minColor, maxColor):
      #print("DetectBlob.setColors      ", minColor, maxColor)
      self.minColor = minColor
      self.maxColor = maxColor
      #print("DetectBlob.setColors(self)", self.minColor, self.maxColor)

  def setFilterShape(self, filterShapeAry):
      self.filterShape = filterShapeAry
      
      ''' Filter Circularity '''
      self.params.filterByCircularity = (self.filterShape[0].activated == 1)
      self.params.minCircularity = self.filterShape[0].minimum
      self.params.maxCircularity = self.filterShape[0].maximum
      
      ''' Filter Inertia '''
      self.params.filterByInertia = (self.filterShape[1].activated == 1)
      self.params.minInertiaRatio = self.filterShape[1].minimum
      self.params.maxInertiaRatio = self.filterShape[1].maximum
      
      ''' Filter Convexity '''
      self.params.filterByConvexity = (self.filterShape[2].activated == 1)
      self.params.minConvexity = self.filterShape[2].minimum
      self.params.maxConvexity = self.filterShape[2].maximum
      
      #print("DetectBlob.setFilterShape(self)", " self.params.filterByCircularity: ", self.params.filterByCircularity, 
	  #  " self.params.filterByInertia: ", self.params.filterByInertia, " self.params.filterByConvexity: ", self.params.filterByConvexity)
    
  def setFilterBlur(self, filterNo):
      self.filterBlur = filterNo
      #print("DetectBlob.setFilterBlur(self)", self.filterBlur)
      

  def identifyColor(self, img):
    img_blur = img
    img_blur = self.doBlurFiltering(img)
    #img_blur = cv2.GaussianBlur(img,(5,5),0)
    #img_blur = cv2.medianBlur(img_blur, 5)
    img_hsv = cv2.cvtColor(img_blur,cv2.COLOR_BGR2HSV)
    # rospy.loginfo(img_hsv[320][240])
    
    # Find selected colors in gui
    img_inrange = cv2.inRange(img_hsv, self.minColor, self.maxColor)
    
    # cv2.imwrite("/tmp/img/hsv_in_range_" + str(DetectBlob.i) + ".jpg", img_inrange)
    #img_erode_gui = cv2.erode(img_inrange, None, iterations = 3)
    #img_dilate_gui = cv2.dilate(img_erode_gui, None, iterations = 10)
    
    img_or = img_inrange
    
    if self.detectExtraRed:
        # Find selected colors 0 - 15
        img_inrange_red = cv2.inRange(img_hsv, np.array([0, 20, 10], np.uint8), np.array([15, 255, 255], np.uint8))
        
        img_or = cv2.bitwise_or(img_inrange, img_inrange_red)
    
    img_erode = cv2.erode(img_or, None, iterations = 3)
    img_dilate = cv2.dilate(img_erode, None, iterations = 3)
    
    #cv2.imshow("or", img_dilate_or)
    
    if self.cvWindows[self.cv_range]:
        cv2.imshow(self.cv_range, img_dilate)
    
    # cv2.imwrite("/tmp/img/hsv_dilate"+str(DetectBlob.i)+".jpg", img_dilate)
    DetectBlob.i += 1
    # cv2.imshow('detect ball', img_dilate)

    return img_dilate
  
  def doBlurFiltering(self, img):
    if(self.filterBlur == 1):
      return img
    elif(self.filterBlur == 2):
      return cv2.GaussianBlur(img,(5,5),0)
    else:
      return cv2.medianBlur(img, 5)

  def getBlobs(self, img):
    binary = self.identifyColor(img)

    # Reuse possible?
    detector = cv2.SimpleBlobDetector(self.params)
    keypoints = detector.detect(binary)

    keypoints.sort(self.compKeypoint)
    keypoints = self.throwOutSmallBlobs(keypoints, 10) #30
    
    im_with_keypoints = cv2.drawKeypoints(img, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    # cv2.imwrite("/tmp/img/key_"+str(DetectBlob.i)+".jpg", im_with_keypoints)
    DetectBlob.i += 1
    return keypoints

  # Comparator for keypoints
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
