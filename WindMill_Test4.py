#-*- coding:utf-8 -*-
import cv2
import time
import numpy as np
#import picamera
#from picamera.array import PiRGBArray
import argparse
import imutils
from collections import deque

tolerance = 17  #瞄准范围宽容度,越小越准确
X_lock = 0  #判断是否已经瞄准物体
Y_lock = 0
currentFrames = 0

#colorUpper = (0, 0, 210)
#colorLower = (255, 120, 255)
colorUpper = (0, 0, 0)
colorLower = (255, 255, 255)

#初始化程序
#ap = argparse.ArgumentParser()
#ap.add_argument("-b", "==buffer", type = int, default = 64, help = "max buffer size")
#args = vers(ap.parse_args)
#pts = deque(maxlen = arge["buffer"])

#设定摄像机大小
#camera = picamera.PiCamera()
#cv2.VideoCapture.set(CV_CAP_PROP_FRAME_WIDTH ,640)

#cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
cap = cv2.VideoCapture(r'D:\Desktop\1\RM2019\WindMill(Blue+Swich_On).mp4')

#camera.resolution = (640,480)
#camera.framerate = 30
#rawCapture = PiRGBArray(camera, size = (640, 480))
cap.set(cv2.CAP_PROP_FRAME_WIDTH ,640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT ,480)
cap.set(cv2.CAP_PROP_FPS,30)

while True:
    if cv2.waitKey(25) & 0xFF != ord('q'):
        _, frame = cap.read()

        #for frame in camera.capture_continuous(rawCapture, format = "bgr", use_video_port = True):
            #frame_image = frame.array

        kernel = np.ones((1,1),np.uint8)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, colorLower, colorUpper)
        mask = cv2.erode(mask, kernel, iterations=2)
        mask = cv2.dilate(mask, kernel, iterations=2)
        res = cv2.bitwise_and(frame, frame, mask = mask)
        #contours, hierarchy = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
        #contours = cv2.findContours(mask.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contours, hierarchy = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        #contours = imutils.grab_contours(contours)
        contours = imutils.grab_contours([contours,hierarchy])
        print('1')
        for i, contour in enumerate(contours):
            area = cv2.contourArea(contour)
            print(area)
            if area < 5000:
                del contours[i]
            print('10')
            if len(contours) > 0:
                cnt = contours[0]
            print('2')

            c = [i, contour]
            # approximate the contour
            peri = cv2.arcLength(c, True)
            approx = cv2.approxPolyDP(c, 0.015 * peri, True)
 
	        # if our approximated contour has four points, then
	        # we can assume that we have found our screen
            if len(approx) == 4:
                screenCnt = approx
                break
            print('3')

            #找出面积最大的轮廓
            c = max([screenCnt], key = cv2.contourArea)
            ((X, Y), radius) = cv2.minEnclosingCircle(c)
            #判断中心点在画面中心哪个方向

            if Y < (240 - tolerance):
                error = (240-Y)/5
                print('up   (error:%d)'%(error))
                Y_lock = 0
            elif Y > (240 + tolerance):
                error = (Y-240)/5
                print('down     (error:%d)'%(error))
                Y_lock = 0
            else:
                Y_lock = 1

            if X < (320 - tolerance):
                error = (320-X)/5
                print('up   (error:%d)'%(error))
                X_lock = 0
            elif X > (320 + tolerance):
                error = (X-320)/5
                print('down     (error:%d)'%(error))
                X_lock = 0
            else:
                X_lock = 1

            if X_lock == 1 and Y_lock == 1:
                print('locked:Fire!')
            else:
                print('detected but not locked:Hold!')
                pass
            print('4')
            cv2.drawContours(cap, cnts, 0, (0, 0, 255), -1)
            cv2.circle(cap, (X, Y), 7, (255, 255, 255), -1)

        #循环播放视频
        if (currentFrames == 329):
            currentFrames = 0
            cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
        currentFrames = currentFrames + 1
        print(currentFrames)

        #清空缓存
        #cap.truncate(0)

        cv2.namedWindow('cap',flags=cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO | cv2.WINDOW_GUI_EXPANDED)
        cv2.namedWindow('mask',flags=cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO | cv2.WINDOW_GUI_EXPANDED)


        cv2.imshow("cap", frame)
        cv2.imshow("mask", res)
        print(contours)
        print('------------------------------------')
    else:
        break

camera.release()
cv2.destroyAllWindows()