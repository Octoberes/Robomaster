#-*- coding:utf-8 -*-
import cv2
import numpy as np
import imutils
from skimage import morphology
import argparse
import time
from imutils.video import FPS


def nothing(x):
    pass

#cap = cv2.VideoCapture(1)
#cap = cv2.VideoCapture(r'D:\BaiduNetdiskDownload\8-11东大3No.4.avi')
#cap = cv2.VideoCapture(r'D:\BaiduNetdiskDownload\8-11东大2No.4.avi')
#cap = cv2.VideoCapture(r'D:\Desktop\1\RM2019\大能量机关（红+开灯）.mov')
cap = cv2.VideoCapture(r'D:\Desktop\1\RM2019\rm2020-_asm120.avi')
time.sleep(1.0)
cap.set(cv2.CAP_PROP_FPS, 120)

logo = cv2.imread(r'D:\Desktop\1\RM2019\R.png', 0)
logo = cv2.resize(logo,(0,0),fx = 0.25,fy = 0.25)

currentFrames = 0

cv2.namedWindow("Trackbars")
cv2.createTrackbar("L - H", "Trackbars", 0, 255, nothing)
cv2.createTrackbar("L - S", "Trackbars", 0, 255, nothing)
cv2.createTrackbar("L - V", "Trackbars", 210, 255, nothing)
cv2.createTrackbar("U - H", "Trackbars", 255, 255, nothing)
cv2.createTrackbar("U - S", "Trackbars", 200, 255, nothing)
cv2.createTrackbar("U - V", "Trackbars", 255, 255, nothing)

#cv2.namedWindow('frame',flags=cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO | cv2.WINDOW_GUI_EXPANDED)
#cv2.namedWindow('mask',flags=cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO | cv2.WINDOW_GUI_EXPANDED)
#cv2.namedWindow('gray',flags=cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO | cv2.WINDOW_GUI_EXPANDED)
cv2.namedWindow('origin',flags=cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO | cv2.WINDOW_GUI_EXPANDED)

cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
#cap.set(cv2.CAP_PROP_FPS, 1)
fps = None
fps = FPS().start()

while True:
    if 0xFF != ord('q'):
        cv2.waitKey(1)
        ret, frame = cap.read()       
        fps.update()
        fps.stop()
        
        #循环播放视频
        if ret:
            print(" ")
        else:
            print('no video')
            cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            ret, frame = cap.read()
        frame = cv2.resize(frame,(0,0),fx = 0.25,fy = 0.25)

        l_h = cv2.getTrackbarPos("L - H", "Trackbars")
        l_s = cv2.getTrackbarPos("L - S", "Trackbars")
        l_v = cv2.getTrackbarPos("L - V", "Trackbars")
        u_h = cv2.getTrackbarPos("U - H", "Trackbars")
        u_s = cv2.getTrackbarPos("U - S", "Trackbars")
        u_v = cv2.getTrackbarPos("U - V", "Trackbars")

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        lower_red = np.array([l_h,l_s,l_v])
        upper_red = np.array([u_h,u_s,u_v])

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        th = cv2.adaptiveThreshold(gray,20,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,cv2.THRESH_BINARY,11,2)
        res = cv2.bitwise_and(frame, frame, mask = th)
        hsv_frame = cv2.cvtColor(res, cv2.COLOR_BGR2HSV)
        lower = np.array([l_h,l_s,l_v])
        upper = np.array([u_h,u_s,u_v])

        mask = cv2.inRange(hsv_frame, lower, upper)
        blur = cv2.bilateralFilter(mask,10,100,100)
        #kernel = np.ones((2,2),np.uint8)
        #erosion = cv2.erode(blur, None, iterations = 1)
        #dilation  = cv2.dilate(blur, None, iterations = 1)
        final = mask
        res = cv2.bitwise_and(frame, frame, mask = final)

        contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours:
            area = cv2.contourArea(contour)

            if area > 4000:
                cnt = contours[0]
                rects = []
                #cv2.rectangle(res, pt, (pt[0] + width, pt[1] + high), (0,255,255), 2)
        
        #cv2.imshow("frame", res)
        #cv2.imshow("mask", mask)
        #cv2.imshow("blur", blur)
        #cv2.imshow("erosion", erosion)
        cv2.imshow("origin", res)
        print(fps.fps())

    else:
        break

cap.release()
cv2.destroyAllWindows()