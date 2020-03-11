#-*- coding:utf-8 -*-
import cv2
import os
import gc
import sys
import math
import time
import dlib
import socket
import numpy as np
import imutils
import queue
from imutils.video import VideoStream
from imutils.video import FPS
import argparse
import threading
import multiprocessing as mp
#from multiprocessing.pool import Pool
from multiprocessing import Process, Queue, Manager
from scipy.spatial import distance as dist
from scipy.integrate import odeint		# 引入数值积分方法
from collections import OrderedDict
from collections import deque
import matplotlib.pyplot as plt

process_result = 'ok'

lock = threading.Lock()

tolerance = 5       #瞄准范围宽容度,越小越准确
X_lock = 0          #判断是否已经瞄准物体
Y_lock = 0
Offset = 2000
Rx = Ry = Sx = Sy = 0
Op = [Rx, Ry]		#创建以R为原点的坐标系
Sp = [Sx, Sy]		#创建以屏幕中心为原点的坐标系
totalFrames = 0
Hit = 0
D = 0
Angle = 0
Scr = 0
Rs = 0				#初始化旋转速度
CameraFocal = 2.8	#F = (P * D) / W	焦距F = （相机拍照测得像素宽度P * 目标距离相机位置D） / 物体宽度W
CameraFOV = 82		#设置相机参数		D' = (W * F) / P
V = 2000			#发射速度t_flight =2*u*math.sin(theta_radians)/g
KnownWidth = perWidth =  FocalLenth = D2C = 0

cap = cv2.VideoCapture(r'D:\Desktop\1\RM2019\WindMill(Blue+Swich_On).mp4')
#cap = cv2.VideoCapture(r'/home/osibereaair/Documents/WindMill(Blue+Swich_On).mp4')
#cap = cv2.VideoCapture(1)
cap.set(cv2.CAP_PROP_FPS, 24)

logo = cv2.imread(r'D:\Desktop\1\RM2019\R.png', 0)
#logo = cv2.imread(r'/home/osibereaair/Documents/R.png', 0)
logo = cv2.resize(logo,(0,0),fx = 0.2,fy = 0.2)
Lw, Lh = logo.shape[::-1]

writer = None
W = None
H = None

def nothing(x):
	pass

class CentroidTracker():
	def __init__(self, maxDisappeared = 10, maxDistance = 10):

		self.nextObjectID = 0
		self.objects = OrderedDict()
		self.disappeared = OrderedDict()
		self.maxDisappeared = maxDisappeared
		self.maxDistance = maxDistance

	def register(self, centroid):
		#注册
		self.objects[self.nextObjectID] = centroid
		self.disappeared[self.nextObjectID] = 0
		self.nextObjectID += 1

	def deregister(self, objectID):
		#注销
		del self.objects[objectID]
		del self.disappeared[objectID]

	def update(self, rects):
		#更新
		if len(rects) == 0:
			for objectID in list(self.disappeared.keys()):
				self.disappeared[objectID] += 1

				if self.disappeared[objectID] > self.maxDisappeared:
					self.deregister(objectID)

			return self.objects

		inputCentroids = np.zeros((len(rects), 2), dtype = "int")	#初始化当前帧的输入质心数组

		for (i, (starX, starY, endX, endY)) in enumerate(rects):
			#使用边界框坐标导出质心
			cX = int((starX + endX) / 2.0)
			cY = int((starY + endY) / 2.0)
			inputCentroids[i] = (cX, cY)

		#如果我们当前未跟踪任何对象，输入质心并注册每个质心
		if len(self.objects) == 0:
			for i in range(0, len(inputCentroids)):
				self.register(inputCentroids[i])

		#否则，当前正在跟踪对象，因此我们需要尝试将输入质心与现有对象质心进行匹配
		else:
			objectIDs = list(self.objects.keys())
			objectCentroids = list(self.objects.values())
			#分别计算每对对象质心和输入质心之间的距离
			D = dist.cdist(np.array(objectCentroids), inputCentroids)
			#为了执行此匹配，我们必须（1）在每行中找到最小值，然后（2）根据行索引的最小值对行索引进行排序，
			#以使具有最小值的行位于索引列表的前面
			rows = D.min(axis = 1).argsort()
			#接下来，我们在列上执行类似的过程，方法是在每一列中找到最小值，然后使用先前计算的行索引列表进行排序
			cols = D.argmin(axis = 1)[rows]
			#为了确定是否需要更新，注册或注销对象，我们需要跟踪已经检查过的行索引和列索引
			usedRows = set()
			usedCols = set()

			for (row, col) in zip(rows, cols):
				#如果我们之前已经检查过行或列的值，请忽略它
				if row in usedRows or col in usedCols:
					continue
				#如果质心之间的距离大于最大距离，请勿将两个质心关联到同一对象
				if D[row, col] > self.maxDistance:
					continue

				#否则，获取当前行的对象ID，设置其新的质心，然后重置消失的计数器
				objectID = objectIDs[row]
				self.objects[objectID] = inputCentroids[col]
				self.disappeared[objectID] = 0

				usedRows.add(row)
				usedCols.add(col)

			unusedRows = set(range(0, D.shape[0])).difference(usedRows)
			unusedCols = set(range(0, D.shape[1])).difference(usedCols)

			#如果对象质心的数量等于或大于输入质心的数量，我们需要检查并查看其中某些对象是否可能消失了
			if D.shape[0] >= D.shape[1]:
				for row in unusedRows:
					objectID = objectIDs[row]
					self.disappeared[objectID] += 1

					if self.disappeared[objectID] > self.maxDisappeared:
						self.deregister(objectID)

			else:
				for col in unusedCols:
					self.register(inputCentroids[col])

		return self.objects

class TrackableObject:
	def __init__(self, objectID, centroid):
		#存储对象ID，然后使用当前质心初始化质心列表
		self.objectID = objectID
		self.centroids = [centroid]
		#初始化一个布尔值，用于指示对象是否已被计数
		self.counted = False

def matchTemplate(Thresholdgray, roi, contour):
	#mask = cv2.cvtColor(final, cv2.COLOR_GRAY2BGR)
	logores = cv2.matchTemplate(Thresholdgray, logo, cv2.TM_CCORR_NORMED, roi)
	#logores = cv2.matchTemplate(Thresholdgray, logo, cv2.TM_CCORR_NORMED)
	cv2.imshow("logores", logores)
	cv2.drawContours(logores, contour, 0, (0,255,0), 4)
	min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(logores)
	loc = min_loc
	Rx = np.int(loc[0] + (Lw / 2))
	Ry = np.int(loc[1] + (Lh / 2))
	return Rx, Ry, loc

def foresee(KnownWidth, D, FocalLenth, Rs, Angle):
	D2C = (KnownWidth * FocalLenth) / D
	if (Angle + Rs) <= 360:
		ForeseeAngle = Angle + Rs
	else:
		ForeseeAngle = (Angle + Rs) - 360

	return D2C, ForeseeAngle

def formulaProjectile(X, Y, V, G):
	#抛物线方程 X Y代表预测落点，V代表炮弹初速，G是重力加速度
	DELTA = math.pow(V, 4) - G*(G*X*X - 2*Y*V*V)
	if DELTA >= 0:
		Theta1 = math.atan((-(V**2) + math.sqrt(DELTA)) / (G*X))
		Theta2 = math.atan((-(V**2) - math.sqrt(DELTA)) / (G*X))
		if Theta1 > Theta2:
			#取最小值
			Theta1 = Theta2
		
		T = X / (V*math.cos(Theta1))	#用抛物线水平运动方程计算飞行时间
		return Theta1, T
	else:
		return 0, 0

#实例化质心跟踪器，然后初始化一个列表以存储每个dlib相关性跟踪器
#然后初始化一个字典以将每个唯一对象ID映射到TrackableObject
ct = CentroidTracker(maxDisappeared = 10, maxDistance = 30)
trackers = []
trackableObjects = {}
start = time.time()
fps = None
fps = FPS().start()
base = 2
count = 0

cv2.namedWindow('origin', flags = cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO | cv2.WINDOW_GUI_EXPANDED)			
cv2.namedWindow('res',flags=cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO | cv2.WINDOW_GUI_EXPANDED)
cv2.namedWindow('logores',flags=cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO | cv2.WINDOW_GUI_EXPANDED)

def  main():
	while True:
		if cv2.waitKey(1) != ord('q'):
			ret, frame = cap.read()
			if ret:
				print(" ")
			else:
				print("No Video")
				cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
				ret, frame = cap.read()

			global W, H
			if W is None or H is None:
				(H, W) = frame.shape[:2]
			#获取屏幕分辨率以及中心点
			Sx = W / 2
			Sy = H / 2

			status = "Waiting"
			rects = []
			print('main')
			gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
			th = cv2.adaptiveThreshold(gray,20,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,cv2.THRESH_BINARY,11,2)
			res = cv2.bitwise_and(frame, frame, mask = th)
			hsv_frame = cv2.cvtColor(res, cv2.COLOR_BGR2HSV)
			lower = np.array([0,0,210])
			upper = np.array([255,200,255])
	
			mask = cv2.inRange(hsv_frame, lower, upper)
			_,binary = cv2.threshold(mask, 0.8, 1, cv2.THRESH_BINARY)
			blur = cv2.bilateralFilter(binary,10,100,100)
			erosion = cv2.erode(blur, None, iterations = 2)
			final = erosion
			res = cv2.bitwise_and(frame, frame, mask = final)

			global totalFrames
			if totalFrames % 5 == 0:

				status = "Detecting"
				trackers = []

				contours, hierarchy = cv2.findContours(final.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
				contours = imutils.grab_contours([contours,hierarchy])
				hierarchy = np.squeeze(hierarchy)
				Total = totalFrames
				for i, contour in enumerate(contours):
					area = cv2.contourArea(contour)
					if area < 4000:
						if Total == totalFrames:

							#**********************************R标识别**********************************
							width, high = logo.shape [::-1 ]
							Thresholdgray = cv2.bitwise_and(gray, gray, mask = final)

							cv2.drawContours(res, contour, -1, (255,255,0),2)

							peri = cv2.arcLength(contour, True)
							approx = cv2.approxPolyDP(contour, 0.015 * peri, True)      #获得边长
							((x, y), (w, h), thtea) = cv2.minAreaRect(contour)
							if w < h:
								q = w
								w = h
								h = q
							if h == 0:
								h = 1
							aspect_ratio = float(w)/h
							print(aspect_ratio)

							if (aspect_ratio > 0.8) and (aspect_ratio < 1.2):
								print(">>>>>>>")
								x = np.int(x-5)
								y = np.int(y-5)
								w = np.int(Lw+5)
								h = np.int(Lh+5)
								roi = Thresholdgray[x:x+w , y:y+h]
								Rx, Ry, loc = matchTemplate(Thresholdgray, final, contour)
								cv2.rectangle(res, loc, (loc[0] + Lw, loc[1] + Lh), (0,255,255), 2)
								#cv2.rectangle(res, (loc[0], loc[1]), (Lw + loc[0], Lh + loc[1]), (0,255,255), 2)
								cv2.circle(res, (Rx, Ry), 4, (0, 255, 255), -1)

								global Ptx, Pty, Ptw, Pth
								Ptx = np.int(Rx - (width)/2 - 10)
								Pty = np.int(Ry - (high)/2 - 10)
								Ptw = np.int(Rx + (width)/2 + 10)
								Pth = np.int(Ry + (high)/2 + 10)	
								Ptx = Pty = Ptw = Pth = 0
						
								box = np.array([Ptx, Pty, Ptx + Ptw, Pty + Pth])
								(startX, startY, endX, endY) = box.astype("int")
								tracker = dlib.correlation_tracker()
								rect = dlib.rectangle(startX, startY, endX, endY)
								tracker.start_track(res, rect)
								trackers.append(tracker)
							
							Total = Total + 1

							continue

						else:
							continue

					peri = cv2.arcLength(contour, True)
					approx = cv2.approxPolyDP(contour, 0.015 * peri, True)      #轮廓近似
					((x, y), (w, h), thtea) = cv2.minAreaRect(contour)

					global Bx, By, Bw, Bh
					Bx = np.int(x - w/2)
					By = np.int(y - h/2)
					Bw = np.int(x + w/2)
					Bh = np.int(y + h/2)

					if w < h:
						q = w
						w = h
						h = q
					aspect_ratio = float(w)/h                                   #计算长宽比

					if (aspect_ratio > 1.3) and (aspect_ratio < 2.2):
						screenCnt = approx

						(x, y), radius = cv2.minEnclosingCircle(approx)
						x = np.float32(x)
						y = np.float32(y)
						cv2.circle(res, (x, y), 10, (0, 0, 255), -1)

						#凸包
						hull = cv2.convexHull(contour)
						#检查比率
						hull_area = cv2.contourArea(hull)
						if hull_area == 0:
							hull_area = float(area)
						solidity = float(area)/hull_area
	
						if ((solidity > 75) and (solidity < 300)) or ((solidity > 0.75) and (solidity < 3)):
	
							if hierarchy[i][2] != -1:		#检测是否为父级
								print("该轮廓是父级")
	
							elif (hierarchy[i][3] != -1):
								if i != None:
									font = cv2.FONT_HERSHEY_DUPLEX
									cX = x
									cY = y
									cX = np.int(cX)
									cY = np.int(cY)
									name = i
									cv2.putText(res, '%s'%(name), (cX, cY), font, 1, (0, 255, 255), 1)
	
								match = cv2.matchShapes((contours[i]), (contours[i-1]), cv2.CONTOURS_MATCH_I1, 1)
								C = cv2.contourArea(contours[i])
								F = cv2.contourArea(contours[i - 1])
								if (C > (F - Offset)) and (C < (F + Offset)) and (match < 0.15):
									c = min([screenCnt], key = cv2.contourArea)
									(X, Y), radius = cv2.minEnclosingCircle(c)
									X = np.float32(X)
									Y = np.float32(Y)
									global D, Angle, end, start, Scr, D2C, ForeseeAngle
									D = math.sqrt((X - Rx)**2 + (Y - Ry)**2)
									Angle = math.atan2( Y - Ry, X - Rx)
									Angle = -(Angle / math.pi * 180)
									end = time.time()
									if Angle < 0:
										Angle = - Angle
									else:
										Angle = 360 - Angle
									#右边是0°，下面是90°，左边是180°，上面是270°
									seconds = end - start
									Rs = (Angle - Scr) / seconds
									start = time.time()
									Scr = Angle
									#KnownWidth, D, FocalLenth, Rs, Angle
									D2C, ForeseeAngle = foresee(700, D, 2.8, Rs, Angle)
									radian = ForeseeAngle * math.pi / 180
									Fx = math.cos(radian) * D
									Fy = math.sin(radian) * D
									Fx = Rx + Fx
									Fy = Ry + Fy
									Fx = np.int(Fx)
									Fy = np.int(Fy)

									G = -9.8
								
									#已知击打预瞄点，把炮台Z轴指向击打预瞄点，然后以重力方向为-Y轴，炮台正前方为X轴建立抛物线
									#得到打击角度和飞行时间
									ShootAngle, FlyTime = formulaProjectile(D2C, Fy, V, G)
									print("ShootAngle = ", ShootAngle, "	FlyTime = ", FlyTime, "s")

									cv2.circle(res, (X, Y), 10, (0, 255, 0), -1)
									cv2.circle(res, (Fx, Fy), 5, (0, 255, 255), -1)
									cv2.polylines(res, [hull], True, (255, 0, 0), 1)

									print("旋转速度：", Rs, "到中心点的距离：" , D2C, "预瞄角度：", ForeseeAngle)

									box = np.array([cX, cY, cX + w, cY + h])
									(startX, startY, endX, endY) = box.astype("int")
									tracker = dlib.correlation_tracker()
									rect = dlib.rectangle(startX, startY, endX, endY)
									tracker.start_track(final, rect)
									trackers.append(tracker)

									info = [
										("FPS", "{:.2f}".format(fps.fps())),
										("Distance", "{:.2f}".format(D)),
										("Angle", "{:.2f}".format(Angle)),
										("Status", status),
									]

									for (i, (k, v)) in enumerate(info):
										text = "{}: {}".format(k, v)
										cv2.putText(res, text, (10, H - ((i * 40) + 20)),
											cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 0, 255), 2)

			else:

				for tracker in trackers:
					status = "Tracking"

					tracker.update(res)
					pos = tracker.get_position()

					startX = int(pos.left())
					startY = int(pos.top())
					endX = int(pos.right())
					endY = int(pos.bottom())

					rects.append((startX, startY, endX, endY))

			objects = ct.update(rects)

			for (objectID, centroid) in objects.items():

				to = trackableObjects.get(objectID, None)

				if to is None:
					to = TrackableObject(objectID, centroid)

				else:
					y = [c[1] for c in to.centroids]
					direction = centroid[1] - np.mean(y)
					to.centroids.append(centroid)

				trackableObjects[objectID] = to

				text = "ID {}".format(objectID)
				cv2.putText(res, text, (centroid[0] - 10, centroid[1] - 10),
					cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 255, 0), 2)
				cv2.circle(res, (centroid[0], centroid[1]), 4, (0, 255, 0), -1)

			fps.update()
			fps.stop()

			info = [
				("FPS", "{:.2f}".format(fps.fps())),
				("Distance", "{:.2f}".format(D)),
				("Angle", "{:.2f}".format(Angle)),
				("Status", status),
			]

			for (i, (k, v)) in enumerate(info):
				text = "{}: {}".format(k, v)
				cv2.putText(res, text, (10, H - ((i * 40) + 20)),
					cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 0, 255), 2)

			cv2.imshow("res", res)
			cv2.imshow('origin', frame)

			totalFrames += 1
		else:
			break

	cv2.destroyAllWindows()
	cap.release()



if __name__ == '__main__':
	main()