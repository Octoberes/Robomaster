#**************************************************************************#
#       __________          _______       _________       _________        #
#      /_______  /\        /__  __/\     /  ______/\     /  ______/\       #
#      \______/ / /        \_/ /\_\/    /  /\_____\/    /  /\_____\/       #
#           / / /     __    / / /      /  /_/___       /  /_/___           #
#         / / /      / /\  / / /      /  ______/\     /  ______/\          #
#       / /_/____    \ \/_/ / /      /  /\_____\/    /  /\_____\/          #
#     /_________/\    \____/ /      /__/ /          /__/ /                 #
#     \_________\/     \___\/       \__\/           \__\/                  #
#                                                                          #
#**************************************************************************#

#-*- coding:utf-8 -*-
import cv2
import os
import gc
import sys
import math
import time
import dlib
import numpy as np
import imutils
import serial	#串口
import binascii	#在BIN(二进制)和ASCII之间转换
import struct	#将数据作为完整的结构传输，用struct模块进行处理
import crcmod	#添加CRC验证
import scipy
import gxipy as gx
from imutils.video import VideoStream
from imutils.video import FPS
import argparse
import threading
from threading import Thread
from scipy.spatial import distance as dist
from scipy.integrate import odeint		# 引入数值积分方法
from collections import OrderedDict
from collections import deque
import matplotlib.pyplot as plt
#import api.gxipy as gx

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
V = 12			#发射速度t_flight =2*u*math.sin(theta_radians)/g
KnownWidth = perWidth = D2C = 0
FocalLenth = 8
#控制X/Y轴的旋转
X_Control = 0
Y_Control = 0
RotateAngle[2] = [0, 0]
packed.gimbal_rotate = RotateAngle

#cap = cam.data_stream[0].get_image()
#frame = cap.convert("RGB")
#numpy_image = frame.get_numpy_array()
#numpy_image = cv2.cvtColor(numpy_image, cv2.COLOR_RGB2BGR)  # opencv采用的是BGR图像， 将RGB转为BGR

#cap = cv2.VideoCapture(r'D:\Desktop\1\RM2019\WindMill(Blue+Swich_On).mp4')
#cap = cv2.VideoCapture(r'D:\Project\Video\rm2020-_asm.mp4')
#cap = cv2.VideoCapture(r'/home/osibereaair/Documents/WindMill(Blue+Swich_On).mp4')
#cap = cv2.VideoCapture(1)
#cap.set(cv2.CAP_PROP_FPS, 24)

#logo = cv2.imread(r'D:\Desktop\1\RM2019\R.png', 0)
#logo = cv2.imread(r'/home/osibereaair/Documents/R.png', 0)
#logo = cv2.resize(logo,(0,0),fx = 0.15,fy = 0.15)
#Lw, Lh = logo.shape[::-1]

writer = None
W = None
H = None

#输入输出的针脚
#output_pin = 8
#input_pin = 10

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

def matchTemplate(Thresholdgray, roi):
	logores = cv2.matchTemplate(Thresholdgray, logo, cv2.TM_CCORR_NORMED, roi)
	min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(logores)
	loc = max_loc
	Rx = np.int(loc[0] + (Lw / 2))
	Ry = np.int(loc[1] + (Lh / 2))
	return Rx, Ry, loc

def foresee(KnownWidth, D, FocalLenth, Rs, Angle):
	#计算相机到目标的距离
	D2C = (KnownWidth * FocalLenth) / D		#KnowWidth = (D * D2C) / FocalLenth
	#FocalLenth = D2C * D / KnowWidth
	if (Angle + Rs) <= 360:
		ForeseeAngle = Angle + Rs
	else:
		ForeseeAngle = (Angle + Rs) - 360

	return D2C, ForeseeAngle

def formulaProjectile(X, Y, V, G):
	#抛物线方程 X Y代表预测落点，V代表炮弹初速，G是重力加速度
	#			  v² ± √v­⁴-g(gx²+2yv²)
	# α = arctan(——————————————————————)
	#						gx
	DELTA = math.pow(V, 4) - G*(G*X*X - 2*Y*V*V)
	if DELTA >= 0:
		Theta1 = math.atan(((V**2) + math.sqrt(DELTA)) / (G*X))
		Theta2 = math.atan(((V**2) - math.sqrt(DELTA)) / (G*X))
		if Theta1 > Theta2:
			#取最小值
			Theta1 = Theta2

		T = X / (V*math.cos(Theta1))	#用抛物线水平运动方程计算飞行时间
		return Theta1, T
	else:
		return 0, 0

#从队列中读取字符
class ReadFromSerial(object):
	
	def __init__(self, port):
		self.port = port
		#self.vals = []
		self.readbyte = 0

	#就读一个字节（八位
	def read_one_struct(self):
		self.read = False
		while not self.read:
			#从队列读取一个字节	👇读取（1字节）
			SOFByte = self.port.read(1)
			#print(SOFByte)
			
			#如果字节的开头是 b'\xa0' 即串口协议的起始指示符 则读取
			if (SOFByte == b'\xa0'):  # 查找开始指示符
				#data = self.port.read_until(b'1') # Z指示结束指示符
				#读到了起始符就继续往下读四个字节
				data = SOFByte + self.port.read(4)
				print("获取到的数据(data):", data)
				#try:							👇这个<是发送数据的大小端排序
					#new_values = struct.unpack('<s2B1B3f1H', data)
				#读到的数据第二位是“这一串数据的字符长度”		👇然后转换为hex
				print("readBytes:", int('0x' + ((data[2:3]).hex()), 16))
				#										👆就是从冒号前面的数字读到后面的数字-1

				#简化
				crc8 = crcmod.predefined.Crc('crc-8-maxim')
				hexData = (data[:4]).hex()				#取前四位
				hexData =binascii.unhexlify(hexData)	#16进制转换
				crc8.update(hexData)
				result = hex(crc8.crcValue)				#得到校验结果
				print("result:", result)				#Output

				print("data[4:]:", data[4:].hex(), "(data[:4]):", data[:4])
				#如果自己校验的和数据后面的校验结果一样的话
				if (('0x' + data[4:].hex()) == result):
					#那数据就没什么问题
					self.read = True
					print("yeeeeeeeeeeeeeeeeeeee") 
					self.readByte = int('0x' + ((data[2:3]).hex()), 16)
					return True

#实例化质心跟踪器，然后初始化一个列表以存储每个dlib相关性跟踪器
#然后初始化一个字典以将每个唯一对象ID映射到TrackableObject
ct = CentroidTracker(maxDisappeared = 5, maxDistance = 50)
trackers = []
trackableObjects = {}
start = time.time()
fps = None
fps = FPS().start()
base = 2
count = 0

#cv2.namedWindow('origin', flags = cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO | cv2.WINDOW_GUI_EXPANDED)			
#cv2.namedWindow('res',flags=cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO | cv2.WINDOW_GUI_EXPANDED)

class RealReadThread(threading.Thread):
	def __init__(self, input):
		super(RealReadThread).__init__()
		self._jobq = input

		#self.cap = cv2.VideoCapture(r'D:\Desktop\1\RM2019\WindMill(Blue+Swich_On).mp4')
		#cv2.waitKey(1)
		#self.cap = cv2.VideoCapture(r'D:\Desktop\1\RM2019\rm2020-_asm120.avi')
		#self.cap = cv2.VideoCapture(r'D:\Desktop\1\RM2019\rm2020-_asm120.mp4')
		#time.sleep(1.0)
		#cv2.waitKey(1)
		#self.cap.set(cv2.CAP_PROP_FPS, 24)

		threading.Thread.__init__(self)

	def run(self):

		# 打开设备
		# 枚举设备
		device_manager = gx.DeviceManager() 
		dev_num, dev_info_list = device_manager.update_device_list()
		if dev_num == 0:
			sys.exit(1)
		# 获取设备基本信息列表
		str_sn = dev_info_list[0].get("sn")
		# 通过序列号打开设备
		self.cam = device_manager.open_device_by_sn(str_sn)
		# set continuous acquisition
		cam.TriggerMode.set(gx.GxSwitchEntry.OFF)
		# 导入配置信息
		# cam.import_config_file("./import_config_file.txt")
		# 开始采集
		self.cam.stream_on()

		# 帧率
		#self.fps = cam.AcquisitionFrameRate.get()  
		# 视频的宽高
		self.size = (cam.Width.get(),cam.Height.get())

		cv2.namedWindow('origin', flags = cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO | cv2.WINDOW_GUI_EXPANDED)

		while self.cap.isOpened():
			print("Cap is Open")
			#ret, frame = self.cap.read()
			#if ret:
				#print(" ")
			#else:
				#print("No Video")
				#self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
				#ret, frame = self.cap.read()

			self.cap = cam.data_stream[0].get_image()
			if cap is None:
				print("Getting image failed.")
				continue
			cap = self.cap.convert("RGB")
			if cap is None:
				continue
			frame = cap.get_numpy_array()

			lock.acquire()
			if len(self._jobq) == 10:
				self._jobq.popleft()
			else:
				self._jobq.append(frame)
			lock.release()
			cv2.imshow('origin', frame)

			if cv2.waitKey(1) == ord('q'):
				break
		cv2.destroyWindow('origin')
		self._jobq.clear()
		cam.stream_off()
		cam.close_device()

class GetThread(threading.Thread):
	def __init__(self, input):
		super(GetThread).__init__()
		self._jobq = input
		threading.Thread.__init__(self)
		
	def run(self):
		cv2.namedWindow('res',flags=cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO | cv2.WINDOW_GUI_EXPANDED)

		flag = False
		while (True):
			if len(self._jobq) != 0:
				lock.acquire()
				frame = self._jobq.pop()
				lock.release()

				global W, H
				if W is None or H is None:
					(H, W) = frame.shape[:2]
				#获取屏幕分辨率以及中心点
				Sx = W / 2
				Sy = H / 2

				status = "Waiting"
				rects = []
				#sprint('main')
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
				final = blur
				thmask = cv2.bitwise_and(th, mask, mask = erosion)
				res = cv2.bitwise_and(frame, frame, mask = final)

				global totalFrames
				if totalFrames % 1 == 0:

					status = "Detecting"
					trackers = []

					contours, hierarchy = cv2.findContours(final.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
					contours = imutils.grab_contours([contours,hierarchy])
					hierarchy = np.squeeze(hierarchy)
					Total = totalFrames
					for i, contour in enumerate(contours):
						area = cv2.contourArea(contour)
						rect = cv2.minAreaRect(contour)
						box = cv2.boxPoints(rect)
						box = np.int0(box)
						cv2.drawContours(res,[box],0,(0,0,255),2)
						if area < 4000:
							#**********************************R标识别**********************************
							#width, high = logo.shape [::-1 ]

							peri = cv2.arcLength(contour, True)
							approx = cv2.approxPolyDP(contour, 0.015 * peri, True)      #获得边长
							((x, y), (w, h), thtea) = cv2.minAreaRect(contour)
							if w < h:
								q = w
								w = h
								h = q
							if h == 0:
								h = 1
							#aspect_ratio = float(w)/h
							aspect_ratio = w/h
							if (aspect_ratio > 0.8) and (aspect_ratio < 1.2) and (area > 5):
								print(">>>>>>>")
								x = np.int(x)
								y = np.int(y)
								w = np.int(w)
								h = np.int(h)
								
								#roi = thmask[x:x+w , y:y+h]
								#global Rx, Ry

								#凸包
								hull = cv2.convexHull(contour)
								#检查比率
								hull_area = cv2.contourArea(hull)
								if hull_area == 0:
									hull_area = float(area)
								solidity = float(area)/hull_area

								if ((solidity > 0.8) and (solidity < 0.9)):

									print(solidity)
									global Ptx, Pty, Ptw, Pth
									Ptx = np.int(x-w/2)
									Pty = np.int(y-h/2)
									Ptw = np.int(x+w/2)
									Pth = np.int(y+h/2)

									cv2.rectangle(res, (Ptx, Pty), (Ptw, Pth), (0,255,255), 2)
									cv2.circle(res, (x, y), 4, (0, 255, 255), -1)

									box = np.array([Ptx, Pty, Ptw, Pth])
									(startX, startY, endX, endY) = box.astype("int")
									tracker = dlib.correlation_tracker()
									rect = dlib.rectangle(startX, startY, endX, endY)
									tracker.start_track(res, rect)
									trackers.append(tracker)
							
							Total = Total + 1

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
										D2C, ForeseeAngle = foresee(750, D, 8, Rs, Angle)
										radian = ForeseeAngle * math.pi / 180
										Fx = math.cos(radian) * D
										Fy = math.sin(radian) * D
										Fx = np.int(Rx + Fx)
										Fy = np.int(Ry + Fy)
										G = 9.8

										#已知击打预瞄点，把炮台Z轴指向击打预瞄点，然后以重力方向为-Y轴，炮台正前方为X轴建立抛物线
										#得到打击角度和飞行时间
										ShootAngle, FlyTime = formulaProjectile(D2C, Fy, V, G)
										ShootAngle = ShootAngle * (180) / (math.pi)
										print("ShootAngle = ", ShootAngle, "	FlyTime = ", FlyTime, "s", "	Fy = ", Fy)

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

				dst = cv2.cornerHarris(blur, 2, 3, 0.04)
				#res[dst > 0.01 * dst.max()] = [0, 255, 0]
				#cv2.imshow("HarrisCorner Detection", img)

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

				global targetX, targetY, yawRotateAngel, pitchRotateAngle, canShootY, canShootX
				targetX = centroid[0]
				targetY = centroid[1]
				canShootX = False
				canShootY = False
				yawRotateAngle = pitchRotateAngle = RotateAngle[0] = RotateAngle[1] = 0
				WdithX = ((targetX - Sx) * D2C) / FocalLenth

				if (targetX < Sx - tolerance):
					yawRotateAngel = WdithX / D2C
					#RotateAngle[0] = WdithX / D2C

				elif (targetX > Sx + tolerance):
					yawRotateAngel = WdithX / D2C
					#RotateAngle[0] = WdithX / D2C

				else:
					canShootX = True

				if (targetY < Sy - tolerance):
					pitchRotateAngle = Fy / D2C
					#RotateAngle[1] = Fy / D2C

				elif (targetY > Sy + tolerance):
					pitchRotateAngle = Fy / D2C
					#RotateAngle[1] = Fy / D2C

				else:
					canShootY = True

				if (canShootX and canShootY):
					shootOrder = 1
				else:
					shootOrder = 0

				#packed.gimbal_rotate = RotateAngle
				#packed.shoot_order = 0
				#serialPort.sendData(yawRotateAngel, pitchRotateAngle, shootOrder)

				#通过串口传输数据
				try:
					ser = serial.Serial(
						port = "COM1",	#COM1按需修改 但是一般都是COM1
						baudrate = 115200,
						bytesize = serial.EIGHTBITS,
						parity = serial.PARITY_NONE,
						stopbits = serial.STOPBITS_ONE,
						timeout = 0.01,
						)
					time.sleep(0.5)
					#start = time.time()	

					try:
						SOF = (b'\xa0 \x00 \x10 \x01 \x1b')
						data = struct.pack("<1B2f1B", 2, 33.21, -21.58, 0)
						#data = struct.pack("<2B1B3f1H", 2, 2, 12, 12.8, 10.3, 0.0, 36)
						ser.write(SOF + data)										#开头和数据主体一起发出去
						print(SOF + data)
						print("structWriteData:", struct.unpack("<1B2f1B", data))	#看看发的对不对
						#print("structWriteData:", struct.unpack("<2B1B3f1H", data))

					except Exception as e:
						print(e)
						ser.write("ERROR\r\n".encode("gbk"))

					if ser.inWaiting() > 0:
						opClass = ReadFromSerial(ser)
						if opClass.read_one_struct():
							print("readbyte:", opClass.readByte)
							print("structReadData:", struct.unpack("<2B1B3f1H", ser.read(opClass.readByte)))

						#ser.close()	# Close Serial Port
						#data = opClass.vals
						#print("structData:", data)

						#print("!1B2f1B", struct.calcsize('!1B2f1B'))		#10
						#print("!2B1B3f1H", struct.calcsize('!2B1B3f1H'))	#17
						#print("structReadData:", struct.unpack("<2B1B3f1H", ser.readline()))

					#end = time.time()
					#print("using Time:", end - start)

				except Exception as e:
					print(e)
				finally:
					ser.close()

				fps.update()
				fps.stop()
				#fps = cv2.dilate()
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


				#res[dst > 0.01 * dst.max()] = [0, 255, 0]

				#GPIO.setwarnings(False)
				#GPIO.setmode(GPIO.BOARD)
				#GPIO.setup(led_pin, GPIO.OUT)
				#try:
					#if (target.x):
						#pass

				#cv2.waitKey(1)
				cv2.imshow("res", res)
				cv2.imshow('origin', frame)

				totalFrames += 1
				flag = True
			elif flag == True and len(self._jobq) == 0:
				break

if __name__ == '__main__':
	q = deque([], 10)
	th1 = RealReadThread(q)
	th2 = GetThread(q)
	th1.start()
	th2.start()

	th1.join()
	th2.join()


"""
	cv2.CAP_PROP_POS_MSEC	0	视频文件的当前位置（以毫秒为单位）或视频捕获时间戳
	cv2.CAP_PROP_POS_FRAMES	1	基于0的索引将被解码/捕获下一帧
	cv2.CAP_PROP_POS_AVI_RATIO	2	视频文件的相对位置：0 - 视频的开始，1 - 视频的结束
	cv2.CAP_PROP_FRAME_WIDTH	3	帧的宽度
	cv2.CAP_PROP_FRAME_HEIGHT	4	帧的高度
	cv2.CAP_PROP_FPS	5	帧速
	cv2.CAP_PROP_FOURCC	6	4个字符表示的视频编码器格式
	cv2.CAP_PROP_FRAME_COUNT	7	帧数
	cv2.CAP_PROP_FORMAT	8	byretrieve()返回的Mat对象的格式
	cv2.CAP_PROP_MODE	9	指示当前捕获模式的后端特定值
	cv2.CAP_PROP_BRIGHTNESS	10	图像的亮度（仅适用于相机）
	cv2.CAP_PROP_CONTRAST	11	图像对比度（仅适用于相机）
	cv2.CAP_PROP_SATURATION	12	图像的饱和度（仅适用于相机）
	cv2.CAP_PROP_HUE	13	图像的色相（仅适用于相机）
	cv2.CAP_PROP_GAIN	14	图像的增益（仅适用于相机）
	cv2.CAP_PROP_EXPOSURE	15	曝光（仅适用于相机）
	cv2.CAP_PROP_CONVERT_RGB	16	表示图像是否应转换为RGB的布尔标志
	cv2.CAP_PROP_WHITE_BALANCE	17	目前不支持
	cv2.CAP_PROP_RECTIFICATION	18	立体摄像机的整流标志
"""

'''
	Format		C Type				Python				字节数
	x			pad byte			no value			1
	c			char				string of length 1	1
	b			signed char			integer				1
	B			unsigned char		integer				1
	?			_Bool				bool				1
	h			short				integer				2
	H			unsigned short		integer				2
	i			int					integer				4
	I			unsigned int		integer or long		4
	l			long				integer				4
	L			unsigned long		long				4
	q			long long			long				8
	Q			unsigned long long	long				8
	f			float				float				4
	d			double				float				8
	s			char[]				string				1
	p			char[]				string				1
	P			void *				long
'''