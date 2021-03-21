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
#                       2021  浙江纺织服装职业技术学院 RoboFuture 连清扬      #
#**************************************************************************#

#-*- coding:utf-8 -*-

#-----------------------------------------------------------------------------
'''
该程序是配合yolo v5深度学习识别算法所写，主要用处是
处理串口的数据，配合深度学习识别结果计算云台旋转角度

该程序用到的环境为：
pyserial, opencv, numpy, scipy, crcmod, gxipy, threading, collections, argparse
'''
				#	-------		串口通讯的规则为：	-------		#
				#	8比特  0校验  1停止  0控制  波特率115200		#
				#	帧头	（1字节）	  校验位（1字节）		  数据	#
				#	---------------------------------------		#

				#收集的数据为：
				#摩擦轮旋转速度（用作初始速度判定）
				#陀螺仪数据（用作当前姿态判定）
				#比赛时间（用作大小幅判定）
				#etc..

				#发送的数据为：
				#云台pich轴旋转角度
				#云台yaw轴旋转角度
				#发射指令
#-----------------------------------------------------------------------------
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
import crcmod.predefined	#添加CRC验证
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

os.system('')
lock = threading.Lock()
def nothing(x):
	pass

class CRCGenerator(object):
	'''
	CRC的校验生成
	'''
	def __init__(self):
		self.module = 'crc-8-maxim'
		
	def create(self,input):
	   crc8 = crcmod.predefined.Crc(self.module)
	   hexData = input
	   print(hexData)						#Output
	   hexData =binascii.unhexlify(hexData)
	   crc8.update(hexData)
	   result = hex(crc8.crcValue)
	   print(result)						#Output
	   return result

class ReadFromSerial(object):
	'''
	从队列中读取字符,读取到的数据放在readByte里面
	'''
	def __init__(self,port):
		self.port = port
		self.readbyte = 0

	def read_one_struct(self):
		self.read = False
		while not self.read:
			#从队列中读取一个字节
			SOFByte = self.port.read(1)
			#读取到帧头就继续往下读
			if (SOFByte == b'\xa0'):
				data = SOFByte + self.port.read(21)
				DecodeData = binascii.b2a_hex(data).decode('utf-8')
				#print("获取到的数据：", DecodeData)

				#CRC8校验
				crc8 = crcmod.predefined.Crc('crc-8')
				hexData = (data[2:]).hex()
				hexData =binascii.unhexlify(hexData)	#16进制转换
				crc8.update(hexData)
				result = hex(crc8.crcValue)				#得到校验结果
				#print("result:", result)				#Output
				#如果校验结果和传输来的结果一致...
				if (('0x' + data[1:2].hex()) == result):
					self.read = True
					#print("The result of CRC8 is currect")
					#self.readByte = int(((data[2:]).hex()), 16)
					self.readByte = struct.unpack("<1i3f1i", data[2:])
					return True
				else:
					print("\033[33mThe result of CRC8 is not currect\033[0m")
					return False
			else:
				print("\033[33mIt's not the Start of data\033[0m")

class CentroidTracker():
	'''
	目标中心质点的跟踪方法
	'''
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

class MathCalculation():
	'''
	集成了几种数学计算方法，包含蒙版匹配， 旋转预测， 抛物线计算
	'''
	def __init__(self, Thresholdgray, roi, KnownWidth, D, FocalLenth, Rs, Angle, X, Y, V, G):
		self.logo = cv2.imread(r'D:\Desktop\1\RM2019\R.png', 0)
		self.logo = cv2.imread(r'/home/osibereaair/Documents/R.png', 0)
		self.logo = cv2.resize(logo,(0,0),fx = 0.15,fy = 0.15)
		self.Thresholdgray = Thresholdgray
		self.roi = roi
		self.KnownWidth = KnownWidth
		self.D = D
		self.FocalLenth = FocalLenth
		self.Rs = Rs
		self.Angle = Angle
		self.X = X
		self.Y = Y
		self.V = V
		self.G = G

	def matchTemplate(Thresholdgray, roi):
		'''
		用matchTemplate方法识别大风车
		'''
		self.logores = cv2.matchTemplate(Thresholdgray, self.logo, cv2.TM_CCORR_NORMED, roi)
		self.min_val, self.max_val, self.min_loc, self.max_loc = cv2.minMaxLoc(self.logores)
		self.loc = self.max_loc
		self.Rx = np.int(self.loc[0] + (Lw / 2))
		self.Ry = np.int(self.loc[1] + (Lh / 2))
		return self.Rx, self.Ry, self.loc

	def foresee(KnownWidth, D, FocalLenth, Rs, Angle):
		'''
		计算相机到目标的距离 (Distance to Camera)
		KnownWidth 为已知宽度，D为图像上目标的宽度， FocalLenth为相机镜头的焦距
		'''
		self.D2C = (KnownWidth * FocalLenth) / D
		#KnowWidth = (D * D2C) / FocalLenth
		#FocalLenth = D2C * D / KnowWidth
		if (Angle + Rs) <= 360:
			self.ForeseeAngle = Angle + Rs
		else:
			self.ForeseeAngle = (Angle + Rs) - 360

		return self.D2C, self.ForeseeAngle

	def formulaProjectile(X, Y, V, G):
		'''
		抛物线方程 X Y代表预测落点，V代表炮弹初速，G是重力加速度
					  v² ± √v­⁴-g(gx²+2yv²)
		 α = arctan(——————————————————————)
							  gx
		'''
		self.DELTA = math.pow(V, 4) - G*(G*X*X - 2*Y*V*V)
		if self.DELTA >= 0:
			self.Theta1 = math.atan(((V**2) + math.sqrt(self.DELTA)) / (G*X))
			self.Theta2 = math.atan(((V**2) - math.sqrt(self.DELTA)) / (G*X))
			if self.Theta1 > self.Theta2:
				#取最小值
				self.Theta1 = self.Theta2

			T = X / (V*math.cos(self.Theta1))	#用抛物线水平运动方程计算飞行时间
			return self.Theta1, self.T
		else:
			return 0, 0

class RealReadThread(threading.Thread):
	'''
	实时读取相机画面并推送到另一线程
	'''
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
		'''
		开始运行，打开相机并开始串流
		进行预设的画面处理并推送到线程
		'''
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
				print("\033[31mGetting image failed.\033[0m")
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
	'''
	开始处理数据，进行一个目标的识别反馈
	'''
	def __init__(self, input):
		super(GetThread).__init__()
		self._jobq = input
		threading.Thread.__init__(self)

	def GetResult(self):
		'''
		获取深度学习的识别反馈，暂时略过
		输出的四个函数为目标中央坐标和长宽（单位px）
		'''
		target = [targetX, targetY, wdith, high]
		return targrt

	def TargetOffset(self):
		'''
		开始计算云台旋转角度（即旋转云台直到目标处于画面中央）
		'''

		#实例化质心跟踪器，然后初始化一个列表以存储每个dlib相关性跟踪器
		#然后初始化一个字典以将每个唯一对象ID映射到TrackableObject
		ct = CentroidTracker(maxDisappeared = 5, maxDistance = 50)
		trackers = []
		trackableObjects = {}
		start = time.time()
		fps = None
		fps = FPS().start()

		cv2.namedWindow('res',flags=cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO | cv2.WINDOW_GUI_EXPANDED)
		flag = False

		#初始化数据
		global totalFrames
		totalFrames = 0
		CameraFocal = 2.8	#F = (P * D) / W	焦距F = （相机拍照测得像素宽度P * 目标距离相机位置D） / 物体宽度W
		CameraFOV = 82		#设置相机参数		D' = (W * F) / P
		V = 0				#发射速度t_flight =2*u*math.sin(theta_radians)/g
		KnownWidth = perWidth = D2C = 0
		FocalLenth = 8		#(8mm)
		#控制X/Y轴的旋转
		X_Control = 0
		Y_Control = 0
		tolerance = 5       #瞄准范围宽容度,越小越准确
		G = 9.8				#设定重力常量

		#从队列管道中接收数据
		while (True):
			if (len(self._jobq) != 0):
				lock.acquire()
				frame = self._jobq.pop()
				lock.release()

				global FrameW, FrameH, targetX, targetY
				if FrameW is None or FrameH is None:
					(FrameH, FrameW) = frame.shape[:2]
				#获取屏幕分辨率以及中心点
				ScreenX = FrameW / 2
				ScreenY = FrameH / 2

				#设置当前状态为Waiting，并且初始化rects
				status = "Waiting"
				rects = []

				if totalFrames % 50 == 0:
					'''
					如果当前帧不为50的倍数就开始检测
					即每50帧检测一次目标位置
					'''
					status = "Detecting"
					trackers = []
					#从深度学习算法中获取目标在画面中的位置
					trackers = GetResult(frame)

					#获取目标相对于相机的距离
					D2C = (KnownWidth * FocalLenth) / D

					#已知击打预瞄点，把炮台Z轴指向击打预瞄点，然后以重力方向为-Y轴，炮台正前方为X轴建立抛物线
					#得到打击角度和飞行时间
					ShootAngle, FlyTime = MathCalculation.formulaProjectile(D2C, Fy, V, G)
					ShootAngle = ShootAngle * (180) / (math.pi)
					print("ShootAngle = ", ShootAngle, "	FlyTime = ", FlyTime, "s", "	Fy = ", Fy)

					cv2.circle(res, (X, Y), 10, (0, 255, 0), -1)
					cv2.circle(res, (Fx, Fy), 5, (0, 255, 255), -1)
					cv2.polylines(res, [hull], True, (255, 0, 0), 1)

					#传递参数给跟踪模块
					box = np.array([cX, cY, cX + w, cY + h])
					(startX, startY, endX, endY) = box.astype("int")
					tracker = dlib.correlation_tracker()
					rect = dlib.rectangle(startX, startY, endX, endY)
					tracker.start_track(final, rect)
					trackers.append(tracker)
			
				else:
					'''
					如果当前帧不为50的倍数，就开始跟踪
					该方法可以有效提高帧数，减少计算检测时间
					'''
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

				#判断目标在画面中心（瞄准点）的什么位置
				targetX, targetY, wdith, high = trackers

				global yawRotateAngel, pitchRotateAngle, canShootY, canShootX
				canShootX = False
				canShootY = False
				yawRotateAngle = pitchRotateAngle = 0
				WdithX = ((targetX - Sx) * D2C) / FocalLenth

				if (targetX < Sx - tolerance):
					yawRotateAngel = WdithX / D2C
				elif (targetX > Sx + tolerance):
					yawRotateAngel = WdithX / D2C
				else:
					canShootX = True
				#-------------------------------------------
				if (targetY < Sy - tolerance):
					pitchRotateAngle = Fy / D2C
				elif (targetY > Sy + tolerance):
					pitchRotateAngle = Fy / D2C
				else:
					canShootY = True
				if (canShootX and canShootY):
					shootOrder = 1
				else:
					shootOrder = 0

				#运算完毕，开始串口传输数据
				try:
					ser = serial.Serial(
						port = "/dev/ttyTHS1",	#COM1	/dev/ttyTHS1
						baudrate = 115200,
						bytesize = serial.EIGHTBITS,
						parity = serial.PARITY_NONE,
						stopbits = serial.STOPBITS_ONE,
						timeout = 0.01,
						)
					print("ser is open")
					time.sleep(0.5)

					#尝试发送数据
					try:
						SOF = (b'\xa0')
						data = struct.pack("<2f1B", yawRotateAngel, pitchRotateAngle, shootOrder)
						decodeData = binascii.b2a_hex(data).decode('utf-8')
						#print("serial SEND RAWdata is: ", decodeData)

						#CRC8校验
						crc = crcmod.predefined.Crc('crc-8')
						hexData = data.hex()
						hexData =binascii.unhexlify(hexData)	#16进制转换
						crc.update(hexData)
						result = hex(crc.crcValue)				#得到校验结果
						result = bytes.fromhex(result[2:])
						print(binascii.b2a_hex(SOF + result + data).decode('utf-8'))
						ser.write(SOF + result + data)

					except Exception as e:
						print("\033[31mserial write data has ERROR: \033[0m", e)

					#如果缓冲区有数据就开始读取
					if ser.inWaiting() > 0:
						opClass = ReadFromSerial(ser)
						if opClass.read_one_struct():
							SerialReadData = opClass.readByte
							print("struct Read Data:", SerialReadData)

				except Exception as e:
					print("\033[31mOpen SERIAL has ERROR:\033[0m", e)
				finally:
					ser.close()

				#开始解析串口接收的数据
				#比赛时间（判断大小幅）（int，4）
				#陀螺仪数据（判断当前姿态（3float，12）
				#炮弹射速（判断子弹落点）（int，4）
				MatchTime, Carx, Cary, Carz, V = SerialReadData

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

				cv2.imshow("res", res)
				cv2.imshow('origin', frame)

				totalFrames += 1
				flag = True

			#退出方法
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

#相机的可调参数
'''
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
'''
#串口不同函数类别的字节长度
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