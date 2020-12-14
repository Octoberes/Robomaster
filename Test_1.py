import serial	#串口
import sys
import time 
import numpy
import binascii	#在BIN(二进制)和ASCII之间转换
import struct	#将数据作为完整的结构传输，struct模块进行处理
import crcmod	#添加CRC验证

#	-------		串口通讯的规则为：	-------		#
#	8比特  0校验  1停止  0控制  波特率115200		#

#CRC校验的生成	（就看看 CRC的原理是这样 代码里没引用这个类）
class CRCGenerator(object):
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

					#readData =  self.vals.append(list(new_values))

				#except KeyboardInterrupt:
					#return False # 停止程序
				#except:
					#return True # 忽略结构错误

	#s = serial.Serial('COM17') # 在设备管理器中检查COM端口
	#opClass = ReadFromSerial(s)
	#while opClass.read_one_value():
		#pass
	#s.close() # 关闭串行端口
	#data = opClass.vals

#这里是主程序
while True:
	#试试
	try:
		ser = serial.Serial(
			port = "COM1",	#COM1按需修改 但是一般都是COM1
			baudrate = 115200,
			bytesize = serial.EIGHTBITS,
			parity = serial.PARITY_NONE,
			stopbits = serial.STOPBITS_ONE,
			timeout = 0.01,
			)
		time.sleep(0.5)		#停一停 太快不好
		#		👆这就cv2.WaitKey()一样
		#start = time.time()	

		#试试
		try:
			#👇这开头都固定的 我干脆算好搁这儿了
			SOF = (b'\xa0 \x00 \x10 \x01 \x1b')
			#然后是传输的主体数据		👇 < 意味着排序方式，1B2f1B意味一个unsigned char +  两个float + 一个unsigned char 详情看最下面
			data = struct.pack("<1B2f1B", 2, 33.21, -21.58, 0)
			#data = struct.pack("<2B1B3f1H", 2, 2, 12, 12.8, 10.3, 0.0, 36)
			ser.write(SOF + data)										#然后就是开头和数据主体一起发出去
			print(SOF + data)
			print("structWriteData:", struct.unpack("<1B2f1B", data))	#看看发的对不对
			#print("structWriteData:", struct.unpack("<2B1B3f1H", data))

		#要是试出毛病了
		except Exception as e:
			#print毛病
			print(e)
			#串口发一个ERROR（虽然没用但是好玩）
			ser.write("ERROR\r\n".encode("gbk"))

		#上面数据发完了就该读数据了，读写不能同时进行，python只能单线程
		if ser.inWaiting() > 0:
			opClass = ReadFromSerial(ser)
			if opClass.read_one_struct():
				print("readbyte:", opClass.readByte)
				print("structReadData:", struct.unpack("<2B1B3f1H", ser.read(opClass.readByte)))
				#											👆对面按照这个发送的
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