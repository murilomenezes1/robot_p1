import rospy
import numpy as np 
import tf
import cv2
import time
from cv_bridge import CvBridge, CvBridgeError



bridge = CvBridge()

cv_image = None
media = []
centro = []
atraso = 1.5E9
dist = None
area = 0.0

class Scan():

	def __init__(self):
		global bridge, cv_image, media, centro, atraso, dist, area



	def roda_todo_frame(imagem):

		now = rospy.get_rostime()
		imgtime = imagem.header.stamp
		lag = now-imgtime
		delay = lag.nsecs
		print("delay", "{:.3f}".format(delay/1.0E9))
		if delay > atraso and check_delay == True:
			print("Descartando por causa do delay do frame:", delay)
			return

		try: 

			antes = time.clock()
			cv_image = bridge.compressed_imgmsg_to_cv2(imagem,"bgr8")
			media, centro, maior_area = cormodule.identifica_cor(cv_image)
			depois = time.clock()
			cv2.imshow("Camera", cv_image)

		except CvBridgeError as e:

			print('ex', e)

	def scaneou(dado):

		dist = dado.ranges[0]
		print(dist)


		