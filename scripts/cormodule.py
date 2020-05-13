import rospy
import numpy as np 
import tf 
import math
import cv2 
import time
from geometry_msgs.msg import Twist, Vector3, Pose
from nav_msgs,msg import Odometry
from sensor_msgs.msgs import Image, COmpressedImage
from cv_bridge import CvBridge, CvBridgeError
import smach
import smach_ros

class identifica_cor():


	def __init__(self, frame):

		self.frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

		self.cor_menor = np.array([50,50,80])
		self.cor_maior = np.array([70,255,255])
		self.segmentado_cor = cv2.inRange(frame_hsv, cor_menor, cor_maior)

		self.centro = (frame.shape[1]//2,frame.shape[0]//2)


	def cross(img_rgb, point, color, width, length):

		cv2.line(img_rgb, (point[0] - length/2, point[1]), (point[0] + length/2, point[1]), color, width, length)
		cv2.line(img_rgb, (point[0], point[1] - length/2), (point[0], point[1] + length/2), color, width, length)

		






