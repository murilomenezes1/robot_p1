import cv2 
import numpy as np 


CLASSES = []
COLORS = []

DEFAULT_PROTOTXT = "MobileNetSSD_deploy.prototxt.txt"
DEFAULT_MODEL = "MobileNetSSD_Deploy.caffemodel"


class Detect():

	def __init__(self,prototxt=None,model=None):

		global DEFAULT_PROTOTXT, DEFAULT_MODEL
		self.prototxt = DEFAULT_PROTOTXT if (prototxt == None) else prototxt
		self.model = DEFAULT_MODEL if (model == None) else model
		self.neuralnet = cv2.dnn.readNetFromCaffe(self.prototxt, self.model)

	def 