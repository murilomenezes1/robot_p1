#! /usr/bin/env python
# -*- coding:utf-8 -*-


from __future__ import print_function, division
import rospy
import numpy as np
import numpy
import tf
import math
import cv2
import time
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
from numpy import linalg
from tf import transformations
from tf import TransformerROS
import tf2_ros
from geometry_msgs.msg import Twist, Vector3, Pose, Vector3Stamped
from ar_track_alvar_msgs.msg import AlvarMarker, AlvarMarkers
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from sensor_msgs.msg import LaserScan


import visao_module

x0 = None
y0 = None

bridge = CvBridge()

cv_image = None
media = []
centro = []
atraso = 1.5E9 # 1 segundo e meio. Em nanossegundos
dist = None


maior_area = 0.0 # Variavel com a area do maior contorno

# Só usar se os relógios ROS da Raspberry e do Linux desktop estiverem sincronizados. 
# Descarta imagens que chegam atrasadas demais
check_delay = False 

resultados = [] # Criacao de uma variavel global para guardar os resultados vistos

x = 0
y = 0
z = 0 
id = 0

frame = "camera_link"
# frame = "head_camera"  # DESCOMENTE para usar com webcam USB via roslaunch tag_tracking usbcam

tfl = 0

tf_buffer = tf2_ros.Buffer()

max_linear = 0.2
max_angular = math.pi/8
contador = 0
pula = 100
alfa = -1
px = None
py = None


def odometria(data):

	global px 
	global py
	global alfa
	global contador
	global x0
	global y0



	px = data.pose.pose.position.x
	py = data.pose.pose.position.y


	quat = data.pose.pose.orientation
	lista = [quat.x, quat.y, quat.z,quat.w]
	angulos_rad = transformations.euler_from_quaternion(lista)
	angulos = np.degrees(angulos_rad)

	alfa = angulos_rad[2]

	if contador == 0:
		x0 = data.pose.pose.position.x
		y0 = data.pose.pose.position.y
		contador += 1


	#if contador % pula == 0:
		#print("Posicao (x,y) ({:.2f}) + angulo {:.2f}".format(x,y,angulos[2]))
		#contador += 1


def angle(alfa,x,y):
	beta = math.atan((y/x))
	angulo_total = beta + math.pi - alfa
	return angulo_total

def distance(px,py):
	hipotenusa = math.sqrt(pow(x,2) + pow(y,2))
	return hipotenusa




def track():

	global fuga_x
	global fuga_y






def roda_todo_frame(imagem):
	print("frame")
	global cv_image
	global media
	global centro
	global centro0
	global resultados
	global maior_area
	global frame


	now = rospy.get_rostime()
	imgtime = imagem.header.stamp
	lag = now-imgtime # calcula o lag
	delay = lag.nsecs
	# print("delay ", "{:.3f}".format(delay/1.0E9))
	if delay > atraso and check_delay==True:
		print("Descartando por causa do delay do frame:", delay)
		return 
	try:
		antes = time.clock()
		temp_image = bridge.compressed_imgmsg_to_cv2(imagem, "bgr8")
		# Note que os resultados já são guardados automaticamente na variável
		# chamada resultados
		centro, saida_net, resultados =  visao_module.processa(temp_image)
		media, centro0, maior_area, frame = visao_module.identifica_cor(temp_image)        
		for r in resultados:
			# print(r) - print feito para documentar e entender
			# o resultado            
			pass

		depois = time.clock()
		# Desnecessário - Hough e MobileNet já abrem janelas
		cv_image = saida_net.copy()
	except CvBridgeError as e:
		print('ex', e)


def scaneou(dado):

		global dist
		dist=dado.ranges[0]
		print(dist)





if __name__ == "__main__":

	rospy.init_node("cor")

	topico_imagem = "/camera/rgb/image_raw/compressed"

	recebe_scan = rospy.Subscriber("/scan", LaserScan, scaneou)

	recebedor = rospy.Subscriber(topico_imagem, CompressedImage, roda_todo_frame, queue_size = 4, buff_size = 2**24)
	print("Usando ", topico_imagem) 

	velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)

	ref_odometria = rospy.Subscriber("/odom", Odometry, odometria)

	tfl = tf2_ros.TransformListener(tf_buffer) #conversao do sistema de coordenadas 
	tolerancia = 25
	creeper_found = False
	go_back = False

	try:


			while not rospy.is_shutdown():

				# for r in resultados:
				# 	print(r)

				if cv_image is not None:
					# Note que o imshow precisa ficar *ou* no codigo de tratamento de eventos *ou* no thread principal, não em ambos
					cv2.imshow("cv_image no loop principal", frame)
					cv2.waitKey(1)
				# rospy.sleep(0.1)

					if len(media) != 0 and len(centro0) !=0 and maior_area > 500:

						# print("Media dos vermelhos: {0}, {1}".format(media[0], media[1]))
						# print("Centro dos vermelhos: {0},{1}".format(centro[0], centro[1]))

						creeper_found = True

						# if dist > 0.2:
					if creeper_found:

						if (media[0] > centro0[0]):
							vel = Twist(Vector3(0.1,0,0), Vector3(0,0,-0.1))
							velocidade_saida.publish(vel)
										
						elif (media[0] < centro0[0]):
							vel = Twist(Vector3(0.1,0,0), Vector3(0,0,0.1))
							velocidade_saida.publish(vel)



						elif (abs(media[0] - centro0[0]) < 10):
					
						  	vel = Twist(Vector3(0.3,0,0), Vector3(0,0,0))
						  	velocidade_saida.publish(vel)

					

						

							# vel = Twist(Vector3(0,0,0), Vector3(0,0,0))
							# velocidade_saida.publish(vel)
							# rospy.sleep(0.2)
							# angle = angle(alfa,px,py)
							# distance = distance(px,py)
						

					if dist <= 0.25 and dist > 0:	

						go_back = True
						# vel = Twist(Vector3(0,0,0), Vector3(0,0,0))
						# velocidade_saida.publish(vel)

					if go_back:
								
						if px > x0 and py > y0: 


							# vel_rot = Twist(Vector3(0,0,0), Vector3(0,0,max_angular))
							vel_trans = Twist(Vector3(-max_linear,0,0), Vector3(0,0,0))

							# sleep_rot = abs(angle/max_angular)
							#sleep_trans = abs(distance/max_linear)

							# velocidade_saida.publish(vel_rot)
							# rospy.sleep(sleep_rot)

							velocidade_saida.publish(vel_trans)
							rospy.sleep(0.2)
							print("-------")
	

						vel_zero = Twist(Vector3(0,0,0), Vector3(0,0,0))
						velocidade_saida.publish()


				cv2.waitKey(1)
			
			rospy.sleep(0.1)
				

	except rospy.ROSInterruptException:
		print("Ocorreu uma exceção com o rospy")