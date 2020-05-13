import cv2 
import numpy as np 




# Classe para manter o robo na pista --AINDA NAO FUNCIONA

class track():

	def __init__():


	def keepintrack():



		while (True):
			ret, frame = #camera do robo --precisa implementar

			grayscale = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
			edges = cv2.Canny(grayscale,50,150,apertureSize = 3)
			lines = cv2.HoughLines(edges, 1,np.pi/180,200)


			# Ponto de Fuga
			k1 = 0
			m1 = 0
			k2 = 0
			m2 = 0 	


			lines_detected0 = False
			lines_detected1 = False

			for l in lines:
				for radius, angle in l:
					a = np.cos(angle)
					b = np.sin(angle)

					x0 = a*radius
					y0 = b*radius
				

					x1 = int(x0 + 1000*(-b))
					y1 = int(y0 + 1000*(a))
					x2 = int(x0 - 1000*(-b))
					y2 = int(y0 - 1000*(a)) 


					distx = x2-x1
					disty = y2-y1


					if distx != 0:
						coef = disty/distx

					if coef < -0.25 and coef > -3:
						if lines_detected0 == False:
							lines_detected0 = True
							m1 = coef
							k1 = (y1-coef*x1)
							cv2.line(frame,(x1,y1), (x2,y2), (0,0,255),2)

					elif coef > 0.25 and coef < 3:
						if lines_detected1 == False:
							lines_detected1 = True
							m2 = coef
							k2 = (y1-coef*x1)
							cv2.line(frame,(x1,y1), (x2,y2), (0,0,255),2)

			if (m1-m2) != 0:
				fuga_x = int((k2-k1)/(m1-m2))
			fuga_y = int(m1*fuga_x + k1)
		cv2.circle(frame,(fuga_x,fuga_y),10,(0,255,0),-1)	