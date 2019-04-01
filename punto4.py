#!/usr/bin/python
import rospy
from std_msgs.msg import Float32MultiArray, Float32
from geometry_msgs.msg import Twist
from pynput.keyboard import Key, Listener 
from pylab import *
import matplotlib.pyplot as plt
import threading
import sys 
import numpy as np
import math
import time

r = 0.0925	##Radio de las ruedas del Pioneer obtenidas en el manual en m
l = 0.1900	##Distancia entre la rueda y el punto con el que se modela el robot en m
J1_1 = np.array([[1,0,l],[1,0,-l],[0,1,0]]) ##Matriz J1 de restriccciones de rodamiento y deslizamiento para un robot diferencial
inv_J1 = np.linalg.inv(J1_1) ##Invesa de la matriz J1
J1 = np.array([[1,0,l],[1,0,-l]])  ##Matriz J1 de restriccciones de rodamiento
J2 = np.array([[r,0],[0,r],[0,0]])  ###Matriz J2 con los radios y la restriccion de deslizamiento
inv_J2 = np.array([[1.0/r,0],[0,1.0/r]]) ###Inversa de la matriz J2 con las restricciones de rodamiento
#ka = 0.99
#kb = 0.5 ##Constantes usadas para el control del robot
ka = 1.2
kb = 0.5
#kp = 0.3
kp = 0.3
pub = rospy.Publisher('/motorsVel', Float32MultiArray, queue_size=10)  ##pub permite publicar las velocidades de cada motor

def arrancar():
	global nombreArchivo, vel_der, vel_izq, tiempo, xfin, yfin, thetafin
	rospy.init_node('punto4', anonymous = True)
	rospy.myargv(argv=sys.argv)
	try:
		xfin = float(sys.argv[1])
		yfin = float(sys.argv[2])
		thetafin = float(sys.argv[3])
	except:    								##Tratamiento de los argumentos ingresador por el usuario
		xfin = 40
		yfin = 40
		thetafin = math.pi/2
	rospy.Subscriber('/pioneerPosition', Twist ,vecto)
	rospy.Subscriber('/simulationTime', Float32, controlTiempo) 
	tasa = rospy.Rate(10)
	try:
		while not rospy.is_shutdown():
			tasa.sleep()		

	except rospy.ServiceException as e:
		pass
		
	
def vecto(data): ##Funcion que manipula la informacion con la posicion del robot
	global posix, posiy, lastheta, vec
	xact = data.linear.x
	yact = data.linear.y
	posix.append(xact)
	posiy.append(yact)  ##Se agregan dichos valores a un vector para mostrarlos en pantalla
	lastheta = data.angular.z  ## Se guarda el ultimo theta obtenido en simulacion
	pub.publish(data = [vec[1],vec[0]])  ##Se publican las velocidades calculadas por la ley de control 

def ThreadInputs():
	with Listener(on_press = keypress) as listener:
		listener.join()

def keypress(key):
	global vel, bandera
	if key == Key.esc:
		bandera = True 	##Se usa la tecla Esc para terminar los diferentes hilos implementados
		return False

def plotPos(): ##Hilo que grafica la informacion solicitada
	global posix, posiy, bandera, posTeox, posTeoy, tiempoSimu, error, fin ##La bandera fin se activa al terminar todos los movimientos dados en el archivo de texto
	while not bandera:
		if fin == True:
			plt.clf()
			plt.plot(tiempoSimu, error)
			plt.ylabel('Magnitud del error')
			plt.xlabel('Tiempo de simulacion')	###Al finalizar el movimiento se grafica el error
			plt.title('Comportamiento del error')
			plt.draw()
			plt.pause(0.8)
		else:
			plt.clf()
			plt.plot(posix,posiy,'p')
			plt.plot(posTeox,posTeoy)
			plt.ylabel('Posicion en y')
			plt.xlabel('Posicion en x')	###Mientras el robot se mueve se muestran en una grafica la posicion teorica y simulada
			plt.title('Posicion del robot')
			plt.draw()
			plt.pause(0.8)
			plt.savefig('src/taller2_6/results/graficaPos4.png')## Se guarda en cada momento la grafica obtenida
	if bandera == True:
		plt.savefig('src/taller2_6/results/graficaError4.png') ### Al presionar esc se guarda la grafica del error y se cierra el plot
		plt.close()
		return False
		
		
def controlTiempo (data):  ##Metodo que recibe la informacion con el tiempo de simulacion
	global posTeox, posTeoy, lastheta, ant, tiempoSimu, error, vec
	## Zero es la variable usada como referencia para calcualar los diferentes tiempos especificados
	## Iterador muestra en que movimiento se encuentra el robot (desde 1 hasta los ingresados por el usuario)
	## ant contiene el tiempo anterior que permite realzar los calculos teoricos para cada delta de tiempo 
	if(ant == -1):
		tiempoSimu.append(data.data)
		ant = data.data 
		errort = math.sqrt((posix[-1]-posTeox[-1])**2 + (posiy[-1]-posTeoy[-1])**2)##Se calcula el error como la distancia entre la prediccion teorica y el resultado arrojado por el simulador
		error.append(errort)
	else:  ## Mientras el robot sigue la ley de control 
		tiempoSimu.append(data.data)
		errort = math.sqrt((posix[-1]-posTeox[-1])**2 + (posiy[-1]-posTeoy[-1])**2)*100
		error.append(errort)
		deltat = data.data - ant	
		ant = data.data
		vecT = invR(lastheta).dot(inv_J1.dot(J2.dot(np.array([vec[0],vec[1]]))))##Se calculan las velocidades teoricas en el marco global a partir de las velocidades de los ruedas
		x = posTeox[-1] + vecT[0]*deltat
		y = posTeoy[-1] + vecT[1]*deltat##Se calcula las nuevas posiciones del robot
		posTeox.append(x)
		posTeoy.append(y)
		
def R(theta): ##funcion que retorna la matriz de rotacion
	return np.array([[math.cos(theta),math.sin(theta),0],[-math.sin(theta),math.cos(theta),0],[0,0,1]])
	
def invR(theta): ##funcion que retorna la inversa de la matriz de rotacion
	return np.array([[math.cos(theta),-math.sin(theta),0],[math.sin(theta),math.cos(theta),0],[0,0,1]])

def control():  ## Metodo que realiza los calculos de la ley de control 
	global posix, posiy, xfin, yfin, thetafin, lastheta, vec, bandera, fin, kb
	while not bandera:
		dy = yfin-posiy[-1]
		dx = xfin-posix[-1]   ##Se calculan los errores en coordenadas cartesiana
		dtheta = lastheta - thetafin
		rho = math.sqrt((dx)**2 + (dy)**2)
		v = kp * rho									##Se calculan los errores en coordenadas esfericas y se calcula la velocidad de acuerdo con kp
		#alpha = -dtheta + math.atan2(dy,dx)
		#beta = -dtheta - alpha
		beta = -math.atan2(dy,dx) + thetafin	#Se calculan los errores en coordenadas esfericas y se calcula la velocidad de acuerdo con kp
		alpha = math.atan2(dy,dx) - lastheta
		if (alpha >= math.pi):
			alpha = alpha - 2*pi
		elif (alpha <= -math.pi):
			alpha = alpha + 2*pi
		if (beta >= math.pi):
			beta = beta - 2*pi
		elif (beta <= -math.pi):
			beta = beta + 2*pi
		if (alpha < -math.pi/2 or alpha > math.pi/2):  ## Si el objetivo no esta frente al robot es necesario moverlo hacia atras
			v = -v
		if(rho < 0.2):
		###Si el robot esta muy cerca del objeivo se detiene y se alinea
			v = 0
			x = v*math.cos(lastheta)
			y = v*math.sin(lastheta)
			w = (0.3*(-dtheta))
			fin = True
		else:		
			x = v*math.cos(dtheta)
			y = v*math.sin(dtheta)
			w = (ka*alpha) + (kb*(beta))
		vec = inv_J2.dot(J1.dot(R(lastheta).dot(np.array([x,y,w])))) ## Usando el modelo cinematico se calcula la velocidad de cada rueda
		time.sleep(0.2)
	return bandera

if __name__ == '__main__':
	global bandera, posix, posiy, posTeox, posTeoy, lastheta, ant, tiempoSimu, error, fin, xfin, yfin, thetafin
	posTeox = []
	posTeoy = []
	posix = []
	posiy = []
	tiempoSimu = []
	error=[]
	vec = [0,0]							##Se les da valor inicial a las variables usadas como globales a lo largo de todo el codigo
	lastheta = -math.pi
	posix.append(0)
	posiy.append(0)
	posTeox.append(0)
	posTeoy.append(0)
	ant = -1
	bandera = False
	fin = False
	xfin = 1
	yfin = 0
	thetafin = math.pi
	try:
		print("Presione una tecla (Esc para salir):")
		threading.Thread(target=ThreadInputs).start()
		threading.Thread(target=plotPos).start()
		threading.Thread(target=control).start()
		arrancar()
	except rospy.ServiceException:
		pass

