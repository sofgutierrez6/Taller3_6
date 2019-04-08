#!/usr/bin/python
import rospy
from std_msgs.msg import Float32MultiArray, Float32
from pylab import *
from pynput.keyboard import Key, Listener 
import matplotlib.pyplot as plt
import threading
import time
import sys 

robot = 2
r = 0.0925	##Radio de las ruedas del Pioneer obtenidas en el manual en m
l = 0.1900	##Distancia entre la rueda y el punto con el que se modela el robot en m
J1 = np.array([[1,0,l],[1,0,-l]]) 
inv_J2 = np.array([[1.0/r,0],[0,1.0/r]])
pub = rospy.Publisher('/motorsVel', Float32MultiArray, queue_size=10) 

def arrancar():
	rospy.init_node('punto2c', anonymous = True)
	rospy.Subscriber('/obstacles', Float32MultiArray ,obstacles)
	rospy.Subscriber('/pioneerPosition', Twist ,vecto)
	tasa = rospy.Rate(10)
	rospy.myargv(argv=sys.argv)
	try:
		xfin = float(sys.argv[1])
		yfin = float(sys.argv[2])
		thetafin = float(sys.argv[3])
	except:    								##Tratamiento de los argumentos ingresador por el usuario
		xfin = 40
		yfin = 40
		thetafin = math.pi/2
	crearCuadricula()
	control(xfin,yfin,thetafin)
	try:
		while not rospy.is_shutdown():
			tasa.sleep()
	except rospy.ServiceException as e:
		pass
		
def obstacles(data):
	global obs
	obs = data.data
	
def crearCuadricula():
	global obs, matriz, matNod
	matriz = [[0  for i in range(200)]for j in range(200)]
	matNod = [[Nodo([i , j]) for i in range(200)]for j in range(200)]
	x = [(5 + obs[0])/0.25 , (5 + obs[1])/0.25 , (5 + obs[2])/0.25 , (5 + obs[3])/0.25 , (5 + obs[4])/0.25]
	y = [(5 + obs[5])/0.25 , (5 + obs[6])/0.25 , (5 + obs[7])/0.25 , (5 + obs[8])/0.25 , (5 + obs[9])/0.25]
	r = [obs[10]/0.25 , obs[11]/0.25 , obs[12]/0.25 , obs[13]/0.25 , obs[14]/0.25]
	for i in range(len(x)):
		matriz[int(x[i])][int(y[i])] = 1
		for j in range(int(x[i]-r[i]-robot)-1, int(x[i]+r[i]+robot+1)):
			for k in range(int(y[i]-r[i]-robot)-1, int(y[i]+r[i]+robot+1)):
				dist = np.linalg.norm(np.array([x[i],y[i]]) - np.array([j+0.5,k+0.5]))
				if dist <= r[i]+robot:
					matriz[j][k] = 1
					x.append(j)
					y.append(k)
	for nod in matNod:
		for nod2 in nod:
			posN = nod2.pos
			if (matriz[posN[0]][posN[1]] == 0):
				for indice in range(posN[0] - 1, posN[0] + 2 ,1):
					for ja in range(posN[1] - 1, posN[1] + 2 ,1):
						if (indice >= 0 and indice <= 199 and ja >= 0 and ja <= 199):
							if (matriz[ja][indice] == 0 and not ((abs(indice - posN[0]) + abs(ja - posN[1])) == 0)):
								nod2.agregarVecinos(matNod[ja][indice])
	if (bandera):
		return False
				
		
def ThreadInputs():
	with Listener(on_press = keypress) as listener:
		listener.join()
		
def keypress(key):
	global bandera
	while True:
		if key == Key.esc:
			bandera = True 	##Se usa la tecla Esc para terminar los diferentes hilos implementados
			return False

class Nodo:
	def __init__(self, pos):
		self.pos = pos
		self.coord = [-5 + (0.25*pos[0]) , -5 + (0.25*pos[1])]
		self.costo = 1000000000
		self.vecinos = []

		self.objetivo = False
		self.visitado = False
		self.padre = None
		self.h = 100000000
		
	def defHeu(self, pos_f):
		self.h = math.sqrt((self.coord[0]-pos_f[0])**2 + (self.coord[1]-pos_f[1])**2) 
		return self.h
		
	def asignarPadre(self, padre):
		self.padre = padre
	
	def cambiarCosto(self, costo):
		self.costo = costo
	
	def agregarVecinos(self, vecino):
		self.vecinos.append(vecino)
		
	def esActual(self, visitado):
		self.visitado = visitado
		
	def esObjetivo(self, objetivo):
		self.objetivo = objetivo
		
		
def Astar(xfin,yfin):
	global bandera
	pos_f = [xfin,yfin]
	goal = buscarNodo(xfin,yfin)
	goal.esObjetivo(True)
	explorados = []
	actual = buscarNodo(0,0)
	actual.asignarPadre(None)
	explorados.append(actual)
	actual.cambiarCosto(0)
	actual.esActual(True)
	costos = {actual : 0}
	rutax = []
	rutay = []
	while not len(explorados) == 0 and not bandera:
		actual = buscarMejor(explorados)
		explorados.remove(actual)
		actual.esActual(True)
		if actual.objetivo:
			break
		vecinos = actual.vecinos
		for vecino in vecinos:
			costo = costos[actual] + 0.25
			if (vecino not in explorados and not vecino.visitado) or costo < costos[vecino]:
				costos[vecino] = costo
				vecino.cambiarCosto(costo + vecino.defHeu(pos_f))
				vecino.asignarPadre(actual)
				explorados.append(vecino)
	
	x = []
	y  = []
	rutax = []
	rutay = []
	while actual.padre != None:
		coord2 = actual.coord
		rutax.append(coord2[0])
		rutay.append(coord2[1])
		actual = actual.padre
	for vecino in explorados:
		x.append(vecino.coord[0])
		y.append(vecino.coord[1])
	return rutax, rutay

def R(theta): ##funcion que retorna la matriz de rotacion
	return np.array([[math.cos(theta),math.sin(theta),0],[-math.sin(theta),math.cos(theta),0],[0,0,1]])

def buscarNodo(x,y):
	global matNod
	a = int(round((5 + x)/0.25))
	b = int(round((5 + y)/0.25))
	return matNod[a][b]
				
def buscarMejor(nodos):
	cost = 10000000
	best = None
	for nod in nodos:
		if(nod.costo < cost):
			cost = nod.costo
			best = nod
	return best

def control(xfin, yfin, thethafin):
	global posix, posiy, lastheta
	x,y = Astar(xfin,yfin)
	x = x.reverse()
	y = y.reverse()
	rho = 10
	for i in range(len(x)):
		while rho >= 0.1:
			dx = x[i] - posix[-1]
			dy = y[i] - posiy[-1]
			if (x[i] == x[-2] and y[i] == y[-2]):
				dtheta = lastheta - thetafin
			else:
				dtheta = math.atan2(y[i+1]-y[i],x[i+1]-x[i]) - thetafin
			rho = math.sqrt((dx)**2 + (dy)**2)
			alpha = -dtheta + math.atan2(dy,dx)	#Se calculan los errores en coordenadas esfericas y se calcula la velocidad de acuerdo con kp
			beta = -dtheta - alpha
			if (alpha >= math.pi):
				alpha = alpha - 2*pi
			elif (alpha <= -math.pi):
				alpha = alpha + 2*pi
			if (beta >= math.pi):
				beta = beta - 2*pi
			elif (beta <= -math.pi):
				beta = beta + 2*pi


def vecto(data): ##Funcion que manipula la informacion con la posicion del robot
	global posix, posiy, lastheta, vec
	xact = data.linear.x
	yact = data.linear.y
	posix.append(xact)
	posiy.append(yact)  ##Se agregan dichos valores a un vector para mostrarlos en pantalla
	lastheta = data.angular.z  ## Se guarda el ultimo theta obtenido en simulacion
	pub.publish(data = [vec[1],vec[0]])  ##Se publican las velocidades calculadas por la ley de control 

if __name__ == '__main__':
	global obs, bandera
	obs = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
	bandera = False
	try:
		threading.Thread(target=ThreadInputs).start()
		arrancar()
	except rospy.ServiceException:
		pass