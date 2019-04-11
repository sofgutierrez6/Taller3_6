#!/usr/bin/python
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray, Float32
from pylab import *
from pynput.keyboard import Key, Listener 
from geometry_msgs.msg import Twist
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
	global xfin, yfin, thetafin
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
	time.sleep(1)
	crearCuadricula()
	threading.Thread(target=control).start()
	try:
		while not rospy.is_shutdown():
			tasa.sleep()
			publicar()
	except rospy.ServiceException as e:
		pass
		
def obstacles(data):	
	global obs
	obs = data.data
	
def publicar():
	global vec
	pub.publish(data = [vec[1],vec[0]])

def crearCuadricula():
	global obs, matriz, matNod, xplot, yplot
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
	xplot = list(map(lambda x: -5 + (0.25*x), x))
	yplot = list(map(lambda x: -5 + (0.25*x), y))

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
		
		
def Astar():
	global bandera, xfin, yfin, posix, posiy, matNod
	for fil in matNod:
		for nod in fil:
			nod.esActual(False)
	pos_f = [xfin,yfin]
	goal = buscarNodo(xfin,yfin)
	goal.esObjetivo(True)
	explorados = []
	actual = buscarNodo(posix[-1], posiy[-1])
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
	rutax = []
	rutay = []
	while actual.padre != None:
		coord2 = actual.coord
		rutax.append(coord2[0])
		rutay.append(coord2[1])
		actual = actual.padre
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

def control():
	global posix, posiy, lastheta, primero, vec, xfin, yfin, thetafin, bandera
	rho = 10
	beta = 20
	x_vec,y_vec = Astar()
	x_vec.reverse()
	y_vec.reverse()
	for i in range(len(x_vec)):
		while rho >= 0.08:
			dx = x_vec[i] - posix[-1]
			dy = y_vec[i] - posiy[-1]
			if (x_vec[i] == x_vec[-2] and y_vec[i] == y_vec[-2]):
				dtheta = lastheta - thetafin
			elif (i != len(x_vec)-1):
				dtheta = lastheta - math.atan2(y_vec[i+1]-y_vec[i],x_vec[i+1]-x_vec[i])
			else:
				dtheta = lastheta - thetafin
			rho = math.sqrt((dx)**2 + (dy)**2)
			alpha = -lastheta + math.atan2(dy,dx)	#Se calculan los errores en coordenadas esfericas y se calcula la velocidad de acuerdo con kp
			beta = -math.atan2(dy,dx) - dtheta
			if (alpha >= 2*math.pi):
				alpha = alpha - 2*pi
			elif (alpha <= -2*math.pi):
				alpha = alpha + 2*pi
			if (beta >= 2*math.pi):
				beta = beta - 2*pi
			elif (beta <= -2*math.pi):
				beta = beta + 2*pi
			if (primero):
				'''if (alpha < -math.pi/2 or alpha > math.pi/2):  ## Si el objetivo no esta frente al robot es necesario moverlo hacia atras
					kb = -0.06
					kp = -0.3
					ka = -0.2
				else:
					kb = 0.1
					kp = 0.4
					ka = 1.3'''
				primero = False
			kb = 0.07
			kp = 0.6
			ka = 1.8
			v = kp * rho
			x = v*math.cos(lastheta)
			y = v*math.sin(lastheta)
			w = (ka*alpha) + (kb*(beta))
			vec = inv_J2.dot(J1.dot(R(lastheta).dot(np.array([x,y,w]))))
			time.sleep(0.2)
			if bandera:
				return False
		rho = 10
		beta = 20
		primero = True
	beta = 0.5
	while abs(beta) >= 0.01:
		kb = 0.3
		beta = -lastheta + thetafin
		if (beta >= 2*math.pi):
				beta = beta - 2*pi
		elif (beta <= -2*math.pi): 
			beta = beta + 2*pi
		w =(kb*(beta))
		x = 0
		y = 0
		vec = inv_J2.dot(J1.dot(R(lastheta).dot(np.array([x,y,w]))))
		time.sleep(0.2)
		if bandera:
			return False
	bandera = True
	vec = [0,0]
	return False


def vecto(data): ##Funcion que manipula la informacion con la posicion del robot
	global posix, posiy, lastheta, vec
	xact = data.linear.x
	yact = data.linear.y
	posix.append(xact)
	posiy.append(yact)  ##Se agregan dichos valores a un vector para mostrarlos en pantalla
	lastheta = data.angular.z  ## Se guarda el ultimo theta obtenido en simulacion 

def plotPos():
	global posix, posiy, xplot, yplot
	while True:
		plt.clf()
		plt.plot(posix,posiy)
		plt.plot(xplot,yplot,'p')
		plt.ylabel('Posicion en y')
		plt.xlabel('Posicion en x')	###Mientras el robot se mueve se muestran en una grafica la posicion simulada
		plt.title('Posicion del robot')
		plt.draw()
		if bandera:
			plt.savefig('src/Taller3_6/taller3_6/results/graficaPunto2c.png')
			plt.close()
			return False
		plt.pause(0.5)

def keypress(key):
	global bandera
	if key == Key.esc:
		bandera = True
		print "fin" 	##Se usa la tecla Esc para terminar los diferentes hilos implementados
		return False
		
def ThreadInputs():
	with Listener(on_press = keypress) as listener:
		listener.join()

if __name__ == '__main__':	
	global obs, bandera, primero, posix, posiy, lastheta, primero, vec, xfin, yfin, thetafin, matriz, matNod, xplot, yplot
	obs = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
	posix = [0]
	posiy = [0]
	xplot = [0]
	yplot = [0]
	vec = [0,0]
	matriz = [[0  for i in range(200)]for j in range(200)]
	matNod = [[Nodo([i , j]) for i in range(200)]for j in range(200)]
	bandera = False
	primero = True
	xfin = 0
	yfin = 0
	lastheta = 0
	thetafin = 0
	try:
		threading.Thread(target=plotPos).start()
		threading.Thread(target=ThreadInputs).start()
		arrancar()
	except rospy.ServiceException:
		pass
