#!/usr/bin/python

#Importaciones
import rospy
from std_msgs.msg import Float32MultiArray, Float32
import time
import sys
import numpy as np
#Se importan el mapa
from pacman.srv import mapService
#Se importan los tipos de Mensajes
from pacman.msg import actions #Se importan los mensajes que indican la direccion a moverse del pacman
from pacman.msg import pacmanPos #se importan los mensajes que indican la posicion del pacman
from pacman.msg import cookiesPos #se importan los mensajes que indican la posicion de las galletas

#Se importan colas, pilas y de prioridad
try:
		import Queue as queue
except ImportError:
		import queue

## Publicador que controla al  pacman con el algoritmo de Dijkstra
pub = rospy.Publisher('pacmanActions0', actions, queue_size=10)	

def iniciar():
	##Variables globales que permiten el manejo del mapa en una matriz, como tambien el acceso a sus dimensiones
	#actual_goal representa el nodo donde esta la galleta a la cual se quiere llegar
	#cookies la lista de galletas
	#nodeList la lista de nodos basados en el mapa/matriz
	global matriz, mapa, cookies, dimX, dimY, actual_goal, cookies, nodeList
	
	##Se inicializa el nodo
	rospy.init_node('punto1Taller3', anonymous = False)

	#Se solicita el mapa
	mapaSolicitado = rospy.ServiceProxy('pacman_world' , mapService)
	mapa = mapaSolicitado("Harvey")
	
	##Inicializacion de la matriz
	dimX = mapa.maxX - mapa.minX + 1
	dimY = mapa.maxY - mapa.minY + 1
	#n es el numero de filas
	#m es el numero de columnas
	matriz = [[" "  for i in range(dimX)]for j in range(dimY)]
	nodeList = [[Nodo([i,j]) for i in range (dimX)] for j in range (dimY)]
	#actual = 2
	#Subscriptions
	rospy.Subscriber("pacmanCoord0", pacmanPos, verifyGoal) 
	rospy.Subscriber("cookiesCoord", cookiesPos, guardarCookies)
	time.sleep(0.2)
	#se suscribe el nodo al topico que envia la posicion de las galletas
	##Se guardan en la matriz los obstaculos
	for i in range(mapa.nObs):
		matriz[-mapa.obs[i].y - mapa.minY][mapa.obs[i].x - mapa.minX] = "%"
	#cookies.append(Nodo([data.cookiesPos[0].x,data.cookiesPos[0].y],True))
	#actual_goal=cookies.pop(0)
	grafo()
	tasa = rospy.Rate(10)
	while not rospy.is_shutdown():
		if (len(cookies) != 0):
			dijkstra()
			moverPacman()
		tasa.sleep()

#Verifica si llego a una galleta		
def verifyGoal(data):
	global llego, posActX, posActY,  actual_goal, mapa
	llego = False
	posActX = data.pacmanPos.x - mapa.minX
	posActY = -data.pacmanPos.y - mapa.minY
	if(actual_goal.darY() == posActX and actual_goal.darX() == posActY):
		llego = True

#Guarda las galletas en una lista
def guardarCookies(data):
	global cookies, mapa, matriz, cookiesYa
	#cookies=queue.Queue(data.nCookies) #lista de objetivos (galletas)
	if (cookiesYa):
		for i in range(data.nCookies):
			#cookies.put(Nodo([data.cookiesPos[i].x,data.cookiesPos[i].y],True))
			cookies.append(Nodo([-data.cookiesPos[i].y - mapa.minY , data.cookiesPos[i].x - mapa.minX]))
			print data.cookiesPos[i].x - mapa.minX, -data.cookiesPos[i].y - mapa.minY , 
			matriz[-data.cookiesPos[i].y - mapa.minY][data.cookiesPos[i].x - mapa.minX] = "."
			cookiesYa = False

##Metodo convertir matriz en grafo
def grafo():
	global matriz, nodeList, costos, cookies, dimX, dimY
	for i in range (dimX):
		for j in range (dimY):
			nodoActual = nodeList[j][i]
			if (matriz[j][i] == " " or matriz[j][i] == "."):
				if (j+1 < dimY and (matriz[j+1][i] == " " or matriz[j+1][i] == ".")):
					nodoActual.agregarVecinos(nodeList[j+1][i])
				if (i+1 < dimX and (matriz[j][i+1] == " " or matriz[j][i+1] == ".")):
					nodoActual.agregarVecinos(nodeList[j][i+1])
				if (i-1 > 0 and (matriz[j][i-1] == " " or matriz[j][i-1] == ".")):
					nodoActual.agregarVecinos(nodeList[j][i-1])
				if (j-1 > 0 and (matriz[j-1][i] == " " or matriz[j-1][i] == ".")):
					nodoActual.agregarVecinos(nodeList[j-1][i])

##Dijkstra
def dijkstra():	
	global matriz ,nodeList, came_from, cookies, actual_goal, llego, posActX, posActY, primera, dimX, dimY, actual
	#posicion inicial del pacman
	if (llego or primera):
		primera = False
		actual = buscarNodo(posActX,posActY)
		#Tamanio de la lista de nodos
		nextList = queue.PriorityQueue(dimX*dimY)
		nextList.put((0,actual))
		actual_goal = masCercana()
		came_from = {}
		cost_so_far = {}
		came_from[actual] = None
		cost_so_far[actual] = 0

		while not nextList.empty():
			tupla = nextList.get()
			actual = tupla[1]
			if (actual.darY() == actual_goal.darX() and actual.darX() == actual_goal.darY()):
				break
			for next in actual.vecinos:
				newCost = cost_so_far[actual] + 1
				if next not in cost_so_far or newCost < cost_so_far[next]:
					cost_so_far[next] = newCost
					nextList.put((newCost,next))
					came_from[next] = actual
	
def buscarNodo(x,y):
	global mapa 
	return nodeList[y][x]

def masCercana():
	global posActX, posActY, cookies, dimX, dimY
	dist = dimX + dimY
	pos = 0
	for i in range(len(cookies)):
		Ndist =  abs(posActY - cookies[i].darX())+ abs(posActX - cookies[i].darY())
		if (Ndist < dist):
			pos = i 
			dist = Ndist
	return cookies.pop(pos)


def moverPacman():
	global came_from, actual, posActX, posActY
	#Lista para reconstruir la ruta
	nextNode = []
	nextNode.append(actual)
	#Se reconstruye la ruta
	print '_______Nuevo objetivo_______'
	print actual.darX(), actual.darY()
	print actual_goal.darX(), actual_goal.darY()
	while came_from[actual] != None:
		actual = nextNode[-1]
		anterior = came_from[actual]
		if (anterior != None):
			print actual.darX(), actual.darY()
			nextNode.append(anterior)
	move = 'nada'
	nextNode.reverse()
	for	j in range(len(nextNode)-1):
		sig = nextNode[j+1]
		act = nextNode[j]
		while (posActX != sig.darX() or posActY != sig.darY()):
			#Revisar movimientos en la matriz
			if(sig.darX() == act.darX()):
				if(sig.darY() > act.darY()):
					move = 'down'
				elif(sig.darY() < act.darY()):
					move = 'up'
				else:
					move = 'nada'
			elif (sig.darY() == act.darY()):
				if(sig.darX() > act.darX()):
					move = 'right'
				elif(sig.darX() < act.darX()):
					move = 'left'
				else:
					move = 'nada'
			else:
				move = 'nada'
			mensajes = {'left':3, 'up':0, 'down':1, 'right':2, 'nada':4}
			action = int(mensajes[move])
			pub.publish(action)
	mensajes = {'left':3, 'up':0, 'down':1, 'right':2, 'nada':4}
	action = int(mensajes[move])
	pub.publish(action)

class Nodo: 
	def __init__(self, pos):
		self.pos = pos
		self.cookie = False
		self.vecinos = []
		self.visitado = False
		self.padre = None
		
	def darX(self):
		return self.pos[0]
		
	def darY(self):
		return self.pos[1]
		
	def darVecinos(self):
		return self.vecinos

	def asignarPadre(self, padre):
		self.padre = padre

	def cambiarCosto(self, costo):
		self.costo = costo

	def agregarVecinos(self, vecino):
		self.vecinos.append(vecino)

	def esActual(self, visitado):
		self.visitado = visitado
        
	def esCookie(self, cookie):
		self.cookie = cookie 
## Main
if __name__ == '__main__':
	try:
		global cookies, actual_goal, primera, cookiesYa
		actual_goal  = Nodo([-1,-1])
		primera = True
		cookiesYa = True
		cookies = []
		iniciar()		
	except rospy.ServiceException:
		pass	