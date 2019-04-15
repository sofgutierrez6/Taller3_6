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
	#nodeList la matriz de nodos basados en el mapa
	global matriz, mapa, cookies, dimX, dimY, actual_goal, cookies, nodeList
	
	##Se inicializa el nodo
	rospy.init_node('punto1Taller3', anonymous = False)

	#Se solicita el mapa
	mapaSolicitado = rospy.ServiceProxy('pacman_world' , mapService)
	mapa = mapaSolicitado("Harvey")
	
	##Inicializacion de la matriz
	dimX = mapa.maxX - mapa.minX + 1
	dimY = mapa.maxY - mapa.minY + 1
	matriz = [[" "  for i in range(dimX)]for j in range(dimY)]
	
	#Matriz con los nodos
	nodeList = [[Nodo([i,j]) for i in range (dimX)] for j in range (dimY)]
	
	#Subscriptions
	rospy.Subscriber("pacmanCoord0", pacmanPos, verifyGoal) 
	rospy.Subscriber("cookiesCoord", cookiesPos, guardarCookies)
	time.sleep(0.2)
	
	##Se guardan en la matriz los obstaculos
	for i in range(mapa.nObs):
		matriz[-mapa.obs[i].y - mapa.minY][mapa.obs[i].x - mapa.minX] = "%"
	
	#Metodo para generar el grafo
	grafo()
	
	#Ciclo logico de ejecucion del nodo 
	tasa = rospy.Rate(10)
	while not rospy.is_shutdown():
		if (len(cookies) != 0):
			#En donde se calcula la ruta por medio del metodo dijkstra
			dijkstra()
			#Posteriormente se realizan los movientos del pacamn con base a la ruta calculada en el metodo anterior
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
			#print data.cookiesPos[i].x - mapa.minX, -data.cookiesPos[i].y - mapa.minY , 
			#Tambien se guardan en la matriz
			matriz[-data.cookiesPos[i].y - mapa.minY][data.cookiesPos[i].x - mapa.minX] = "."
			#cookiesYa es una bandera que permite cargarsolamente una vez las galletas
			cookiesYa = True

##Metodo convertir matriz en grafo
def grafo():
	global matriz, nodeList, costos, cookies, dimX, dimY
	for i in range (dimX):
		for j in range (dimY):
			nodoActual = nodeList[j][i]
			#Se verifica si es un espacio libre o una galleta
			if (matriz[j][i] == " " or matriz[j][i] == "."):
				#Se verifican si los nodos adyacentes estan libres o son una galleta, en dicho caso es una casilla factible por lo tanto se agregan a los vecino
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
	#Se verifica si se llego al objetivo o si es la primera vez que se corre ya que si es la primera el nodo corresponde a la posicion inicial y no ala posicion de una galleta
	if (llego or primera):
		primera = False
		#Se actualiza el nodoa ctual
		actual = buscarNodo(posActX,posActY)
		#Se inicializa la cola de prioridad
		nextList = queue.PriorityQueue(dimX*dimY)
		#Se agrega el nodo actual a la cola de prioridad
		nextList.put((0,actual))
		#El objetivo sera la galleta mas cercana
		actual_goal = masCercana()
		#Diccionario con entrada nodo que devulve el nodo de donde vino
		came_from = {}
		#Diccionario con los costos acomulados
		cost_so_far = {}
		#acutal alse elprimer nodo tienen costo cero y no vienen de ninguno
		came_from[actual] = None
		cost_so_far[actual] = 0
		
		#Se recorren los nodos
		while not nextList.empty():
			#Se obtienen el siguiente nodo y su costo y se almacenan en la variable tupla
			tupla = nextList.get()
			#Se obtiene el siguiente nodo
			actual = tupla[1]
			#Se verifica si ya llego al objetivo, en caso de que si se termina el metodo
			if (actual.darY() == actual_goal.darX() and actual.darX() == actual_goal.darY()):
				break
			#Se exploran los vecinos
			for next in actual.vecinos:
				#Se actualiza el costo acomulado hasta el nodo
				newCost = cost_so_far[actual] + 1
				#Si hay un costo menor se pone como siguiente nodo, y se agrega al diccionario de donde vino
				if next not in cost_so_far or newCost < cost_so_far[next]:
					#Se acualiza el costo acomulado
					cost_so_far[next] = newCost
					#Se pone en la cola de prioridad de siguientes
					nextList.put((newCost,next))
					#Se pone de donde vino
					came_from[next] = actual
					
#Metodo que dependiendo las cordenadas devuelve el nodo buscado	
def buscarNodo(x,y):
	global mapa 
	return nodeList[y][x]

#Metodo que devuelve la galleta mas cercana
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

#Metodo que calcula y publica los movimientos del pacman
def moverPacman():
	global came_from, actual, posActX, posActY
	#Lista para reconstruir la ruta
	nextNode = []
	#Se agrega el nodo actual 
	nextNode.append(actual)
	
	#print '_______Nuevo objetivo_______'
	#print actual.darX(), actual.darY()
	#print actual_goal.darX(), actual_goal.darY()
	
	#Se reconstruye la ruta
	while came_from[actual] != None:
		#Actual es el ultimo nodo
		actual = nextNode[-1]
		#Se guarda de donde viene actual
		anterior = came_from[actual]
		if (anterior != None):
			#print actual.darX(), actual.darY()
			#Se agrega a la lista de siguientes
			nextNode.append(anterior)
			
	#Diccionario con los numeros a publicar para el movimiento del pacman
	mensajes = {'left':3, 'up':0, 'down':1, 'right':2, 'nada':4}
	#Movimiento a publicar
	move = 'nada'
	#Cmo la lista se construyo al contrario es necesario invertir el orden
	nextNode.reverse()
	#Se recorre la lista de nodos siguientes
	for	j in range(len(nextNode)-1):
		sig = nextNode[j+1]
		act = nextNode[j]
		#Si las posiciones act y sig son iguales significa que llego
		while (posActX != sig.darX() or posActY != sig.darY()):
			#Se realiza la logica de movimientos dependiendo de las coordenadas
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
			#Se obtiene el numero a publicar
			action = int(mensajes[move])
			#Se publica
			pub.publish(action)

	action = int(mensajes[move])
	pub.publish(action)
	
	
##Clase interna nodo
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
		#Se inicializan las variables
		actual_goal  = Nodo([-1,-1])
		primera = True
		cookiesYa = True
		cookies = []
		#Se inicia el algoritmo
		iniciar()		
	except rospy.ServiceException:
		pass	
