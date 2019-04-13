import rospy
from std_msgs.msg import Float32MultiArray, Float32
from pylab import *
from pynput.keyboard import Key, Listener 
import matplotlib.pyplot as plt
import threading
import time
import sys
import numpy as np

from pacman.srv import mapService
#Se importan los tipos de Mensajes
from pacman.msg import actions
from pacman.msg import pacmanPos
from pacman.msg import cookiesPos #se importan los mensajes que indican la posicion de las galletas

#Importar colas, pilas y de prioridad
try:
		import Queue as queue
	except ImportError:
		import queue
#Bandera y contador para ver si es la primera vez
primera=True
vez=0
pub = rospy.Publisher('pacmanActions0', actions, queue_size=10)	## Publicador que controla al  pacman con el algoritmo d
def iniciar():
	##Variables globales que permiten el manejo del mapa (matriz), actual * almacenamiento del mapa. Itera es la variable que verifica cuantas veces se ha verificado la direccion de la derecha del pacman
	global matriz, actual, mapa, itera, cookies, primera, vez
	##Se inicializa el nodo
	rospy.init_node('punto1Taller3', anonymous = False)
	#Se solicita el mapa
	mapaSolicitado = rospy.ServiceProxy('pacman_world' , mapService)
	mapa = mapaSolicitado("Harvey")
	##Inicializacion de la matriz
	dimX=mapa.maxX - mapa.minX + 1
	dimY=mapa.maxY - mapa.minY + 1
	matriz = [[" "  for i in range(dimX)]for j in range(dimY)]
	#actual = 2
	#Subscriptions
	rospy.Subscriber("pacmanCoord0", pacmanPos, verifyGoal) # Verificar donde meter DIJKSTRA
	rospy.Subscriber("cookiesCoord", cookiesPos, guardarCookies) #se suscribe el nodo al topico que envia la posicion de las galletas
	##Se guardan en la matriz los obstaculos
	for i in range(mapa.nObs):
		matriz[-mapa.obs[i].y - mapa.minY][mapa.obs[i].x - mapa.minX] = "%"
	##Se guardan en la matriz las galletas
	guardarCookies(data)
	##Metodo convertir matriz en grafo
	grafo()
	vez=vez+1
	if(vez>0):
		primera=False

	tasa = rospy.Rate(10)
	while not rospy.is_shutdown():
		tasa.sleep()

#Verifica si llego a una galleta		
def verifyGoal(data):
	global info, llego, posActX, posActY
	info=data
	global actual_goal, posActX, posActY
	llego=False
	posActX=data.pacmanPos.x
	posActY=data.pacmanPos.y
	if(actual_goal.darX == posActX and actual_goal.darY == posActY):
		llego=True

#Guarda las galletas en una lista
def guardarCookies(data):
	global cookies, data, mapa
	#cookies=queue.Queue(data.nCookies) #lista de objetivos (galletas)
	cookies=[]
	for i in range(data.nCookies):
		#cookies.put(Nodo([data.cookiesPos[i].x,data.cookiesPos[i].y],True))
		cookies.append(Nodo([data.cookiesPos[i].x,data.cookiesPos[i].y],True))
		matriz[-data.cookiesPos[i].y - mapa.minY][data.cookiesPos[i].x - mapa.minX] = "."

##Metodo convertir matriz en grafo
def grafo():
	global matriz, nodeList, costos, cookies
	##En primer lugar, se crea una lista con los nodos, los cuales corresponden a las esquinas
	nodeList=[] #lista de nodos
	
	for i in range(1,dimX):
		for j in range(1,dimY):
			libre=False
			if((matriz[i][j-1]==" ") and (matriz[i-1][j]==" ")):
				libre=True
			if((matriz[i-1][j]==" ") and (matriz[i][j+1]==" ")):
				libre=True
			if((matriz[i][j+1]==" ") and (matriz[i+1][j]==" ")):
				libre=True
			if((matriz[i][j-1]==" ") and (matriz[i+1][j]==" ")):
				libre=True
		#Falta mirar esquinas, revisar, complicarse menos****
		if(libre):
			nodeList.append(Nodo([i, j],False))
			#Si es galleta
		elif (matriz[i][j]== "."):
			nodeList.append(Nodo([i, j],True))
			#galletas.append(Nodo([i, j],True))

	#Teniendo los vertices o nodos, se procede a construir los arcos
	for i in range(len(nodeList)):
		for j in range(i+1,len(nodeList)):
			#medioSure
			x_igual=False
			y_igual=False
			costos = {[nodeList[i],nodeList[j]] : 0}
			if(nodeList[i].darX() == nodeList[j].darX()):
				#Revisar
				while k in range(nodeList[i].darY(), nodeList[j].darY()) and obstaculo==False:
					if(matriz[nodeList[i].darX()][k] == "%")
						obstaculo=True
						y_igual=True
					else:
						pass
			elif (nodeList[i].darY() == nodeList[j].darY()):
				while k in range(nodeList[i].darX(), nodeList[j].darX()) and obstaculo==False:
					if(matriz[k][nodeList[i].darY()] == "%")
						obstaculo=True
						x_igual=True
					else:
						pass
			if(obstaculo==False):
				nodeList[i].agregarVecinos(nodeList[j])
				nodeList[j].agregarVecinos(nodeList[i])
				#AlmostSure
				if(x_igual):
					costo=nodeList[j].darX()-nodeList[i].darX()
					if(costo<0): costo=-costo
					costos[[nodeList[i],nodeList[j]] = costo
				elif(y_igual):
					costo=nodeList[j].darY()-nodeList[i].darY()
					if(costo<0): costo=-costo
					costos[[nodeList[i],nodeList[j]] = costo

##Dijkstra
def dijkstra():	
	global matriz,nodeList, came_from, cookies, actual_goal, llego, posActX, posActY, primera
	#posicion inicial del pacman
	if(primera or llego):
		start=Nodo([posActX, posActY], False)

	nodeList.append(start)
	 #Tamanio de la lista de nodos
	n=len(nodeList)
	nextList=queue.PriorityQueue(n)
	nextList.put((0,start))
	actual_goal = cookies.pop(0)

	came_from ={}
	cost_so_far={}
	came_from[actual_goal] = None
	cost_so_far[actual_goal] = 0

	while not nextList.empty()
		actual = 	nextList.get()

		if actual == actual_goal:
			break
		
		for  next in actual.darVecinos:
			newCost = cost_so_far[actual] + costos[[actual, next]]
			if next not in cost_so_far or newCost < cost_so_far[next]:
				cost_so_far[next] = newCost
				nextList.put((newCost,next))
				came_from[next]=actual

def moverPacman():
	global came_from, nextNode, actual_goal
	#Lista para reconstruir la ruta
	nextNode=[]
	nextNode[0]= actual_goal
	anterior=nextNode[0]
	#Se reconstruye la ruta
	for nodo in came_from:
		anterior = came_from.pop(anterior)
		nextNode.append(anterior)
	for	j in range(len(nextNode)-1,0,-1):
		sig = nextNode[j-1]
		act= nextNode[j]
		#Revisar movimientos en la matriz
		if(sig.darX()==act.darX()):
			if(sig.darY() > act.darY()):
				move= 'down'
			elif(sig.darY() < act.darY()):
				move= 'up'
			else
				move = 'nada'
		elif (sig.darY() == act.darY()):
			if(sig.darX() > act.darX()):
				move= 'right'
			elif(sig.darX() < act.darX()):
				move= 'left'
			else
				move = 'nada'
		else
			move = 'nada'

	mensajes = {'left':3, 'up':0, 'down':1, 'right':2, 'nada':4}
	action = int(mensajes[move])
	pub.publish(action)

## Main
if __name__ == '__main__':
	try:
		global cookies
		iniciar()
		while not len(cookies)==0:
			dijkstra()
			moverPacman()
	except rospy.ServiceException:
		pass	
	
	

class Nodo: 
    def __init__(self, pos, cookie):
		self.pos = pos
		self.cookie = cookie
		self.vecinos = []

		self.objetivo = False
		self.visitado = False
		self.padre = None
		
		def darX():
			return self.pos[0]
		
		def darY():
			return self.pos[1]
		
		def darVecinos():
			return vecinos

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
		




