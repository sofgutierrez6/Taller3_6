import rospy
from std_msgs.msg import Float32MultiArray, Float32
from pylab import *
from pynput.keyboard import Key, Listener 
import matplotlib.pyplot as plt
import threading
import time
import sys

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

pub = rospy.Publisher('pacmanActions0', actions, queue_size=10)	## Publicador que controla al  pacman con el algoritmo d
def iniciar():
	##Variables globales que permiten el manejo del mapa (matriz), actual * almacenamiento del mapa. Itera es la variable que verifica cuantas veces se ha verificado la direccion de la derecha del pacman
	global matriz, actual, mapa, itera, galletas
	
	##Se inicializa el nodo
	rospy.init_node('punto1Taller3', anonymous = False)
	mapaSolicitado = rospy.ServiceProxy('pacman_world' , mapService)
	mapa = mapaSolicitado("mediumCorners")
	##Inicializacion de la matriz
	dimX=mapa.maxX - mapa.minX + 1
	dimY=mapa.maxY - mapa.minY + 1
	matriz = [[" "  for i in range(dimX)]for j in range(dimY)]
	actual = 2
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

	itera = 0
	tasa = rospy.Rate(10)
	while not rospy.is_shutdown():
		tasa.sleep()
def verifyGoal():
	llego=False
	if():
		llego=True
	return llego


def guardarCookies(data):
	global cookies, data, mapa
	cookies=queue.Queue(data.nCookies) #lista de objetivos (galletas)
	for i in range(data.nCookies):
		cookies[i]=[data.cookiesPos[i].x,data.cookiesPos[i].y]
		matriz[-data.cookiesPos[i].y - mapa.minY][data.cookiesPos[i].x - mapa.minX] = "."

##Metodo convertir matriz en grafo
def grafo():
	global matriz, nodeList, costos, galletas
	##En primer lugar, se crea una lista con los nodos, los cuales corresponden a las esquinas
	nodeList=[] #lista de nodos
	
	for i in range(dimX):
		for j in range(dimY):
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
			galletas.append(Nodo([i, j],True))

	#Teniendo los vertices o nodos, se procede a construir los arcos
	for i in range(len(nodeList)):
		for j in range(i+1,len(nodeList)):
			#notSure
			costos = {[nodeList[i],nodeList[j]] : 0}
			if(nodeList[i].darX == nodeList[j].darX):
				#Revisar
				while k in range(nodeList[i].darY, nodeList[j].darY) and obstaculo==False:
					if(matriz[nodeList[i].darX][nodeList[k].darY+1] == "%")
						obstaculo=True
					else:
						pass
			elif (nodeList[i].darY == nodeList[j].darY):
				while k in range(nodeList[i].darY, nodeList[j].darY) and obstaculo==False:
					if(matriz[nodeList[i].darX][nodeList[k].darY+1] == "%")
						obstaculo=True
					else:
						pass
			if(obstaculo==False):
				nodeList[i].agregarVecinos(nodeList[j])
				nodeList[j].agregarVecinos(nodeList[i])
				#notSure
				costos[nodeList[i],nodeList[j]]=k

##Dijkstra
def dijkstra(data):	
	global matriz,nodeList, came_from, galletas, actual_goal
	#posicion inicial del pacman
	#Se puede crear un conflicto por lo que la pos se va actualizando??**
	start=Nodo(data.pos, False)
	nodeList.append(start)
	#Falta mirar lo de ponerle costo cero... 
	n=len(nodeList)
	nextList=queue.PriorityQueue(n)
	
	nextList.put(,0)
	actual_goal = galletas.pop(0)
	#nextList.put(actual_goal,0)
	
	came_from ={}
	cost_so_far={}
	came_from[actual_goal] = None
	cost_so_far[actual_goal] = 0

	while not nextList.empty()
		actual = 	nextList.get()

		if actual == actual_goal:
			break
		
		for  next in actual.darVecinos:
			newCost = cost_so_far[actual] + costos[actual, next]
			if next not in cost_so_far or newCost < cost_so_far[next]:
				cost_so_far[next] = newCost
				nextList.put(next, newCost)
				came_from[next]=actual
			#Hay que revisar e implementar el movimiento del pacman

def moverPacman():
	global came_from, nextNode, actual_goal
	nextNode=[]
	nextNode[0]= actual_goal
	anterior=nextNode[0]
	i=1
	#Se reconstruye la ruta
	for nodo in came_from:
		anterior = came_from.pop(anterior)
		nextNode[i]=anterior
		i = i+1
	for	j in range(len(nextNode),0,-1):
		sig = nextNode[j]
		act= nextNode[j-1]
		#Revisar movimientos en la matriz
		if(sig.darX==act.darX):
			if(sig.darY>act.darY):
				move= 'right'
			elif(sig.darY<act.darY):
				move= 'left'
			else
				move = 'nada'
		elif (sig.darY==act.darY):
			if(sig.darX>act.darX):
				move= 'up'
			elif(sig.darX<act.darX):
				move= 'down'
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
		iniciar()
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
		




