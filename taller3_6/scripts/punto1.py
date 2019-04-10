import rospy
from std_msgs.msg import Float32MultiArray, Float32
from pylab import *
from pynput.keyboard import Key, Listener 
import matplotlib.pyplot as plt
import threading
import time
import sys
from pacman.msg import actions
from pacman.srv import mapService
from pacman.msg import pacmanPos

pub = rospy.Publisher('pacmanActions0', actions, queue_size=10)	## Publicador que controla al  pacman con el algoritmo d
def iniciar():
	##Variables globales que permiten el manejo del mapa (matriz), actual * almacenamiento del mapa. Itera es la variable que verifica cuantas veces se ha verificado la direccion de la derecha del pacman
	global matriz, actual, mapa, itera
	
	##Se inicializa el nodo
	rospy.init_node('punto1Taller3', anonymous = False)
	mapaSolicitado = rospy.ServiceProxy('pacman_world' , mapService)
	mapa = mapaSolicitado("mediumCorners")
	##Inicializacion de la matriz
	dimX=mapa.maxX - mapa.minX + 1
	dimY=mapa.maxY - mapa.minY + 1
	matriz = [[" "  for i in range(dimX)]for j in range(dimY)]
	actual = 2
	rospy.Subscriber("pacmanCoord0", pacmanPos, Dijkstra) # CUANDO HAGAMOS DIJKSTRA
	##Se guardan en la matriz los obstaculos
	for i in range(mapa.nObs):
		matriz[-mapa.obs[i].y - mapa.minY][mapa.obs[i].x - mapa.minX] = "%"
	##Se guardan en la matriz las galletas
	for i in range(data.nCookies):
		matriz[-data.cookiesPos[i].y - mapa.minY][data.cookiesPos[i].x - mapa.minX] = "."
	##En primer lugar, se crea una lista con los nodos, los cuales corresponden a las esquinas
	#lista de nodos
	nodeList=[]
	for i in range(dimX):
		for j in range(dimY):
			numLibres=0
			if(matriz[i][j+1]==" "):
				numLibres=numLibres+1
			if(matriz[i][j-1]==" "):
				numLibres=numLibres+1
			if(matriz[i+1][j]==" "):
				numLibres=numLibres+1
			if(matriz[i-1][j]==" "):
				numLibres=numLibres+1
		#Falta mirar esquinas**
		if(numLibres>=2):
			nodeList.append(Nodo([i , j]),False)
		elif (matriz[i][j]== "."):
			nodeList.append(Nodo([i , j]),True)
	#Teniendo los vertices o nodos, se procede a construir los arcos
	for i in range(len(nodeList)):
		for j in range(i+1,len(nodeList)):
			if(nodeList[i].darX == nodeList[j].darX):
				#Revisar
				for k in range(nodeList[i].darY, nodeList[j].darY):
					if(matriz[nodeList[i].darX][nodeList[k].darY+1] == "%")
						break
					else:
						pass
			elif (nodeList[i].darY == nodeList[j].darY):
				for k in range(nodeList[i].darY, nodeList[j].darY):
					if(matriz[nodeList[i].darX][nodeList[k].darY+1] == "%")
						break
					else:
						pass



	##Metodo convertir matriz en grafo
	grafo()

	itera = 0
	tasa = rospy.Rate(10)
	while not rospy.is_shutdown():
		tasa.sleep()
def grafo():
	global matriz, graf



class nodo: 
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
		
try:
	import Queue as queue
except ImportError:
	import queue
n=10
nextList=queue.PriorityQueue(n)



