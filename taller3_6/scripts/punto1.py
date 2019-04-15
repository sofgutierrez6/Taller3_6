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
	##Variables globaales que permiten el manejo del mapa (matriz), actual * almacenamiento del mapa. Itera es la variable que verifica cuantas veces se ha verificado la direccion de la derecha del pacman
	global matriz, actual, mapa, itera
	
	##Se inicializa el nodo
	rospy.init_node('punto1Taller3', anonymous = False)
	mapaSolicitado = rospy.ServiceProxy('pacman_world' , mapService)
	mapa = mapaSolicitado("mediumCorners")
	##Inicializacion de la matriz
	matriz = [[" "  for i in range(mapa.maxX - mapa.minX + 1)]for j in range(mapa.maxY - mapa.minY + 1)]
	actual = 2
	rospy.Subscriber("pacmanCoord0", pacmanPos, Dijkstra) # CUANDO HAGAMOS DIJKSTRA
	##Se guardan en la matriz los obstaculos
	for i in range(mapa.nObs):
		matriz[-mapa.obs[i].y - mapa.minY][mapa.obs[i].x - mapa.minX] = "%"
	itera = 0
	tasa = rospy.Rate(10)
	while not rospy.is_shutdown():
		tasa.sleep()

class nodo: 
    def __init__(self, pos):
		self.pos = pos
		self.coord = [-5 + (0.25*pos[0]) , -5 + (0.25*pos[1])]
		self.costo = 1000000000
		self.vecinos = []

		self.objetivo = False
		self.visitado = False
		self.padre = None
		
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