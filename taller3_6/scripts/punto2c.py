#!/usr/bin/python
import rospy
from std_msgs.msg import Float32MultiArray, Float32
from pylab import *
from pynput.keyboard import Key, Listener 
import threading
import time

robot = 0

def arrancar():
	rospy.init_node('punto2b', anonymous = True)
	rospy.Subscriber('/obstacles', Float32MultiArray ,obstacles)
	tasa = rospy.Rate(10)
	try:
		while not rospy.is_shutdown():
			tasa.sleep()
	except rospy.ServiceException as e:
		pass
		
def obstacles(data):
	global obs
	obs = data.data
	
def crearCuadricula():
	global obs, matriz
	while True:
		matriz = [[0  for i in range(500)]for j in range(500)]
		matNod = []
		for i in range(500):
			for j in range(500):
				matNod.append(Nodo([i , j]))
		x = [(5-obs[0])/0.1 , (5-obs[1])/0.1 , (5-obs[2])/0.1 , (5-obs[3])/0.1 , (5-obs[4])/0.1]
		y = [(5-obs[5])/0.1 , (5-obs[6])/0.1 , (5-obs[7])/0.1 , (5-obs[8])/0.1 , (5-obs[9])/0.1]
		r = [obs[10]/0.1 , obs[11]/0.1 , obs[12]/0.1 , obs[13]/0.1 , obs[14]/0.1]
		for i in range(len(x)):
			matriz[int(x[i])][int(y[i])] = 1
			for j in range(int(x[i]-r[i]-robot)-1, int(x[i]+r[i]+robot)):
				for k in range(int(y[i]-r[i]-robot)-1, int(y[i]+r[i]+robot)):
					dist = np.linalg.norm(np.array([x[i],y[i]]) - np.array([j+0.5,k+0.5]))
					if dist <= r[i]+robot:
						matriz[j][k] = 1
						x.append(j)
						y.append(k)
		for nod in matNod:
			posN = nod.pos
			if (matriz[posN[0]][posN[1]] == 0):
				for i in range(posN[0] - 1, posN[0] + 1 ,1):
					for j in range(posN[1] - 1, posN[1] + 1 ,1):
						if (i >= 0 and i <= 499 and j >= 0 and j <= 499):
							if (matriz[i][j] == 0 and not i == posN[0] and not j == posN[1]):
								print i,j,posN
								nod.agregarVecinos(matNod[i+j])
				print "--"
		xplot = list(map(lambda x: 5 - (0.1*x), x))
		yplot = list(map(lambda x: 5 - (0.1*x), y))
		'''for i in range(500):
			for j in range(500):
				print(matriz[i][j]),
			print""	'''
		if (bandera):
			return False
		time.sleep(0.2)
				
		
def ThreadInputs():
	with Listener(on_press = keypress) as listener:
		listener.join()
		
def keypress(key):
	global vel, bandera
	if key == Key.esc:
		bandera = True 	##Se usa la tecla Esc para terminar los diferentes hilos implementados
		return False

class Nodo:
	def __init__(self, pos):
		self.pos = pos
		self.coord = [5 - (0.1*pos[0]) , 5 - (0.1*pos[1])]
		costo = 1000000000
		self.vecinos = []
		self.objetivo = False
		self.actual = False
		
	def defHeu(self, pos_f):
		self.h = math.sqrt((self.pos[0]-pos_f[0])**2 + (self.pos[1]-pos_f[1])**2) 
		return self.h
		
	def asignarPadre(self, padre):
		self.padre = padre
	
	def cambiarCostoActual(self, costo):
		self.costo = costo
	
	def agregarVecinos(self, vecino):
		self.vecinos.append(vecino)
		
	def esActual(self, actual):
		self.actual = actual
		
	def esObejtivo(self, objetivo):
		self.objetivo = objetivo
				
if __name__ == '__main__':
	global xplot, yplot, obs, bandera
	obs = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
	bandera = False
	try:
		threading.Thread(target=ThreadInputs).start()
		threading.Thread(target=crearCuadricula).start()
		arrancar()
	except rospy.ServiceException:
		pass
