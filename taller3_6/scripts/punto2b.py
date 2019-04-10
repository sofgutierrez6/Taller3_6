#!/usr/bin/python
import rospy
from std_msgs.msg import Float32MultiArray, Float32
from pylab import *
from graphviz import Digraph
from pynput.keyboard import Key, Listener 
import matplotlib.pyplot as plt
import threading
import time

robot = 0
#Metodo de inicializacion 
def arrancar():
	rospy.init_node('punto2b', anonymous = True)
	rospy.Subscriber('/obstacles', Float32MultiArray ,obstacles)
	tasa = rospy.Rate(10)
	grafo()
	try:
		while not rospy.is_shutdown():
			tasa.sleep()
	except rospy.ServiceException as e:
		pass
		
def obstacles(data):
	global obs
	obs = data.data
	
def crearCuadricula():
	global obs, xplot, yplot, matriz
	while True:
		matriz = [[0  for i in range(500)]for j in range(500)]
		#Se obtienen las posiciones del 
		x = [(5+obs[0])/0.1 , (5+obs[1])/0.1 , (5+obs[2])/0.1 , (5+obs[3])/0.1 , (5+obs[4])/0.1]
		y = [(5+obs[5])/0.1 , (5+obs[6])/0.1 , (5+obs[7])/0.1 , (5+obs[8])/0.1 , (5+obs[9])/0.1]
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
		xplot = list(map(lambda x: -5 + (0.1*x), x))
		yplot = list(map(lambda x: -5 + (0.1*x), y))
		'''for i in range(500):
			for j in range(500):
				print(matriz[i][j]),
			print""	'''
		if (bandera):
			return False
		time.sleep(0.2)
		
def grafo():
	global matriz, bandera
	time.sleep(3)
	dot = Digraph(comment = 'Mapa')
	for i in range(50,105,5):
		for j in range(50,105,5):
			if (matriz[i][j] == 0):
				dot.node(str(i)+","+str(j) , str(-5+(0.1*i))+","+str(-5+(0.1*j)))
	for i in range(50,100,5):
		for j in range(50,100,5):
			for k in range(i-5,i+10,5):
				for m in range(j-5,j+10,5):
					if (matriz[i][j] == 0 and matriz[k][m] == 0 and k >= 50 and k <= 100 and m >= 50 and m <= 100 and not ((abs(k-i) + abs(m-j))==0)):
						dot.edge(str(i)+","+str(j) , str(k)+","+str(m))
	dot.render('src/Taller3_6/taller3_6/results/grafoPunto2.gv', view=True) 
				

def plotMap():
	global xplot, yplot, bandera
	while True:
		plt.clf()
		plt.plot(xplot, yplot, 'p')
		plt.draw()
		if (bandera):
			plt.close()
			return False
		plt.pause(0.8)
		
def ThreadInputs():
	with Listener(on_press = keypress) as listener:
		listener.join()
		
def keypress(key):
	global vel, bandera
	if key == Key.esc:
		bandera = True 	##Se usa la tecla Esc para terminar los diferentes hilos implementados
		return False
			
				
if __name__ == '__main__':
	global xplot, yplot, obs, bandera, matriz
	xplot = [0]
	yplot = [0]
	obs = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
	bandera = False
	matriz = [[0  for i in range(500)]for j in range(500)]
	try:
		threading.Thread(target=ThreadInputs).start()
		threading.Thread(target=plotMap).start()
		threading.Thread(target=crearCuadricula).start()
		arrancar()
	except rospy.ServiceException:
		pass
