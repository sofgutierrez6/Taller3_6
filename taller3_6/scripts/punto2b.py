#!/usr/bin/python
import rospy
from std_msgs.msg import Float32MultiArray, Float32
from pylab import *
import matplotlib.pyplot as plt
from pynput.keyboard import Key, Listener 
import threading
import graphviz as gv
import time

robot = 0

def arrancar():
	rospy.init_node('punto2b', anonymous = True)
	rospy.Subscriber('/obstacle_1', Float32MultiArray ,obstacle1)
	rospy.Subscriber('/obstacle_2', Float32MultiArray ,obstacle2)
	rospy.Subscriber('/obstacle_3', Float32MultiArray ,obstacle3)
	rospy.Subscriber('/obstacle_4', Float32MultiArray ,obstacle4)
	rospy.Subscriber('/obstacle_5', Float32MultiArray ,obstacle5)
	tasa = rospy.Rate(10)
	try:
		while not rospy.is_shutdown():
			tasa.sleep()
	except rospy.ServiceException as e:
		pass
		
def obstacle1(data):
	global obs1
	obs1 = data.data
	
def obstacle2(data):
	global obs2
	obs2 = data.data
	
def obstacle3(data):
	global obs3
	obs3 = data.data
	
def obstacle4(data):
	global obs4
	obs4 = data.data
	
def obstacle5(data):
	global obs5
	obs5 = data.data
	
def crearCuadricula():
	global obs1, obs2, obs3, obs4, obs5, xplot, yplot
	while True:
		matriz = [[0  for i in range(500)]for j in range(500)]
		x = [(25-obs1[0])/0.1 , (25-obs2[0])/0.1 , (25-obs3[0])/0.1 , (25-obs4[0])/0.1 , (25-obs5[0])/0.1]
		y = [(25-obs1[1])/0.1 , (25-obs2[1])/0.1 , (25-obs3[1])/0.1 , (25-obs4[1])/0.1 , (25-obs5[1])/0.1]
		r = [obs1[2]/0.1 , obs2[2]/0.1 , obs3[2]/0.1 , obs4[2]/0.1 , obs5[2]/0.1]
		for i in range(len(x)):
			matriz[int(x[i])][int(y[i])] = 1
			for j in range(int(x[i]-r[i]-robot), int(x[i]+r[i]+robot)):
				for k in range(int(y[i]-r[i]-robot), int(y[i]+r[i]+robot)):
					dist = np.linalg.norm(np.array([x[i],y[i]]) - np.array([j+0.1,k+0.1]))
					if dist <= r[i]:
						matriz[j][k] = 1
						x.append(j)
						y.append(k)
		xplot = list(map(lambda x: 25 - (0.1*x), x))
		yplot = list(map(lambda x: 25 - (0.1*x), y))
		'''for i in range(500):
			for j in range(500):
				print(matriz[i][j]),
			print""	'''
		if (bandera):
			return False
		time.sleep(0.5)

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
	global xplot, yplot, obs1, obs2, obs3, obs4, obs5, bandera
	xplot = [0]
	yplot = [0]
	obs1 = [0,0,0]
	obs2 = [0,0,0]
	obs3 = [0,0,0]
	obs4 = [0,0,0]
	obs5 = [0,0,0]
	bandera = False
	try:
		threading.Thread(target=ThreadInputs).start()
		threading.Thread(target=plotMap).start()
		threading.Thread(target=crearCuadricula).start()
		arrancar()
	except rospy.ServiceException:
		pass
