#!/usr/bin/python
import rospy
from std_msgs.msg import Float32MultiArray, Float32
from pylab import *
import matplotlib.pyplot as plt

robot = 0

def arrancar():
	rospy.init_node('punto2b', anonymous = True)
	rospy.Subscriber('/obstacle_1', Float32MultiArray ,obstacle1)
	rospy.Subscriber('/obstacle_2', Float32MultiArray ,obstacle2)
	rospy.Subscriber('/obstacle_3', Float32MultiArray ,obstacle3)
	rospy.Subscriber('/obstacle_4', Float32MultiArray ,obstacle4)
	rospy.Subscriber('/obstacle_5', Float32MultiArray ,obstacle5)
	tasa = rospy.Rate(10)
	tasa.sleep()
	crearCuadricula()
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
	global obs1, obs2, obs3, obs4, obs5
	matriz = [["."  for i in range(500)]for j in range(500)]
	x = [round((25-obs1[0])/0.1) , round((25-obs2[0])/0.1) , round((25-obs3[0])/0.1) , round((25-obs4[0])/0.1) , round((25-obs5[0])/0.1)]
	y = [round((25-obs1[1])/0.1) , round((25-obs2[1])/0.1) , round((25-obs3[1])/0.1) , round((25-obs4[1])/0.1) , round((25-obs5[1])/0.1)]
	r = [round(obs1[2]/0.1) , round(obs2[2]/0.1) , round(obs3[2]/0.1) , round(obs4[2]/0.1) , round(obs5[2]/0.1)]
	for i in range(len(x)):
		matriz[int(x[i])][int(y[i])] = "*"
		for j in range(int(x[i]-r[i]-robot), int(x[i]+r[i]+robot+1)):
			for k in range(int(y[i]-r[i]-robot), int(y[i]+r[i]+robot+1)):
				matriz[j][k] = "*"
	for i in range(500):
		for j in range(500):
			print(matriz[i][j]),
		print""	
			
				
if __name__ == '__main__':
	try:
		arrancar()
	except rospy.ServiceException:
		pass
