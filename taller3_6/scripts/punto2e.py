#!/usr/bin/python



""" librerias usadas"""

import rospy
from std_msgs.msg import Float32MultiArray, Float32
from geometry_msgs.msg import Twist
from pylab import *
from pynput.keyboard import Key, Listener 
import matplotlib.pyplot as plt
import threading
import time
import sys 
import random
import math
import copy

robot = 2
show_animation = True

r = 0.0925	##Radio de las ruedas del Pioneer obtenidas en el manual en m
l = 0.1900	##Distancia entre la rueda y el punto con el que se modela el robot en m
J1 = np.array([[1,0,l],[1,0,-l]]) 
inv_J2 = np.array([[1.0/r,0],[0,1.0/r]])
pub = rospy.Publisher('/motorsVel', Float32MultiArray, queue_size=10) 


def arrancar():
	global xfin, yfin, thetafin, obstacleList, path, obs
	rospy.init_node('punto2e', anonymous = True)
	rospy.Subscriber('/obstacles', Float32MultiArray , obstacles)
	rospy.Subscriber('/pioneerPosition', Twist ,vecto)
	tasa = rospy.Rate(10)
	rospy.myargv(argv=sys.argv)
	try:
		xfin = float(sys.argv[1])
		yfin = float(sys.argv[2])
		thetafin = float(sys.argv[3])
	except:    								##Tratamiento de los argumentos ingresador por el usuario
		xfin = 2.5
		yfin = 2.5
		thetafin = math.pi/2
	time.sleep(1)
	obstacleList = [
	(obs[0],obs[5],(2*obs[10])+0.4), 
	(obs[1],obs[6],(2*obs[11])+0.4),
	(obs[2],obs[7],(2*obs[12])+0.4), 
	(obs[3],obs[8],(2*obs[13])+0.4),
	(obs[4],obs[9],(2*obs[14])+0.4)
	]  # [x,y,size]
	rrt = RRT(start=[0, 0], goal=[xfin, yfin],randArea=[-5, 45], obstacleList=obstacleList)
	path = rrt.Planning()
	crearCuadricula()
	threading.Thread(target=plotPos).start()
	threading.Thread(target=control).start()
    # Draw final path
	'''if show_animation:  # pragma: no cover
		plt.plot([x for (x, y) in path], [y for (x, y) in path], '-r')
		plt.grid(True)
		plt.show()'''
	
	try:
		while not rospy.is_shutdown():
			tasa.sleep()
			publicar()
	except rospy.ServiceException as e:
		pass
    

def obstacles(data):	
	global obs
	obs = data.data
	
def publicar():
	global vec
	pub.publish(data = [vec[1],vec[0]])

def crearCuadricula():
	global obs, xplot, yplot
	x = [(5 + obs[0])/0.25 , (5 + obs[1])/0.25 , (5 + obs[2])/0.25 , (5 + obs[3])/0.25 , (5 + obs[4])/0.25]
	y = [(5 + obs[5])/0.25 , (5 + obs[6])/0.25 , (5 + obs[7])/0.25 , (5 + obs[8])/0.25 , (5 + obs[9])/0.25]
	r = [obs[10]/0.25 , obs[11]/0.25 , obs[12]/0.25 , obs[13]/0.25 , obs[14]/0.25]
	for i in range(len(x)):
		for j in range(int(x[i]-r[i]-robot)-1, int(x[i]+r[i]+robot+1)):
			for k in range(int(y[i]-r[i]-robot)-1, int(y[i]+r[i]+robot+1)):
				dist = np.linalg.norm(np.array([x[i],y[i]]) - np.array([j+0.5,k+0.5]))
				if dist <= r[i]+robot:
					x.append(j)
					y.append(k)
	xplot = list(map(lambda x: -5 + (0.25*x), x))
	yplot = list(map(lambda x: -5 + (0.25*x), y))

class RRT():
    """
    Class for RRT Planning
    """

    def __init__(self, start, goal, obstacleList,
                 randArea, expandDis=0.25, goalSampleRate=5, maxIter=500):
        """
        Setting Parameter
        start:Start Position [x,y]
        goal:Goal Position [x,y]
        obstacleList:obstacle Positions [[x,y,size],...]
        randArea:Ramdom Samping Area [min,max]
        """
        self.start = Node(start[0], start[1])
        self.end = Node(goal[0], goal[1])
        self.minrand = randArea[0]
        self.maxrand = randArea[1]
        self.expandDis = expandDis
        self.goalSampleRate = goalSampleRate
        self.maxIter = maxIter
        self.obstacleList = obstacleList

    def Planning(self, animation=False):
        """
        Pathplanning
        animation: flag for animation on or off
        """

        self.nodeList = [self.start]
        while True:
            # Random Sampling
            if random.randint(0, 100) > self.goalSampleRate:
                rnd = [random.uniform(self.minrand, self.maxrand), random.uniform(
                    self.minrand, self.maxrand)]
            else:
                rnd = [self.end.x, self.end.y]

            # Find nearest node
            nind = self.GetNearestListIndex(self.nodeList, rnd)
            # print(nind)

            # expand tree
            nearestNode = self.nodeList[nind]
            theta = math.atan2(rnd[1] - nearestNode.y, rnd[0] - nearestNode.x)

            newNode = copy.deepcopy(nearestNode)
            newNode.x += self.expandDis * math.cos(theta)
            newNode.y += self.expandDis * math.sin(theta)
            newNode.parent = nind

            if not self.__CollisionCheck(newNode, self.obstacleList):
                continue

            self.nodeList.append(newNode)
            print("nNodelist:", len(self.nodeList))

            # check goal
            dx = newNode.x - self.end.x
            dy = newNode.y - self.end.y
            d = math.sqrt(dx * dx + dy * dy)
            if d <= self.expandDis:
                print("Goal!!")
                break

            if animation:
                self.DrawGraph(rnd)

        path = [[self.end.x, self.end.y]]
        lastIndex = len(self.nodeList) - 1
        while self.nodeList[lastIndex].parent is not None:
            node = self.nodeList[lastIndex]
            path.append([node.x, node.y])
            lastIndex = node.parent
        path.append([self.start.x, self.start.y])

        return path

    def DrawGraph(self, rnd=None):  # pragma: no cover
        """
        Draw Graph
        """
        plt.clf()
        if rnd is not None:
            plt.plot(rnd[0], rnd[1], "^k")
        for node in self.nodeList:
            if node.parent is not None:
                plt.plot([node.x, self.nodeList[node.parent].x], [
                         node.y, self.nodeList[node.parent].y], "-g")

        for (ox, oy, size) in self.obstacleList:
            plt.plot(ox, oy, "ok", ms=30 * size)

        plt.plot(self.start.x, self.start.y, "xr")
        plt.plot(self.end.x, self.end.y, "xr")
        plt.axis([-2, 15, -2, 15])
        plt.grid(True)
        plt.pause(0.01)

    def GetNearestListIndex(self, nodeList, rnd):
        dlist = [(node.x - rnd[0]) ** 2 + (node.y - rnd[1])
                 ** 2 for node in nodeList]
        minind = dlist.index(min(dlist))
        return minind

    def __CollisionCheck(self, node, obstacleList):

        for (ox, oy, size) in obstacleList:
            dx = ox - node.x
            dy = oy - node.y
            d = math.sqrt(dx * dx + dy * dy)
            if d <= size:
                return False  # collision

        return True  # safe


class Node():
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None



def R(theta): ##funcion que retorna la matriz de rotacion
	return np.array([[math.cos(theta),math.sin(theta),0],[-math.sin(theta),math.cos(theta),0],[0,0,1]])


def control():
	global posix, posiy, lastheta, vec, xfin, yfin, thetafin, bandera, path
	rho = 10
	beta = 20
	x_vec = []
	y_vec = []
	for vec in path:
		x_vec.append(vec[0])
		y_vec.append(vec[1])
	x_vec.reverse()
	y_vec.reverse()
	for i in range(len(x_vec)):
		while rho >= 0.08:
			dx = x_vec[i] - posix[-1]
			dy = y_vec[i] - posiy[-1]
			if (x_vec[i] == x_vec[-2] and y_vec[i] == y_vec[-2]):
				dtheta = lastheta - thetafin
			elif (i != len(x_vec)-1):
				dtheta = lastheta - math.atan2(y_vec[i+1]-y_vec[i],x_vec[i+1]-x_vec[i])
			else:
				dtheta = lastheta - thetafin
			rho = math.sqrt((dx)**2 + (dy)**2)
			alpha = -lastheta + math.atan2(dy,dx)	#Se calculan los errores en coordenadas esfericas y se calcula la velocidad de acuerdo con kp
			beta = -math.atan2(dy,dx) - dtheta
			if (alpha >= 2*math.pi):
				alpha = alpha - 2*pi
			elif (alpha <= -2*math.pi):
				alpha = alpha + 2*pi
			if (beta >= 2*math.pi):
				beta = beta - 2*pi
			elif (beta <= -2*math.pi):
				beta = beta + 2*pi
			kb = 0.07
			kp = 0.6
			ka = 1.8
			v = kp * rho
			x = v*math.cos(lastheta)
			y = v*math.sin(lastheta)
			w = (ka*alpha) + (kb*(beta))
			vec = inv_J2.dot(J1.dot(R(lastheta).dot(np.array([x,y,w]))))
			time.sleep(0.2)
			if bandera:
				return False
		rho = 10
		beta = 20
	beta = 0.5
	while abs(beta) >= 0.01:
		kb = 0.3
		beta = -lastheta + thetafin
		if (beta >= 2*math.pi):
				beta = beta - 2*pi
		elif (beta <= -2*math.pi): 
			beta = beta + 2*pi
		w =(kb*(beta))
		x = 0
		y = 0
		vec = inv_J2.dot(J1.dot(R(lastheta).dot(np.array([x,y,w]))))
		time.sleep(0.2)
		if bandera:
			return False
	bandera = True
	vec = [0,0]
	return False



def vecto(data): ##Funcion que manipula la informacion con la posicion del robot
	global posix, posiy, lastheta, vec
	xact = data.linear.x
	yact = data.linear.y
	posix.append(xact)
	posiy.append(yact)  ##Se agregan dichos valores a un vector para mostrarlos en pantalla
	lastheta = data.angular.z  ## Se guarda el ultimo theta obtenido en simulacion 


def plotPos():
	global posix, posiy, xplot, yplot
	while True:
		plt.clf()
		plt.plot(posix,posiy)
		plt.plot(xplot,yplot,'p')
		plt.ylabel('Posicion en y')
		plt.xlabel('Posicion en x')	###Mientras el robot se mueve se muestran en una grafica la posicion simulada
		plt.title('Posicion del robot')
		plt.draw()
		if bandera:
			plt.savefig('src/Taller3_6/taller3_6/results/graficaPunto2e.png')
			plt.close()
			return False
		plt.pause(0.5)



def keypress(key):
	global bandera
	if key == Key.esc:
		bandera = True
		print "fin" 	##Se usa la tecla Esc para terminar los diferentes hilos implementados
		return False
		
def ThreadInputs():
	with Listener(on_press = keypress) as listener:
		listener.join()




if __name__ == '__main__':
	global obs, bandera, posix, posiy, lastheta, vec, xfin, yfin, thetafin, xplot, yplot
	obs = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
	posix = [0]
	posiy = [0]
	xplot = [0]
	yplot = [0]
	vec = [0,0]
	bandera = False
	xfin = 0
	yfin = 0
	lastheta = 0
	thetafin = 0
	try:
		threading.Thread(target=ThreadInputs).start()
		arrancar()
	except rospy.ServiceException:
		pass








