#!/usr/bin/python
import rospy
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
from pynput.keyboard import Key, Listener 
import matplotlib.pyplot as plt
import threading
import sys
from pylab import *

pub = rospy.Publisher('/motorsVel', Float32MultiArray, queue_size=10) ## publicador de las velocidades de los motores

    
def arrancar(): ### Metodo que inicia el nodo y solicita los argumentos del sistema 
	global vel ## la variable vel guarda la velocidad ingresada por el usuario
	rospy.init_node('punto3a', anonymous = True)
	rospy.myargv(argv=sys.argv)
	rospy.Subscriber('/pioneerPosition', Twist ,vecto)
	rospy.Subscriber('/scanner', Float32MultiArray, scanner) ## Se suscribe al topico que publica la posicion del robot llamando al metodo vecto
	vel = int(sys.argv[1]) 
	try:
		tasa = rospy.Rate(10)
		while not rospy.is_shutdown():
			publicar()
			tasa.sleep()				

	except rospy.ServiceException as e:
		pass

def vecto(data): ##Este metodo se encarga de manipular la informacion de la posicion del robot
	global posix, posiy
	posix = data.linear.x
	posiy = data.linear.y

def keypress(key):
	global vel, bandera, vec
	if key == Key.esc:
		bandera = True  ##En caso de presionar esc el hilo se finaliza y activa una bandera para finalizar los demas hilos
		return False
	else:
		try:
			if key.char == 'a' or key.char == 'w' or key.char == 's' or key.char == 'd': 
				mensajes = {'a':[-vel,vel], 'w':[vel,vel], 's':[-vel,-vel], 'd':[vel,-vel]}
				vec = mensajes[key.char]  ##En caso de presionar una de las teclas a,s,d,w se mueve el robot con la velocidad especificada en la direccion correspondiente
			else:
				vec = [0,0] 
		except:
			vec = [0,0] 
		  

def keydown(key):
	global vec
	vec = [0,0]   ##Al soltar cualquier tecla se detiene el robot

def publicar():
    global vec
    pub.publish(data = vec)
    
def ThreadInputs():
	with Listener(on_press = keypress, on_release = keydown) as listener:
		listener.join()  ##Thread para leer teclado

def scanner(data):
	global scannerX, scannerY, posix, posiy
	scannerX = []
	scannerY = []
	for i in range(0,len(data.data),2):
		x = (math.cos(data.data[i])*data.data[i+1]) + posix
		y = (math.sin(data.data[i])*data.data[i+1]) + posiy
		scannerX.append(x)
		scannerY.append(y)
		

def plotPos(): ##Funcion que realiza el plot de la posicion del robot
	global posix, posiy, bandera, scannerX, scannerY
	while not bandera: ##Mientras no se haya presionado Esc
		plt.clf() 
		plt.plot(posix,posiy, 'p')
		plt.plot(scannerX,scannerY,'p')
		plt.ylabel('Posicion en y') ##Grafico la posicion del robot esperando 0.5 segundos entre cada grafica
		plt.xlabel('Posicion en x')
		plt.title('Posicion del robot')
		plt.draw()
		plt.pause(0.5)
	if bandera == True:
		plt.close() ##Si se presiona Esc se guarda la imagen en la ruta especificada 
		return False

if __name__ == '__main__':
	global bandera, posix, posiy, vec, scannerX, scannerY
	vec = [0,0]
	scannerX = [0]
	scannerY = [0]
	posix = 0
	posiy = 0  ##Se inicializan las variables con el fin de evitar errores y la bandera en False dado que no se ha presionado Esc
	bandera = False
	try:
		print("Presione una tecla (Esc para salir):")
		threading.Thread(target=ThreadInputs).start()
		threading.Thread(target=plotPos).start()  ##Se inicia los hilos y se arranca el nodo
		arrancar()
	except rospy.ServiceException:
		pass