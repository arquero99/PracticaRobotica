#!/usr/bin/python3

print('### Script:', __file__)

import math
import sys
import time

import sim

#----DEFINICION PARÁMETROS CONFIGURACIÓN BÁSICOS PARA EL PROGRAMA FOLLOW WALL----#

direction=1 #1 para Dcha, -1 para Izqda
radius= 0.0975
a=	0 #Distancia en eje delantero y trasero
b=	0.2764 #Distancia en eje ruedas de un eje  NOTA: Si no funciona probar con la mitad
dWallSide=	0.25 #Distancia esperada a la pared
v=	1 #Factor Velocidad
kWall=	0.2 #Ganancia para calcular diferencia entre medida acutual y medida esperada
e = 	0.5#Distancia desde el centro del robot al punto descentrado

dFR=7	#Cupla de sensores frontales/traseros derecha/izquierda
dRR=10
dFL=2
dRL=15

tolarance=0.1

vTurn=0.3
vCurve=0.5
cellSize=1

#minTurnT=0
#minCurveT=0

# --------------------------------------------------------------------------

def setState():
	if (state==1):
		currentT=sim.getSimulationTime();
		if (wallF):
			state=2;
		if (!wallS):
			state=3;
	elif (state==2)
		elapsedT=((sim.getSimulationTime() - currentT)>minTurnT)
		if (wallS and elapsedT):
			state=1;
	elif (state==3):
		elapsedT=((sim.getSimulationTime() - currentT)>minCurveT)
		if (wallS and elapsedT):
			state=1;
			
def followWall():
	if (direction==1):
		phi=math.atan((sonar[dFR]-sonar[dRR])/a)
		d=(0.5*((sonar[dFR]+sonar[dRR]))-dWallSide)
	else:
		phi=math.atan((sonar[dRL]-sonar[dFL])/a)
		d=(dWallSide-0.5*(sonar[dFL]+sonar[dRL])))
	
	gamma=kWall*d
	alpha=phi+gamma
	
	wL=(v-radius)*(math.cos(alpha)+(b/e)*math.sin(alpha))
	wR=(v-radius)*(math.cos(alpha)-(b/e)*math.sin(alpha))
	return wL, wR

#Devuelve valor booleano que indica sise ha detectado una pared
def wallDetected(distance, expectedDistance, tolerance):
	return math.abs(distance-expectedDistance<tolerance)  #Se sugiere usar una tolerancia de 0.3

#Calcula la velocidad angular de las ruedas para realizar un giro sobre si mismo de 90º	
def precomputeTurn():
	wL = -direction*vTurn/radius
	wR = direction*vTurn/radius
	t=b*math.pi/2/vTurn
	return wL, wR, t
	
#Calcula la velocidad angular de las ruedas para realizar una curva de 90º	
def precomputeCurve():
	wRef=vCurve/(cellSize/2)
	wL=(vCurve+b*direction*wRef)/radius
	wL=(vCurve-b*direction*wRef)/radius
	t=(math.pi/2)/wRef
	return wL, wR, t
	
def finish():
	#Funcion que devuelva true si se ha terminado el laberinto. (Espacio de 2x2)

def getTurnParam():
	TurnL, TurnR, minTurnT = precomputeTurn();

def getCurveParam():
	CurveL, CurveR, minCurveT = precomputeCurve();

def getWallState():
	wallF=wallDetected((math.min(sonar[4],sonar[5])),0.1,tolerance)
	if (direction==1):
		wallS=wallDetected(sonar[8],0.2,tolerance)
	else:
		wallS=wallDetected(sonar[1],0.2,tolerance)
			
	
def getRobotHandles(clientID):
    # Robot handle
    _,rbh = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx',
                                     sim.simx_opmode_blocking)

    # Motor handles
    _,lmh = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx_leftMotor',
                                     sim.simx_opmode_blocking)
    _,rmh = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx_rightMotor',
                                     sim.simx_opmode_blocking)

    # Sonar handles
    str = 'Pioneer_p3dx_ultrasonicSensor%d'
    sonar = [0] * 16
    for i in range(16):
        _,h = sim.simxGetObjectHandle(clientID, str % (i+1),
                                       sim.simx_opmode_blocking)
        sonar[i] = h
        sim.simxReadProximitySensor(clientID, h, sim.simx_opmode_streaming)

    return [lmh, rmh], sonar, rbh

# --------------------------------------------------------------------------

def setSpeed(clientID, hRobot, lspeed, rspeed, state):
	
	if (state==1): #Estado basico. Sigue la pared
		lspeed, rspeed = followWall()
	elif (state==2): #Estado que representa un giro de 90º
		lspeed=TurnL
		rspeed=TurnR
	elif (state==3): #Estado que representa curva de 90º
		lspeed=CurveL
		rspeed=CurveR
	else:
		lspeed=0	#Estado final en el que se ha llegado a la meta
		rspeed=0	

    sim.simxSetJointTargetVelocity(clientID, hRobot[0][0], lspeed,
                                    sim.simx_opmode_oneshot)
    sim.simxSetJointTargetVelocity(clientID, hRobot[0][1], rspeed,
                                    sim.simx_opmode_oneshot)

# --------------------------------------------------------------------------

def getSonar(clientID, hRobot):
    r = [1.0] * 16
    for i in range(16):
        handle = hRobot[1][i]
        e,s,p,_,_ = sim.simxReadProximitySensor(clientID, handle,
                                                 sim.simx_opmode_buffer)
        if e == sim.simx_return_ok and s:
            r[i] = math.sqrt(p[0]*p[0]+p[1]*p[1]+p[2]*p[2])

    return r

# --------------------------------------------------------------------------
# --------------------------------------------------------------------------

def avoid(sonar):
		
				 
    if (sonar[3] < 0.2) or (sonar[4] < 0.2):
        lspeed, rspeed = +0.5, -0.5
    elif (sonar[6] < 0.2) or (sonar[5] < 0.2):
    	lspeed, rspeed = -0.5, +0.5
    elif sonar[1] < 0.3:
        lspeed, rspeed = +1.0, +0.3 
    elif sonar[8] < 0.3:
       lspeed, rspeed = +0.3, +1.0    
    elif sonar[5] < 0.2:
        lspeed, rspeed = +0.2, +0.7
    elif sonar[4] < 0.2:
        lspeed, rspeed = +0.7, +0.2
    else:
        lspeed, rspeed = +2.0, +2.0

    return lspeed, rspeed

# --------------------------------------------------------------------------

def main():
    print('### Program started')

    print('### Number of arguments:', len(sys.argv), 'arguments.')
    print('### Argument List:', str(sys.argv))

    sim.simxFinish(-1) # just in case, close all opened connections

    port = int(sys.argv[1])
    clientID = sim.simxStart('127.0.0.1', port, True, True, 2000, 5)

    if clientID == -1:
        print('### Failed connecting to remote API server')

    else:
        print('### Connected to remote API server')
        hRobot = getRobotHandles(clientID)
        int state=1

        while sim.simxGetConnectionId(clientID) != -1:
            # Perception
            sonar = getSonar(clientID, hRobot)
            getWallState();
            
           print '### s', sonar
            

            # Planning
            getTurnParam()
            getCurveParam()
            
            #lspeed, rspeed = avoid(sonar)
			
			print wL
			print wR
			
            # Action
            setSpeed(clientID, hRobot, lspeed, rspeed, state)
            
            setState();
            
            time.sleep(0.1)

        print('### Finishing...')
        sim.simxFinish(clientID)

    print('### Program ended')

# --------------------------------------------------------------------------

if __name__ == '__main__':
    main()
