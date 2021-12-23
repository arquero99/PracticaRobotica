#!/usr/bin/python3

print('### Script:', __file__)

import math
import sys
import time
import os

import sim

#Variables------------------

state_ = 0;
states_ = { 0: 'Wall following',
			1: 'Turning left',
			2: 'Turning right',
			3: 'Endpoint'}

wall_dist = 0.23 #distancia a la pared


frange= 1
lrange=0.5
rotating=0

regions_ = {
	'bright': 0,
	'right': 0,
	'fright': 0,
	'front': 0,
	'fleft': 0,
	'left': 0,
	'bleft': 0,
	'fbleft':0
}


clear = lambda:os.system('clear')

# --------------------------------------------------------------------------

def getRobotHandles(clientID):
    # Robot handle
	_,rbh = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx', sim.simx_opmode_blocking)

    # Motor handles
	_,lmh = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx_leftMotor', sim.simx_opmode_blocking)
	_,rmh = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx_rightMotor', sim.simx_opmode_blocking)

    # Sonar handles
	str = 'Pioneer_p3dx_ultrasonicSensor%d'
	sonar = [0] * 16
	i=0
	while i<16:
		_,h = sim.simxGetObjectHandle(clientID, str % (i+1), sim.simx_opmode_blocking)
		sonar[i] = h
		sim.simxReadProximitySensor(clientID, h, sim.simx_opmode_streaming)
		i=i+1
	return [lmh, rmh], sonar, rbh
	
# --------------------------------------------------------------------------

def setSpeed(clientID, hRobot, lspeed, rspeed):
    sim.simxSetJointTargetVelocity(clientID, hRobot[0][0], lspeed,
                                    sim.simx_opmode_oneshot)
    sim.simxSetJointTargetVelocity(clientID, hRobot[0][1], rspeed,
                                    sim.simx_opmode_oneshot)

# --------------------------------------------------------------------------

def getSonar(clientID, hRobot):
	r = [1.0] * 16
	i = 0
	print('getSonar started')
	i = 0
	while i<16:
		handle = hRobot[1][i]
		e,s,p,_,_ = sim.simxReadProximitySensor(clientID, handle,
                                                 sim.simx_opmode_buffer)
		if e == sim.simx_return_ok and s:
			r[i] = math.sqrt(p[0]*p[0]+p[1]*p[1]+p[2]*p[2])
		i=i+1
	return r

# --------------------------------------------------------------------------
def printMeasures(sensor):
	x = 0
	while x<16 :
		print('-----------------')
		print(x)
		print(sensor[x])
		x=x+1

def printRegions():
	global regions_
	for reg in regions_:
		print('--------')
		print(reg)
		print(regions_[reg])

# --------------------------------------------------------------------------

def avoid():
	
	global state_
	
# print('starting AVOID')
	 
	if(state_ == 0):
		lspeed, rspeed = +2.0, +2.0
		lspeed, rspeed= wall_following(lspeed,rspeed)
	elif(state_==1): 
		lspeed, rspeed = +0.1, +2.0
		lspeed, rspeed= wall_following(lspeed,rspeed)
	elif(state_==2):
		lspeed, rspeed = +2.0, +0.1
		lspeed, rspeed= wall_following(lspeed,rspeed)
	elif(state_==3):
 		lspeed, rspeed = 0.0, 0.0
	return lspeed, rspeed
    

# --------------------------------------------------------------------------
def setState(state):					#Sets the state of the SM
	
	global state_, states_

	if state is not state_:
		state_=state

#---------------------------------------------------------------------------
def analyseSonar(sonar):			#Set distances between Pioneer and obstacles in all cordinates
	global regions_
	#print('analyse started')
	regions_ = {								
		'bright' : min(sonar[10],sonar[9]),
		'right' : min(sonar[7],sonar[8]),
		'fright' : min(sonar[5],sonar[4]),
 		'front' : min(sonar[4],sonar[3]),
		'fleft' : min(sonar[2],sonar[1]),
		'left' : min(sonar[1],sonar[0]),
		'bleft' : min(sonar[13],sonar[12]),
		'fbleft': min(sonar[0], sonar[15])
	}
	#print('analyse finish')
 	
#--------------------------------------------------

def control():						#decides whats the next state based on perception
	global regions_, rotating, wall_dist
	
	#print ('CONTROL STARTED')
	regions=regions_
	
	state_description = ''

	if (isEndpoint(regions)):
		state_description= 'Case 3. EndPoint'
		setState(3)
	elif (regions['left']> lrange or regions['fbleft'] > lrange):		#(regions['fleft']==range and regions['bleft']>range-0.13)
		state_description = 'Case 1. Turn Left'
		setState(1)
	elif (regions['front']>wall_dist + 0.15 and regions['fright']> wall_dist + 0.15):
		state_description = 'Case 0. Wall Following'
		setState(0)
	else :
		state_description = 'Case 2. Turn Right'
		setState(2)
	print(state_description)
#----------------------------------------------------------
def wall_following (lspeed, rspeed):
	global regions_, wall_dist
	corection_factor=0.3
	regions= regions_
	if (regions['left'] > wall_dist + 0.05):
		rspeed= rspeed + corection_factor
		print('Corrigiendo separación')
	elif (regions['fleft'] < (wall_dist - 0.05)):
		lspeed= lspeed + corection_factor
		print('Corrigiendo acercamiento')
	if (regions['fleft'] < 0.1 or regions['left'] < 0.1):
		lspeed=1.0
		rspeed=-0.5
		print('Avoiding left colision')
	return lspeed, rspeed
	
#----------------------------------
def isEndpoint(regions):
	endpoint = False
	if ((regions['front']==frange) and (regions['right']==frange) and (regions['fright']==frange) and (regions['bright']==frange)):
		endpoint=True
	return endpoint
	

#----------------------------------


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
		while sim.simxGetConnectionId(clientID) != -1:
            # Perception
			sonar = getSonar(clientID, hRobot)
			#print '### s', sonar
			#printMeasures(sonar)
			printRegions()

			analyseSonar(sonar)
			
			control()
			
            # Planning
			lspeed, rspeed = avoid()
			

            # Action
			setSpeed(clientID, hRobot, lspeed, rspeed)
			time.sleep(0.1)
			clear()

		print('### Finishing...')
		sim.simxFinish(clientID)

	print('### Program ended')

# --------------------------------------------------------------------------

if __name__ == '__main__':
    main()
