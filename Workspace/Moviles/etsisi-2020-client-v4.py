#!/usr/bin/python3

print('### Script:', __file__)

import math
import sys
import time

import sim

#Variables------------------

state_ = 0;
states_ = { 0: 'Wall following',
			1: 'Turning left',
			2: 'Turning right',
			3: 'Endpoint'}

wall_dist = 0.5 #distancia a la pared
dist_front = 0
wall_found = 0


range=1


regions_ = {
	'bright': 0,
	'right': 0,
	'fright': 0,
	'front': 0,
	'fleft': 0,
	'left': 0,
	'bleft': 0
}




# --------------------------------------------------------------------------

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

def setSpeed(clientID, hRobot, lspeed, rspeed):
    sim.simxSetJointTargetVelocity(clientID, hRobot[0][0], lspeed,
                                    sim.simx_opmode_oneshot)
    sim.simxSetJointTargetVelocity(clientID, hRobot[0][1], rspeed,
                                    sim.simx_opmode_oneshot)

# --------------------------------------------------------------------------

def getSonar(clientID, hRobot):
	print('Starting GETSONAR')
    r = [1.0] * 16
    for i in range(16):
        handle = hRobot[1][i]
        e,s,p,_,_ = sim.simxReadProximitySensor(clientID, handle,
                                                 sim.simx_opmode_buffer)
        if e == sim.simx_return_ok and s:
            r[i] = math.sqrt(p[0]*p[0]+p[1]*p[1]+p[2]*p[2])
		print(i + ': ' + r[i])

    return r

# --------------------------------------------------------------------------
# --------------------------------------------------------------------------

def avoid():

	print('Starting AVOID')
	 
    switch (_state){
    	case 0: lspeed, rspeed = +2.0, +2.0
    	case 1: lspeed, rspeed = -3.0, +3.0
    	case 2: lspeed, rspeed = +2.0, -2.0
    	case 3: lspeed, rspeed = 0.0, 0.0
    }

# --------------------------------------------------------------------------
def setSate(state):					#Sets the state of the SM
	
	global state_, states_

	if (state=!state_):
		state_=state

#---------------------------------------------------------------------------
def analyseSonar(sonar):			#Set distances between Pioneer and obstacles in all cordinates
	global regions_

	print('Starting ANALYSIS')
 	regions_ = {								
 		'bright' = min(sonar[11],sonar[10]),
 		'right' = min(sonar[8],sonar[9]),
 		'fright' = min(sonar[7],sonar[6]),
 		'front' = min(sonar[4],sonar[5]),
 		'fleft' = min(sonar[2],sonar[3]),
 		'left' = min(sonar[1],sonar[16]),
		'bleft' = min(sonar[14],sonar[15])
		}
 	
 	
#--------------------------------------------------
def control():						#decides whats the next state based on perception
	global regions, rotating, wall_dist

	print('Starting CONTROL')
	
	regions=regions_
	
	state_description = ''
	
	if (regions['left']==1):
		if (regions['front']==1 or (regions['bright']==1 and regions['right']==1))
			state_description= 'Case 3. EndPoint'
			setState(3)
		else 
			state_description = 'Case 1. Turn Left'
			rotating=1
			setState(1)
	elif (regions['front']>wall_dist and regions['fright']>wall_dist):
		state_description = 'Case 0. Wall Following'
		setState(0)
	else 
		state_description = 'Case 2. Turn Right'
		rotating=1
		setState(2)
	print(state_description)
		
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
            # print '### s', sonar
		
			analyseSonar(sonar)
			control()
			
            # Planning
            lspeed, rspeed = avoid()

            # Action
            setSpeed(clientID, hRobot, lspeed, rspeed)
            time.sleep(0.1)

        print('### Finishing...')
        sim.simxFinish(clientID)

    print('### Program ended')

# --------------------------------------------------------------------------

if __name__ == '__main__':
    main()
