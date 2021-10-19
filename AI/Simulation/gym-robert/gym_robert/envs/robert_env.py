import gym
from gym import error, utils, spaces
from gym.utils import seeding
import pybullet as p
import time
import pybullet_data
import numpy as np
import os
import math
import random

physicsClient = p.connect(p.GUI)#p.GUI or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally contains the plane.urdf
print("Additionally Search Path: ",pybullet_data.getDataPath())
#HS645MG max torque: 9.6kg/cm -> 0.096kg/m -> 0.9414nm
maxForce = 0.9414

#heightInput = p.addUserDebugParameter("height", -3.14, 3.14, 0)
#widthInput = p.addUserDebugParameter("width", -3.14, 3.14, 0)

def quaternion_to_euler(x, y, z, w):

    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = math.asin(t2)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)
    return [yaw, pitch, roll]


class RobertEnv(gym.Env):
    metadata = {'render.modes': ['human']}

    def __init__(self,res=[384,384]):
        self.width=res[0]
        self.height=res[1]
        self.numLegs = 6
        self.numMotorsPerLeg = 3
        self.numMotors = self.numLegs * self.numMotorsPerLeg
        self.robertId = 0
        self.motors = [None]*self.numMotors
        self.motorTargetPositions = [None]*self.numMotors
       
        self.lastPosition = [0,0,0]
        self.jointNameToId = {}
        self.linkNameToId = {}
        self.rotation = [0,0,0]
        self.angularVelocity = [0,0,0]
        self.legs = [None]*self.numLegs
        self.randomMotorOffsetDegree = [None]*self.numMotors
        self.currentStep = 0

        #2x Rotation 12x motors
        self.state = np.zeros(3+18)

        self.action_space = spaces.Box(np.array([-0.5,-0.25,-0.5,-0.5,-0.25,-0.5,0,0,0,0,0,0]), np.array([0.5,0.25,0.5,0.5,0.25,0.5,0.5,0.5,0.5,0.5,0.5,0.5]))
        self.observation_space = spaces.Box(np.array([-1.57 ,-1.57  ,-1.57  ,-1.57  ,0      ,-2.51  ,-1.57  ,0      ,-2.51  ,-1.57  ,0      ,-2.51  ,-1.57  ,-2.356 ,0      ,-1.57  ,-2.356 ,0      ,-1.57  ,-2.356 ,0]),
                                            np.array([1.57  ,1.57   ,1.57   ,1.57   ,2.356  ,0      ,1.57   ,2.356  ,0      ,1.57   ,2.356  ,0      ,1.57   ,0      ,2.51   ,1.57   ,0      ,2.51   ,1.57   ,0      ,2.51]))

        self.renderCurrentSim = False
        self.renderedFrames = []
    def GetState(self):

        self.state[0] =  self.rotation[0]
        self.state[1] =  self.rotation[1] 
        self.state[2] =  self.rotation[2]

        offset = 3
        self.state[offset+0] = self.GetMotorPosition(self.legs[0][0])
        self.state[offset+1] = self.GetMotorPosition(self.legs[0][1])
        self.state[offset+2] = self.GetMotorPosition(self.legs[0][2])

        self.state[offset+3] = self.GetMotorPosition(self.legs[1][0])
        self.state[offset+4] = self.GetMotorPosition(self.legs[1][1])
        self.state[offset+5] = self.GetMotorPosition(self.legs[1][2])

        self.state[offset+6] = self.GetMotorPosition(self.legs[2][0])
        self.state[offset+7] = self.GetMotorPosition(self.legs[2][1])
        self.state[offset+8] = self.GetMotorPosition(self.legs[2][2])

        self.state[offset+9] = self.GetMotorPosition(self.legs[3][0])
        self.state[offset+10] = self.GetMotorPosition(self.legs[3][1])
        self.state[offset+11] = self.GetMotorPosition(self.legs[3][2])

        self.state[offset+12] = self.GetMotorPosition(self.legs[4][0])
        self.state[offset+13] = self.GetMotorPosition(self.legs[4][1])
        self.state[offset+14] = self.GetMotorPosition(self.legs[4][2])

        self.state[offset+15] = self.GetMotorPosition(self.legs[5][0])
        self.state[offset+16] = self.GetMotorPosition(self.legs[5][1])
        self.state[offset+17] = self.GetMotorPosition(self.legs[5][2])


        return np.copy(self.state)

    def ChangeMotorPosition(self, bodyUniqueId,motor,targetPos):
        maxForces = []
        positionGains =[]
        targetV = []
        for i in range(len(self.motors)):
            m = self.motors[i]
            if m == motor:
                self.motorTargetPositions[i] = targetPos
                #,maxVelocity=10.0
                p.setJointMotorControl2(bodyUniqueId,jointIndex=m,controlMode=p.POSITION_CONTROL,force=maxForce,targetPosition=targetPos)

            jointInfo = p.getJointInfo(bodyUniqueId,m)
            lowerLimit = jointInfo[8]
            upperLimit = jointInfo[9]

            maxForces.append(maxForce)
            positionGains.append(1)
            targetV.append(0.001)


    def GetMotorPosition(self,joint):
        return p.getJointState(self.robertId,joint)[0]


    #Input:     12 actions
    #Output:    18 motor rotation in degree
    def ActionsToAngle(self,actions):
        #18Motors.
        #3Elements per leg.
        #First element inside leg = Motor A 
        #Second element inside leg = Motor B 
        #Second element inside leg = Motor C
        #Motor layout:  C-B-A-[Robo]-A-B-C
        #0..2 right first leg
        #3..5 right second leg
        #6..8 right third leg
        #9..11 left first leg
        #12..14 left second leg
        #15..17 left third leg
        angles = [0]*self.numMotors
        
        for i in range(0,self.numLegs):
            anglesArrayOffset = i*3
            #The first 6 actions are in radians for all 6 motors.
            #So only map them to degree
            #Motor A
            angles[0+anglesArrayOffset] = actions[i]

            #Left side angle B and C are swapped
            sign = 1
            if i > 2:
                sign = -1

            #The last 6 actions are in range 0..1.
            #So calculate the degrees.
            #Motor B
            angles[1+anglesArrayOffset] = sign*(90 - (actions[6 + i] * 90))
            #Motor C
            angles[2+anglesArrayOffset] = sign*(-135 + (actions[6 + i] * 90))

        return angles




    def render(self, mode='human', close=False):
        self.renderCurrentSim = True

        return self.renderedFrames

    def renderInternal(self):
        #z is up
        basePosition = p.getLinkState(self.robertId,0)[0]
        camPosition = np.array(basePosition) + np.array([0,-1,0.7])
        camTarget = basePosition
        camUP = [0,0,1]
        viewMatrix = p.computeViewMatrix( camPosition, camTarget,camUP)


        fov = 60
        aspect = self.width/self.height
        nearPlane = 0.1
        farPlane = 100
        projectionMatrix = p.computeProjectionMatrixFOV(fov,aspect,nearPlane,farPlane)

        img = p.getCameraImage(self.width,self.height,viewMatrix,projectionMatrix)
        rgb = np.reshape(img[2],(self.height,self.width,4)) * (1./255.)
        return rgb


    def step(self,action):
        #if  type(action) is not np.ndarray :
        #    return self.GetState()

        #right first
        B = (90 - (action[6] * 90)) + self.randomMotorOffsetDegree[1]
        C = (-135 + (action[6]*90)) + self.randomMotorOffsetDegree[2]
        self.ChangeMotorPosition(self.robertId,self.legs[0][0],action[0] + math.radians(self.randomMotorOffsetDegree[0]))
        self.ChangeMotorPosition(self.robertId,self.legs[0][1],math.radians(B))
        self.ChangeMotorPosition(self.robertId,self.legs[0][2],math.radians(C))
        #right second
        B = (90 - (action[7] * 90)) + self.randomMotorOffsetDegree[4]
        C = (-135 + (action[7]*90)) + self.randomMotorOffsetDegree[5]
        self.ChangeMotorPosition(self.robertId,self.legs[1][0],action[1] + math.radians(self.randomMotorOffsetDegree[3]))
        self.ChangeMotorPosition(self.robertId,self.legs[1][1],math.radians(B))
        self.ChangeMotorPosition(self.robertId,self.legs[1][2],math.radians(C))
        #right third
        B = (90 - (action[8] * 90)) + self.randomMotorOffsetDegree[7] 
        C = (-135 + (action[8]*90)) + self.randomMotorOffsetDegree[8]
        self.ChangeMotorPosition(self.robertId,self.legs[2][0],action[2] + math.radians(self.randomMotorOffsetDegree[6]))
        self.ChangeMotorPosition(self.robertId,self.legs[2][1],math.radians(B))
        self.ChangeMotorPosition(self.robertId,self.legs[2][2],math.radians(C))
        #left first
        B = (90 - (action[9] * 90)) + self.randomMotorOffsetDegree[10]
        C = (-135 + (action[9]*90)) + self.randomMotorOffsetDegree[11]
        self.ChangeMotorPosition(self.robertId,self.legs[3][0],action[3] + math.radians(self.randomMotorOffsetDegree[9]))
        self.ChangeMotorPosition(self.robertId,self.legs[3][1],-math.radians(B))
        self.ChangeMotorPosition(self.robertId,self.legs[3][2],-math.radians(C))
        #left second
        B = (90 - (action[10] * 90)) + self.randomMotorOffsetDegree[13]
        C = (-135 + (action[10]*90)) + self.randomMotorOffsetDegree[14]
        self.ChangeMotorPosition(self.robertId,self.legs[4][0],action[4]+ math.radians(self.randomMotorOffsetDegree[12]))
        self.ChangeMotorPosition(self.robertId,self.legs[4][1],-math.radians(B))
        self.ChangeMotorPosition(self.robertId,self.legs[4][2],-math.radians(C))
        #left third
        B = (90 - (action[11] * 90)) + self.randomMotorOffsetDegree[16]
        C = (-135 + (action[11]*90)) + self.randomMotorOffsetDegree[17]
        self.ChangeMotorPosition(self.robertId,self.legs[5][0],action[5]+ math.radians(self.randomMotorOffsetDegree[15]))
        self.ChangeMotorPosition(self.robertId,self.legs[5][1],-math.radians(B))
        self.ChangeMotorPosition(self.robertId,self.legs[5][2],-math.radians(C))

        self.renderedFrames = []
        #Step 24 times -> ~100ms and add some latency +-16ms
        #Step 240 times -> 1S
        latencyFrames = random.randrange(-4,4)
        for i in range(240+latencyFrames):
            p.stepSimulation()
            if self.renderCurrentSim == True and (i+1)%4==0:
                self.renderedFrames.append(self.renderInternal())
            #Active this command if you want so have a "real-time" simulation -> only for visuals NOT for training!
            #time.sleep(1/200)
        
        self.currentStep += 1

        #Reward-----------------------------------------------------------------------------------------------
        #Position gain reward:
        baseState = p.getLinkState(self.robertId,0)
        basePosition = baseState[0]
        targetFWD = np.asarray([1,0,0])
        deltaPos = np.asarray(basePosition) - np.asarray(self.lastPosition)
        positionGain = np.dot(deltaPos,targetFWD)
        
        #Energy reward:
        jointStates = p.getJointStates(self.robertId,jointIndices=self.motors)
        motorVelocities = [item[0] for item in jointStates]
        motorTorques = [item[1] for item in jointStates]
        w = -0.001
        deltaTime = 1.0/240.0
        energyConsumtion = w * deltaTime * np.absolute(np.dot(motorTorques,motorVelocities))

        #Rotation reward:
        #Calculate the fwd and right vector
        baseOrientation = p.getLinkState(self.robertId,0)[1]
        eulerAngles = p.getEulerFromQuaternion(baseOrientation)
        M = p.getMatrixFromQuaternion(baseOrientation)
        row1 = np.asarray(M[0:3])
        row2 = np.asarray(M[3:6])
        row3 = np.asarray(M[6:9])
        Q = np.asarray([row1,row2,row3])
        baseFwd = np.dot(Q,targetFWD)
        targetRight = np.asarray([0,1,0])
        baseRight = np.dot(Q, targetRight)

        #p.addUserDebugLine(basePosition,np.asarray(basePosition)+np.asarray(baseRight),lifeTime=0.1)
        rotationRewardFactor = 0.01
        rotationReward = rotationRewardFactor * (np.dot(targetFWD,baseFwd) + np.dot(targetRight,baseRight)) / 2
        
        #Height reward:
        heightRewardFactor = 0.005
        targetHeight = 0.14
        heightReward = -1 * heightRewardFactor * abs(targetHeight - basePosition[2])
        
        #Total reward:
        reward = positionGain + rotationReward  + heightReward + energyConsumtion + heightReward
        #print(round(positionGain,4),round(rotationReward,4),round(heightReward,4),round(energyConsumtion,4),round(heightReward,4))



        #Store state
        self.angularVelocity = p.getBaseVelocity(self.robertId)[1]
        self.lastPosition = basePosition
        self.rotation = [eulerAngles[0],eulerAngles[1],eulerAngles[2]]

        done = False
        #Check if robot has fallen on his "back"
        terminateRotationDegree = 20
        if self.rotation[0] < math.radians(-terminateRotationDegree) or self.rotation[0] > math.radians(terminateRotationDegree) or self.rotation[1] < math.radians(-terminateRotationDegree) or self.rotation[1] > math.radians(terminateRotationDegree):
            done = True
            reward -= 0.1 
        else:
            done = False

        if basePosition[2] < 0.1:
            reward -= 0.1

        
        if self.currentStep > 500:
            done = True
        
        info = {}

        return self.GetState(), reward, done, info

    def reset(self):    
        self.motors = [None]*self.numMotors
        self.motorTargetPositions = [None]*self.numMotors
        self.lastPosition = [0,0,0]
        self.jointNameToId = {}
        self.linkNameToId = {}
        self.currentStep = 0
        self.legs = [None]*self.numLegs
        self.renderCurrentSim = False
        self.randomMotorOffsetDegree = [None]*self.numMotors

        p.resetSimulation()

        #default timestep is 1/240second = 4.17 ms
        p.setGravity(0,0,-9.81)
        #p.setGravity(0,0,0)
        p.setPhysicsEngineParameter(numSolverIterations=100,enableConeFriction=0)

        planeId = p.loadURDF("plane.urdf")


        robertStartPos = [0,0,0.25]
        self.lastPosition = robertStartPos
        robertStartOrientation = p.getQuaternionFromEuler([0,0,0])


        urdfPath = os.path.join(os.path.join(os.path.join(os.path.join(os.path.join(os.path.dirname(os.path.realpath(__file__)),".."),".."),"data")),"Robert2.urdf")
        #print("Load URDF file:",urdfPath)
        #|p.URDF_USE_SELF_COLLISION
        self.robertId = p.loadURDF(urdfPath,robertStartPos, robertStartOrientation,flags=p.URDF_MAINTAIN_LINK_ORDER|p.URDF_USE_SELF_COLLISION)

        #Names to ids-----------------------------------------------------------------------------------------------------------
        nJoints = p.getNumJoints(self.robertId)
        for i in range(nJoints):
            jointInfo = p.getJointInfo(self.robertId, i)
            self.jointNameToId[jointInfo[1].decode('UTF-8')] = jointInfo[0]

        self.linkNameToId = {p.getBodyInfo(self.robertId)[0].decode('UTF-8'):-1,}#base = -1
        for i in range(nJoints):
            jointInfo = p.getJointInfo(self.robertId, i)
            name = jointInfo[12].decode('UTF-8')
            self.linkNameToId[name] = i



        #Motors------------------------------------------------------------------------------------------------------------------------------
        #right side
        self.motors[0] = self.jointNameToId['right_first_leg_Base_to_A']
        self.motors[1] = self.jointNameToId['right_first_leg_A_to_B']
        self.motors[2] = self.jointNameToId['right_first_leg_B_to_C']
        self.legs[0] = [self.motors[0],self.motors[1],self.motors[2]]
        self.motors[3] = self.jointNameToId['right_second_leg_Base_to_A']
        self.motors[4] = self.jointNameToId['right_second_leg_A_to_B']
        self.motors[5] = self.jointNameToId['right_second_leg_B_to_C']
        self.legs[1] = [self.motors[3],self.motors[4],self.motors[5]]
        self.motors[6] = self.jointNameToId['right_third_leg_Base_to_A']
        self.motors[7] = self.jointNameToId['right_third_leg_A_to_B']
        self.motors[8] = self.jointNameToId['right_third_leg_B_to_C']
        self.legs[2] = [self.motors[6],self.motors[7],self.motors[8]]
        #left side
        self.motors[9] = self.jointNameToId['left_first_leg_Base_to_A']
        self.motors[10] = self.jointNameToId['left_first_leg_A_to_B']
        self.motors[11] = self.jointNameToId['left_first_leg_B_to_C']
        self.legs[3] = [self.motors[9],self.motors[10],self.motors[11]]
        self.motors[12] = self.jointNameToId['left_second_leg_Base_to_A']
        self.motors[13] = self.jointNameToId['left_second_leg_A_to_B']
        self.motors[14] = self.jointNameToId['left_second_leg_B_to_C']
        self.legs[4] = [self.motors[12],self.motors[13],self.motors[14]]
        self.motors[15] = self.jointNameToId['left_third_leg_Base_to_A']
        self.motors[16] = self.jointNameToId['left_third_leg_A_to_B']
        self.motors[17] = self.jointNameToId['left_third_leg_B_to_C']
        self.legs[5] = [self.motors[15],self.motors[16],self.motors[17]]
  
        

        for i in range(len(self.motors)):
            motor = self.motors[i]
            motorStartPosition = p.getJointState(self.robertId,motor)[0]
            self.motorTargetPositions[i] = motorStartPosition
            p.enableJointForceTorqueSensor(self.robertId,motor)
            #Add 5 Degree random offset because the real motors are not perfect aligned.
            self.randomMotorOffsetDegree[i] = random.randrange(-5,5)



        #Mass
        dynamicsInfo = p.getDynamicsInfo(self.robertId,self.linkNameToId["base_bottom"])
        massWeight = random.uniform(0.7,1.5)
        p.changeDynamics(self.robertId,self.linkNameToId["base_bottom"],mass=dynamicsInfo[0]*massWeight)

        dynamicsInfo = p.getDynamicsInfo(self.robertId,self.linkNameToId["base_top"])
        massWeight = random.uniform(0.7,1.5)
        p.changeDynamics(self.robertId,self.linkNameToId["base_top"],mass=dynamicsInfo[0]*massWeight)

        #Friction-------------------------------------------------------------------------------------------------------------------------
        #Ground
        #groundFriction = random.uniform(0.7, 1.0)
        #dynamicsInfo = p.getDynamicsInfo(planeId,-1)
        #p.changeDynamics(planeId,-1,lateralFriction=groundFriction)
        #Base
        #p.changeDynamics(self.robertId,-1,lateralFriction=0.3)
        #Robert
        #dynamicsInfo = p.getDynamicsInfo(self.robertId,self.linkNameToId['right_first_leg_C'])

        #lFriction = 0.9

        #p.changeDynamics(self.robertId,self.linkNameToId['right_first_leg_C'],lateralFriction=lFriction)
        #p.changeDynamics(self.robertId,self.linkNameToId['left_first_leg_C'],lateralFriction=lFriction)
        
        #p.changeDynamics(self.robertId,self.linkNameToId['right_third_leg_C'],lateralFriction=lFriction)
        #p.changeDynamics(self.robertId,self.linkNameToId['left_third_leg_C'],lateralFriction=lFriction)
        
        #p.changeDynamics(self.robertId,self.linkNameToId['right_second_leg_C'],lateralFriction=lFriction)
        #p.changeDynamics(self.robertId,self.linkNameToId['left_second_leg_C'],lateralFriction=lFriction)
        

        
        #default position
        self.ChangeMotorPosition(self.robertId,self.jointNameToId['right_first_leg_Base_to_A'],math.radians(0+self.randomMotorOffsetDegree[0]))
        self.ChangeMotorPosition(self.robertId,self.jointNameToId['right_second_leg_Base_to_A'],math.radians(0+self.randomMotorOffsetDegree[1]))
        self.ChangeMotorPosition(self.robertId,self.jointNameToId['right_third_leg_Base_to_A'],math.radians(0+self.randomMotorOffsetDegree[2]))

        self.ChangeMotorPosition(self.robertId,self.jointNameToId['left_first_leg_Base_to_A'],math.radians(0+self.randomMotorOffsetDegree[3]))
        self.ChangeMotorPosition(self.robertId,self.jointNameToId['left_second_leg_Base_to_A'],math.radians(0+self.randomMotorOffsetDegree[4]))
        self.ChangeMotorPosition(self.robertId,self.jointNameToId['left_third_leg_Base_to_A'],math.radians(0+self.randomMotorOffsetDegree[5]))
  

        self.ChangeMotorPosition(self.robertId,self.jointNameToId['right_first_leg_A_to_B'],math.radians(90+self.randomMotorOffsetDegree[6]))
        self.ChangeMotorPosition(self.robertId,self.jointNameToId['right_second_leg_A_to_B'],math.radians(90+self.randomMotorOffsetDegree[7]))
        self.ChangeMotorPosition(self.robertId,self.jointNameToId['right_third_leg_A_to_B'],math.radians(90+self.randomMotorOffsetDegree[8]))

        self.ChangeMotorPosition(self.robertId,self.jointNameToId['left_first_leg_A_to_B'],math.radians(-90+self.randomMotorOffsetDegree[9]))
        self.ChangeMotorPosition(self.robertId,self.jointNameToId['left_second_leg_A_to_B'],math.radians(-90+self.randomMotorOffsetDegree[10]))
        self.ChangeMotorPosition(self.robertId,self.jointNameToId['left_third_leg_A_to_B'],math.radians(-90+self.randomMotorOffsetDegree[11]))

        self.ChangeMotorPosition(self.robertId,self.jointNameToId['right_first_leg_B_to_C'],math.radians(-135+self.randomMotorOffsetDegree[12]))
        self.ChangeMotorPosition(self.robertId,self.jointNameToId['right_second_leg_B_to_C'],math.radians(-135+self.randomMotorOffsetDegree[13]))
        self.ChangeMotorPosition(self.robertId,self.jointNameToId['right_third_leg_B_to_C'],math.radians(-135+self.randomMotorOffsetDegree[14]))

        self.ChangeMotorPosition(self.robertId,self.jointNameToId['left_first_leg_B_to_C'],math.radians(135+self.randomMotorOffsetDegree[15]))
        self.ChangeMotorPosition(self.robertId,self.jointNameToId['left_second_leg_B_to_C'],math.radians(135+self.randomMotorOffsetDegree[16]))
        self.ChangeMotorPosition(self.robertId,self.jointNameToId['left_third_leg_B_to_C'],math.radians(135+self.randomMotorOffsetDegree[17]))

        for i in range(1000):
            p.stepSimulation()

        return self.GetState()



