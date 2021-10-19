# requires PySerial: pip install pyserial
# make sure call close() when done to close the used pose
import serial
import math
import time
import re
import pandas as pd
import numpy as np


# Input: 12 actions #Output: 18 motor rotation in degree
def ActionsToAngle(actions, numMotors, numLegs):
    # 18Motors.
    # 3Elements per leg. #First element inside leg = Motor A
    # Second element inside leg = Motor B
    # Second element inside leg = Motor C
    # Motor layout: C-B-A-[Robo]-A-B-C
    # 0..2 right first leg
    # 3..5 right second leg
    # 6..8 right third leg
    # 9..11 left first leg
    # 12..14 left second leg
    # 15..17 left third leg
    angles = [0] * numMotors
    for i in range(0, numLegs):
        anglesArrayOffset = i * 3
        # The first 6 actions are in radians for all 6 motors.
        # So only map them to degree
        # Motor A
        angles[0 + anglesArrayOffset] = int(math.degrees(actions[i]))

        # Left side angle B and C are swapped
        sign = 1
        # if i > 2:
        # sign = -1

        # The last 6 actions are in range 0..1. #So calculate the degrees.
        # Motor B
        angles[1 + anglesArrayOffset] = sign * (90 - (actions[6 + i] * 90))
        # Motor C
        angles[2 + anglesArrayOffset] = sign * (-135 + (actions[6 + i] * 90))
    return angles


class Schnittstelle:
    portname = None
    numberOfMotors = None
    motorangles = None
    port = None

    def __init__(self, portname, number_of_motors):
        self.portname = portname
        self.numberOfMotors = number_of_motors
        self.motorangles = [0] * number_of_motors
        self.port = serial.Serial(portname, timeout=1)  # open serial port
        print("opening Port...")
        # you need to sleep after opening the port for a few seconds
        time.sleep(5)  # arduino takes a few seconds to be ready ...
        print("using port: ", self.port.name)  # check which port was really used

    def go_for_angles(self, angles):
        # angles: angles of servomotors from 0 to Pi
        serial_string = ""
        # check which positions changed and append to serialString
        for index, angle in enumerate(angles, start=0):
            # print(index)
            # if angle != self.motorangles[index]:
            # serialString looks like this: #1 90#3 60#4 70...
            serial_string += "#" + str(index) + " " + str(round(angle, ndigits=3)) + ";"
            # update self.motorangles
            self.motorangles[index] = angle
        # send serialString
        self.port.reset_output_buffer()
        print("SENT:", serial_string + "\r")
        self.port.write((serial_string + "\r").encode())

    def init(self):
        self.port.write(("init" + "\r").encode())

    def close(self):
        self.port.close()

    def readStatus(self):
        self.port.reset_input_buffer()
        line = ""
        while True:
            line = self.port.readline().decode()
            # print(line)
            if "!" in line:
                line = re.sub(r'.*!', '!', line)[1:]
                if '§' in line:
                    return line[:-2]
                else:
                    while True:
                        lineappend = self.port.readline().decode("utf-8")
                        if '§' in lineappend:
                            line += lineappend.split('§')[0]
                            return line
                        else:
                            line += lineappend



# networkoutput = [-0.5029466, -0.36549976, 0.24844654, 0.22432256, -0.26745978, -0.62270576, 0.6313647, 0.90693754,
#                 0.05260056, 0.23428851, 0.9118128, 0.6541247]

# val 0-5 = obere motoren winkel in bgm
# val 6-11 fuer andere mototren zustaendig

# networkoutput1 = [0, 0, 0, 0, 0, 0,
#                  0, 0, 0, 0, 0, 0]
# networkoutput2 = [0, 0, 0, 0, 0, 0,
#                  1, 1, 1, 1, 1, 1]

datasmoothed = pd.read_csv("actionlistsmoothed3.csv")
#data = pd.read_csv("actionlist.csv")


networkOutputList = datasmoothed.values.tolist()


def setTopMotors(angle):
    for i in [0, 3, 6, 9, 12, 15]:
        angles[i] = angle


def setTopFrontMotors(angle):
    for i in [0, 9]:
        angles[i] = angle


def setTopBackMotors(angle):
    for i in [6, 15]:
        angles[i] = angle


def setMiddleMotors(angle):
    for i in [1, 4, 7, 10, 13, 16]:
        angles[i] = angle


def setBottomMotors(angle):
    for i in [2, 5, 8, 11, 14, 17]:
        angles[i] = angle


#open port
mySchnittStelle = Schnittstelle(portname="\\COM3", number_of_motors=18)
# Microcontoller needs 4 seconds to Init
time.sleep(5)

for actions in networkOutputList:

    #actions[5] = 1

    #actions[6:12] = 0.5
    # actions[0] = -0.78
    # actions[2] = +0.78
    # actions[3] = +0.78
    # actions[5] = -0.78
    # actions[1] = +0.4
    # actions[4] = -0.4

    #for i in range(6,12):
        #actions[i] /= 2

    newAngles = ActionsToAngle(actions, 18, 6)
    print("Angles without offset: ", newAngles)
    angles = [90] * 18
    # topmotors
    angles[0] = newAngles[0] + 45
    angles[6] = newAngles[6] + 135
    angles[9] = newAngles[9] + 135
    angles[15] = newAngles[15] + 45
    angles[3] = newAngles[3] + 90
    angles[12] = newAngles[12] + 90
    # middle motors
    angles[1] = -newAngles[1] + 135
    angles[4] = -newAngles[4] + 135
    angles[7] = -newAngles[7] + 135
    angles[10] = newAngles[10] + 45
    angles[13] = newAngles[13] + 45
    angles[16] = newAngles[16] + 45
    # bottom motors
    angles[2] = newAngles[2] + 180
    angles[5] = newAngles[5] + 180
    angles[8] = newAngles[8] + 180
    angles[11] = -newAngles[11]
    angles[14] = -newAngles[14]
    angles[17] = -newAngles[17]

    """
    #raw offsets
    angles = [90] * 18
    angles[0] = +45
    angles[6] = 135
    angles[9] = 135
    angles[15] = 45
    
    angles[1] = 45
    angles[4] = 45
    angles[7] = 45
    
    angles[10] = 135
    angles[13] = 135
    angles[16] =135
    
    angles[2] = 45
    angles[5] = 45
    angles[8] = 45
    
    angles[11] = 135
    angles[14] = 135
    angles[17] = 135
    """

    # setTopMotors(0)
    # setTopMotors(90)
    # setTopFrontMotors(45)
    # setTopBackMotors(135)

    # setMiddleMotors(40)
    # setBottomMotors(35)

    # angles = getAnglesFromNetworkOutput(networkoutput1)


    # middlemotors=

    print("Final angles:", angles)

    mySchnittStelle.go_for_angles(angles)
    # readStatus Poll modus --> wais for response!
    status = mySchnittStelle.readStatus().split('!')[0].split('§')[0].split(';')
    #time.sleep(0.5)
"""
for statusSet in networkOutputList:
    newAngles = ActionsToAngle(statusSet, 18, 6)
    angles[0] = newAngles[0] + 90
    angles[3] = newAngles[3] + 90
    angles[6] = newAngles[6] + 90
    angles[9] = newAngles[9] + 90
    angles[12] = newAngles[12] + 90
    angles[15] = newAngles[15] + 90

    print(angles[0])
    print(angles[3])
    print(angles[6])
    print(angles[9])
    print(angles[12])
    print(angles[15])



    # Microcontoller needs 0.1 seconds to go for angle


    mySchnittStelle.go_for_angles(angles)
    # readStatus Poll modus --> wais for response!
    status = mySchnittStelle.readStatus().split('!')[0].split('§')[0].split(';')
    # print(status)
    #print("Ultraschallabstand in cm: ", status[0])
    #print("InfrarotRichtung in Grad: ", status[1])
    #print("accelerationX: ", status[2])
    #print("accelerationY: ", status[3])
    #print("accelerationZ: ", status[4])
    #print("temprature: ", status[5])
    #print("rotationAccelX: ", status[6])
    #print("rotationAccelY: ", status[7])
    #print("rotationAccelZ: ", status[8])
    #print("rotationX in Grad: ", status[9])
    #print("rotationY in Grad: ", status[10])
    #print("rotationZ in Grad: ", status[11])
    time.sleep(3)
    
"""
