# Die Klassen (Baupläne) für den Roboter und den GPS-Sensor werden importiert, damit Roboter- und GPS-Objekte erstellt werden können.
from controller import Robot, GPS # pyright: ignore[reportMissingImports]
from collections import deque
import numpy as np
import math
import struct # Use the struct module in order to pack the message sent to the Main Supervisor


# Die Zeit für einen Simulationsschritt des Simulators wird gesetzt
timestep = 32            

# Die maximale Geschwindigkeit ist für den Roboter vorgegeben (geht aus der Dokumentation des Roboter hervor). Hier wird sie einer Variable zugewiesen 
max_velocity = 6.28      

# Ein Roboter-Objekt wird erstellt
robot = Robot()

receiver = robot.getDevice("receiver") # Retrieve the receiver and emitter by device name
emitter = robot.getDevice("emitter")

receiver.enable(timestep) # Enable the receiver. Note that the emitter does not need to call enable()


#Definiere die Räder
wheel1 = robot.getDevice("wheel1 motor")   #Ein Objekt des Motors des linken Rades wird erstellt
wheel2 = robot.getDevice("wheel2 motor")   #Ein Objekt des Motors des rechten Rades wird erstellt

#Setzte die Motoren der Räder auf unendliche Roration, d. h. die Räder hören nicht auf zu drehen, fallls mit einer bestimmten Geschwindigkeit angestellt wurden 
wheel1.setPosition(float("inf"))       
wheel2.setPosition(float("inf"))

rotationsSensorLinks = wheel1.getPositionSensor()    # Encoder initialization
rotationsSensorRechts = wheel2.getPositionSensor()

#Aktiviere die Rotationssensoren (ACHTUNG: Alle Sensoren müssen aktiviert werden, damit man sie nutzen kann).
rotationsSensorLinks.enable(timestep)   
rotationsSensorRechts.enable(timestep)

lidar = robot.getDevice("lidar") # Step 2: Retrieve the sensor, named "lidar", from the robot. Note that the sensor name may differ between robots.
lidar.enable(timestep) # Step 3: Enable the sensor, using the timestep as the update rate

'''
Die Bezeichner ps0, ps2, usw entsprechen den Vorgaben des e-puck Roboters. 
Achtung: Falls ein eigener Roboter verwendet wird, müssen diese Bezeichner in "distance sensor1", "distance sensor2", usw geändert werden.
Die Datei "custom_robot.proto" sollte immer für solche Differenzen gelesen werden.
'''
"""
#Definiere die US-Sensoren
usVorneRechts_0 = robot.getDevice("ps0")
usHalbVornerechts_1 = robot.getDevice("ps1")
usrechts_2 = robot.getDevice("ps2")
usHintenRechts_3 = robot.getDevice("ps3")
usHintenLinks_4 = robot.getDevice("ps4")
usLinks_5 = robot.getDevice("ps5")
usHalbVorneLinks_6 = robot.getDevice("ps6")
usVorneLinks_7 = robot.getDevice("ps7")
"""
#Definiere den GPS-Sensor
gps = robot.getDevice("gps")

#Definiere den Farbsensor
colorSensor = robot.getDevice("colour_sensor")

"""
#Aktiviere die US-Sensoren (ACHTUNG: Alle Sensoren müssen aktiviert werden, damit man sie nutzen kann).
usVorneRechts_0.enable(timestep)
usHalbVornerechts_1.enable(timestep)
usrechts_2.enable(timestep)
usHintenRechts_3.enable(timestep)
usHintenLinks_4.enable(timestep)
usLinks_5.enable(timestep)
usHalbVorneLinks_6.enable(timestep)
usVorneLinks_7.enable(timestep)
"""
#Aktiviere den GPS-Sensor
gps.enable(timestep)

#Aktiviere den Farbsensor
colorSensor.enable(timestep) 


#Die Startzeit für den Roboter wird gespeichert. Damit kann man später ausrechnen, wie lange der Roboter schon aktiv ist und wieviel Zeit verbleibt.
startZeit = robot.getTime()
    
def delay(time:int):
	initTime = robot.getTime()
	while robot.step(timestep) != -1:
		if (robot.getTime() - initTime) * 1000.0 > time:
			return None

class MappingBase:
    
    def __init__(self):
        
        self.gpsX = gps.getValues()[0]
        self.gpsZ = gps.getValues()[2]
        
        self.tile = [0],[0]
        
        self.largestX = 0
        self.largestZ = 0
        
        self.smallestX = 0
        self.smallestZ = 0

        self.resetted = False
    
        
            
    def calculateToStartingTile(self, coordinateX, coordinateZ):
            return [int(round(coordinateX/12,2)*100),
                    int(round(coordinateZ/12,2)*100)]
    
    def calculateToTile(self, coordinateX, coordinateZ):
        return [int(round(coordinateX/12,2)*100 - self.startingtile[0] - self.smallestX),
        int(round(coordinateZ/12,2)*100 - self.startingtile[1] - self.smallestZ)]
        
    def calculateArea(self):
        self.areaSize = (self.largestX+1)*(self.largestZ+1)
        print(f'area: {self.areaSize}')
        
        
    
    
    
    def updateMapArray(self):
        print(f'Feld:{self.tile}')
        self.mapArray[self.tile[0], self.tile[1]] = 1
        print(f'MapArray: {self.mapArray}')
        
        
    def padMapArray(self, width):
        print(f'width= {width}')
        self.mapArray = np.pad(self.mapArray, pad_width = ((0, width[1][1]), (0, width[0][1])), mode='constant', constant_values=0)
        print(f'mapArray{self.mapArray}')
       
    
    
        
        
    

class WorldMapping(MappingBase):
    def __init__(self):
        super().__init__()
        
        self.mapArray = np.array([['1', '1', '1', '1'], 
                                  ['1', '5', '0', '5'], 
                                  ['1', '0', '0', '0'],
                                  ['1', '5', '0', '5']], dtype=str)
        
        self.largestX = 0
        self.largestZ = 0
        

        
        self.xTiles = 1
        self.zTiles = 1
        
        self.gpsX = gps.getValues()[0]
        self.gpsZ = gps.getValues()[2]
        self.ninth = [1],[1]
        self.startingtile = self.calculateToNinthStartingTile(self.gpsX, self.gpsZ)
        self.smallestX = self.startingtile[0]
        self.smallestZ = self.startingtile[1]
        print(f'startfingtile= {self.startingtile}')
        

        self.resetted = False
        
    def calculateToNinthTile(self, coordinateX, coordinateZ):
        tempX= int(round((coordinateX/12)*3,2)*100 - self.smallestX)
        tempZ= int(round((coordinateZ/12)*3,2)*100 - self.smallestZ)
        print(f'ohne start: {(coordinateX/12)*3*100},ergebnis: {tempX + int((tempX+1)/3)},start: {self.startingtile[0]}, tempX: {tempX}, kleinstes: {self.smallestX}, teile: {(tempX+1)/3}')
        print(f'tempZ: {tempZ}, kleinstes: {self.smallestZ}, teile: {(tempZ+1)/3}')
        return [tempX + int((tempX+1)/3+2/3), tempZ + int((tempZ+1)/3+2/3)]
    
    def calculateToNinthStartingTile(self, coordinateX, coordinateZ):
        return [int(round((coordinateX/12)*3,2)*100),
        int(round((coordinateZ/12)*3,2)*100)] #muss zu niedrigstes Feld geändert werden!
    
    def updateGpsValues(self):
        self.gpsX = gps.getValues()[0]
        self.gpsZ = gps.getValues()[2]
        self.ninth = self.calculateToNinthTile(self.gpsX, self.gpsZ)

        
    def updateArraySize(self):
        difference:list = [[0, 0],[0, 0]]
        if self.ninth[0] > self.largestX:
            difference[0][1] = int(self.ninth[0]-self.largestX)
            self.largestX = self.ninth[0]
            
        elif self.ninth[0] < self.smallestX:
            difference[0][0] = int(abs(self.ninth[0]-self.smallestX))
            self.smallestX = self.ninth[0]
         
           
        if self.ninth[1] > self.largestZ:
            difference[1][1] = int(self.ninth[1]-self.largestZ)
            self.largestZ = self.ninth[1]
            
        elif self.ninth[1] < self.smallestZ:
            difference[1][0] = int(abs(self.ninth[1]-self.smallestZ))
            self.smallestZ = self.ninth[1]
        

        if sum(difference[0]) + sum(difference[1])>0:  
            print("distanzen:" + str([self.smallestX, self.largestX, self.smallestZ, self.largestZ]))
            self.padMapArray(difference)    

        self.xTiles = (self.largestX+1)/3 + (self.smallestX+1)/3 +1
        self.zTiles = (self.largestZ+1)/3 + (self.smallestX+1)/3 +1
        
    
    def calculateNinthTileToTile(self, xTile:int, zTile:int):
        return [int(xTile/4), int(zTile/4)]
        
    def calculateTileToCenterOfNinthTile(self, xTile:int, zTile:int):
        return [int(xTile*4+3), int(zTile*4+3)]

class MotionController:
    
    def forward():
        wheel1.setVelocity(6.0)
        wheel2.setVelocity(6.0)
    
    def backward():
        wheel1.setVelocity(-6.0)
        wheel2.setVelocity(-6.0)

    def turnRight():
        wheel1.setVelocity(-6.0)
        wheel2.setVelocity(6.0)

    def turnLeft():
        wheel1.setVelocity(6.0)
        wheel2.setVelocity(-6.0)

    def stop():
        wheel1.setVelocity(0)
        wheel2.setVelocity(0)      

                
    def ninetyDegreeRotationRight(): 
        startwert = rotationsSensorRechts.getValue() 
        print("anfang rechtsdrehung")

        i:int = 0
        while i < 30 and robot.step(timestep) == 0 and rotationsSensorRechts.getValue() < (startwert + math.pi/2 + math.pi/6):
            MotionController.turnRight()
            delay(20)
            i+= 1
            pass
        robot.step(timestep)
        MotionController.stop()

    def ninetyDegreeRotationLeft():
        startwert = rotationsSensorLinks.getValue() 
        print("anfang linksdrehung")

        
        i:int = 0
        while i < 30 and robot.step(timestep) == 0 and rotationsSensorLinks.getValue() < (startwert + math.pi/2 + math.pi/6):
            MotionController.turnLeft()
            delay(20)
            i+= 1
            pass
        robot.step(timestep)
        MotionController.stop()



MotionController.stop()
delay(1000)
MotionController.stop()
startcoords = gps.getValues()
print(gps.getValues())
print(startcoords)
startcoords.pop(1)
print(f'startcoords: {startcoords}')
startfield = [startcoords[0]/12, startcoords[1]/12]


while robot.step(timestep) != -1:
    MotionController.forward()
    x = gps.getValues()[0]
    z = gps.getValues()[2]
    xField = (round(x/12-startfield[0],2))*100
    zField = (round(z/12-startfield[1],2))*100
    print(x)
    print(z)
    print(xField)
    print(zField)
    print(startfield)
    print(xField)
    print(zField)
    xTemp = (round(10/3*x/12-startfield[0],2))*100
    zTemp = (round(10/3*z/12-startfield[1],2))*100
    print(xTemp)
    print(zTemp)    
