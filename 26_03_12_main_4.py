# Die Klassen (Baupläne) für den Roboter und den GPS-Sensor werden importiert, damit Roboter- und GPS-Objekte erstellt werden können.
from controller import Robot, Camera, GPS # pyright: ignore[reportMissingImports]
from collections import deque
import numpy as np
import math
import struct # Use the struct module in order to pack the message sent to the Main Supervisor
import cv2

debug = True
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

distanceSensor = robot.getDevice("distance sensor1")
distanceSensor.enable(timestep)

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

cam_c:Camera = robot.getDevice("camera2")
cam_r:Camera = robot.getDevice("camera3")
cam_l:Camera = robot.getDevice("camera1")

cam_c.enable(timestep)
cam_r.enable(timestep)
cam_l.enable(timestep)

#Die Startzeit für den Roboter wird gespeichert. Damit kann man später ausrechnen, wie lange der Roboter schon aktiv ist und wieviel Zeit verbleibt.
startZeit = robot.getTime()
    
def delay(time:int):
	initTime = robot.getTime()
	while robot.step(timestep) != -1:
		if (robot.getTime() - initTime) * 1000.0 > time:
			return None

class SensorBase:
    pass

class UsSensorEvaluator:
    def __init__(self):
        self.holeInFront = False

    def updateHole(self):
        if distanceSensor.getValue()>0.79:
            self.holeInFront = True
    
    def checkHoleInFront(self):
        if 0.51<distanceSensor.getValue()<0.55:
            print("us loch")
            return True
        
        elif distanceSensor.getValue()<0.2:
                  print("kein Loch")
                  self.holeInFront = False

class ColorSensor:
    
    def readColor():
        image = colorSensor.getImage()

        r = colorSensor.imageGetRed(image, 1, 0, 0)
        g = colorSensor.imageGetGreen(image, 1, 0, 0)
        b = colorSensor.imageGetBlue(image, 1, 0, 0)
        
        return [r,g,b]
    
    def checkHole():
        image = colorSensor.getImage()

        r = colorSensor.imageGetRed(image, 1, 0, 0)
        g = colorSensor.imageGetGreen(image, 1, 0, 0)
        b = colorSensor.imageGetBlue(image, 1, 0, 0)
        
        if ((r<50)and(g<50)and(b<50)):#überpüfe vorne auf Loch mit Colorsensor
                print ("Loch")
                
                return True

class LidarEvaluator:
    def __init__(self):
        self.leftRotation = False
        
    def checkleftRotation(self):
        rangeImage = lidar.getRangeImage()
        if self.leftRotation == True and round(rangeImage[1024], 3) <0.069: #bleibt solange im Modus "nach links gedreht,"" bis man links an einer Wand vorbeigefahren ist
            self.leftRotation = False

    
    def checkNoLeftWall(self):
        rangeImage = lidar.getRangeImage(); # Step 4: Retrieve the range image      
        if  (round(rangeImage[1024], 3)) >0.1 and (round(rangeImage[1067], 3)) >0.1 and (round(rangeImage[1494], 3))>0.1 and self.leftRotation == False:
            print('keine Wand links')
            print('lv:'+str(round(rangeImage[1067],3)) + " ", end='')
            print('l'+str(round(rangeImage[1152],3)) + " ", end='')
            print('lh:'+str(round(rangeImage[1494],3)) + " ", end='')
            return True
        
    def checkFrontWall():
        rangeImage = lidar.getRangeImage(); # Step 4: Retrieve the range image
        if (round(rangeImage[1152], 3)) < 0.069 and (round(rangeImage[1142], 3)) < 0.08 and (round(rangeImage[1162], 3)) < 0.08:
            print('Wand vorne')
            print(str(round(rangeImage[1152],3)) + " ", end='')
            return True
        
    def checkLeftWall():
        rangeImage = lidar.getRangeImage(); # Step 4: Retrieve the range image
        if (round(rangeImage[1152], 3)) < 0.069:
            print('Wand vorne')
            print(str(round(rangeImage[1024],3)) + " ", end='')
            return True
        
    def checkRightWall():
        rangeImage = lidar.getRangeImage(); # Step 4: Retrieve the range image
        if(round(rangeImage[1280], 3)) <0.069:
            print('Wand rechts')
            print(str(round(rangeImage[1152],3)) + " ", end='')
            return True
        
    def checkLeftFrontWall():
        rangeImage = lidar.getRangeImage(); # Step 4: Retrieve the range image
        if (round(rangeImage[1067],3))<0.059:
            print('Wand links-vorne')
            print('lv:'+str(round(rangeImage[1067],3)) + " ", end='')
            return True
        
    def checkFrontLeftWall():
        rangeImage = lidar.getRangeImage(); # Step 4: Retrieve the range image
        if (round(rangeImage[1109],3))<0.059:
            print('Wand vorne-links')
            print('vl:'+str(round(rangeImage[1109],3)) + " ", end='')
            return True
        
    def checkRightFrontWall():
        rangeImage = lidar.getRangeImage(); # Step 4: Retrieve the range image
        if (round(rangeImage[1237],3))<0.06:
            print('Wand rechts-vorne')
            print('rv:'+str(round(rangeImage[1237],3)) + " ", end='')
            return True
        
    def checkFrontRightWall():
        rangeImage = lidar.getRangeImage(); # Step 4: Retrieve the range image 
        if (round(rangeImage[1195],3))<0.06:
            print('Wand vorne-rechts')
            print('vr:'+str(round(rangeImage[1195],3)) + " ", end='')   
            return True
            
class EmitterCommunicator:
    def signalEoP():
        emitter.send(bytes('E', "utf-8")) # Send the letter 'E' to signify exit
    
    def signalLoP():
        message = struct.pack('c', 'L'.encode()) # message = 'L' to activate lack of progress
        emitter.send(message) # Send message
        
    def requestGameInformation():
        message = struct.pack('c', 'G'.encode()) # message = 'G' for game information
        emitter.send(message) # send message
        
    def sendMap():
        ## Get shape
        s = worldMap.mapArray.shape
        ## Get shape as bytes
        s_bytes = struct.pack('2i',*s)

        ## Flattening the matrix and join with ','
        flatMap = ','.join(worldMap.mapArray.flatten())
        ## Encode
        sub_bytes = flatMap.encode('utf-8')

        ## Add togeather, shape + map
        a_bytes = s_bytes + sub_bytes

        ## Send map data
        emitter.send(a_bytes)

        #STEP3 Send map evaluate request
        map_evaluate_request = struct.pack('c', b'M')
        emitter.send(map_evaluate_request)

        #STEP4 Send an Exit message to get Map Bonus
        ## Exit message
        exit_mes = struct.pack('c', b'E')
        emitter.send(exit_mes)
        
    def sendPos(victimType:str):
        if victimType.isspace():
            print("Fehler: Kein VictimType angegeben, abbrechen")
            return
        print("Position für " + victimType + " wird gesendet")
        delay(1500)
        emitter.send(struct.pack("i i c", int(gps.getValues()[0] * 100), int(gps.getValues()[2] * 100), bytes(victimType, "utf-8")))
        delay(1500)


class ReceiverCommunicator:
    def checkInput()->bool:
        if receiver.getQueueLength() > 0: # If receiver queue is not empty
            print(f'Receiverqueue{receiver.getQueueLength()}')
            return True    
    def checkLoP()->bool:
        if ReceiverCommunicator.checkInput():
            receivedData = receiver.getBytes()
            tup = struct.unpack('c', receivedData) # Parse data into character
            if tup[0].decode("utf-8") == 'L': # 'L' means lack of progress occurred
                print("Detected Lack of Progress!")
                receiver.nextPacket() # Discard the current data packet
                return True
        else: return False
        
    def checkGameInformation():
        if ReceiverCommunicator.checkInput():
            receivedData = receiver.getBytes()
            tup = struct.unpack('c f i', receivedData) # Parse data into char, float, int
            if tup[0].decode("utf-8") == 'G':
                print(f'Game Score: {tup[1]}  Remaining time: {tup[2]}')
                receiver.nextPacket() # Discard the current data packet

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
            camE.check()
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
            camE.check()
        robot.step(timestep)
        MotionController.stop()

class BaseStrategy:
    pass

class RightHandStrategy(BaseStrategy):
    
    def __init__(self):
        self.leftRotation = False
    
    def followPath(self):
        rangeImage = lidar.getRangeImage(); # Step 4: Retrieve the range image      
        #überprüfe vorne und vorne auf linker sowie rechter Seite


        if self.leftRotation == True and round(rangeImage[1024], 3) <0.069: #bleibt solange im Modus "nach links gedreht,"" bis man links an einer Wand vorbeigeMotionControllern ist
            self.leftRotation = False

        
        if  LidarEvaluator.checkLeftWall() and self.leftRotation == False:
            MotionController.ninetyDegreeRotationLeft()
            self.leftRotation = True

        elif LidarEvaluator.checkFrontWall():
            MotionController.ninetyDegreeRotationRight()

        elif LidarEvaluator.checkRightWall():
            MotionController.ninetyDegreeRotationLeft()
            
        else:
            while LidarEvaluator.checkLeftFrontWall() and robot.step(timestep) != -1:#links-vorne
                MotionController.turnRight()            

            while LidarEvaluator.checkFrontLeftWall() and robot.step(timestep) != -1:#vorne liks
                MotionController.turnRight()

            while LidarEvaluator.checkRightFrontWall() and robot.step(timestep) != -1:#rechts-vorne
                MotionController.turnLeft()
                
            while LidarEvaluator.checkFrontRightWall() and robot.step(timestep) != -1:#vorne-rechts
                MotionController.turnLeft()
                
            print('drehen abgeschlossen')

class MapStrategy(BaseStrategy):
    pass

class GPSEvaluator:
    def __init__(self):
        self.newPoint = [gps.getValues()[0], gps.getValues()[2]]
        self.lastPoint = [gps.getValues()[0], gps.getValues()[2]]
    
    def updateValues(self):
        self.lastPoint = self.newPoint
        self.newPoint = [round(gps.getValues()[0], 3), round(gps.getValues()[2],3)]

    def stuck(self):
        self.updateValues()
        if self.lastPoint == self.newPoint:
            return True


class MapManager:
    pass

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
        self.mapArray[self.tile[1], self.tile[0]] = 1
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
        
        self.smallestX = 0
        self.smallestZ = 0
        
        self.xTiles = 1
        self.zTiles = 1
        
        self.gpsX = gps.getValues()[0]
        self.gpsZ = gps.getValues()[2]
        self.ninth = [1],[1]
        self.startingtile = self.calculateToNinthStartingTile(self.gpsX, self.gpsZ)

        print(f'startfingtile= {self.startingtile}')
        

        self.resetted = False
        
    def calculateToNinthTile(self, coordinateX, coordinateZ):
        tempX= int(round((coordinateX/12)*3,2)*100 - self.startingtile[0])
        tempZ= int(round((coordinateZ/12)*3,2)*100 - self.startingtile[1])
        print(f'ohne start: {(coordinateX/12)*3*100},ergebnis: {tempX + int((tempX+1)/3)},start: {self.startingtile[0]}, tempX: {tempX}, kleinstes: {self.smallestX}, teile: {(tempX+1)/3}')
        print(f'tempZ: {tempZ}, kleinstes: {self.smallestZ}, teile: {(tempZ+1)/3}')
        return [tempX + int((tempX+1)/3), tempZ + int((tempZ+1)/3)]
    
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

class RoomMapping(MappingBase):
    
    def __init__(self, pAreaNr):
        super().__init__()
        self.mapArray = np.array([[0]], dtype=int)
        
        self.areaNr = pAreaNr

        self.startingtile = self.calculateToStartingTile(self.gpsX, self.gpsZ)

        self.areaSize: int
        self.countVisited:int
        self.countNonVisited:int
        

        
        self.xDirection = True #True = Forward; False = Backward; Standing no change
        self.zDirection = True
        self.xDistance: float
        self.zDistance: float
        
    def updateGpsValues(self):
        self.gpsX = gps.getValues()[0]
        self.gpsZ = gps.getValues()[2]
        self.tile = self.calculateToTile(self.gpsX, self.gpsZ)

    
    def updateArraySize(self):
        difference:list = [[0, 0],[0, 0]]
        if self.tile[0] > self.largestX:
            difference[0][1] = int(self.tile[0]-self.largestX)
            self.largestX = self.tile[0]
            
        elif self.tile[0] < self.smallestX:
            difference[0][0] = int(abs(self.tile[0]-self.smallestX))
            self.smallestX = self.tile[0]
         
           
        if self.tile[1] > self.largestZ:
            difference[1][1] = int(self.tile[1]-self.largestZ)
            self.largestZ = self.tile[1]
            
        elif self.tile[1] < self.smallestZ:
            difference[1][0] = int(abs(self.tile[1]-self.smallestZ))
            self.smallestZ = self.tile[1]
        

        if sum(difference[0]) + sum(difference[1])>0:  
            print("distanzen:" + str([self.smallestX, self.largestX, self.smallestZ, self.largestZ]))
            self.padMapArray(difference)    
       

    def floodFillZeros(self):
        visited = np.zeros_like(self.mapArray, dtype=bool)
        rows, cols = self.mapArray.shape
        
        # Flood-Fill von außen starten, wo garantiert 0en sind (z. B. Rand)
        queue = deque()

        # Alle Ränder durchsuchen
        for r in range(rows):
            for c in [0, cols - 1]:
                if self.mapArray[r, c] == 0 and not visited[r, c]:
                    queue.append((r, c))
                    visited[r, c] = True
        for c in range(cols):
            for r in [0, rows - 1]:
                if self.mapArray[r, c] == 0 and not visited[r, c]:
                    queue.append((r, c))
                    visited[r, c] = True

        # BFS-Flood-Fill auf die 0er starten
        while queue:
            r, c = queue.popleft()
            for dr, dc in [(-1,0), (1,0), (0,-1), (0,1)]:
                nr, nc = r + dr, c + dc
                if 0 <= nr < rows and 0 <= nc < cols:
                    if self.mapArray[nr, nc] == 0 and not visited[nr, nc]:
                        visited[nr, nc] = True
                        queue.append((nr, nc))

        # Alle 0er, die vom Roboter nicht besucht wurden (also innen), bleiben 0
        # Alle 0er, die vom Roboter nicht besucht wurden (außen), werden zu 9
        for r in range(rows):
            for c in range(cols):
                if self.mapArray[r, c] == 0 and visited[r, c]:
                    self.mapArray[r, c] = 9
                    
        print(f'map: {self.mapArray}')
        self.countVisited = np.count_nonzero(self.mapArray == 1)
        self.countNonVisited = np.count_nonzero(self.mapArray == 0)
        self.areaSize = self.countNonVisited + self.countVisited
        print(f'tatsächliche area: {self.areaSize} visited: {self.countVisited} nonvisited: {self.countNonVisited}')

        
    
    


    
    def getAxisDistances(self):
        gpsX = gps.getValues()[0] # Step 4: Use the getValues() function to get the sensor readings
        gpsZ = gps.getValues()[2]
        
        print(f'Achsendistanzen:{self.gpsX}{self.gpsZ}{gpsX}{gpsZ}')
        
        self.xDistance = self.gpsX - gpsX
        self.zDistance = self.gpsZ - gpsZ
        
    def getDirection(self):
        self.getAxisDistances()
        if self.xDistance > 0:
            self.xDirection = True
        elif self.xDistance < 0:
            self.xDirection = False
        
        if self.zDistance > 0:
            self.zDirection = True
        elif self.zDistance < 0:
            self.zDirection = False
        print("directions:")
        print(f'xDirection {self.xDirection}')
        print(f'zDicection {self.zDirection}')
        
    def checkRoundCompleted(self)->bool:
        if ReceiverCommunicator.checkLoP():
            self.resetted = True
            return False
        elif self.checkStartingTile(): 
            if self.resetted == False:
                print("resetted: false und startingtile true")
                return True
        else: 
            if self.resetted == True: self.resetted = False
    
    def checkStartingTile(self)->bool:
        if self.largestX > 0 or self.largestZ > 0:
            if self.tile == [0,0]:
                print("Runde vollendet")
                return True
            else: print(f'Runde noch nicht vollendet {self.tile[0]} + {self.tile[1]} ungleich {0} + {0}')
        else: print("noch auf Startfeld")
             
class CamEvaluator:
    
    def check(self) -> str:
        bild:list = []
        img_l:np.ndarray = np.frombuffer(cam_l.getImage(), np.uint8).reshape((cam_l.getHeight(), cam_l.getWidth(), 4))
        img_c:np.ndarray = np.frombuffer(cam_c.getImage(), np.uint8).reshape((cam_c.getHeight(), cam_c.getWidth(), 4))
        img_r:np.ndarray = np.frombuffer(cam_r.getImage(), np.uint8).reshape((cam_r.getHeight(), cam_r.getWidth(), 4))

        bildL:list[int] = []
        bildC:list[int] = []        
        bildR:list[int] = []

        #print( "Links:" + str(erkennen.ecke()[1]))
        #print("Rechts:" + str(erkennen.ecke()[0]))
        # zonen.ecke kleiner halb Feld (2) oder kleiner ein Feld (1,2)
        if LidarEvaluator.checkFrontWall() :             #usVorneRechts_0.getValue() < 0.2: # or True:
            if debug: print("Überprüfe Vorne")
            bildR:list[int] = self.checkImg(img_c)
        if LidarEvaluator.checkLeftWall():             #usHalbVorneLinks_6.getValue() < 0.7:
            if debug: print("Überprüfe Links")
            bildL:list[int] = self.checkImg(img_l)
        if LidarEvaluator.checkRightWall():             #usHalbVorneRechts_1.getValue() < 0.7 and usVorneRechts_0.getValue() < 0.2:
            if debug: print("Überprüfe Rechts")
            bildR:list[int] = self.checkImg(img_r)
        
        waende:list = [2,2,2]#erkennen.ecke()  # [rechts, links, vorne]
        print(f'Bildlängen: C:{len(bildC)}, L:{len(bildL)}, R:{len(bildR)}')
        if len(bildC) > 5 and waende[2] == 2 :       # != 9
            bild = ["c", str(bildC[0]), str(bildC[1]), str(bildC[2]), str(bildC[3]), str(bildC[4]), str(bildC[5])]
        if len(bildL) > 5 and waende[1] == 2 :
            bild = ["l", str(bildL[0]), str(bildL[1]), str(bildL[2]), str(bildL[3]), str(bildL[4]), str(bildL[5])]
        if len(bildR) > 5 and waende[0] == 2 :
            bild = ["r", str(bildR[0]), str(bildR[1]), str(bildR[2]), str(bildR[3]), str(bildR[4]), str(bildR[5])]
        
        if len(bild) > 0:
            print("Bild auf Kamera " + str(bild[0]) + ": " + str(bild))
            
            self.scanImageOld(bild[0])  # ,bild
            return True
        return ""

    
    def getImage(cam) -> np.ndarray:
        return np.frombuffer( cam.getImage() , np.uint8).reshape(cam.getHeight(), cam.getWidth(), 4)
    
    
    def countColour(img) -> list:
        '''
        Counts pixels of specific power
        Returns:
        list of pixels: red, orange, black, white
        '''
        lower_red = np.array([ 60,  0, 185,   0], dtype = "uint8") 
        upper_red = np.array([100, 20, 240, 255], dtype = "uint8")
        mask_red:cv2.Mat = cv2.inRange(img, lower_red, upper_red)
        count_red = cv2.countNonZero(mask_red)
        
        lower_orange = np.array([0, 170, 185,   0], dtype = "uint8") 
        upper_orange = np.array([20, 220, 240, 255], dtype = "uint8") 
        mask_orange:cv2.Mat = cv2.inRange(img, lower_orange, upper_orange)
        count_orange = cv2.countNonZero(mask_orange)
        
        lower_black = np.array([ 0,  0,  0,   0], dtype = "uint8") 
        upper_black = np.array([20, 20, 20, 255], dtype = "uint8")
        mask_black:cv2.Mat = cv2.inRange(img, lower_black, upper_black)
        count_black = cv2.countNonZero(mask_black)
        
        lower_white = np.array([200, 200, 200, 0  ], dtype = "uint8") 
        upper_white = np.array([255, 255, 255, 255], dtype = "uint8")
        mask_white:cv2.Mat = cv2.inRange(img, lower_white, upper_white)
        count_white = cv2.countNonZero(mask_white)
        
        return[count_red, count_orange, count_black, count_white]
    
    def checkImg(self, img:np.ndarray) -> list:
        "@return Liste mit den Werten [x, y, w, h, area, ratio] des gefundenen Zeichens"
        or_img:np.ndarray = img.copy()      # Originalbild sichern
        o2_img:np.ndarray = img.copy()      # Originalbild sichern
        img:cv2.Mat = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) # Grayscale image
        #if saveImages:
            #cv2.imwrite(imagePath + "grayscale.png", img)
        #    pass
        img, thresh = cv2.threshold(img, 120, 255, cv2.THRESH_BINARY_INV) # Inverse threshold image (0-80 -> white; 80-255 -> black)
        contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) # Find all shapes within thresholded image
        #if debug:                     #120 -255                      #cv2.CHAIN_APPROX_NONE  #SIMPLE
            #pass
            #print(len(contours)) #gibt wieder wie oft wir auf ein Bild prüfen  
        #if saveImages:
        #   cam_r.saveImage(imagePath + ".png", 100)
        #  cv2.drawContours(or_img, contours, -1, (0, 255, 0), 3)  # Erst einzeichnen und dann als Bild speichern (Vergleichsbild zu Image.png)
            #cv2.imwrite(imagePath + "_contours.png", or_img)
            
        for cnt in contours:
            x, y, w, h = cv2.boundingRect(cnt)   # Find width and height of contour
            contArea = cv2.contourArea(cnt)   # Area covered by the shape
            ratio = w / h    # Calculate width to height ratio of contour
            
            # Wenn Kamera auf Bild zeigt: contArea > 400 and contArea < 900 and ratio > 0.5 and ratio < 1.5
            #if contArea > 200 and contArea < 900 and ratio > 0.5 and ratio < 1.5 and (x != 0 and y != 0 and x+w != 64 and y+h != 40):
            if ((contArea > 90 and contArea < 900 and ratio > 0.5 and ratio < 1.5) or ( contArea > 35 and ratio > 0.95 and ratio < 1.05)) : #sand (x != 0 and y != 0 and x+w != 64):  # y+h != 40)
                if debug:
                    print("Bild x;y: " + str(x) + ";" + str(y) + "  w;h: " + str(w) + ";" + str(h))
                    print("Flaeche: " + str(contArea))
                    print("w/h Verhaeltnis: " + str(ratio))
                    pass
                    
                colours:list = CamEvaluator.countColour(o2_img)
                print("Farben auf Kamera: " + str(colours))
                if colours == [0,0,0,0]: return ""
                
                #timeNew = com.timeLeft
                return [x, y, w, h, contArea, ratio]
            else:
                if debug:
                    pass
                    print("Nichts")
                    print("Bild x;y: " + str(x) + ";" + str(y) + "  w;h: " + str(w) + ";" + str(h))
                    print("Flaeche: " + str(contArea))
                    print("w/h Verhaeltnis: " + str(ratio))
                    print("Nichts2")
        return []
    
    def scanImageOld(self, img_side:str):
        print("Bild auf Seite: " + str(img_side))
        MotionController.stop()
        delay(30)
        
        if img_side == "c": 
            colours:list = CamEvaluator.countColour(CamEvaluator.getImage(cam_c))
            img_c = CamEvaluator.getImage(cam_c)
            #print("Farben auf Kamera 'C': " + str(colours))
        
            robot.step(timestep)
            delay(1000)
            if(not self.scanSide(img_c, "c")):         # Wenn nicht 1px weiß in der Mitte
                #das Bewegen raus und schauen,dass das mithilfe der Lidarwerten vergleichen
                if self.scanWhite(self.crop_pos(img_c, 6, 20)):      # Weiß links mitte
                    #bewegen.drehe_links()
                    j = 0
                    while robot.step(timestep) == 0 and not self.scanSide(img_c, "c") and j < 41:
                        img_c = CamEvaluator.getImage(cam_c)
                        j += 1
                elif self.scanWhite(self.crop_pos(img_c, 60, 20)):    # Weiß rechts mitte
                    #bewegen.drehe_rechts()
                    j = 0
                    while robot.step(timestep) == 0 and not self.scanSide(img_c, "c") and j < 41:
                        img_c = CamEvaluator.getImage(cam_c)
                        j += 1

            MotionController.stop()
            robot.step(timestep)
            delay(2000)
        
            sign:str = self.getsign(colours)
            if (sign != ""):
              pass
              #com.send(sign)

            MotionController.forward()
            delay(500)

        if img_side == "r": 
            #bewegen.abstand()
            #print("drehe")
            #print("L: vl:" + str(round(rangeImage[1088],3)) + "hl:" + str(round(rangeImage[1472],3)) + "vr" + str(round(rangeImage[1216],3)) + "vl" + str(round(rangeImage[1344],3)))
            
            colours:list = CamEvaluator.countColour(CamEvaluator.getImage(cam_r))
            img_r = CamEvaluator.getImage(cam_r)
            #print("Farben auf Kamera 'R': " + str(colours))
        
            robot.step(timestep)
            delay(1000)
            if(not self.scanSide(img_r, "r")):         # Wenn nicht 1px weiß in der Mitte
                # das Bewegen raus und schauen, dass das mithilfe der Lidarwerte vergleicht
                if self.scanWhite(self.crop_pos(img_r, 6, 20)):      # Weiß links mitte
                    #bewegen.drehe_links()
                    j = 0
                    while robot.step(timestep) == 0 and not self.scanSide(img_r, "r") and j < 41:
                        img_c = CamEvaluator.getImage(cam_r)
                        j += 1
                elif self.scanWhite(self.crop_pos(img_r, 60, 20)):    # Weiß rechts mitte
                    #bewegen.drehe_rechts()
                    j = 0
                    while robot.step(timestep) == 0 and not self.scanSide(img_r, "r") and j < 41:
                        img_r = CamEvaluator.getImage(cam_r)
                        j += 1

            MotionController.stop()
            robot.step(timestep)
            delay(2000)
        
            sign:str = self.getsign(colours)
            if (sign != ""):
              pass
              #com.send(sign)
            
            MotionController.forward()
            delay(500)

        if img_side == "l": 
            #bewegen.abstand()
            #print("drehe")
            #print("L: vl:" + str(round(rangeImage[1088],3)) + "hl:" + str(round(rangeImage[1472],3)) + "vr" + str(round(rangeImage[1216],3)) + "vl" + str(round(rangeImage[1344],3)))
            colours:list = CamEvaluator.countColour(CamEvaluator.getImage(cam_l))
            img_l = CamEvaluator.getImage(cam_l)
            #print("Farben auf Kamera 'C': " + str(colours))
        
            robot.step(timestep)
            delay(1000)
            if(not self.scanSide(img_l, "l")):         # Wenn nicht 1px weiß in der Mitte
                # das Bewegen raus und schauen, dass das mithilfe der Lidarwerte vergleicht
                if self.scanWhite(self.crop_pos(img_l, 6, 20)):      # Weiß links mitte
                    #bewegen.drehe_links()
                    j = 0
                    while robot.step(timestep) == 0 and not self.scanSide(img_l, "l") and j < 41:
                        img_c = CamEvaluator.getImage(cam_l)
                        j += 1
                elif self.scanWhite(self.crop_pos(img_l, 60, 20)):    # Weiß rechts mitte
                    #bewegen.drehe_rechts()
                    j = 0
                    while robot.step(timestep) == 0 and not self.scanSide(img_l, "l") and j < 41:
                        img_l = CamEvaluator.getImage(cam_l)
                        j += 1

            MotionController.stop()
            robot.step(timestep)
            delay(2000)
        
            sign:str = self.getsign(colours)
            if (sign != ""):
              pass
              #com.send(sign)
            
            MotionController.forward()
            delay(500)

    def getsign(self, colours:list) -> str:
        
        """
        teil_a:(teil | None) = karte.getTeil(int(bewegen.kästchen_x_Richtung()), int(bewegen.kästchen_z_Richtung()))
        if (teil_a == None): 
            print("Koordinate unbekannt!!!")
        else:
            if teil_a.victims != "":     
                return ""
        """

        #rangeImage = lidar.getRangeImage(); # Step 4: Retrieve the range image
        if cam_c: #and round(rangeImage[1152] ,3) < 0.5: 
            img_c = CamEvaluator.getImage(cam_c)
            abst = self.abstand_c(img_c) 
        if cam_l : #and round(rangeImage[1024] ,3) < 0.5: 
            img_l = CamEvaluator.getImage(cam_l)
            abst = self.abstand_l(img_l)
        if cam_r: # and round(rangeImage[1280] ,3) < 0.5:
            img_r = CamEvaluator.getImage(cam_r)
            abst = self.abstand_r(img_r)

        if debug:
            print("abst") 
            print(abst)
        
        if (abst[0] + abst[1]) > 32:
            return ""

        #colours:list = CamEvaluator.countColour(img_c)
        
        count_red = colours[0] 
        count_orange = colours[1]
        count_black = colours[2]
        count_white = colours[3]

        if debug or True:  # Testweise immer ausgeben
            print("Red:    " + str(count_red))
            print("Orange: " + str(count_orange))
            print("Black:  " + str(count_black))
            print("White:  " + str(count_white))
            pass
        
        if (count_black == 0 and count_white == 0 and count_red == 0 and count_orange == 0):
            return ""   # Keine Pixel erkannt. Wahrscheinlich von Wand weggedreht

            ## TODO TODO TODO
            ##      Hier noch die anderen Kameras auf Farbe testen
            ## TODO TODO TODO
        
        # Wenn nicht Rechteck an Farbe erkennen
        if debug: print("F, P, C, O")
        #if saveImages: cam_c.saveImage(imagePath + "_red.png", 1)
    
        if (int(count_orange) > 5):
            EmitterCommunicator.sendPos("O")
        elif (int(count_red) > 5):
            EmitterCommunicator.sendPos("F")
        elif (700 < int(count_white) < 980) and (300 < int(count_black) < 550):
            EmitterCommunicator.sendPos("H")
        elif (230 < int(count_black) < 280) and (530 < int(count_white) < 600):
            EmitterCommunicator.sendPos("H")
        elif ( 700 < int(count_white) < 1200) and (240 < int(count_black) < 550):
            EmitterCommunicator.sendPos("U") 
        elif (460 < int(count_black) < 500) and (1100 < int(count_white) < 1200):
            EmitterCommunicator.sendPos("U")
        elif (880 < int(count_white)) and (370 < int(count_black) < 480):
            EmitterCommunicator.sendPos("S")
        elif (160 < int(count_black) < 480) and (430 < int(count_white) < 1000):
            EmitterCommunicator.sendPos("S")
        elif (int(count_black) >= int(count_white)) and count_white > 10 and count_black > 20:
            EmitterCommunicator.sendPos("C")
      #  elif (int(count_black) < 100) and (int(count_white < 350)) :
       #     EmitterCommunicator.sendPos("P") 
        elif (int(count_orange == 0)) and (int(count_red == 0)) and (int(count_black < 10)) and (60 < int(count_white) < 300) : 
            EmitterCommunicator.sendPos("P")
        else: 
            print("Nicht erkennbar")
            pass
        if count_white > 0 and count_black > 0:
            #print("Verhältnis: " + str(int(count_white) / int(count_black) ))
            pass
        
        try:
            img_c:np.ndarray = np.frombuffer(cam_c.getImage(), np.uint8).reshape((cam_c.getHeight(), cam_c.getWidth(), 4))
            data:list = self.checkImg(img_c)

            if len(data) < 3:
                return ""
                                   #     y   :   y    +    h    /2     x   :   x    +    w    /2        -> Obere linke Ecke
            img_crop:np.ndarray = img_c[data[1]:data[1] + data[3] /2, data[0]:data[0] + data[2] /2]
            lower = np.array([200, 200, 200, 0  ], dtype = "uint8") 
            upper = np.array([255, 255, 255, 255], dtype = "uint8")
            mask:cv2.Mat = cv2.inRange(img_crop, lower, upper)
            count = cv2.countNonZero(mask)

            # Schwarz und Weiß vergleichen. Vorsicht: Blau/Grau (Wand/Himmer) nicht mit zählen

            # Mittleres Drittel in 3 Teile teilen (besser: 3 4x4 Bereiche)

            # H Nur Mitte 
            # S Mitte, Oben und Unten
            # U Nur Unten


            if debug: print ("non zero"+count)
        except:
            return ""

        return "U"
        
    def crop_pos(self, img:np.ndarray, posX, posY) -> np.ndarray:
        y,x = img.shape[0], img.shape[1]
        if posY <= y or posX <= x:
            return img[posY:posY+1,posX:posX+1]
        else:
            #print("Dimensions!!!")
            robot.step(timestep)
            return None     # Error - Dimensions wrong
    
    def crop_center(img:np.ndarray) -> np.ndarray:
        return CamEvaluator().crop_pos(img, 32, 20)
    
    def crop_right(img:np.ndarray, cropx:int, cropy:int): 
        y,x = img.shape[0], img.shape[1]
        startx = (x-x//4)-(cropx//2)
        starty = y//2-(cropy//2)    
        return img[starty:starty+cropy,startx:startx+cropx]
    
    def crop_left(img:np.ndarray, cropx:int, cropy:int):
        y,x = img.shape[0], img.shape[1]
        startx = (x-(3*x//4))-(cropx//2)
        starty = y//2-(cropy//2)    
        return img[starty:starty+cropy,startx:startx+cropx]
    
    def text(self, img) -> str:
        return "" #pytesseract.image_to_string(img, config=("--psm 10")).strip()
    
    def scanSide(self, img:np.ndarray, side:str) -> bool:
        if side == "r":
            img_new:np.ndarray = CamEvaluator.crop_center(img)
        elif side == "c":
            img_new:np.ndarray = CamEvaluator.crop_center(img)
        elif side == "l":
            img_new:np.ndarray = CamEvaluator.crop_center(img)
        else:
            pass
            #print("Missing or wrong parameter for scanWhite(); side:str")
        return self.scanWhite(img_new)
    
    def scanWhite(self, img_new:np.ndarray) -> bool:
        if img_new.size == 0:
            return False
        print(img_new [0, 0])         ## Debugging / testing
        b, g, r, a = img_new[0, 0]
        if (b > 200) and (g > 200) and (r > 200):
            return True
        return False
    
    def scanColor(self, img_new:np.ndarray) -> str:    #(schwarz) / rot / gelb / white
        #print(img_new [0, 0])         ## Debugging / testing
        b, g, r, a = img_new[0, 0]
        if (b > 200) and (g > 200) and (r > 200):
            return "w"
        if ( 65 < b < 85) and (0 < g < 15) and (190 < r < 210):       # rot
            return "r"
        if ( 0 < b < 10) and (180 < g < 200) and (190 < r < 215):       # Gelb / Gold
            return "g"
        if ( 0 < b < 15) and (0 < g < 15) and (0 < r < 15):       # Schwarz (Aufpassen! - besser weiß suchen und schwarz finden)
            return "s"
        return ""
    
    def abstand_c(self, img_c): 
        place, dirpx, direction = 0, 0, "r"      #place == px von mitte zum Rand des Bildes (y=20 == Mitte)
        while robot.step(timestep) == 0 and place<32 and self.scanWhite(self.crop_pos(img_c, 32+place, 20)) and self.scanWhite(self.crop_pos(img_c, 32-place, 20)):
            place += 1                      # dirpx == px with only one side with white (difference); direction = side
        while robot.step(timestep) == 0 and place<32 and (self.scanWhite(self.crop_pos(img_c, 32+place+dirpx, 20)) or self.scanWhite(self.crop_pos(img_c, 32-place-dirpx, 20))):
            dirpx += 1
        if dirpx != 0:
            if self.scanWhite(self.crop_pos(img_c, 32-place, 20)):
                direction = "l"
        else:
            direction = ""
        return [place, dirpx, direction]
    
    def abstand_l(self, img_l): 
        place, dirpx, direction = 0, 0, "r"      #place == px von mitte zum Rand des Bildes (y=20 == Mitte)
        while robot.step(timestep) == 0 and place<32 and self.scanWhite(self.crop_pos(img_l, 32+place, 20)) and self.scanWhite(self.crop_pos(img_l, 32-place, 20)):
            place += 1                      # dirpx == px with only one side with white (difference); direction = side
        while robot.step(timestep) == 0 and place<32 and (self.scanWhite(self.crop_pos(img_l, 32+place+dirpx, 20)) or self.scanWhite(self.crop_pos(img_l, 32-place-dirpx, 20))):
            dirpx += 1
        if dirpx != 0:
            if self.scanWhite(self.crop_pos(img_l, 32-place, 20)):
                direction = "l"
        else:
            direction = ""
        return [place, dirpx, direction]
    
    def abstand_r(self, img_r): 
        place, dirpx, direction = 0, 0, "r"      #place == px von mitte zum Rand des Bildes (y=20 == Mitte)
        while robot.step(timestep) == 0 and place<32 and self.scanWhite(self.crop_pos(img_r, 32+place, 20)) and self.scanWhite(self.crop_pos(img_r, 32-place, 20)):
            place += 1                      # dirpx == px with only one side with white (difference); direction = side
        while robot.step(timestep) == 0 and place<32 and (self.scanWhite(self.crop_pos(img_r, 32+place+dirpx, 20)) or self.scanWhite(self.crop_pos(img_r, 32-place-dirpx, 20))):
            dirpx += 1
        if dirpx != 0:
            if self.scanWhite(self.crop_pos(img_r, 32-place, 20)):
                direction = "l"
        else:
            direction = ""
        return [place, dirpx, direction]
camE = CamEvaluator()    


delay(100)
#areaOneMap = RoomMapping(1)
#worldMap = WorldMapping()
strategy = RightHandStrategy()
lidarEv = LidarEvaluator()
gpsEv = GPSEvaluator()
usEv = UsSensorEvaluator()
while robot.step(timestep) != -1:
    if gpsEv.stuck():
        print("Stuck")
        MotionController.backward()
        delay(50)
        MotionController.ninetyDegreeRotationLeft()
    
    usEv.updateHole()
    if usEv.holeInFront:
        if usEv.checkHoleInFront():
            MotionController.ninetyDegreeRotationLeft()
    
    if ColorSensor.checkHole():
        #behandle Loch als Wand
        MotionController.backward()
        delay(50)
        MotionController.ninetyDegreeRotationLeft()
        MotionController.forward()
        
        if gpsEv.stuck():
            MotionController.backward()
            delay(50)
            MotionController.ninetyDegreeRotationLeft()
           
        MotionController.forward()
        delay(2000)
        
    camE.check()
    MotionController.forward()
    lidarEv.checkleftRotation()
    if lidarEv.leftRotation == False and lidarEv.checkNoLeftWall():
        MotionController.ninetyDegreeRotationLeft()
        lidarEv.leftRotation = True
    elif LidarEvaluator.checkFrontLeftWall():
        MotionController.ninetyDegreeRotationRight()
    elif LidarEvaluator.checkFrontRightWall():
        MotionController.ninetyDegreeRotationLeft()
    elif LidarEvaluator.checkFrontWall():
        MotionController.ninetyDegreeRotationLeft()
