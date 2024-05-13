from scipy.spatial import KDTree
import math

def defaultMap():
    # Build Map
    obstacleX, obstacleY = [], []

    for i in range(51):
        obstacleX.append(i)
        obstacleY.append(0)

    for i in range(51):
        obstacleX.append(0)
        obstacleY.append(i)

    for i in range(51):
        obstacleX.append(i)
        obstacleY.append(50)

    for i in range(51):
        obstacleX.append(50)
        obstacleY.append(i)
    
    for i in range(10,20):
        obstacleX.append(i)
        obstacleY.append(30) 

    for i in range(30,51):
        obstacleX.append(i)
        obstacleY.append(30) 

    for i in range(0,31):
        obstacleX.append(20)
        obstacleY.append(i) 

    for i in range(0,31):
        obstacleX.append(30)
        obstacleY.append(i) 

    for i in range(40,50):
        obstacleX.append(15)
        obstacleY.append(i)

    for i in range(25,40):
        obstacleX.append(i)
        obstacleY.append(35)

    return obstacleX, obstacleY

def parkingMap():
    # Parking Map
    for i in range(51):
        obstacleX.append(i)
        obstacleY.append(0)

    for i in range(51):
        obstacleX.append(0)
        obstacleY.append(i)

    for i in range(51):
        obstacleX.append(i)
        obstacleY.append(50)

    for i in range(51):
        obstacleX.append(50)
        obstacleY.append(i)

    for i in range(51):
        obstacleX.append(i)
        obstacleY.append(40)

    for i in range(0,20):
        obstacleX.append(i)
        obstacleY.append(30) 

    for i in range(29,51):
        obstacleX.append(i)
        obstacleY.append(30) 

    for i in range(24,30):
        obstacleX.append(19)
        obstacleY.append(i) 

    for i in range(24,30):
        obstacleX.append(29)
        obstacleY.append(i) 

    for i in range(20,29):
        obstacleX.append(i)
        obstacleY.append(24)

    return obstacleX,obstacleY


class MapParameters:
    def __init__(self, xyResolution, yawResolution, mapName="default"):
        obstacleX, obstacleY = self.buildMap(mapName)
        self.calculate_map_params(obstacleX, obstacleY, xyResolution, yawResolution)

    def set_params(self, mapMinX, mapMinY, mapMaxX, mapMaxY, xyResolution, yawResolution, ObstacleKDTree, obstacleX, obstacleY):
        self.mapMinX = mapMinX               # map min x coordinate(0)
        self.mapMinY = mapMinY               # map min y coordinate(0)
        self.mapMaxX = mapMaxX               # map max x coordinate
        self.mapMaxY = mapMaxY               # map max y coordinate
        self.xyResolution = xyResolution     # grid block length
        self.yawResolution = yawResolution   # grid block possible yaws
        self.ObstacleKDTree = ObstacleKDTree # KDTree representating obstacles
        self.obstacleX = obstacleX           # Obstacle x coordinate list
        self.obstacleY = obstacleY           # Obstacle y coordinate list

    def calculate_map_params(self, obstacleX, obstacleY, xyResolution, yawResolution):
        # calculate min max map grid index based on obstacles in map
        mapMinX = round(min(obstacleX) / xyResolution)
        mapMinY = round(min(obstacleY) / xyResolution)
        mapMaxX = round(max(obstacleX) / xyResolution)
        mapMaxY = round(max(obstacleY) / xyResolution)

        # create a KDTree to represent obstacles
        ObstacleKDTree = KDTree([[x, y] for x, y in zip(obstacleX, obstacleY)])

        self.set_params(mapMinX, mapMinY, mapMaxX, mapMaxY, xyResolution, yawResolution, ObstacleKDTree, obstacleX, obstacleY)  

    def obstaclesMap(self):
        """returns obstacle map with grid indices for all obstacles"""
        obstacleX, obstacleY, xyResolution = self.obstacleX, self.obstacleY, self.xyResolution

        # Compute Grid Index for obstacles
        obstacleX = [round(x / xyResolution) for x in obstacleX]
        obstacleY = [round(y / xyResolution) for y in obstacleY]

        # Set all Grid locations to No Obstacle
        obstacles =[[False for i in range(max(obstacleY))]for i in range(max(obstacleX))]

        # Set Grid Locations with obstacles to True
        for x in range(max(obstacleX)):
            for y in range(max(obstacleY)):
                for i, j in zip(obstacleX, obstacleY):
                    if math.hypot(i-x, j-y) <= 1/2:
                        obstacles[i][j] = True
                        break

        return obstacles

    def buildMap(self, mapName="default"):
        if mapName=="default":
            return defaultMap()
        if mapName=="parking":
            return parkingMap()