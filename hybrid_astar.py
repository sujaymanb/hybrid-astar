import math
import sys
import os
import heapq
import numpy as np
from heapdict import heapdict
import reeds_shepp as rsCurve
from node import Node
from car import Car
from cost import Cost
from map import MapParameters
from heuristic import HolonomicHeuristic
from draw_map import draw


class HybridAStar:
    def __init__(self, mapParameters, car):
        self.map = mapParameters
        self.car = car
        self.heuristic = HolonomicHeuristic(mapParameters)
        self.cost = Cost()

    def kinematicSimulationNode(self, currentNode, motionCommand, simulationLength=4, step = 0.8 ):
        """Simulate trajectory using given current Node and Motion Commands"""
        
        traj = []
        angle = rsCurve.pi_2_pi(currentNode.traj[-1][2] + motionCommand[1] * step / self.car.wheelBase * math.tan(motionCommand[0]))
        
        traj.append([currentNode.traj[-1][0] + motionCommand[1] * step * math.cos(angle),
                    currentNode.traj[-1][1] + motionCommand[1] * step * math.sin(angle),
                    rsCurve.pi_2_pi(angle + motionCommand[1] * step / self.car.wheelBase * math.tan(motionCommand[0]))])
        
        for i in range(int((simulationLength/step))-1):
            traj.append([traj[i][0] + motionCommand[1] * step * math.cos(traj[i][2]),
                        traj[i][1] + motionCommand[1] * step * math.sin(traj[i][2]),
                        rsCurve.pi_2_pi(traj[i][2] + motionCommand[1] * step / self.car.wheelBase * math.tan(motionCommand[0]))])

        # Find grid index
        gridIndex = [round(traj[-1][0]/self.map.xyResolution), \
                     round(traj[-1][1]/self.map.xyResolution), \
                     round(traj[-1][2]/self.map.yawResolution)]

        # Check if node is valid
        if not self.isValid(traj, gridIndex):
            return None

        # Calculate Cost of the node
        cost = self.cost.simulatedPathCost(currentNode, motionCommand, simulationLength)

        return Node(gridIndex, traj, motionCommand[0], motionCommand[1], cost, currentNode.index())


    def reedsSheppNode(self, currentNode, goalNode):

        # Get x, y, yaw of currentNode and goalNode
        startX, startY, startYaw = currentNode.traj[-1][0], currentNode.traj[-1][1], currentNode.traj[-1][2]
        goalX, goalY, goalYaw = goalNode.traj[-1][0], goalNode.traj[-1][1], goalNode.traj[-1][2]

        # Instantaneous Radius of Curvature
        radius = math.tan(self.car.maxSteerAngle)/self.car.wheelBase

        #  Find all possible reeds-shepp paths between current and goal node
        reedsSheppPaths = rsCurve.calc_all_paths(startX, startY, startYaw, goalX, goalY, goalYaw, radius, 1)

        # Check if reedsSheppPaths is empty
        if not reedsSheppPaths:
            return None

        # Find path with lowest cost considering non-holonomic constraints
        costQueue = heapdict()
        for path in reedsSheppPaths:
            costQueue[path] = self.cost.reedsSheppCost(self.car.maxSteerAngle, currentNode, path)

        # Find first path in priority queue that is collision free
        while len(costQueue)!=0:
            path = costQueue.popitem()[0]
            traj=[]
            traj = [[path.x[k],path.y[k],path.yaw[k]] for k in range(len(path.x))]
            if not self.collision(traj):
                cost = self.cost.reedsSheppCost(self.car.maxSteerAngle, currentNode, path)
                return Node(goalNode.gridIndex ,traj, None, None, cost, currentNode.index())
                
        return None


    def isValid(self, traj, gridIndex):

        # Check if Node is out of map bounds
        if gridIndex[0]<=self.map.mapMinX or gridIndex[0]>=self.map.mapMaxX or \
           gridIndex[1]<=self.map.mapMinY or gridIndex[1]>=self.map.mapMaxY:
            return False

        # Check if Node is colliding with an obstacle
        if self.collision(traj):
            return False
        return True


    def collision(self, traj):

        carRadius = (self.car.axleToFront + self.car.axleToBack)/2 + 1
        dl = (self.car.axleToFront - self.car.axleToBack)/2
        for i in traj:
            cx = i[0] + dl * math.cos(i[2])
            cy = i[1] + dl * math.sin(i[2])
            pointsInObstacle = self.map.ObstacleKDTree.query_ball_point([cx, cy], carRadius)

            if not pointsInObstacle:
                continue

            for p in pointsInObstacle:
                xo = self.map.obstacleX[p] - cx
                yo = self.map.obstacleY[p] - cy
                dx = xo * math.cos(i[2]) + yo * math.sin(i[2])
                dy = -xo * math.sin(i[2]) + yo * math.cos(i[2])

                if abs(dx) < carRadius and abs(dy) < self.car.width / 2 + 1:
                    return True

        return False

    def backtrack(self, startNode, goalNode, closedSet):
        """trace path back from goal to start"""
        # Goal Node data
        startNodeIndex= startNode.index()
        currentNodeIndex = goalNode.parentIndex
        currentNode = closedSet[currentNodeIndex]
        x=[]
        y=[]
        yaw=[]

        # Iterate till we reach start node from goal node
        while currentNodeIndex != startNodeIndex:
            a, b, c = zip(*currentNode.traj)
            x += a[::-1] 
            y += b[::-1] 
            yaw += c[::-1]
            currentNodeIndex = currentNode.parentIndex
            currentNode = closedSet[currentNodeIndex]

        return x[::-1], y[::-1], yaw[::-1]

    
    def run(self, s, g):
        """run hybrid a* for car on map given start and goal"""

        # Compute Grid Index for start and Goal node
        sGridIndex = [round(s[0] / self.map.xyResolution), \
                      round(s[1] / self.map.xyResolution), \
                      round(s[2] / self.map.yawResolution)]

        gGridIndex = [round(g[0] / self.map.xyResolution), \
                      round(g[1] / self.map.xyResolution), \
                      round(g[2] / self.map.yawResolution)]

        # Generate all Possible motion commands to car
        motionCommand = self.car.motionCommands()

        # Create start and goal Node
        startNode = Node(sGridIndex, [s], 0, 1, 0 , tuple(sGridIndex))
        goalNode = Node(gGridIndex, [g], 0, 1, 0, tuple(gGridIndex))

        # Find Holonomic Heuristric
        holonomicHeuristics = self.heuristic.holonomicCostsWithObstacles(goalNode)

        # Add start node to open Set
        openSet = {startNode.index():startNode}
        closedSet = {}

        # Create a priority queue for acquiring nodes based on their cost's
        costQueue = heapdict()

        # Add start mode into priority queue
        costQueue[startNode.index()] = max(startNode.cost , Cost.hybridCost * holonomicHeuristics[startNode.gridIndex[0]][startNode.gridIndex[1]])
        counter = 0

        # loop until path is found or open set is empty
        while True:
            counter +=1

            # Check if openSet is empty, if empty no solution available
            if not openSet:
                return None

            # Get first node in the priority queue
            currentNodeIndex = costQueue.popitem()[0]
            currentNode = openSet[currentNodeIndex]

            # Revove currentNode from openSet and add it to closedSet
            openSet.pop(currentNodeIndex)
            closedSet[currentNodeIndex] = currentNode

            # Get Reed-Shepp path to node if available
            rSNode = self.reedsSheppNode(currentNode, goalNode)

            # If Reeds-Shepp Path is found, done
            if rSNode:
                closedSet[rSNode.index()] = rSNode
                break

            # if not using reeds-shepp expansion or if start=goal
            if currentNodeIndex == goalNode.index():
                print("Path Found")
                print(currentNode.traj[-1])
                break

            # Get all simulated Nodes from current node
            for i in range(len(motionCommand)):
                simulatedNode = self.kinematicSimulationNode(currentNode, motionCommand[i])

                # Check if path is within map bounds and is collision free
                if not simulatedNode:
                    continue

                # Draw Simulated Node
                #x,y,z =zip(*simulatedNode.traj)
                #plt.plot(x, y, linewidth=0.3, color='g')

                # Check if simulated node is already in closed set
                simulatedNodeIndex = simulatedNode.index()
                if simulatedNodeIndex not in closedSet: 

                    # Check if simulated node is already in open set, if not add it open set as well as in priority queue
                    if simulatedNodeIndex not in openSet:
                        openSet[simulatedNodeIndex] = simulatedNode
                        costQueue[simulatedNodeIndex] = max(simulatedNode.cost , Cost.hybridCost * holonomicHeuristics[simulatedNode.gridIndex[0]][simulatedNode.gridIndex[1]])
                    elif simulatedNode.cost < openSet[simulatedNodeIndex].cost:
                        openSet[simulatedNodeIndex] = simulatedNode
                        costQueue[simulatedNodeIndex] = max(simulatedNode.cost , Cost.hybridCost * holonomicHeuristics[simulatedNode.gridIndex[0]][simulatedNode.gridIndex[1]])
        
        # Backtrack
        x, y, yaw = self.backtrack(startNode, goalNode, closedSet)

        return x, y, yaw


def main():
    # Start and Goal pose (x, y, theta)
    s = [10, 10, np.deg2rad(90)]
    g = [25, 20, np.deg2rad(90)]

    # get car params
    car = Car()

    # Calculate map Paramaters
    mapParameters = MapParameters(xyResolution=4, 
                                  yawResolution=np.deg2rad(15.0))
    
    # Run Hybrid A*
    search = HybridAStar(mapParameters, car)
    x, y, yaw = search.run(s, g)

    # animate path
    draw(x,y,yaw,car,mapParameters)


if __name__ == '__main__':
    main()

