import math
from node import HolonomicNode
import heapq
import numpy as np

class HolonomicHeuristic:
    """Point, Omni-Directional/Holonomic Robot"""

    def __init__(self,mapParameters):
        self.map = mapParameters
        self.obstacles = self.map.obstaclesMap()

    def holonomicMotionCommands(self):
        """returns Action set (8-Directions)"""
        holonomicMotionCommand = [[-1, 0], [-1, 1], [0, 1], [1, 1], [1, 0], [1, -1], [0, -1], [-1, -1]]
        return holonomicMotionCommand

    def eucledianCost(self, holonomicMotionCommand):
        """Compute Eucledian Distance between two nodes"""
        return math.hypot(holonomicMotionCommand[0], holonomicMotionCommand[1])

    def holonomicNodeIsValid(self, neighbourNode):

        # Check if Node is out of map bounds
        if neighbourNode.gridIndex[0]<= self.map.mapMinX or \
           neighbourNode.gridIndex[0]>= self.map.mapMaxX or \
           neighbourNode.gridIndex[1]<= self.map.mapMinY or \
           neighbourNode.gridIndex[1]>= self.map.mapMaxY:
            return False

        # Check if Node on obstacle
        if self.obstacles[neighbourNode.gridIndex[0]][neighbourNode.gridIndex[1]]:
            return False

        return True

    def holonomicCostsWithObstacles(self, goalNode):
        """computes Holonomic Cost Map given a goal node"""

        gridIndex = [round(goalNode.traj[-1][0]/self.map.xyResolution), round(goalNode.traj[-1][1]/self.map.xyResolution)]
        gNode = HolonomicNode(gridIndex, 0, tuple(gridIndex))

        obstacles = self.obstacles

        holonomicMotionCommand = self.holonomicMotionCommands()

        openSet = {gNode.index(): gNode}
        closedSet = {}

        priorityQueue =[]
        heapq.heappush(priorityQueue, (gNode.cost, gNode.index()))

        while True:
            if not openSet:
                break

            _, currentNodeIndex = heapq.heappop(priorityQueue)
            currentNode = openSet[currentNodeIndex]
            openSet.pop(currentNodeIndex)
            closedSet[currentNodeIndex] = currentNode

            for i in range(len(holonomicMotionCommand)):
                neighbourNode = HolonomicNode([currentNode.gridIndex[0] + holonomicMotionCommand[i][0],\
                                          currentNode.gridIndex[1] + holonomicMotionCommand[i][1]],\
                                          currentNode.cost + self.eucledianCost(holonomicMotionCommand[i]), currentNodeIndex)

                if not self.holonomicNodeIsValid(neighbourNode):
                    continue

                neighbourNodeIndex = neighbourNode.index()

                if neighbourNodeIndex not in closedSet:            
                    if neighbourNodeIndex in openSet:
                        if neighbourNode.cost < openSet[neighbourNodeIndex].cost:
                            openSet[neighbourNodeIndex].cost = neighbourNode.cost
                            openSet[neighbourNodeIndex].parentIndex = neighbourNode.parentIndex
                    else:
                        openSet[neighbourNodeIndex] = neighbourNode
                        heapq.heappush(priorityQueue, (neighbourNode.cost, neighbourNodeIndex))

        holonomicCostMap = [[np.inf for i in range(max(self.map.obstacleY))]for i in range(max(self.map.obstacleX))]

        for nodes in closedSet.values():
            holonomicCostMap[nodes.gridIndex[0]][nodes.gridIndex[1]]=nodes.cost

        return holonomicCostMap