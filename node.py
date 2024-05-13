
class Node:
    def __init__(self, gridIndex, traj, steeringAngle, direction, cost, parentIndex):
        self.gridIndex = gridIndex         # grid block x, y, yaw index
        self.traj = traj                   # trajectory x, y  of a simulated node
        self.steeringAngle = steeringAngle # steering angle throughout the trajectory
        self.direction = direction         # direction throughout the trajectory
        self.cost = cost                   # node cost
        self.parentIndex = parentIndex     # parent node index

    def index(self):
        # return tuple consisting grid index, 
        # used for checking if two nodes are near/same
        return tuple([self.gridIndex[0], self.gridIndex[1], self.gridIndex[2]])


class HolonomicNode:
    def __init__(self, gridIndex, cost, parentIndex):
        self.gridIndex = gridIndex
        self.cost = cost
        self.parentIndex = parentIndex

    def index(self):
        # Index is a tuple consisting grid index, used for checking if two nodes are near/same
        return tuple([self.gridIndex[0], self.gridIndex[1]])
