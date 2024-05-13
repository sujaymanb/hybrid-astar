class Cost:
    reverse = 10
    directionChange = 150
    steerAngle = 1
    steerAngleChange = 5
    hybridCost = 50

    def reedsSheppCost(self, maxSteerAngle, currentNode, path):

        # Previos Node Cost
        cost = currentNode.cost

        # Distance cost
        for i in path.lengths:
            if i >= 0:
                cost += 1
            else:
                cost += abs(i) * Cost.reverse

        # Direction change cost
        for i in range(len(path.lengths)-1):
            if path.lengths[i] * path.lengths[i+1] < 0:
                cost += Cost.directionChange

        # Steering Angle Cost
        for i in path.ctypes:
            # Check types which are not straight line
            if i!="S":
                cost += maxSteerAngle * Cost.steerAngle

        # Steering Angle change cost
        turnAngle=[0.0 for _ in range(len(path.ctypes))]
        for i in range(len(path.ctypes)):
            if path.ctypes[i] == "R":
                turnAngle[i] = - maxSteerAngle
            if path.ctypes[i] == "WB":
                turnAngle[i] = maxSteerAngle

        for i in range(len(path.lengths)-1):
            cost += abs(turnAngle[i+1] - turnAngle[i]) * Cost.steerAngleChange

        return cost


    def simulatedPathCost(self, currentNode, motionCommand, simulationLength):

        # Previous Node Cost
        cost = currentNode.cost

        # Distance cost
        if motionCommand[1] == 1:
            cost += simulationLength 
        else:
            cost += simulationLength * Cost.reverse

        # Direction change cost
        if currentNode.direction != motionCommand[1]:
            cost += Cost.directionChange

        # Steering Angle Cost
        cost += motionCommand[0] * Cost.steerAngle

        # Steering Angle change cost
        cost += abs(motionCommand[0] - currentNode.steeringAngle) * Cost.steerAngleChange

        return cost