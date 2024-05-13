import numpy as np

class Car:
    maxSteerAngle = 0.6
    steerPrecision = 10
    wheelBase = 3.5
    axleToFront = 4.5
    axleToBack = 1
    width = 3

    def motionCommands(self):
        """List of motion commands for a Non-Holonomic Robot/Car/Bicycle 
        (for Trajectories with Steer Angle and Direction)"""
        direction = 1
        motionCommand = []
        
        for i in np.arange(self.maxSteerAngle, -(self.maxSteerAngle + self.maxSteerAngle/self.steerPrecision), -self.maxSteerAngle/self.steerPrecision):
            motionCommand.append([i, direction])
            motionCommand.append([i, -direction])
        
        return motionCommand