import numpy as np
import random

def ForwardKinematics(legAngles):
    return centerOfGravity

def InGoalRegion(centerOfGravity):
    return False

def GetCost(centerOfGravity):
    return 0.0 # Distance of CenterOfGravity to simultaneous stability, penalize heavily for leaving any stability

# Assume leg one desired, so lift leg four then find intersection of stability between the two

notStable = True

legAngles = [30, 150, 30, 150, 30, 150, 30, 30]
testAngles = legAngles
costToGo = 0.0

anglePath = []

while notStable:
    # Compare how each perturbance affects the cost
    bestAngleIndex = 0
    bestCost = 10000000
    bestDirection = 0

    for idx in range(0,6): # For each joint angle...
        for j in range(0,2): # For each direction...
            testLegAngles = legAngles
            if j == 0:
                testLegAngles[idx] = testLegAngles + epsilon
            else:
                testLegAngles[idx] = testLegAngles - epsilon

            centerOfGravity = ForwardKinematics(testLegAngles)
            testCost = GetCost(centerOfGravity)

            if testCost < bestCost:
                bestCost = testCost
                bestAngleIndex = idx
                bestDirection = j

    # Update with the best values
    if bestDirection == 0:
        legAngles[bestAngleIndex] = testLegAngles + epsilon
    else:
        legAngles[bestAngleIndex] = testLegAngles - epsilon
    costToGo += bestCost

    # Log it [NewAngle, AngleIndex, CenterOfGravity]
    anglePath.append([legAngles[bestAngleIndex], bestAngleIndex, ForwardKinematics(legAngles)])

    # Test if we made it, exit if yes
    if InGoalRegion(ForwardKinematics(legAngles)):
        notStable = False

# Plot/Return outcome somehow
