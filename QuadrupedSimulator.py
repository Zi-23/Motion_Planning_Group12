# test push
# Jordan testing pushing
import numpy as np
import itertools
import random

def ForwardKinematics(legAngles):
    return centerOfGravity

def InGoalRegion(current_cog):
    return False

def GetNewJointAngles(init_joint_angles, step_size, total_joints):
    """Generate all combinations of [-step, 0, +step] for each joint"""
    # 16 directions from binary combinations of +/- step per joint
    new_joint_angles = [0] * (len(init_joint_angles) * 2)
    i = 0
    while i <= range(init_joint_angles):
        new_joint_angles[i] = -step_size
        i + 1
        new_joint_angles[i] = step_size
        i + 1

    return new_joint_angles

def FindMinCostJoint(new_joint_angles, target_cog):
    """Find the minimum cost out of moving each joint angles and return the minimum cost joint"""
    cost_vals = [0] * len(new_joint_angles)
    for i in range(new_joint_angles):
        current_cog = ForwardKinematics(new_joint_angles[i])
        cost_vals[i] = GetCost(current_cog, target_cog, new_joint_angles)

    min_cost_index = cost_vals.index(np.min(cost_vals))
    min_joint = new_joint_angles[min_cost_index]

    return min_joint

def GetCost(current_cog, target_cog, new_joint_angles):

    return ((current_cog - target_cog) ** 2)



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
