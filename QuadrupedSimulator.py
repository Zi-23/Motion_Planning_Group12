# test push
# Jordan testing pushing
import numpy as np
import itertools
import random

def ForwardKinematics(legAngles):
    L = 6 # Leg segment length, inches
    bl = 1 # Body length
    bw = 1 # Body width
    hipPositions = [] # X,Y coords then next hip

    for i in range(0, 4):
        thetaOne = legAngles[2*i] # Upper joint angle
        thetaTwo = legAngles[2*i+1] # Lower joint angle

        hipPositions.append(L * (cos(thetaOne) * cos(thetaTwo) - sin(thetaOne) * sin(thetaTwo)))
        hipPositions.append(L * (sin(thetaOne) * cos(thetaTwo) + cos(thetaOne) * sin(thetaTwo)))

    # from hips need CoG

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
    current_cogs = [0] * len(new_joint_angles)
    cost_vals = [0] * len(new_joint_angles)

    for i in range(new_joint_angles):
        current_cogs[i] = ForwardKinematics(new_joint_angles[i])
        cost_vals[i] = GetCost(current_cogs[i], target_cog)

    min_cost_index = cost_vals.index(np.min(cost_vals))
    min_joint = new_joint_angles[min_cost_index]
    min_cog = current_cogs[min_cost_index]  # center of gravity corresponding to minimum cost movement (for keeping track of COG path)

    return min_joint, min_cog

def GetCost(current_cog, target_cog):
    return ((current_cog - target_cog) ** 2)


# initilize variables
init_joint_angles = [30, 150, 30, 150, 30, 150, 30, 30]
step_size = 0.1
total_joints = len(init_joint_angles)
target_cog = 0
min_cog = 1000000
minimum_cogs = [] # center of gravity logation

while min_cog != target_cog:
    # Get array of incremented joint angles (2 directions for all joints)
    new_joint_angles = GetNewJointAngles(init_joint_angles, step_size, total_joints)

    # Get the joint and updated cog location from incremented
    min_joint, min_cog = FindMinCostJoint(new_joint_angles, target_cog)

    # Add minimum joint and it's corresponding center of gravity to an array to plot
    minimum_cogs.append(min_cog)



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
