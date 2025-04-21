# test push
# Jordan testing pushing
import matplotlib.pyplot as plt
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

        FeetPositions.append(L * (cos(thetaOne) * cos(thetaTwo) - sin(thetaOne) * sin(thetaTwo)))
        FeetPositions.append(L * (sin(thetaOne) * cos(thetaTwo) + cos(thetaOne) * sin(thetaTwo)))

    # from hips need CoG

    return FeetPositions

def InGoalRegion(current_cog, hipPositions, movingLeg):
    """
    Determines if a point is inside a triangle using barycentric coordinates.

    Parameters:
    - pt: tuple (x, y) - the point to check
    - v1, v2, v3: tuples (x, y) - vertices of the triangle

    Returns:
    - True if pt is inside the triangle, False otherwise
    """
    # Get verticies from hip positions depending on which legs we're using
    if movingLeg == 1:
        v1 = 1
        v2 = 2
        v3 = 3
    elif movingLeg == 2:
        v1 = 1
        v2 = 2
        v3 = 3
    elif movingLeg == 3:
        v1 = 1
        v2 = 2
        v3 = 3
    elif movingLeg == 4:
        v1 = 1
        v2 = 2
        v3 = 3
    else:
        print("Choose one of the 4 legs that's moving (movingLeg should equal 1 through 4)")

    def area(p1, p2, p3):
        return 0.5 * abs((p1[0] * (p2[1] - p3[1]) +
                          p2[0] * (p3[1] - p1[1]) +
                          p3[0] * (p1[1] - p2[1])))

    # Compute total area of triangle
    A = area(v1, v2, v3)

    # Compute areas of triangles using center of gravity
    A1 = area(current_cog, v2, v3)
    A2 = area(v1, current_cog, v3)
    A3 = area(v1, v2, current_cog)

    # Allow for a small margin of error due to floating point arithmetic
    isInGoalRegion = abs(A - (A1 + A2 + A3)) < 1e-10

    return isInGoalRegion






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

# Plot/Return outcome somehow

def PlotPath(minimum_cogs):
    """
    Plot the trajectory of the center of gravity over time.
    Parameters:
    minimum_cogs: List of cog values (can be 1D or 2D depending on output of ForwardKinematics)
    """
    if not minimum_cogs:
        print("No cog data to plot.")
        return

    # Check if cog is 2D (x, y) or scalar
    if isinstance(minimum_cogs[0], (list, tuple, np.ndarray)) and len(minimum_cogs[0]) == 2:
        xs = [cog[0] for cog in minimum_cogs]
        ys = [cog[1] for cog in minimum_cogs]
        plt.plot(xs, ys, marker='o')
        plt.xlabel("X Position of cog")
        plt.ylabel("Y Position of cog")
        plt.title("Center of Gravity Trajectory")
        plt.grid(True)
    else:
        # Scalar cog value case
        plt.plot(minimum_cogs, marker='o')
        plt.xlabel("Step")
        plt.ylabel("CoG (scalar)")
        plt.title("Center of Gravity Trajectory")
        plt.grid(True)

    plt.show()


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
