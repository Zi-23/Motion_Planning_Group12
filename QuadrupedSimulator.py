import matplotlib.pyplot as plt
import numpy as np
import itertools
import math
from Plotter import plotter2d, plotCoG, plandrw

_ROBOT_WIDTH = 8 # inches
_ROBOT_LENGTH = 15

def ForwardKinematics(legAngles):
    # Given angles of the leg, assuming the feet are all in contact with
    # the ground, returns the feet positions relative to each hip's frame
    L = 6 # Leg segment length, inches
    bl = _ROBOT_LENGTH # Body length, inches
    bw = _ROBOT_WIDTH # Body width, inches
    FeetPositions = [] # X,Y coords then next foot

    for i in range(0, 4):
        thetaOne = legAngles[2*i] # Upper joint angle
        thetaTwo = legAngles[2*i+1] # Lower joint angle

        FeetPositions.append(L * (np.cos(np.deg2rad(thetaOne)) * np.cos(np.deg2rad(thetaTwo)) - np.sin(np.deg2rad(thetaOne)) * np.sin(np.deg2rad(thetaTwo))))
        FeetPositions.append(L * (np.sin(np.deg2rad(thetaOne)) * np.cos(np.deg2rad(thetaTwo)) + np.cos(np.deg2rad(thetaOne)) * np.sin(np.deg2rad(thetaTwo))))

    return FeetPositions

def GetTargetCg(FeetPositions, movingLeg):
    """
    Calculate the centroid (center) of the support polygon (triangle) given three vertices. These vertices depend on which foot is moving.
    Returns:
        A tuple (x_center, y_center) representing the centroid of the triangle.
    """
    # Get vertices from hip positions depending on which legs we're using
    if movingLeg == 1:
        v1 = (FeetPositions[2], _ROBOT_WIDTH) # Leg 2 tuple (x, y)
        v2 = (_ROBOT_LENGTH + FeetPositions[4], 0) # Leg 3 tuple (x, y)
        v3 = (_ROBOT_LENGTH + FeetPositions[6], _ROBOT_WIDTH) # Leg 4 tuple (x, y)
    elif movingLeg == 2:
        v1 = (0, 0) # Leg 1 tuple (x, y)
        v2 = (_ROBOT_LENGTH + FeetPositions[4], 0) # Leg 3 tuple (x, y)
        v3 = (_ROBOT_LENGTH + FeetPositions[6], _ROBOT_WIDTH) # Leg 4 tuple (x, y)
    elif movingLeg == 3:
        v1 = (0, 0) # Leg 1 tuple (x, y)
        v2 = (FeetPositions[2], _ROBOT_WIDTH) # Leg 2 tuple (x, y)
        v3 = (_ROBOT_LENGTH + FeetPositions[6], _ROBOT_WIDTH) # Leg 4 tuple (x, y)
    elif movingLeg == 4:
        v1 = (0, 0) # Leg 1 tuple (x, y)
        v2 = (FeetPositions[2], _ROBOT_WIDTH) # Leg 2 tuple (x, y)
        v3 = (_ROBOT_LENGTH + FeetPositions[4], 0) # Leg 3 tuple (x, y)
    else:
        print("Choose one of the 4 legs that's moving (movingLeg should equal 1 through 4)")

    # Compute the center of the support polygon
    x_center = (v1[0] + v2[0] + v3[0]) / 3
    y_center = (v1[1] + v2[1] + v3[1]) / 3
    target_cog = (x_center, y_center)

    return target_cog

def GetCg(FeetPositions):

    FrontWidth = _ROBOT_WIDTH
    SideLength = _ROBOT_LENGTH

    FootOne = tuple(FeetPositions[0:2])
    FootTwo = tuple(FeetPositions[2:4])
    FootThree = tuple(FeetPositions[4:6])
    FootFour = tuple(FeetPositions[6:-1])

    # Assume feet one two and three are on the ground and disregard foot four

    # Inital location in base frame of feet [x,y,z] (this test case is with the legs straight down)

    FootOneInit = [0,0,0]
    FootTwoInit = [SideLength,0,0]
    FootThreeInit = [SideLength,FrontWidth,0]

    Theta1 = np.arctan((FootOne[1] - FootTwo[1]) / (SideLength - FootOne[0] + FootTwo[0]))

    Hip = tuple([0,0])

    L1 = math.dist(Hip,FootOne)
    L2 = math.dist(Hip,FootTwo)
    L3 = math.dist(Hip,FootThree)

    Alpha1 = np.arcsin(FootOne[0]/L1)
    Alpha2 = np.arcsin(FootTwo[0]/L2)
    Alpha3 = np.arcsin(FootThree[0]/L3)

    d1 = L1*np.cos(Theta1 - Alpha1)
    d2 = L2*np.cos(Theta1 - Alpha2)
    d3 = L3*np.cos(Theta1 - Alpha3)

    a1 = L1*np.sin(Theta1 - Alpha1)
    a2 = L2*np.sin(Theta1 - Alpha2)
    a3 = L3*np.sin(Theta1 - Alpha3)

    Theta2 = np.arctan((d2-d3)/FrontWidth)

    b1 = d1*np.sin(Theta2)
    b2 = d2*np.sin(Theta2)
    b3 = d3*np.sin(Theta2)

    c1 = d1*np.cos(Theta2)
    c2 = d2*np.cos(Theta2)
    c3 = d3*np.cos(Theta2)

    HipOne = [FootOneInit[0] + a1, FootOneInit[1] + b1, FootOneInit[2] + c1]
    HipTwo = [FootTwoInit[0] + a2, FootTwoInit[1] + b2, FootTwoInit[2] + c2]
    HipThree = [FootThreeInit[0] + a3, FootThreeInit[1] + b3, FootThreeInit[2] + c3]

    v1 = GetUnitVector(HipTwo,HipOne)
    v2 = GetUnitVector(HipTwo,HipThree)

    CenterOfGravity = [HipTwo + v1*0.4*SideLength + v2*0.5*FrontWidth]

    return CenterOfGravity

def GetUnitVector(p1,p2):

    vector = (p2[0]-p1[0], p2[1]-p1[1], p2[2]-p1[2])
    magnitude = np.sqrt(vector[0]**2 + vector[1]**2 + vector[2]**2)
    UnitVector = vector/magnitude

    return UnitVector

def InGoalRegion(current_cog, FeetPositions, movingLeg):
    """
    Determines if a point is inside a triangle using barycentric coordinates.

    Parameters:
    - pt: tuple (x, y) - the point to check
    - v1, v2, v3: tuples (x, y) - vertices of the triangle

    Returns:
    - True if pt is inside the triangle, False otherwise
    """
    # Get vertices from hip positions depending on which legs we're using
    if movingLeg == 1:
        v1 = (FeetPositions[2], FeetPositions[3]) # Leg 2 tuple (x, y)
        v2 = (FeetPositions[4], FeetPositions[5]) # Leg 3 tuple (x, y)
        v3 = (FeetPositions[6], FeetPositions[7]) # Leg 4 tuple (x, y)
    elif movingLeg == 2:
        v1 = (FeetPositions[0], FeetPositions[1]) # Leg 1 tuple (x, y)
        v2 = (FeetPositions[4], FeetPositions[5]) # Leg 3 tuple (x, y)
        v3 = (FeetPositions[6], FeetPositions[7]) # Leg 4 tuple (x, y)
    elif movingLeg == 3:
        v1 = (FeetPositions[0], FeetPositions[1]) # Leg 1 tuple (x, y)
        v2 = (FeetPositions[2], FeetPositions[3]) # Leg 2 tuple (x, y)
        v3 = (FeetPositions[6], FeetPositions[7]) # Leg 4 tuple (x, y)
    elif movingLeg == 4:
        v1 = (FeetPositions[0], FeetPositions[1]) # Leg 1 tuple (x, y)
        v2 = (FeetPositions[2], FeetPositions[3]) # Leg 2 tuple (x, y)
        v3 = (FeetPositions[4], FeetPositions[5]) # Leg 3 tuple (x, y)
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

    new_joint_angles = [0] * (len(init_joint_angles) * 2)  # Use for creating a normal array below
    # new_joint_angles = [0] * len(init_joint_angles)  # Use for creating an array of tuples below

    i = 0
    while i < len(init_joint_angles):
        # Create an array of joint angles [angle[i]-step_size, angle[i]+step_size, ...]
        new_joint_angles[2*i] = init_joint_angles[i] - step_size
        new_joint_angles[2*i+1] = init_joint_angles[i] + step_size
        i = i + 1

        # # Create an array of tuples (angle-step_size, angle+step_size)
        # new_joint_angles[i] = (init_joint_angles[i] - step_size, init_joint_angles[i] + step_size)
        # i = i + 1

    return new_joint_angles

def FindMinCostJoint(new_joint_angles, target_cog, current_joint_angles):
    """Find the minimum cost out of moving each joint angles and return the minimum cost joint"""
    feetPositions = [0] * len(new_joint_angles)
    current_cogs = [0] * len(new_joint_angles)
    cost_vals = [0] * len(new_joint_angles)
    test_indicies = [0] * len(new_joint_angles)

    for idx, i in enumerate(new_joint_angles):
        test_joint_angles = current_joint_angles

        # Update each joint with the associated angle to test
        if idx == 0 or idx == 1:
            test_joint_angles[0] = i
            test_indicies[idx] = 0
        elif idx == 2 or idx == 3:
            test_joint_angles[1] = i
            test_indicies[idx] = 1
        elif idx == 4 or idx == 5:
            test_joint_angles[2] = i
            test_indicies[idx] = 2
        elif idx == 6 or idx == 7:
            test_joint_angles[3] = i
            test_indicies[idx] = 3
        elif idx == 8 or idx == 9:
            test_joint_angles[4] = i
            test_indicies[idx] = 4
        elif idx == 10 or idx == 11:
            test_joint_angles[5] = i
            test_indicies[idx] = 5
        elif idx == 12 or idx == 13:
            test_joint_angles[6] = i
            test_indicies[idx] = 6
        elif idx == 14 or idx == 15:
            test_joint_angles[7] = i
            test_indicies[idx] = 7

        feetPositions = ForwardKinematics(test_joint_angles)
        unprojected_cg = GetCg(feetPositions)
        current_cogs[idx] = [unprojected_cg[0][0], unprojected_cg[0][1]] # Assuming that it is of the from x,y,z
        cost_vals[idx] = GetCost(current_cogs[idx], target_cog)

    # Need to log which joint is improving the cost so we update that one too
    min_cost_index = cost_vals.index(np.min(cost_vals))
    min_joint = new_joint_angles[min_cost_index]
    min_cog = current_cogs[min_cost_index]  # center of gravity corresponding to minimum cost movement (for keeping track of COG path)

    return min_joint, test_indicies[min_cost_index], min_cog

def GetCost(current_cog, target_cog):
    return np.linalg.norm(np.array(current_cog) - np.array(target_cog))

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
joint_angles = [30, 150, 30, 150, 30, 150, 30, 150] # Corresponds to a neutral standing position
step_size = 0.1 # Amount to perturb the joints at a given step
total_joints = len(joint_angles)
target_cog = 0
min_cog = 1000000
minimum_cogs = [] # center of gravity location
leg_positions = [] # set of leg positions
movingLeg = 1
movingToTargetCG = True
threshold = 1

# Compute target_cog using initial joint angles
initFeetPositions = ForwardKinematics(joint_angles)

target_cog = GetTargetCg(initFeetPositions, movingLeg)
plotter2d(movingLeg, target_cog)

print("\nThe target COG is: ", target_cog, "\n")

while movingToTargetCG == True:
    # Get array of incremented joint angles (2 directions for all joints)
    new_joint_angles = GetNewJointAngles(joint_angles, step_size, total_joints)
    #print("New Joint Angles: ", new_joint_angles, "\n")

    # Test each move to see which results in the most improvement; return that one
    min_joint, min_joint_index, min_cog = FindMinCostJoint(new_joint_angles, target_cog, joint_angles)
    plotCoG(min_cog)
    print("Min COG: ", min_cog, "\n")
    print("Vs Target: ", target_cog, "\n")

    # Update the current position so we actually go somewhere
    joint_angles[min_joint_index] = min_joint
    
    # Add new minimum center of gravity, associated leg positions to an array for future plotting
    minimum_cogs.append(min_cog)
    leg_positions.append(joint_angles)
    
    if target_cog[0] - threshold < min_cog[0] and min_cog[0] < target_cog[0] + threshold and target_cog[1] - threshold < min_cog[1] and min_cog[1] < target_cog[1] + threshold:
        movingToTargetCG = False

PlotPath(minimum_cogs)

# # Assume leg one desired, so lift leg four then find intersection of stability between the two

# notStable = True

# legAngles = [30, 150, 30, 150, 30, 150, 30, 30]
# testAngles = legAngles
# costToGo = 0.0

# anglePath = []

# while notStable:
#     # Compare how each perturbance affects the cost
#     bestAngleIndex = 0
#     bestCost = 10000000
#     bestDirection = 0

#     for idx in range(0,6): # For each joint angle...
#         for j in range(0,2): # For each direction...
#             testLegAngles = legAngles
#             if j == 0:
#                 testLegAngles[idx] = testLegAngles + epsilon
#             else:
#                 testLegAngles[idx] = testLegAngles - epsilon

#             centerOfGravity = ForwardKinematics(testLegAngles)
#             testCost = GetCost(centerOfGravity)
#     WHY DID WE REWRITE ALL THIS
#             if testCost < bestCost:
#                 bestCost = testCost
#                 bestAngleIndex = idx
#                 bestDirection = j

#     # Update with the best values
#     if bestDirection == 0:
#         legAngles[bestAngleIndex] = testLegAngles + epsilon
#     else:
#         legAngles[bestAngleIndex] = testLegAngles - epsilon
#     costToGo += bestCost

#     # Log it [NewAngle, AngleIndex, CenterOfGravity]
#     anglePath.append([legAngles[bestAngleIndex], bestAngleIndex, ForwardKinematics(legAngles)])

#     # Test if we made it, exit if yes
#     if InGoalRegion(ForwardKinematics(legAngles)):
#         notStable = False
