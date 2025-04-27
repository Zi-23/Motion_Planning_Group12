import matplotlib.pyplot as plt
import numpy as np
import itertools
import random
import math

CoG_i = []

def plotter2d(FeetPositions, movingLeg, target_cog):
    ## plots the legs and target center of gravity
    plt.figure()
    plt.grid(True)
    plt.axis('equal')

    FeetPositions = np.array(FeetPositions).reshape(4,2)

    for i in range(4):
        plt.plot(FeetPositions[i,0], FeetPositions[i,1], 'ko', markersize=8, markerfacecolor='k')

    support_legs = [i for i in range(4) if (i+1) != movingLeg]
    supportFeet = np.array(FeetPositions)[support_legs, :]

    plt.fill(supportFeet[:,0], supportFeet[:,1], 'c', alpha=0.3, edgecolor='none')

    plt.plot(target_cog[0], target_cog[1], 'go', markersize=7, linewidth=2)

    plt.title('Stability Region vs Center of Gravity')

def plotCoG(CoG):
    ## add new CoG to Plot
    plt.plot(CoG[0], CoG[1], 'ro', markersize=5)
    CoG_i.append(CoG)

def plandrw(index):
    ## adds lines for the cogs for the final path
    for i in range(len(index) - 1):
        p_1 = CoG_i[index[i]]
        p_2 = CoG_i[index[i+1]]

        plt.plot([p_1[0], p_2[0]], [p_1[1], p_2[1]], 'g-', linewidth=1)
        plt.plot(p_1[0], p_1[1], 'o', markersize=6, markeredgecolor='g', markerfacecolor='none')
    plt.plot(p_2[0], p_2[1], 'o', markersize=6, markeredgecolor='g', markerfacecolor='none')






        