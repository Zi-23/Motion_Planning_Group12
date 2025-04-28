import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

def plotter3d(Feet_p, Knees_p, Hips_p, animation_on = 0):
    ##hipsb is determined by the order of the joints 
    ##feet_P, etc are(4,3), (x,y,z) in structure.
    ##animation_on if you want to see change ovver time.
    hipsb = [0, 1, 3, 2, 0] #change this if nessessary
    
    if animation_on == 1: 
        plt.pause(0.5)

    Feet = np.array(Feet_p).reshape(4,3)
    Knees = np.array(Knees_p).reshape(4,3)
    Hips = np.array(Hips_p).reshape(4,3)

    spline = plt.figure(1)
    s3d = spline.add_subplot(111, projection='3d')
    s3d.set_box_aspect([1,.8,0.5])
    s3d.grid(True)

    for i in range(4):
        s3d.scatter(Feet[i,0], Feet[i,1], Feet[i,2], color='b', s=25)
        s3d.plot([Feet[i,0], Knees[i,0]],[Feet[i,1], Knees[i,1]],[Feet[i,2], Knees[i,2]],'b-', linewidth=2)
        
        s3d.scatter(Knees[i,0], Knees[i,1], Knees[i,2], color='b', s=25)
        s3d.plot([Knees[i,0], Hips[i,0]],[Knees[i,1], Hips[i,1]],[Knees[i,2], Hips[i,2]],'b-', linewidth=2)
        
        s3d.scatter(Hips[i,0], Hips[i,1], Hips[i,2], color='k', s=25)
        
    s3d.plot(Hips[hipsb,0],Hips[hipsb,1],Hips[hipsb,2],'k-', linewidth=2)
    
    plt.show()