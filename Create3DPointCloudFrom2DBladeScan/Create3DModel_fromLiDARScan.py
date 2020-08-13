# -*- coding: utf-8 -*-
"""
Created on Tue Jan 23 08:37:59 2018

@author: ivan
"""

import numpy as np
import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.spatial.distance import pdist
import scipy.interpolate
import sys

import time
import math
from os import walk

import csv



def proper3DAspect(ax,array3d_beginning, array3d_end):
    
    min_x = min(array3d_beginning[:,0].min(), array3d_end[:,0].min())
    min_y = min(array3d_beginning[:,1].min(), array3d_end[:,1].min())
    min_z = min(array3d_beginning[:,2].min(), array3d_end[:,2].min())
    
    max_x = max(array3d_beginning[:,0].max(), array3d_end[:,0].max())
    max_y = max(array3d_beginning[:,1].max(), array3d_end[:,1].max())
    max_z = max(array3d_beginning[:,2].max(), array3d_end[:,2].max())

    
    
    max_range = np.array([max_x-min_x, max_y-min_y, max_z-min_z]).max() / 2.0
    
    mid_x = (max_x+min_x) * 0.5
    mid_y = (max_y+min_y) * 0.5
    mid_z = (max_z+min_z) * 0.5
    ax.set_xlim(mid_x - max_range, mid_x + max_range)
    ax.set_ylim(mid_y - max_range, mid_y + max_range)
    ax.set_zlim(mid_z - max_range, mid_z + max_range)
    
    
def loadBlade(stringpath, whichFile, useDel=False):
    
    f = []
    for (dirpath, dirnames, filenames) in walk(stringpath):
        f.extend(filenames)
        break
    
    airfoilDir = dirpath + "\\" +filenames[whichFile]

    if useDel:
        
        airfoil = np.loadtxt(airfoilDir,delimiter=",")
        
    else:
        airfoil = np.loadtxt(airfoilDir)

    
    airfoilArray = np.delete(airfoil,0,0)

    return airfoilArray
    
    
def intermediates(p1, p2, nb_points=10):
    """"Return a list of nb_points equally spaced points
    between p1 and p2"""
    # If we have 8 intermediate points, we have 8+1=9 spaces
    # between p1 and p2
    x_spacing = (p2[0] - p1[0]) / (nb_points )
    y_spacing = (p2[1] - p1[1]) / (nb_points)

    return [[p1[0] + i * x_spacing, p1[1] +  i * y_spacing] 
            for i in range(1, nb_points)]    

def interpolateCurve(bladeArray, numPoints):
    x,y = bladeArray.T
    xd = np.diff(x)
    yd = np.diff(y)
    dist = np.sqrt(xd**2+yd**2)
    u = np.cumsum(dist)
    u = np.hstack([[0],u])
    t = np.linspace(0,u.max(),numPoints)
    xn = np.interp(t, u, x)
    yn = np.interp(t, u, y)
    
    return xn, yn
    
    
    


def visualizeCalcBlade(bladeArray_init_3d, bladeArray_end_3d, interm_points_3d):

    fig3D = plt.figure()
    ax = Axes3D(fig3D)
    
    
    ax.scatter(bladeArray_init_3d[:,0],bladeArray_init_3d[:,1],bladeArray_init_3d[:,2],lw=0.5)
    
    ax.scatter(bladeArray_end_3d[:,0],bladeArray_end_3d[:,1],bladeArray_end_3d[:,2],lw=0.5)
    
    ax.scatter(interm_points_3d[:,0],interm_points_3d[:,1],interm_points_3d[:,2],lw=0.5)
    
    proper3DAspect(ax, bladeArray_init_3d, bladeArray_end_3d)




def makeBladeSegments(z_initial, z_end, bladeArray_init, bladeArray_end, numCurveInterp, numHeightInterp):
    
#  Init blade prep
    z_array_init = np.ones(len(bladeArray_init[:,0])) * z_initial
    z_array_init = z_array_init.reshape((len(z_array_init),1))
    
    bladeArray_init_3d = np.concatenate([bladeArray_init,z_array_init],axis=1)

    x_init,y_init = interpolateCurve(bladeArray_init,numCurveInterp)
    
#  End blade prep  
    z_array_end = np.ones(len(bladeArray_end[:,0])) * z_end
    z_array_end = z_array_end.reshape((len(z_array_end),1))
    
    bladeArray_end_3d = np.concatenate([bladeArray_end,z_array_end],axis=1)
    
    x_end,y_end = interpolateCurve(bladeArray_end,numCurveInterp)
    
    
    allIntermPoints = []
    for i in range(0,len(x_end)):
        tempPoint_init = [x_init[i],y_init[i]]
        tempPoint_end = [x_end[i],y_end[i]]
        interm_points = intermediates(tempPoint_init, tempPoint_end, numHeightInterp)
        interm_points = np.array(interm_points) 
        allIntermPoints.append(interm_points)
    
    
    allIntermPoints = np.vstack(allIntermPoints)
    
    interm_points_z = np.arange(z_initial-(z_initial - z_end)/numHeightInterp, z_end, -(z_initial - z_end)/numHeightInterp)
    interm_points_z = interm_points_z.reshape((len(interm_points_z),1))
    
    interm_points_z = np.tile(interm_points_z,(numCurveInterp,1))
    
    interm_points_3d = np.concatenate([allIntermPoints,interm_points_z],axis=1)
    
    return bladeArray_init_3d, bladeArray_end_3d, interm_points_3d
    

    
if __name__ == '__main__': 
    
    z_initial = 1200
    z_end = 0
    numCurveInterp = 100
    numHeightInterp = 100
    
    
    stringpath = r"NACA blades\NACA4"
    
    airfoil_initial = loadBlade(stringpath,25)
    

    
    bladeArray_init = np.array(airfoil_initial)
    
    bladeArray_init[:,0] -= np.mean(bladeArray_init[:,0])
    bladeArray_init = bladeArray_init * 1400   
    
    
    airfoil_end = loadBlade(stringpath,25)

    
    bladeArray_end = np.array(airfoil_end)
    
    bladeArray_end[:,0] -= np.mean(bladeArray_end[:,0])
    bladeArray_end = bladeArray_end * 1400 
    
    
    bladeArray_init_3d, bladeArray_end_3d, interm_points_3d = makeBladeSegments(z_initial, z_end, bladeArray_init, bladeArray_end, numCurveInterp, numHeightInterp)
        
    visualizeCalcBlade(bladeArray_init_3d, bladeArray_end_3d, interm_points_3d)

#    numCurveInterp = 50
#    x_init,y_init = interpolateCurve(bladeArray_init,numCurveInterp)
#    arrayBlade = np.array([x_init,y_init]).T
#    
#
#    fig2d_anim = plt.figure(figsize=(12,6) )
#    
#    ax = fig2d_anim.add_subplot(1,2,1)
#    
#    ax.plot(x_init, y_init, 'r.')
#    
#    axisSize = 2000
#    ax.axis([-axisSize, axisSize, -axisSize, axisSize])
#    
#    ax = fig2d_anim.add_subplot(1,2,2)
#    
#    ax.plot(bladeArray_init[:,0], bladeArray_init[:,1], 'b.')
#    
#    axisSize = 2000
#    ax.axis([-axisSize, axisSize, -axisSize, axisSize])

# Initial blade segment

airfoil_initial = loadBlade(stringpath,23)

airfoil_initial = airfoil_initial[0:17,:]

bladeArray = np.array(airfoil_initial)

bladeArray[:,0] -= np.mean(bladeArray[:,0])





z_array = np.ones(len(bladeArray[:,0])) * z_initial
z_array = z_array.reshape((len(z_array),1))


bladeArray = bladeArray * 3000

bladeArray_3d = np.concatenate([bladeArray,z_array],axis=1)


x_init,y_init = interpolateCurve(bladeArray,30)






#End blade segment
z_end = 500

airfoil_end = loadBlade(stringpath,5)

airfoil_end = airfoil_end[0:17:2,:]

bladeArray_end = np.array(airfoil_end)

bladeArray_end[:,0] -= np.mean(bladeArray_end[:,0])




z_array_end = np.ones(len(bladeArray_end[:,0])) * z_end
z_array_end = z_array_end.reshape((len(z_array_end),1))


bladeArray_end = bladeArray_end * 1000

bladeArray_end_3d = np.concatenate([bladeArray_end,z_array_end],axis=1)


x_end,y_end = interpolateCurve(bladeArray_end,30)


allIntermPoints = []
for i in range(0,len(x_end)):
    tempPoint_init = [x_init[i],y_init[i]]
    tempPoint_end = [x_end[i],y_end[i]]
    interm_points = intermediates(tempPoint_init, tempPoint_end, 20)
    interm_points = np.array(interm_points) 
    allIntermPoints.append(interm_points)


allIntermPoints = np.vstack(allIntermPoints)

interm_points_z = np.arange(z_initial-(z_initial - z_end)/20, z_end, -(z_initial - z_end)/20)
interm_points_z = interm_points_z.reshape((len(interm_points_z),1))

interm_points_z = np.tile(interm_points_z,(30,1))

interm_points_3d = np.concatenate([allIntermPoints,interm_points_z],axis=1)







fig = plt.figure()
ax = fig.add_subplot(111)
ax.scatter(bladeArray[:,0],bladeArray[:,1], c='r')


ax.scatter(allIntermPoints[:,0],allIntermPoints[:,1], c='g')

ax.scatter(bladeArray_end[:,0],bladeArray_end[:,1], c='b')


ax.set_aspect('equal', 'datalim')


fig2 = plt.figure()
ax2 = Axes3D(fig2)


ax2.scatter(bladeArray_3d[:,0],bladeArray_3d[:,1],bladeArray_3d[:,2],lw=0.5)

ax2.scatter(bladeArray_end_3d[:,0],bladeArray_end_3d[:,1],bladeArray_end_3d[:,2],lw=0.5)

ax2.scatter(interm_points_3d[:,0],interm_points_3d[:,1],interm_points_3d[:,2],lw=0.5)

proper3DAspect(ax2, bladeArray_3d,bladeArray_end_3d)





