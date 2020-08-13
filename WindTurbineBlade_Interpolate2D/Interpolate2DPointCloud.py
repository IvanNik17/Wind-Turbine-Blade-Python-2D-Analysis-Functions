# -*- coding: utf-8 -*-
"""
Created on Mon Apr  9 13:51:34 2018

@author: ivan
"""


import numpy as np
import matplotlib.pyplot as plt
from os import walk, getcwd

def loadBlade(bladeName, scale):
    stringpath = getcwd() + "\\NACA4"
    
    whichBlade = ""
    f = []
    for (dirpath, dirnames, filenames) in walk(stringpath):
        f.extend(filenames)
        
        break
    
    for i in range(0, len(filenames)):
        if filenames[i] == bladeName + '.txt':
            whichBlade = filenames[i]
    
    if len(whichBlade) == 0:
        return -1
        
    
    airfoilDir = dirpath + "\\" +whichBlade
    airfoil = np.loadtxt(airfoilDir)
    bladeArray = np.array(airfoil)
    
    bladeArray[:,0] -= np.mean(bladeArray[:,0])
    
    bladeArray = bladeArray * scale

    return bladeArray





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

    
if __name__ == '__main__': 
    
    
    bladeName = 'NACA 2421_removedPoints'
    bladeScale = 3000
    numCurveInterp = 50
    
    axisSize = 2000
    
    bladeArray_init = loadBlade(bladeName, bladeScale)
    
    
    x_init,y_init = interpolateCurve(bladeArray_init,numCurveInterp)
    arrayBlade = np.array([x_init,y_init]).T
    
    
    fig2d_anim = plt.figure(figsize=(12,6) )
    
    ax_before = fig2d_anim.add_subplot(1,2,1)
    ax_before.plot(bladeArray_init[:,0], bladeArray_init[:,1], 'r.')
    ax_before.axis([-axisSize, axisSize, -axisSize, axisSize])
    
    ax_after = fig2d_anim.add_subplot(1,2,2)
    ax_after.plot(x_init, y_init, 'b.')
    ax_after.axis([-axisSize, axisSize, -axisSize, axisSize])
    
    