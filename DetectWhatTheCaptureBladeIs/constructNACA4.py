# -*- coding: utf-8 -*-
"""
Created on Wed Nov  1 15:42:25 2017

Construct a NACA4 blade from given values 
Initial idea taken from https://github.com/dgorissen/naca and expanded
"""


import math
import matplotlib.pyplot as pyplot
import numpy as np


from os import walk

#https://en.wikipedia.org/wiki/NACA_airfoil#Equation_for_a_cambered_4-digit_NACA_airfoil
def camber_line( x, m, p, c ):
    return np.where((x>=0)&(x<=(c*p)),
                    m * (x / np.power(p,2)) * (2.0 * p - (x / c)),
                    m * ((c - x) / np.power(1-p,2)) * (1.0 + (x / c) - 2.0 * p ))

def dyc_over_dx( x, m, p, c ):
    return np.where((x>=0)&(x<=(c*p)),
                    ((2.0 * m) / np.power(p,2)) * (p - x / c),
                    ((2.0 * m ) / np.power(1-p,2)) * (p - x / c ))

def thickness( x, t, c ):
    term1 =  0.2969 * (np.sqrt(x/c))
    term2 = -0.1260 * (x/c)
    term3 = -0.3516 * np.power(x/c,2)
    term4 =  0.2843 * np.power(x/c,3)
    term5 = -0.1015 * np.power(x/c,4)
    return 5 * t * c * (term1 + term2 + term3 + term4 + term5)

def naca4(x, m, p, t, c=1):
    dyc_dx = dyc_over_dx(x, m, p, c)
    th = np.arctan(dyc_dx)
    yt = thickness(x, t, c)
    yc = camber_line(x, m, p, c)  
    
    return ((x - yt*np.sin(th), yc + yt*np.cos(th)), 
            (x + yt*np.sin(th), yc - yt*np.cos(th)))
    
    
def naca4FromStr(currNACA):
    
    m = 0.01 * int(currNACA[0])
    p = 0.1 * int(currNACA[1])
    t = 0.01 * int(currNACA[2:4])
    c = 1.0
    
    #naca2412 
#    m = 0.02
#    p = 0.4
#    t = 0.12
#    c = 1.0


    x = np.linspace(0,1,200)
    item = naca4(x, m, p, t, c)
    
    fullCalculatedBladeX = np.array([])
    fullCalculatedBladeY = np.array([])
    
    fullCalculatedBladeX = np.concatenate([fullCalculatedBladeX, item[0][0]])
    fullCalculatedBladeX = np.concatenate([fullCalculatedBladeX, item[1][0]])
    
    fullCalculatedBladeY = np.concatenate([fullCalculatedBladeY, item[0][1]])
    fullCalculatedBladeY = np.concatenate([fullCalculatedBladeY, item[1][1]])
    
    fullCalculatedBlade = np.concatenate([[fullCalculatedBladeX,fullCalculatedBladeY]]).T
    
    fullCalculatedBlade= np.matrix(fullCalculatedBlade)
    
    
    return fullCalculatedBlade
    
    
if __name__ == '__main__':
    
    stringpath = r"NACA blades\NACA4"

    f = []
    for (dirpath, dirnames, filenames) in walk(stringpath):
        f.extend(filenames)
        break
    
    
    currFileName = filenames[13]
    airfoilDir = dirpath + "\\" +currFileName
    airfoil = np.loadtxt(airfoilDir)
    
    currNACA = currFileName[5:9]
    currNACA = "3411"
    fullCalculatedBlade = naca4FromStr(currNACA)

    

    pyplot.plot(fullCalculatedBlade[:,0], fullCalculatedBlade[:,1], 'bo')
    

    pyplot.axes().set_aspect('equal', 'datalim')
    
    pyplot.plot(airfoil[:,0],airfoil[:,1],'ro')