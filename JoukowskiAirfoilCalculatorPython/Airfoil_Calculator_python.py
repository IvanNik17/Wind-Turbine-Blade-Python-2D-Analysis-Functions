# -*- coding: utf-8 -*-
"""
Created on Fri Mar 16 10:09:28 2018

@author: In Matlab Emil Krog Kruse / Rewriten in Python - Ivan Nikolov 

Joukowsky Airfoil
"""

import numpy as np
import math
import matplotlib.pyplot as plt

import timeit

start_time = timeit.default_timer()

ChordLength = 3.5
Thickness = 20
Camber = 5

numP = 256

Cl_J_vec = []
m1oc = Thickness/100 * (4/(3*math.sqrt(3)))
m2oc = 2*0.01*Camber


i = 1

tc = 0


Thickness_Residual = 1
Camber_Residual = 1
Convergence_Criteria = 0.001


R = 1.0
angle =list(np.arange(0.0,2*math.pi,2*math.pi/numP))
angle = np.array(angle)
init_circle = R*(np.cos(angle)+1j*np.sin(angle))


while (abs(Thickness_Residual) > Convergence_Criteria or abs(Camber_Residual) > Convergence_Criteria):
    
    if i == 1000:
        break

    elif i > 1:

        if Thickness_Residual < -Convergence_Criteria:
            m1oc = m1oc-0.001*abs(Thickness_Residual)
        elif Thickness_Residual > Convergence_Criteria:
            m1oc = m1oc+0.001*abs(Thickness_Residual)


        if Camber_Residual < -Convergence_Criteria:
            m2oc = m2oc-0.001*abs(Camber_Residual)
        elif Camber_Residual > Convergence_Criteria:
            m2oc = m2oc+0.001*abs(Camber_Residual)


    c = R/math.sqrt(m2oc**2+(m1oc+1)**2)
    c2_J = c**2
    
    m1 = m1oc*c
    m2 = m2oc*c
    s = -m1 + 1j*m2
    
    
    z_circle = init_circle + s
    z_airfoil = z_circle+c2_J/z_circle
    xc=np.real(z_airfoil)
    yc=np.imag(z_airfoil)
    
    num1 = int(np.argmin(xc))
    y_max_vector_J = np.zeros([num1,1])
    y_camber_vector_J = np.zeros([num1,1])
    
    for n in range(0, num1):
        y_max = 0
        y_max_old = y_max
        y_camber = 0
        y_camber_old = y_camber
        
        for m in range(num1,len(yc)-1):
 
            if xc[m]==xc[n]:
                y_max = 100*((yc(n)-yc(m))/(np.max(xc)-np.min(xc)))
                y_camber = 100*((0.5*(yc[n]+yc[m]))/(np.max(xc)-np.min(xc)))
                
            elif xc[m] < xc[n] and xc[m+1] > xc[n]:
                y_interpolated1 = yc[m]+(((yc[m+1]-yc[m])/(xc[m+1]-xc[m]))*(xc[n]-xc[m]))
                y_camber = 100*((0.5*(yc[n] + y_interpolated1))/(np.max(xc)-np.min(xc)))
                
                y_max = 100*((yc[n]-y_interpolated1)/(np.max(xc)-np.min(xc)))

            
            if y_camber > y_camber_old:
                y_camber_vector_J[n] = y_camber

            if y_max > y_max_old:
                y_max_vector_J[n] = y_max

            y_max_old = y_max
            y_camber_old = y_camber
            

    tc = np.max(y_max_vector_J)
    
    camber_J = np.max(y_camber_vector_J)
    Thickness_Residual = Thickness-tc
    Camber_Residual = Camber-camber_J
    
    i = i+1
    
X_t = np.real(z_airfoil)
Y_t = np.imag(z_airfoil)

L = np.max(X_t)-np.min(X_t)
Scale = L/ChordLength
X = (X_t-np.min(X_t))/Scale
Y = Y_t/Scale

elapsed = timeit.default_timer() - start_time

print(elapsed)

plt.plot(X,Y,'b.')
plt.axes().set_aspect('equal', 'datalim')
