# -*- coding: utf-8 -*-
"""
Created on Tue Dec 12 09:54:48 2017

@author: ivan
"""
import numpy as np

import matplotlib.pyplot as plt

from os import walk
import math



def testEllipse(distance1,distance2, orientation, center):
#    ellipseInfo = collections.namedtuple('ellipseInfo', ['radiusAnglesEllipse', 'ellipsePointsPos'])
    numberOfPoints = 360
    centerX = center[0]
    centerY = center[1]
    orientation = -orientation
    theta = np.linspace(0, 2*math.pi, numberOfPoints)
    orientation=orientation*math.pi/180
    
    radiusDist = []
    xx2 = []
    yy2 = []

    for i in range(0,numberOfPoints):
        xx = -(distance1/2) * math.sin(theta[i]) + centerX
        yy = -(distance2/2) * math.cos(theta[i]) + centerY
    
        xx2_temp = (xx-centerX)*math.cos(orientation) - (yy-centerY)*math.sin(orientation) + centerX
        yy2_temp = (xx-centerX)*math.sin(orientation) + (yy-centerY)*math.cos(orientation) + centerY

        xx2.append(xx2_temp)
        yy2.append(yy2_temp)
    
        radiusDist.append(math.sqrt(xx2_temp**2 + yy2_temp**2))


    degrees = range(0,numberOfPoints)
#    theta_deg = numpy.degrees(theta)
    
    
    radiusAngles = np.column_stack((radiusDist,degrees))
    ellipsePos = np.column_stack((xx2,yy2))

    return radiusAngles, ellipsePos  


def calculateMinMax(pointCloud2D):
    distAll = np.linalg.norm(pointCloud2D - pointCloud2D[:,None], axis=-1)

    distAll_noDup = np.triu(distAll)
    
    minDist = np.min(distAll_noDup[np.nonzero(distAll_noDup)])
    maxDist = np.max(distAll_noDup[np.nonzero(distAll_noDup)])
    minDist_index = np.where(distAll_noDup==minDist)
    minDist_points = np.array([pointCloud2D[minDist_index[0][0],:], pointCloud2D[minDist_index[1][0],:]])
    maxDist_index = np.where(distAll_noDup==maxDist)
    maxDist_points = np.array([pointCloud2D[maxDist_index[0][0],:], pointCloud2D[maxDist_index[1][0],:]])
    
    return minDist, maxDist, minDist_points, maxDist_points
    
    
def split(start, end, segments):
    x_delta = (end[0] - start[0]) / float(segments)
    y_delta = (end[1] - start[1]) / float(segments)
    points = []
    points.append(start)
    for i in range(1, segments):
        points.append([start[0] + i * x_delta, start[1] + i * y_delta])
        
    points.append(end)
    return np.array(points)
    
    
def rotateBlade(bladeArray, angle):
    
    angle = -angle
    qx = 0 + math.cos(math.radians(angle)) * (bladeArray[:,0] - 0) - math.sin(math.radians(angle)) * (bladeArray[:,1] - 0)
    qy = 0 + math.sin(math.radians(angle)) * (bladeArray[:,0] - 0) + math.cos(math.radians(angle)) * (bladeArray[:,1] - 0)
    
    

    
    newCoord = np.array([qx,qy]).T

    return newCoord
    



stringpath = r"NACA blades\Captured"
    
f = []
for (dirpath, dirnames, filenames) in walk(stringpath):
    f.extend(filenames)
    break


airfoilDir = dirpath + "\\" +filenames[0]
airfoil = np.loadtxt(airfoilDir,delimiter=",")
  
testPoints = np.delete(airfoil,0,0)

count = 0
currPiece = testPoints[testPoints[:,2] == count ,0:2]  


currPiece = rotateBlade(currPiece, 0)


minDist,maxDist,minDist_points,maxDist_points = calculateMinMax(currPiece)



cov_mat = np.cov(currPiece.T)

eig_val_cov, eig_vec_cov = np.linalg.eig(cov_mat)
        
maxIndEigenval = np.argmax(eig_val_cov)
evec1 = eig_vec_cov[:,maxIndEigenval]

angleOffsetMeasured = math.degrees(np.arctan2( evec1[0],evec1[1]   )) 



#Reproject points to a line oriented depending on the PCA analysis
test = evec1.reshape((1,2))

currPieceRot = currPiece.T
c_values = test.dot(currPieceRot)
projected = test.T.dot(c_values).T


#Calculat the distances from each point of the current piece  to the projections, find the maximum
dist_currToFlat = np.sqrt(((currPiece-projected)**2).sum(axis=-1))
dist_largestCurrToFlat = dist_currToFlat.max()
smallestRadiusDistance = max( abs(dist_largestCurrToFlat-dist_currToFlat[0]), abs(dist_largestCurrToFlat-dist_currToFlat[-1]) )
print(smallestRadiusDistance)

distance1 = smallestRadiusDistance*2;
distance2 = maxDist;


radiusAnglesEllipse, ellipsePointsPos = testEllipse(distance1,distance2,angleOffsetMeasured, [0,0])

plt.plot(ellipsePointsPos[:,0],ellipsePointsPos[:,1],'ro') 

plt.plot(maxDist_points[:,0],maxDist_points[:,1],'ro',markersize=10)     
plt.plot(minDist_points[:,0],minDist_points[:,1],'go',markersize=10)  

plt.plot(projected[:,0],projected[:,1],'ro')

plt.plot([currPiece[:,0],projected[:,0]], [currPiece[:,1],projected[:,1]],'k-')


plt.plot(currPiece[:,0], currPiece[:,1], 'bo')

plt.axes().set_aspect('equal', 'datalim')