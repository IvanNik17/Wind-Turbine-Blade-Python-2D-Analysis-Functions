# -*- coding: utf-8 -*-
"""
Created on Mon Oct 16 14:21:03 2017

@author: ivan nikolov

If a slices of LiDAR data from a blade are given, calculate what type of blade it is, either NACA4 or NACA5 and specific numbers
"""
import numpy as np

import matplotlib.pyplot as plt

from scipy.spatial.distance import pdist

from os import walk

import math

from constructNACA4 import naca4, camber_line



def rotate(origin, point, angle):
    """
    Rotate a point counterclockwise by a given angle around a given origin.

    The angle should be given in radians.
    """
    ox, oy = origin
    px, py = point
    
    angle = math.radians(angle)

    qx = ox + math.cos(angle) * (px - ox) - math.sin(angle) * (py - oy)
    qy = oy + math.sin(angle) * (px - ox) + math.cos(angle) * (py - oy)
    return qx, qy

def split(start, end, segments):
    x_delta = (end[0] - start[0]) / float(segments)
    y_delta = (end[1] - start[1]) / float(segments)
    points = []
    for i in range(1, segments):
        points.append([start[0] + i * x_delta, start[1] + i * y_delta])
    return np.array(points)
    
    
def find_nearest(array,value):
    idx = (np.abs(array-value)).argmin()
    return array[idx]    



def intersectionLineCurveOLD(lineFirst, lineSecond, curvePoints):

    b = (lineSecond[1] - lineFirst[1]) / (lineSecond[0] - lineFirst[0]) # gradient
    a = lineFirst[1] - b * lineFirst[0] # intercept
    B = (a + curvePoints[:,0] * b) - curvePoints[:,1] # distance of y value from line
    ix = np.where(B[1:] * B[:-1] < 0)[0] # index of points where the next point is on the other side of the line
    
    
    d_ratio = B[ix] / (B[ix] - B[ix + 1]) # similar triangles work out crossing points
    cross_points = np.zeros((len(ix), 2)) # empty array for crossing points
    cross_points[:,0] = curvePoints[ix,0] + d_ratio * (curvePoints[ix+1,0] - curvePoints[ix,0]) # x crossings
    cross_points[:,1] = curvePoints[ix,1] + d_ratio * (curvePoints[ix+1,1] - curvePoints[ix,1]) # y crossings
    

    return cross_points
    
def calculateEndPoints(pointCloud):
    maxXInd = int(np.argmax(pointCloud[:,0]))
    minXInd = int(np.argmin(pointCloud[:,0]))
    
    minPoint = pointCloud[minXInd,:].copy()
    maxPoint = pointCloud[maxXInd,:].copy() 
    
    return minPoint, maxPoint
    
    
    
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

    
whatNaca = 5




stringpath = r"NACA blades\Captured"

airfoil = loadBlade(stringpath, 0, True)

stringpath_ground = r"NACA blades\NACA4"

airfoil_ground = loadBlade(stringpath_ground,25, False)

bladeScale = 2881 # this is a magic number, but change to the proper scale of the real world blade
airfoil_ground = airfoil_ground*bladeScale  

airfoilArray = np.delete(airfoil,0,0)

#Use first and last scan
uniqueScans = np.unique(airfoilArray[:,2])

scanFirst = airfoilArray[airfoilArray[:,2] == 0,:]

scanLast = airfoilArray[airfoilArray[:,2] == max(uniqueScans),:]

minPointF, maxPointF = calculateEndPoints(scanFirst)                   
minPointL, maxPointL = calculateEndPoints(scanLast)


scanFirst[:,0] = scanFirst[:,0] -  minPointF[0]
scanLast[:,0] = scanLast[:,0] - minPointL[0] 


minPointF, maxPointF = calculateEndPoints(scanFirst)                   
minPointL, maxPointL = calculateEndPoints(scanLast)


scanFirst[:,1] = scanFirst[:,1] -  maxPointF[1]
scanLast[:,1] = scanLast[:,1] - maxPointL[1] 



scanCombinedFL = np.concatenate((scanFirst[:,0:2], scanLast[:,0:2]),axis = 0)

minPointF, maxPointF = calculateEndPoints(scanFirst)                   
minPointL, maxPointL = calculateEndPoints(scanLast)

minPoint =np.array([minPointF[0], np.mean([minPointF[1],minPointL[1] ])])     # or np.mean             
maxPoint = np.array([maxPointF[0],np.mean([maxPointF[1],maxPointL[1] ]) ])    # or np.mean 


fig1 = plt.figure()
ax1 = fig1.add_subplot(111)
fig2 = plt.figure()
ax2 = fig2.add_subplot(111)
fig3 = plt.figure()
ax3 = fig3.add_subplot(111)

ax1.plot(minPoint[0],minPoint[1],'kx',markersize= 15)

ax1.plot(maxPoint[0],maxPoint[1],'kx',markersize= 15)


ax3.plot(airfoilArray[:,0],airfoilArray[:,1],'ro',markersize= 5)



chord = np.linalg.norm(maxPoint - minPoint)


segmentPoints = split(minPoint, maxPoint, 20)

segmentPointsSecondPoint = np.copy(segmentPoints)
segmentPointsSecondPoint[:,1] += 1000
segmentPointsSecondPoint[:,0] += 0.00001


intersectPoints = []
distanceIntersectPoints = []
camberLine = []




for i in range(0, len(segmentPoints)):
    
    currPoints = intersectionLineCurveOLD( segmentPointsSecondPoint[i,:], segmentPoints[i,:], scanCombinedFL)
    
    intersPoint_min = currPoints[np.argmin(currPoints[:,1]),:]
    intersPoint_max = currPoints[np.argmax(currPoints[:,1]),:]
                                 
    inters_temp = np.array([intersPoint_min,intersPoint_max])
    
    intersectPoints.append(inters_temp)
    ax1.plot(inters_temp[:,0],inters_temp[:,1])

    distTocurr = np.array([ inters_temp[0,:],  inters_temp[1,:] ])
    distTocurr = np.asscalar(pdist(distTocurr,'euclidean'))
    
    distanceIntersectPoints.append(distTocurr)
    
    centerPointX = np.asscalar((inters_temp[0,0] - inters_temp[1,0])/2 + inters_temp[1,0])
    centerPointY = np.asscalar((inters_temp[0,1] - inters_temp[1,1])/2 + inters_temp[1,1])
    camberLine.append([centerPointX,centerPointY])
#    
camberLine = np.array(camberLine)
maximumThickness = max(distanceIntersectPoints)



distancesCamberChord = np.sqrt(np.sum((segmentPoints-camberLine)**2,axis=1))
maximumCamber = max(distancesCamberChord)
maximumCamberIndex = int(np.argmax(distancesCamberChord))
maximumCamberDist = math.fabs(minPoint[0] - segmentPoints[maximumCamberIndex,0])


maximumCamberPercentCord = 100* (maximumCamberDist/chord) 
camberPercentCord = 100* (maximumCamber/chord)
thicknessPercentCord = 100* (maximumThickness/chord)


#if whatNaca == 4:
    
print("Blade is NACA4 - ", int(round(camberPercentCord)),int(round(round(maximumCamberPercentCord)/10)%10), str( int(round(thicknessPercentCord))).zfill(2) )
#elif whatNaca == 5:
print("OR")
    
print("Blade is NACA5 - ","2 ",int(round(maximumCamberPercentCord/100*20)), " ? " ,str( int(round(thicknessPercentCord))).zfill(2) )





#naca2412 
#m = 0.02
#p = 0.4
#t = 0.12
#c = 1.0

m = (maximumCamber/chord)
p = (maximumCamberDist/chord)
t = (maximumThickness/chord)
c = 1.0


sizeX = maxPoint[0] - minPoint[0]
sizeY = maxPoint[1] - minPoint[1]

distBetweenEdges = np.array([ minPoint,  maxPoint ])
distBetweenEdges = np.asscalar(pdist(distBetweenEdges,'euclidean'))


x = np.linspace(0,1,100)
item = naca4(x, m, p, t, c)
    
fullCalculatedBladeX = np.array([])
fullCalculatedBladeY = np.array([])


fullCalculatedBladeX = np.concatenate([fullCalculatedBladeX, item[0][0]])
fullCalculatedBladeX = np.concatenate([fullCalculatedBladeX, item[1][0]])

fullCalculatedBladeY = np.concatenate([fullCalculatedBladeY, item[0][1]])
fullCalculatedBladeY = np.concatenate([fullCalculatedBladeY, item[1][1]])

fullCalculatedBlade = np.concatenate([[fullCalculatedBladeX,fullCalculatedBladeY]]).T

fullCalculatedBlade[:,0] = fullCalculatedBlade[:,0]*distBetweenEdges

fullCalculatedBlade[:,1] = fullCalculatedBlade[:,1]*distBetweenEdges


fullCalculatedBlade= np.matrix(fullCalculatedBlade)



ax1.plot(segmentPoints[:,0],segmentPoints[:,1],'bo')


ax1.plot(camberLine[:,0],camberLine[:,1],'go')

ax1.plot(scanCombinedFL[:,0],scanCombinedFL[:,1],'ro')

#Calculate distance between 2D point clouds
allMinDist = []
for i in range(0,len(fullCalculatedBlade)):
    temp_minDist = (np.sqrt(np.sum(np.asarray(airfoil_ground - fullCalculatedBlade[i,:])**2, axis=1))).min()
    allMinDist.append(temp_minDist)

distAverage = np.average(allMinDist)
distStdDev = np.std(allMinDist, ddof=1)
ax2.plot(fullCalculatedBlade[:,0], fullCalculatedBlade[:,1], 'bo')
ax2.plot(airfoil_ground[:,0],airfoil_ground[:,1],'ro')

ax2.set_aspect('equal', 'datalim')
ax1.set_aspect('equal', 'datalim')
ax3.set_aspect('equal', 'datalim')