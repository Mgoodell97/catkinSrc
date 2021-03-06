# -*- coding: utf-8 -*-
import matplotlib.pyplot as plt
import numpy as np
import math



# For creating videos
import os 
#import cv2  
from PIL import Image 
import re 

_ACTIONS_2 = ['u','d','l','r','ne','nw','sw','se']
_ACTIONS_2_ANGLES = np.pi*np.array([(1.0/2.0),(-1.0/2.0),(1.0),(0.0),(1.0/4.0),(3.0/4.0),(-3.0/4.0),(-1.0/4.0)])

# *************** Functions ***************

def find_nearest(array, value):
    array = np.asarray(array)
    idx = (np.abs(array - value)).argmin()
    return array[idx],idx

def DiscretizeMap(xParticle, yParticle, xSteps, ySteps, xRange, yRange):
    # Dicretizes the particle map and places the number of particles in bins
    # Return a matrix with all particles in there respective bins
    # also returns the dirtitized x and y locations for all bins 
    # i.e xBins = [41,51,61,71,81]
    
    xEdgesTemp = np.linspace(xRange[0], xRange[1], num=xSteps+1)
    yEdgesTemp = np.linspace(yRange[0], yRange[1], num=ySteps+1)
    
    patriclesArray, _, _ = np.histogram2d(yParticle, xParticle, bins=(yEdgesTemp, xEdgesTemp))
    _, xedges, yedges = np.histogram2d(xParticle, yParticle, bins=(xEdgesTemp, yEdgesTemp))
    
    patriclesArray = np.flip(patriclesArray,axis=0)
    
    xBins = [];
    yBins = [];
    
    for i in range(xSteps):
        xBins.append((xedges[i] + xedges[i+1])/2)

    for i in range(ySteps):
        yBins.append((yedges[i] + yedges[i+1])/2)   
    yBins.reverse()
    
    particleGraph = np.empty((ySteps,xSteps),object)

    for i in range(ySteps):
        for j in range(xSteps):
            particleGraph[i][j] = (xBins[j],yBins[i],patriclesArray[i][j])
            
    return particleGraph,xBins,yBins,patriclesArray

def DiscretizeRobotPose(robotPose, xBins, yBins):
    # %needs robot position in x and y and the x and y bins. 
    # %returns the xy position the the disctritized bin and 
    xDiscretizedLoc,xIndex = find_nearest(xBins, robotPose[0])
    yDiscretizedLoc,yIndex = find_nearest(yBins, robotPose[1])
    robotPoseNew = np.zeros(2)
    robotPoseNew[0] = xDiscretizedLoc
    robotPoseNew[1] = yDiscretizedLoc
        
    return robotPoseNew,xIndex,yIndex

def FindBestGoal(particleGraph, patriclesArray, robotPose, xBins, yBins):
    # Returns bin location with max particles.
    # If there is a tie it returns the closest goal to the robot

    _,xIndexRobot,yIndexRobot = DiscretizeRobotPose(robotPose, xBins, yBins)

    # print(patriclesArray)
    result = np.where(patriclesArray == np.amax(patriclesArray))
    
    xMaxLocIndex =  result[0]
    yMaxLocIndex =  result[1]

    dMin = 1000000000
    xBestGoal = 0
    yBestGoal = 0
    xBestGoalIndex = 0
    yBestGoalIndex = 0
    if len(xMaxLocIndex) > 1:
        for i in range(len(xMaxLocIndex)):
            # print("X loc: ",particleGraph[xMaxLocIndex[i]][yMaxLocIndex[i]][0] ,"   Y loc: ", particleGraph[xMaxLocIndex[i]][yMaxLocIndex[i]][1])
            xGoal = particleGraph[xMaxLocIndex[i]][yMaxLocIndex[i]][0]
            yGoal = particleGraph[xMaxLocIndex[i]][yMaxLocIndex[i]][1]
            
            d = math.sqrt( pow((robotPose[0] - xGoal),2) + pow((robotPose[1] - yGoal),2) )
            if d < dMin: 
                dMin = d
                xBestGoal = xGoal
                yBestGoal = yGoal
                xBestGoalIndex = xMaxLocIndex[i]
                yBestGoalIndex = yMaxLocIndex[i]
    else:
        xBestGoal = particleGraph[xMaxLocIndex[0]][yMaxLocIndex[0]][0]
        yBestGoal = particleGraph[xMaxLocIndex[0]][yMaxLocIndex[0]][1]
        xBestGoalIndex = xMaxLocIndex
        yBestGoalIndex = yMaxLocIndex
        
    return xBestGoal,yBestGoal,xBestGoalIndex,yBestGoalIndex

def rasterScanGen(xRange, xNumSteps, yRange, yNumSteps):
    xArray = np.linspace(xRange[0], xRange[1], xNumSteps, endpoint=True)
    yArray = np.linspace(yRange[0], yRange[1], yNumSteps, endpoint=True)
    
    
    xScan = []
    yScan = []
    
    for i, yi in enumerate(yArray):
        xScan.append(xArray[::(-1)**i]) # reverse when i is odd
        yScan.append(np.ones_like(xArray) * yi)   
    
    # squeeze lists together to vectors
    xScan = np.concatenate(xScan)
    yScan = np.concatenate(yScan)
    
    desiredWaypoints = np.array([xScan, yScan]).T
    return desiredWaypoints
    
def geedyMovementsAction(xBestGoal, yBestGoal, robotPose, xBins, yBins):
    # Moves directly to the goal
    # If robot is on goal it will move right
    
    # xDiscretizedLoc, yDiscretizedLoc, _, _ = DiscretizeRobotPose(robotPose, xBins, yBins)
    
    xDiff = xBestGoal - robotPose[0]
    yDiff = yBestGoal - robotPose[1]
    
    
    angleRad = np.arctan2(yDiff,xDiff)
    # print(yDiff,xDiff,angleRad)
    if angleRad == 0 and yDiff == 0 and xDiff == 0:
        return "Do Nothing" 
    else:
        angle, angleIndex = find_nearest(_ACTIONS_2_ANGLES, angleRad)
        # print(angleRad, angle)
        return _ACTIONS_2[angleIndex]

def NextRobotPos(action, xyPoseRobot, particleGraph, xBins, yBins):
    xyPoseRobot,xIndexRobot,yIndexRobot = DiscretizeRobotPose(xyPoseRobot, xBins, yBins)
    
    # ['u','d','l','r','ne','nw','sw','se']
    yIndexRobotNew = yIndexRobot
    xIndexRobotNew = xIndexRobot
    
    if action == "u":
        yIndexRobotNew = yIndexRobot - 1

    elif action == "d":
        yIndexRobotNew = yIndexRobot + 1

    elif action == "r":
        xIndexRobotNew = xIndexRobot + 1    
    
    elif action == "l":
        xIndexRobotNew = xIndexRobot - 1  
        
    elif action == "ne":
        yIndexRobotNew = yIndexRobot - 1
        xIndexRobotNew = xIndexRobot + 1  

    elif action == "nw":
        yIndexRobotNew = yIndexRobot - 1
        xIndexRobotNew = xIndexRobot - 1  
    
    elif action == "sw":
        yIndexRobotNew = yIndexRobot + 1
        xIndexRobotNew = xIndexRobot - 1   
        
    elif action == "se":
        yIndexRobotNew = yIndexRobot + 1
        xIndexRobotNew = xIndexRobot + 1   
        
    else: # Do nothing
        yIndexRobotNew = yIndexRobot
        xIndexRobotNew = xIndexRobot  
        
    NextRobotPoseX = particleGraph[yIndexRobotNew][xIndexRobotNew][0]
    NextRobotPoseY = particleGraph[yIndexRobotNew][xIndexRobotNew][1]

    return (NextRobotPoseX, NextRobotPoseY)


# Video Generating function 
def generate_video(video_name = 'TestVid.avi'):     
    
    try:
        image_folder = os.getcwd() + '/VideoFig' # make sure to use your folder 
        os.chdir("VideoFig/") 
        images = [img for img in os.listdir(image_folder) if img.endswith(".jpg") or img.endswith(".jpeg") or img.endswith("png")] 
    except:
        image_folder = os.getcwd() 
        images = [img for img in os.listdir(image_folder) if img.endswith(".jpg") or img.endswith(".jpeg") or img.endswith("png")]     
    # print(os.getcwd())
    
    
    # print(images)
     
    images.sort(key=lambda test_string : list(map(int, re.findall(r'\d+', test_string)))[0])  
    # Array images should only consider 
    # the image files ignoring others if any 
  
    frame = cv2.imread(os.path.join(image_folder, images[0])) 
  
    # setting the frame width, height width 
    # the width, height of first image 
    height, width, layers = frame.shape   
  
    video = cv2.VideoWriter(video_name, 0, 7, (width, height))  
  
    # Appending the images to the video one by one 
    for image in images:  
        video.write(cv2.imread(os.path.join(image_folder, image)))  
      
    # Deallocating memories taken for window creation 
    cv2.destroyAllWindows()  
    video.release()  # releasing the video generated 
    # filelist = [ f for f in os.listdir(image_folder) if f.endswith(".png") ]
    # for f in filelist:
    #     os.remove(os.path.join(image_folder, f))
    
    print("Created video")



# generate_video()
'''
from scipy.io import loadmat
Bugs_X_Locations = loadmat('Bugs_X_Locations.mat')
Bugs_Y_Locations = loadmat('Bugs_Y_Locations.mat')
Bugs_X_Locations = Bugs_X_Locations['Bugs_X_Locations']
Bugs_Y_Locations = Bugs_Y_Locations['Bugs_Y_Locations']
Bug_Locations = np.array([Bugs_X_Locations, Bugs_Y_Locations])



#*************** Main ***************

plt.close('all')

plots = 1

particles = 10000
mu, sigma = 0, 0.25 # mean and standard deviation


xMinMap = 0
yMinMap = 0
xMaxMap = 100
yMaxMap = 100






# x = Bugs_X_Locations#np.random.uniform(xMinMap,xMaxMap,particles)#
# y = Bugs_Y_Locations#np.random.uniform(yMinMap,yMaxMap,particles)#

for i in range(1):

    x = np.random.uniform(xMinMap,xMaxMap,particles)
    y = np.random.uniform(yMinMap,yMaxMap,particles)
    # y = 5*np.random.normal(0,1,particles)+50
    
    x = np.ndarray.flatten(x)
    y = np.ndarray.flatten(y)
    
    xyPoseRobotNoneDicritized = np.random.uniform(0,100,2)
    
    rowSteps = 3;
    colSteps = 3;
    
    xRange = [xMinMap, xMaxMap]
    yRange = [yMinMap, yMaxMap]
    
    particleGraph,xBins,yBins,patriclesArray = DiscretizeMap(x, y, colSteps, rowSteps, xRange, yRange)
    
    xyPoseRobot,xIndexRobot,yIndexRobot = DiscretizeRobotPose(xyPoseRobotNoneDicritized, xBins, yBins)
    
    xBestGoal, yBestGoal, xBestGoalIndex, yBestGoalIndex = FindBestGoal(particleGraph, patriclesArray, xyPoseRobot, xBins, yBins)
    
    action = geedyMovementsAction(xBestGoal, yBestGoal, xyPoseRobot, xBins, yBins)
    
    
    # print(xIndexRobot,yIndexRobot)
    # print(action)
    
    nextRobotPose = NextRobotPos(action, xyPoseRobot, particleGraph, xBins, yBins)
    
    # print(nextPose[0], nextPose[1])
    # xSteps = 10
    # ySteps = 10
    
    # xRangeRaster = [min(xBins), max(xBins)]
    # yRangeRaster = [min(yBins), max(yBins)]
    # rasterWaypoints = rasterScanGen(xRangeRaster, xSteps, yRangeRaster, ySteps)
    
    
    #print(patriclesArray)
    # print(xBestGoal,yBestGoal,xBestGoalIndex,yBestGoalIndex)
    
    
    if plots:
        # plt.figure()

        csfont = {'fontname':'Times New Roman'}
        fig = plt.figure()
        # fig.set_size_inches(50, 10)
        
        ax1 = fig.add_subplot(1, 2, 1)
        ax2 = fig.add_subplot(1, 2, 2)
        # ax1.set_aspect(1)
        # ax2.set_aspect(1)
        
        ax1.set_xlabel('x (m)',**csfont)
        ax1.set_ylabel('y (m)',**csfont)
        ax1.set_xlim(xMinMap, xMaxMap)
        ax1.set_ylim(yMinMap, yMaxMap)
        ax1.grid(True)
        
        ax2.set_xlabel('x (m)',**csfont)
        ax2.set_ylabel('y (m)',**csfont)
        ax2.set_xlim(xMinMap, xMaxMap)
        ax2.set_ylim(yMinMap, yMaxMap)
        
        fig.set_size_inches(26.5, 10)
        plot1_1 = ax1.plot(x,y,'k.')
        # plt.plot(rasterWaypoints[:,0],rasterWaypoints[:,1],'r-')
        # plt.plot(rasterWaypoints[:,0],rasterWaypoints[:,1],'r.')
        
        
        # plt.figure()
        xEdgesTemp = np.linspace(xMinMap, xMaxMap, num=colSteps+1)
        yEdgesTemp = np.linspace(yMinMap, yMaxMap, num=rowSteps+1)
    
        patriclesArray, xedgesPlot, yedgesPlot = np.histogram2d(y, x, bins=(yEdgesTemp, xEdgesTemp))
        plot2_1 = ax2.imshow(patriclesArray, interpolation='nearest', origin='low', extent=[yedgesPlot[0], yedgesPlot[-1],xedgesPlot[0], xedgesPlot[-1]])
        
        plot2_2 = ax2.plot(xyPoseRobotNoneDicritized[0],xyPoseRobotNoneDicritized[1],'ro')
        plot2_3 = ax2.plot(xyPoseRobot[0],xyPoseRobot[1],'bo')
        plot2_4 = ax2.plot(xBestGoal,yBestGoal,'rx')
        plot2_3 = ax2.plot(nextRobotPose[0],nextRobotPose[1],'bx')
        
        
        plot2_1.set_clim(0,patriclesArray.max())
        plt.colorbar(plot2_1)
        
        
        # plt.plot(rasterWaypoints[:,0],rasterWaypoints[:,1],'r-')
        # plt.plot(rasterWaypoints[:,0],rasterWaypoints[:,1],'r.')
        figureName = "VideoFig/figure_" + str(i) + ".jpg"
        plt.savefig(figureName)
        fig.clf()
        
plt.close('all')

# Calling the generate_video function 
generate_video() 
'''