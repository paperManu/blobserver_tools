#!/usr/bin/env python

import liblo
import sys
import cv2 as cv
from copy import deepcopy
from time import time, sleep
from numpy import *

VERBOSE = False
# Set this to True to see OpenCV buffers. Useful for debugging.
SHOW_CV = True

# Input resolution (from camera)
IMAGE_SIZE = [640, 480]

# Projection parameters
PROJECTION_IN = array([[0, 0], [640, 0], [640, 480], [0, 480]], float32)
PROJECTION_OUT = array([[0, 0], [640, 0], [640, 480], [0, 480]], float32)

#*************#
class TimedPoint(object):
    def __init__(self, point):
        self.point = point
        self.time = time()

#*************#
# Contains the detected point, and its projection on the path
class ProjectedPoint(object):
    def __init__(self, point, projection, distance, time):
        self.point = point
        self.projection = projection
        self.distance = distance
        self.time = time

#*************#
# The class which compares an object's path to a pathway
class Pathway(object):
    # Constructor of the class
    def __init__(self, maxHistoryLength, maxTime, args = []):
        self._history = []
        self._maxLength = maxHistoryLength
        self._maxTime = maxTime
        self._args = args
        self._maxLifetime = 30
        self._minStep = 4

        self._updated = False
        self._lifetime = 30

        self._maxDistance = 32
        self._margin = 8

        self._traveled = 0.0
        self._error = 1e100

        self._path = zeros((512, 512))

        self._mask = self.__createDistanceMask(self._maxDistance)
        self._negMask = ones((self._maxDistance * 2 + 1, self._maxDistance * 2 + 1)) * 255

        self._projectionMat = array([])

    # This creates a mask where each pixel contains its own distance to the center
    def __createDistanceMask(self, size):
        mask = zeros((self._maxDistance * 2 + 1, self._maxDistance * 2 + 1))

        for i in range(-self._maxDistance, self._maxDistance + 1):
            for j in range(-self._maxDistance, self._maxDistance + 1):
                distance = sqrt(i*i + j*j)
                mask[i + self._maxDistance][j + self._maxDistance] = distance

        return mask

    def updateProjection(self, inPoints, outPoints):
        self._projectionMat = cv.getPerspectiveTransform(inPoints, outPoints)

    # Checks is this instance of the class is still in use or not
    def isAlive(self):
        if self._lifetime > 0:
            return True
        else:
            return False

    # Adds a new position of the object to the path, computes its projection on the pathway (if close to it),
    # and updates the history to get rid of older positions
    def follow(self, point):
        pos = deepcopy(point.point)
        if len(self._projectionMat) > 0:
            projPoint = array([[pos]], float32)
            projPoint = cv.perspectiveTransform(projPoint, self._projectionMat)
            pos = array(projPoint[0][0], integer)

        minDist = self._margin + self._maxDistance
        if pos[0] < minDist or pos[0] > self._path.shape[0] - minDist or pos[1] < minDist or pos[1] > self._path.shape[0] - minDist:
            return

        ixgrid = ix_(range(pos[1] - self._maxDistance, pos[1] + self._maxDistance + 1),
                     range(pos[0] - self._maxDistance, pos[0] + self._maxDistance + 1))
        local = self._path[ixgrid]
        outMat = cv.multiply(local, self._mask)
        outMat = outMat + cv.multiply(1 - local, self._negMask)

        distance = amin(outMat)
        index = argmin(outMat)
        projection = [mod(index, self._maxDistance * 2 + 1), floor(index / (self._maxDistance * 2 + 1))]
        projection[0] = int(projection[0] - self._maxDistance + pos[0])
        projection[1] = int(projection[1] - self._maxDistance + pos[1])
        
        if len(self._history) > 0:
            lastProjection = self._history[len(self._history)-1].projection
            dist = sqrt(pow(projection[0] - lastProjection[0], 2.0) + pow(projection[1] - lastProjection[1], 2.0))
            if dist < self._minStep:
                self._updated = True
                return

        pPoint = ProjectedPoint(pos, projection, distance, point.time)
        self._history.append(pPoint)

        while len(self._history) > self._maxLength:
            self._history.remove(self._history[0])

        while True:
            currentTime = self._history[len(self._history) - 1].time
            if time() - self._history[0].time > self._maxTime:
                self._history.remove(self._history[0])
            else:
                break

        self._updated = True

    # Sets the grayscale image which describes the pathway
    def setPath(self, path):
        newPath = zeros((path.shape[0], path.shape[1]))
        for i in range(path.shape[0]):
            for j in range(path.shape[1]):
                if path[i][j] != 0:
                    newPath[i][j] = 1
        self._path = newPath

    # Computes the completion of a pathway according to the history of positions, as well as
    # the squared sum of the error of these positions (which gives an indication of how well
    # the path has been followed)
    def travel(self):
        if self._updated == False:
            self._lifetime -= 1
            return self._traveled, self._error
        self._lifetime = self._maxLifetime

        pathMask = zeros(self._path.shape)

        for i in arange(len(self._history) - 1):
            pos1 = (self._history[i].projection[0], self._history[i].projection[1])
            pos2 = (self._history[i+1].projection[0], self._history[i+1].projection[1])
            pos1 = (min(pos1[0], pos2[0]) - self._margin, min(pos1[1], pos2[1]) - self._margin)
            pos2 = (max(pos1[0], pos2[0]) + self._margin, max(pos1[1], pos2[1]) + self._margin)
            cv.rectangle(pathMask, pos1, pos2, (1), -1)

        path = cv.multiply(pathMask, self._path)
        totalPath = sum(self._path)
        travelPath = sum(path)
        pathTraveled = travelPath / totalPath

        if SHOW_CV:
            cv.imshow("pathMask", pathMask)
            cv.imshow("traveled", path)

        sqSum = 0
        for i in arange(len(self._history)):
            sqSum += pow(self._history[i].distance, 2.0)

        self._traveled = pathTraveled
        self._error = sqrt(sqSum / len(self._history))

        self._updated = False
        return self._traveled, self._error

#*************#
# Callback used by liblo, when a new adequate OSC message is caught
def pathway_callback(path, args, types, src, user_data):
    blobPos = array([args[1], args[2]])
    blobId = args[0]

    pathways = user_data[0]
    pathmaps = user_data[1]
    maxHistory = user_data[2]
    pointLifetime = user_data[3]

    # If this blobId is new, we create has many new pathway objects as there are pathmaps
    if pathways.has_key(blobId) == False:
        index = 0
        pathways[blobId] = []
        for path in pathmaps:
            pathways[blobId].append(Pathway(maxHistory, pointLifetime))
            pathways[blobId][index].setPath(path)
            pathways[blobId][index].updateProjection(PROJECTION_IN, PROJECTION_OUT)
            index += 1

    tPoint = TimedPoint(blobPos)
    # The new blob position is added to all its related pathways
    for index in range(len(pathmaps)):
        pathways[blobId][index].follow(tPoint)

#*************#
def loadImage(path):
    img = cv.imread(path, cv.CV_LOAD_IMAGE_GRAYSCALE)
    if SHOW_CV:
        cv.imshow("path", img)
    return img

#*************#
def mainLoop(maxHistory = 300, pointLifetime = 1e6):
    try:
        oscServer = liblo.Server(9000);
    except liblo.AddressError, err:
        print(str(err))
        sys.exit()

    # The list containing all possible pathways.
    # Here, only one is loaded
    pathmaps = []
    pathmaps.append(loadImage("assets/path.png"))

    # This dict contains one list of Pathway (the class) per blob ID
    pathways = {}
    user_data = [pathways, pathmaps, maxHistory, pointLifetime]

    # The positions of the blobs are updated through this callback
    oscServer.add_method("/blobserver/bgsubtractor", "iiiffiii", pathway_callback, user_data)

    while True:
        if VERBOSE:
            print("-----------------------")
        oscServer.recv(33)

        # cleanLog contains the list of the blob ID which are not active anymore
        cleanLog = []
        for i in pathways:
            if pathways[i][0].isAlive() == False:
                cleanLog.append(i)
                continue

            for j in range(len(pathways[i])):
                # Update of the completion of the pathways, for each blob and each pathmap
                completion, error = pathways[i][j].travel()
                if VERBOSE:
                   print(i, j, completion, error)

        # Cleaning of inactive blobs is done afterwards
        for i in cleanLog:
            pathways.pop(i)

        if SHOW_CV:
            cv.waitKey(5)

#*************#
def usage():
    print("Usage: pathway.py [maxHistory [pointLifetime]]")

#*************#
if __name__ == "__main__":
    if len(sys.argv) > 1 and (sys.argv[1] == "-h" or sys.argv[1] == "--help"):
        print("Pathway, a small script which compares objects movement with predefined pathways")
        usage()
        sys.exit()

    maxHistory = 300
    pointLifetime = 1e6

    try:
        maxHistory = sys.argv[1]
        pointLifetime = sys.argv[2]
    except:
        usage()

    mainLoop(maxHistory, pointLifetime)
