#!/usr/bin/env python

from trail import *
from pathway import *

# A few parameters
VERBOSE = False
OSC = True
SHOW_CV = True

MAX_PATHWAY_HISTORY = 300 # Maximum history length for the Pathway objects
MAX_TRAIL_HISTORY = 50 # Maximum history length for the Trail object
POINT_LIFETIME = 1e6 # Maximum lifetime (in seconds) of a new position
LINE_DETECTION_LEVEL = 64 # Maximum indice of confidence for a line to be detected by Trail
CIRCLE_DETECTION_LEVEL = 8192 # Maximum indice of confidence for a circle to be detected by Trail_Circle
CIRCLE_MAX_RADIUS = 256 # Maximum radius of the circles detected by Trail_Circle (with no limit, it will always find a circle)

# Input resolution (from camera)
IMAGE_SIZE = [640, 480]

# Projection parameters
PROJECTION_IN = array([[0, 0], [640, 0], [640, 480], [0, 480]], float32)
PROJECTION_OUT = array([[0, 0], [640, 0], [640, 480], [0, 480]], float32)

#*************#
def bigBrother_callback(path, args, types, src, user_data):
    pathway_callback(path, args, types, src, user_data["pathway"])
    trail_callback(path, args, types, src, user_data["trail"])

#*************#
def mainLoop(maxPathwayHistory = MAX_PATHWAY_HISTORY, maxTrailHistory = MAX_TRAIL_HISTORY, pointLifetime = POINT_LIFETIME,
             lineDetectionLevel = LINE_DETECTION_LEVEL, circleDetectionLevel = CIRCLE_DETECTION_LEVEL, circleMaxRadius = CIRCLE_MAX_RADIUS):
    try:
        oscServer = liblo.Server(9000);
    except liblo.AddressError, err:
        print(str(err))
        sys.exit()

    try:
        oscClient = liblo.Address(9100)
    except liblo.AddressError, err:
        print(str(err))
        sys.exit()

    # Set the pathways list
    pathmaps = []
    pathmaps.append(loadImage("assets/path.png"))

    pathways = {}
    trails = {}

    user_data = {}
    user_data["pathway"] = [pathways, pathmaps, maxPathwayHistory, pointLifetime]
    user_data["trail"] = [trails, maxTrailHistory, pointLifetime, lineDetectionLevel, circleDetectionLevel, circleMaxRadius]
    # The positions of the blobs are updated through this callback
    oscServer.add_method("/blobserver/bgsubtractor", "iiiffiii", bigBrother_callback, user_data)

    while True:
        if VERBOSE:
            print("-----------------------")
        oscServer.recv(33)

        #-----#
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
                if OSC:
                    # OSC message: blobID, pathway number, completion, error
                    liblo.send(oscClient, "/bigBrother/pathway", "iiff", i, j, completion, error)

        # Cleaning of inactive blobs is done afterwards
        for i in cleanLog:
            pathways.pop(i)
        
        #-----#
        cleanLog = []
        for i in trails:
            if trails[i][0].isAlive() == False:
                cleanLog.append(i)
                continue

            # Line trails are updated first
            sol, res = trails[i][0].track()
            eq = trails[i][0].identify().T
            if VERBOSE:
                print(eq, res)
            if OSC and len(eq) == 1:
                # OSC message: blobID, slope, delta at x=0
                liblo.send(oscClient, "/bigBrother/trail", "iff", i, eq[0][0], eq[0][1])

            # Then, circle trails
            sol, res = trails[i][1].track()
            eq = trails[i][1].identify().T
            if VERBOSE:
                print(eq, res)
            if OSC and len(eq) == 1:
                # OSC message: blobID, center_x, center_y, radius, completeness
                liblo.send(oscClient, "/bigBrother/trail_circle", "iffff", i, eq[0][0], eq[0][1], eq[0][2], eq[0][3])

        for i in cleanLog:
            trails.pop(i)

        if SHOW_CV:
            drawTrails(trails)

        if SHOW_CV:
            key = cv.waitKey(5)
            if key == 1048603:
                break;

#*************#
if __name__ == "__main__":
    mainLoop()
