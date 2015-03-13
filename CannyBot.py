# Filename: CannyBot.py
# Authors: Tommy Lin, TJ Maynes

import os, sys, math, motion, almath
from PIL import ImageChops
import Image
from numpy import *
import cv2.cv as cv
from naoqi import ALProxy

"""

helper functions and values

"""

rows = 4
columns = 4
ELBOW_OFFSET_Y = 15
UPPER_ARM_LENGTH = 105
SHOULDER_OFFSET_Y = 98
SHOULDER_OFFSET_Z = 100
LOWER_ARM_LENGTH = 55.95
ip = "169.254.226.148"
port = 9559
eyes = None
video_proxy = None
video_client = None
pilcolorspace = "RGB"
resolution = 2
colorSpace = 11
fps = 30

def pretty_print(name, matrix):
    print "\nThis is matrix = " + name
    for row in matrix:
        print row

"""

connect to NAO Robot

"""

try:
    motionProxy = ALProxy("ALMotion", ip, port)
except Exception, e:
    print "Could not create proxy to ALMotion"
    print "Error was: ", e
try:
    postureProxy = ALProxy("ALRobotPosture", ip, port)
except Exception, e:
    print "Could not create proxy to ALRobotPosture"
    print "Error was: ", e

"""

main functions
    
"""
    
def stiffness_on(proxy):
    #We use the "Body" name to signify the collection of all joints
    pNames = "Body"
    pStiffnessLists = 1.0
    pTimeLists = 1.0
    proxy.stiffnessInterpolation(pNames, pStiffnessLists, pTimeLists)
    
def robo_vision():
    while True:
        # set up data structure for shape
        shape_name = ""
        start_pos = 0.0
        way_points = []
        shape = [shape_name, start_pos, way_points]
        
        video_proxy = ALProxy("ALVideoDevice", ip, port)
        video_client = video_proxy.subscribe("eyes", resolution, colorSpace, fps)

        # Get a camera image.
        # image[6] contains the image data passed as an array of ASCII chars.
        naoImage = camProxy.getImageRemote(video_client)

        # Time the image transfer.
        print "acquisition delay ", t1 - t0
        
        camProxy.unsubscribe(video_client)

        # Now we work with the image returned and run

        # Get the image size and pixel array.
        imageWidth = naoImage[0]
        imageHeight = naoImage[1]
        frame = naoImage[6]

        # display image from Robot Camera
        cv.ShowImage('RoboVision', frame)

        # Convert to greyscale
        grey = cv.CreateImage(cv.GetSize(frame), frame.depth, 1)
        cv.CvtColor(frame, grey, cv.CV_RGB2GRAY)

        # Gaussian blur to remove noise
        blur = cv.CreateImage(cv.GetSize(grey), cv.IPL_DEPTH_8U, grey.channels)
        cv.Smooth(grey, blur, cv.CV_GAUSSIAN, 5, 5)
            
        # And do Canny edge detection
        canny = cv.CreateImage(cv.GetSize(blur), blur.depth, blur.channels)
        cv.Canny(blur, canny, 10, 100, 3)
        cv.ShowImage('RoboVision', canny)

        contours,h = cv.findContours(canny,1,2)
            
        # only return value when you find a circle or square
        for cnt in contours:
            approx = cv.approxPolyDP(cnt,0.01*cv2.arcLength(cnt,True),True)
            print len(approx)
            if len(approx)==5:
                return ["pentagon", 0.0, [0.0,0.1,1.0]]
                #cv.drawContours(img,[cnt],0,255,-1)
            elif len(approx)==3:
                return ["triangle", 0.0, [0.0,0.1,1.0]]
                #cv.drawContours(img,[cnt],0,(0,255,0),-1)
            elif len(approx)==4:
                return ["square", 0.0, [0.0,0.1,1.0]]
                #cv.drawContours(img,[cnt],0,(0,0,255),-1)
            elif len(approx) == 9:
                return ["half-circle", 0.0, [0.0,0.1,1.0]]
                #cv.drawContours(img,[cnt],0,(255,255,0),-1)
            elif len(approx) > 15:
                return ["circle", 0.0, [0.0,0.1,1.0]]
                #cv.drawContours(img,[cnt],0,(0,255,255),-1)

        c = cv.WaitKey(50)
        if c == 27:
            exit(0)
                
def robo_motion(shape):
    print "\nNAO Robot will draw this shape: " + shape + "."
    # Set NAO in Stiffness On
    stiffness_on(motionProxy)

    # Send NAO to Pose Init
    postureProxy.goToPosture("StandInit", 0.5)

    effector   = "RArm"
    space      = 2 #motion.FRAME_ROBOT
    axisMask   = almath.AXIS_MASK_VEL    # just control position
    isAbsolute = False

    # Since we are in relative, the current position is zero
    currentPos = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    
    # Define the changes relative to the current position
    dx         =  0.06      # translation axis X (meters)
    dy         =  0.03      # translation axis Y (meters)
    dz         =  0.00      # translation axis Z (meters)
    dwx        =  0.00      # rotation axis X (radians)
    dwy        =  0.00      # rotation axis Y (radians)
    dwz        =  0.00      # rotation axis Z (radians)
    targetPos  = [dx, dy, dz, dwx, dwy, dwz]

    # Go to the target and back again
    path       = [targetPos, currentPos]
    times      = [2.0, 10.0] # seconds

    motionProxy.positionInterpolation(effector, space, path,
                                      axisMask, times, isAbsolute)

def transformation_matrix(name_of_matrix, matrix, rows, columns, a, alpha, distance, theta):
    temp = 0.0
    for i in range(rows):
        for j in range(columns):
            if i == 0 and j == 0:
                temp = math.cos(theta*math.pi/180.0)
                if temp == -0:
                    temp = 0.0
                matrix[i][j] = round(temp)
            if i == 0 and j == 1:
                temp = (-(math.sin(theta*math.pi/180.0))*math.cos(alpha*math.pi/180.0))
                if temp == -0:
                    temp = 0.0
                matrix[i][j] = round(temp)
            if i == 0 and j == 2:
                temp = (math.sin(theta*math.pi/ 180.0) * math.sin(alpha*math.pi/ 180.0))
                if temp == -0:
                    temp = 0.0
                matrix[i][j] = round(temp)
            if i == 0 and j == 3:
                temp = (a * math.cos(theta*math.pi/180.0))
                if temp == -0:
                    temp = 0.0
                matrix[i][j] = round(temp)
            if i == 1 and j == 0:
                temp = math.sin(theta*math.pi/180.0)
                if temp == -0:
                    temp = 0.0
                matrix[i][j] = round(temp)
            if i == 1 and j == 1:
                temp = (math.cos(theta*math.pi/180.0)*math.cos(alpha*math.pi/180.0))
                if temp == -0:
                    temp = 0.0
                matrix[i][j] = round(temp)
            if i == 1 and j == 2:
                temp = (-(math.cos(theta*math.pi/180.0))*math.sin(alpha*math.pi/180.0))
                if temp == -0:
                    temp = 0.0
                matrix[i][j] = round(temp)
            if i == 1 and j == 3:
                temp = a*math.sin(theta*math.pi/180.0)
                if temp == -0:
                    temp = 0.0
                matrix[i][j] = round(temp)
            if i == 2 and j == 0:
                matrix[i][j] = 0.0
            if i == 2 and j == 1:
                temp = math.sin(alpha * math.pi/180.0)
                if temp == -0:
                    temp = 0.0
                matrix[i][j] = round(temp)
            if i == 2 and j == 2:
                temp = math.cos(alpha * math.pi/180.0)
                if temp == -0:
                    temp = 0.0
                matrix[i][j] = round(temp)
            if i == 2 and j == 3:
                temp = distance
                if temp == -0:
                    temp = 0.0
                matrix[i][j] = round(temp)
            if i == 3 and j == 0:
                matrix[i][j] = 0.0
            if i == 3 and j == 1:
                matrix[i][j] = 0.0
            if i == 3 and j == 2:
                matrix[i][j] = 0.0
            if i == 3 and j == 3:
                matrix[i][j] = 1.0
    return matrix

def  multiply_matrices(RShoulderPitch, RShoulderRoll, RElbowYaw, RElbowRoll, RWristRoll):
    # initialize temp matrices
    m0 = [[0 for x in range(4)] for x in range(4)]
    m1 = [[0 for x in range(4)] for x in range(4)]
    m2 = [[0 for x in range(4)] for x in range(4)]
    m3 = [[0 for x in range(4)] for x in range(4)]

    for i in range(rows):
        for j in range(columns):
            for inner in range(4):
                m0[i][j] = round(m0[i][j]+RShoulderPitch[i][inner]*RShoulderRoll[inner][j])

    for i in range(rows):
        for j in range(columns):
            for inner in range(4):
                m1[i][j] = round(m1[i][j]+m0[i][inner]*RElbowYaw[inner][j])

    for i in range(rows):
        for j in range(columns):
            for inner in range(4):
                m2[i][j] = round(m2[i][j]+m1[i][inner]*RElbowRoll[inner][j])

    for i in range(rows):
        for j in range(columns):
            for inner in range(4):
                m3[i][j] = round(m3[i][j]+m2[i][inner]*RWristRoll[inner][j])

    return m3

if __name__ == '__main__':
    print("\nWelcome to the CannyBot Program!\n")
    shape = robo_vision()
    print("The shape found on the workspace was a %d", shape[0])
    
    # initialize matrices
    RShoulderPitch = [[0 for x in range(4)] for x in range(4)]
    RShoulderRoll = [[0 for x in range(4)] for x in range(4)]
    RElbowYaw = [[0 for x in range(4)] for x in range(4)]
    RElbowRoll = [[0 for x in range(4)] for x in range(4)]
    RWristRoll = [[0 for x in range(4)] for x in range(4)]
    base_to_start = [[0 for x in range(4)] for x in range(4)]

    # process file
    print("Processing files in input.txt.")
    f = open('input.txt', 'r')
    theta0 = f.readline()
    theta1 = f.readline()
    theta2 = f.readline()
    theta3 = f.readline()
    theta4 = f.readline()
    f.close()
    print("Finished processing file.")
    print "Thetas are %d %d %d %d %d" % (float(theta0), float(theta1), float(theta2), float(theta3), float(theta4))

    # transformation matrices
    RShoulderPitch = transformation_matrix("RShoulderPitch",RShoulderPitch,rows,columns,0,-(math.pi/2.0), 0, float(theta0))
    pretty_print("RShoulderPitch", RShoulderPitch)
    RShoulderRoll = transformation_matrix("RShoulderRoll",RShoulderRoll,rows,columns,0,math.pi/2.0, 0, float(theta1) + (math.pi/2.0))
    pretty_print("RShoulderRoll", RShoulderRoll)
    RElbowYaw = transformation_matrix("RElbowYaw",RElbowYaw,rows,columns,-ELBOW_OFFSET_Y, (math.pi / 2.0), UPPER_ARM_LENGTH, float(theta2))
    pretty_print("RElbowYaw",RElbowYaw)
    RElbowRoll = transformation_matrix("RElbowRoll",RElbowRoll,rows,columns,0, -(math.pi / 2.0), 0, float(theta3))
    pretty_print("RElbowRoll",RElbowRoll)
    RWristRoll = transformation_matrix("RWristRoll",RWristRoll,rows,columns,LOWER_ARM_LENGTH, (math.pi/ 2.0), 0, float(theta4))
    pretty_print("RWristRoll",RWristRoll)
    base_to_start = multiply_matrices(RShoulderPitch,RShoulderRoll,RElbowYaw,RElbowRoll,RWristRoll)
    pretty_print("base_to_start", base_to_start)

    # make movement
    robo_motion(shape)

    # end of line
    print("End of Program.")
