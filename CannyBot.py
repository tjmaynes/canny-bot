# Filename: CannyBot.py
# Authors: Tommy Lin, TJ Maynes

import os, sys, math, motion, almath
from StringIO import StringIO
from PIL import Image
import cv2
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
pil_color_space = "RGB"
resolution = 2   #VGA
color_space = 11  #RGB
fps = 30

def pretty_print(name, matrix):
    print "\nThis is matrix = " + name
    for row in matrix:
        print row
"""
Connect to NAO Robot on startup

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
try:
    voice = ALProxy("ALTextToSpeech", ip, port)
except Exception, e:
    print "Could not create proxy to ALTextToSpeech"
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
  # leave area commented out for debugging purposes (uncomment for final demo).

  # First get an image from Nao, then show it on the screen with PIL.
  video_proxy = ALProxy("ALVideoDevice", ip, port)

  video_client = video_proxy.subscribe("NAO_CAM", resolution, color_space, fps)

  # Get a camera image.
  # image[6] contains the image data passed as an array of ASCII chars.
  naoImage = video_proxy.getImageRemote(video_client)
  
  # disconect from video_proxy
  video_proxy.unsubscribe(video_client)

  # Get the image size and pixel array.
  imageWidth = naoImage[0]
  imageHeight = naoImage[1]
  buffer = naoImage[6]

  # Create a PIL Image from our pixel array.
  im = Image.fromstring(pil_color_space, (imageWidth, imageHeight), buffer)

  # Save the image.
  im.save("noognagnook.png")

  # use opencv to read from image
  frame = cv2.imread("noognagnook.png")

  # Convert to greyscale
  gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
  
  # Gaussian blur to remove noise
  blur = cv2.GaussianBlur(gray, (3,3), 0)
  
  # And do Canny edge detection
  canny = cv2.Canny(blur, 10, 100)
  
  # debugging -- write canny to file
  cv2.imwrite("NAOVISION.png", canny)

  # debugging -- what does NAO see
  cv2.imshow("canny", canny)
  
  # press 0 to get out of image view
  cv2.waitKey(0)

  # contour detection
  contours,h = cv2.findContours(canny,1,2)
            
  # only return value when you find a circle or square
  for cnt in contours:
      approx = cv2.approxPolyDP(cnt,0.01*cv2.arcLength(cnt,True),True)
      if len(approx)==2:
          robo_motion("line")
          break
      if len(approx)==5:
          robo_motion("pentagon")
          break
          #return ["pentagon", 0.0, [0.0,0.1,1.0]]
          #cv.drawContours(img,[cnt],0,255,-1)
      elif len(approx)==3:
          robo_motion("triangle")
          break
      elif len(approx)==4:
          robo_motion("square")
          break
      elif len(approx) == 9:
          robo_motion("half-circle")
          break
      elif len(approx) > 15:
          robo_motion("circle")
          break

  c = cv2.waitKey(50)
  if c == 27:
      exit(0)
      
def robo_motion(shape):
    voice.say("I will draw a " + shape);

    print "\nNAO Robot will draw this shape: " + shape + "."

    """
    stiffness_on(motionProxy)

    shape_name = ""
    start_pos = 0.0
    way_points = []
    shape = [shape_name, start_pos, way_points]

    # Send NAO to Pose Init
    postureProxy.goToPosture("StandInit", 0.5)

    effector   = "RArm"
    space      = motion.FRAME_TORSO
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
    """
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

# for debugging purposes
def test_transformation_matrices():
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
    print "(Before check): Thetas are %d, %d, %d, %d, %d" % (float(theta0), float(theta1), float(theta2), float(theta3), float(theta4))

    # if else conditions bounded by certain degrees Prevent overheating
    if float(theta0) > 0 and float(theta0) >= 119.5:
        print "\nthis theta ", theta0, " is wrong."
        theta0 = 110
        print "\nthis theta ", theta0, " is now correct."
    elif float(theta0) < 0.0 and float(theta0) < -119.5:
            print "\nthis theta ", theta0, " is wrong."
            theta0 = -110
            print "\nthis theta ", theta0, " is now correct."
    if float(theta1) > 0.0 and float(theta1) > 18.0:
            print "\nthis theta ", theta1, " is wrong."
            theta1 = 15
            print "\nthis theta ", theta1, " is now correct."
    elif float(theta1) < 0.0 and float(theta1) < -76:
            print "\nthis theta ", theta1, " is wrong."
            theta1 = -70
            print "\nthis theta ", theta1, " is now correct."
    if float(theta2) > 0.0 and float(theta2) > 119.5:
            print "\nthis theta ", theta2, " is wrong."
            theta2 = 110
            print "\nthis theta ", theta2, " is now correct."
    elif float(theta2) < 0.0 and float(theta2) < -119.5:
            print "\nthis theta ", theta2, " is wrong."
            theta2 = -110
            print "\nthis theta ", theta2, " is now correct."
    if float(theta3) > 2 and float(theta3) > 88.5:
            print "\nthis theta ", theta3, " is wrong."
            theta3 = 80
            print "\nthis theta ", theta3, " is now correct."
    elif float(theta3) < 2:
        print "\nthis theta ", theta3, " is wrong."
        theta3 = 5
        print "\nthis theta ", theta3, " is now correct."
    if float(theta4) > 0 and float(theta4) > 104.5:
            print "\nthis theta ", theta4, " is wrong."
            theta4 = 100
            print "\nthis theta ", theta4, " is now correct."
    elif float(theta4) < 0 and float(theta4) < -104.5:
            print "\nthis theta ", theta4, " is wrong."
            theta4 = -100
            print "\nthis theta ", theta4, " is now correct."
    
    print "\n(After check): Thetas are %d, %d, %d, %d, %d" % (float(theta0), float(theta1), float(theta2), float(theta3), float(theta4))

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

if __name__ == '__main__':
    print("\nWelcome to the CannyBot Program!\n")

    # have nao look at shapes!
    robo_vision()

    """
    while (true):
      print "Would you like to run this program again?"
      # input
      if input == "n" or input == "no":
         break;
    2  else:
         robo_vision()
    """

    # debugging
    test_transformation_matrices()

    # end of line
    print("End of Program.")
