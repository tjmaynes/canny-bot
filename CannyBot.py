"""
@file: CannyBot.py
@authors: Tommy Lin, TJ Maynes
@subject: getting the NAO Robot to draw the shapes it "sees" using image processing
"""

import os, sys, math, motion, almath, time
from StringIO import StringIO
from PIL import Image
import cv2
from naoqi import ALProxy

"""

global variables

"""
# nao right arm defaults
ELBOW_OFFSET_Y = 15
UPPER_ARM_LENGTH = 105
LOWER_ARM_LENGTH = 55.95
SHOULDER_OFFSET_Y = 98
SHOULDER_OFFSET_Z = 100

# nao settings
ip = "169.254.226.148"
port = 9559
eyes = None
video_proxy = None
video_client = None
pil_color_space = "RGB"
resolution = 2   #VGA
color_space = 11  #RGB
fps = 30

"""

helper functions

"""
def pretty_print(name, matrix):
  print "\nThis is matrix = " + name
  for row in matrix:
    print row
def stiffness_on(proxy):
  #We use the "Body" name to signify the collection of all joints
  pNames = "Body"
  pStiffnessLists = 0.0
  pTimeLists = 1.0
  proxy.stiffnessInterpolation(pNames, pStiffnessLists, pTimeLists)
def stiffness_off(proxy):
  #We use the "Body" name to signify the collection of all joints
  pNames = "Body"
  pStiffnessLists = 0.0
  pTimeLists = 1.0
  proxy.stiffnessInterpolation(pNames, pStiffnessLists, pTimeLists)

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
@function: get_joint_angles()
@description: manually get joint angles within NAO's workspace
@return: a list of joint angles =>  [rshoulderpitch, rshoulderroll, relbowyaw, rwristyaw, rhand]
"""
def get_joint_angles():
    # which coordinate location to record
    input_value = raw_input("\Which coordinate?")
    print input_value

    # posture
    postureProxy.goToPosture("StandInit", 0.5)

    #We use the "Body" name to signify the collection of all joints
    pNames = "RArm"
    pStiffnessLists = 0.0
    pTimeLists = 1.0
    motionProxy.stiffnessInterpolation(pNames, pStiffnessLists, pTimeLists)

    # Example that finds the difference between the command and sensed angles.
    names         = "RArm"
    useSensors    = False
    commandAngles = motionProxy.getAngles(names, useSensors)
    print "Command angles:"
    print str(commandAngles)
    print ""

    useSensors  = True
    sensorAngles = motionProxy.getAngles(names, useSensors)
    print "Sensor angles:"
    print str(sensorAngles)
    print ""

    errors = []
    for i in range(0, len(commandAngles)):
        errors.append(commandAngles[i]-sensorAngles[i])
    print "Errors"
    print errors

"""
@function: lookup_table
@description: create lookup table of size 5 by 5 with each coordinate
containing the joint angles (theta values) to get to that position in
NAO's workspace.
@return: a table of joint angles
"""
def lookup_table(start):#, canny, points):
  rows = 5
  columns = 5
  grid = [[0 for x in range(rows)] for x in range(columns)]

  # traverse through matrix and add theta values based on measurements of space in "invisible" grid
  for i in range(rows):
    for j in range(columns):
      if i == 0 and j == 0:
        grid[i][j] = start
      elif i == 0 and j == 1:
        grid[i][j] = [1,2,1,2,1]
      elif i == 0 and j == 2:
        grid[i][j] = [1,2,1,2,1]
      elif i == 0 and j == 3:
        grid[i][j] = [1,2,1,2,1]
      elif i == 0 and j == 4:
        grid[i][j] = [1,2,1,2,1]
      elif i == 1 and j == 0:
        grid[i][j] = [1,2,1,2,1]
      elif i == 1 and j == 1:
        grid[i][j] = [1,2,1,2,1]
      elif i == 1 and j == 2:
        grid[i][j] = [1,2,1,2,1]
      elif i == 1 and j == 3:
        grid[i][j] = [1,2,1,2,1]
      elif i == 1 and j == 4:
        grid[i][j] = [1,2,1,2,1]
      elif i == 2 and j == 0:
        grid[i][j] = [1,2,1,2,1]
      elif i == 2 and j == 1:
        grid[i][j] = [1,2,1,2,1]
      elif i == 2 and j == 2:
        grid[i][j] = [1,2,1,2,1]
      elif i == 2 and j == 3:
        grid[i][j] = [1,2,1,2,1]
      elif i == 2 and j == 4:
        grid[i][j] = [1,2,1,2,1]
      elif i == 3 and j == 0:
        grid[i][j] = [1,2,1,2,1]
      elif i == 3 and j == 1:
        grid[i][j] = [1,2,1,2,1]
      elif i == 3 and j == 2:
        grid[i][j] = [1,2,1,2,1]
      elif i == 3 and j == 3:
        grid[i][j] = [1,2,1,2,1]
      elif i == 3 and j == 4:
        grid[i][j] = [1,2,1,2,1]
      elif i == 4 and j == 0:
        grid[i][j] = [1,2,1,2,1]
      elif i == 4 and j == 1:
        grid[i][j] = [1,2,1,2,1]
      elif i == 4 and j == 2:
        grid[i][j] = [1,2,1,2,1]
      elif i == 4 and j == 3:
        grid[i][j] = [1,2,1,2,1]
      elif i == 4 and j == 4:
        grid[i][j] = [1,2,1,2,1]
      else:
        break
  return grid

"""
@function: robot_vision
@description: Use NAO's camera to detect the shape to draw. Uses
PIL at first, then uses Canny Edge Detection via OpenCV.
"""
def robo_vision():
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
  im.save("debug/noognagnook.png")

  # use opencv to read from image
  frame = cv2.imread("debug/noognagnook.png")

  # Convert to greyscale
  gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)

  # Gaussian blur to remove noise
  blur = cv2.GaussianBlur(gray, (3,3), 0)

  # And do Canny edge detection
  canny = cv2.Canny(blur, 10, 100)

  # uncomment area to see what the NAO "sees"
  # debugging -- write canny to file
  cv2.imwrite("debug/NAOVISION.png", canny)

  """
  # debugging -- what does NAO see
  cv2.imshow("canny", canny)

  # press 0 to get out of image view
  cv2.waitKey(0)
  """

  # contour detection
  contours,h = cv2.findContours(canny,1,2)

  # only return value when you find a circle or square
  for cnt in contours:
    approx = cv2.approxPolyDP(cnt,0.01*cv2.arcLength(cnt,True),True)
    if len(approx)==2:
      robo_motion("line", canny, cnt)
      break
    elif len(approx)==5:
      robo_motion("pentagon", canny, cnt)
      break
    elif len(approx)==3:
      robo_motion("triangle", canny, cnt)
      break
    elif len(approx)==4:
      robo_motion("square", canny, cnt)
      break
    elif len(approx) == 9:
      robo_motion("half-circle", canny, cnt)
      break
    elif len(approx) > 15:
      robo_motion("circle", canny, cnt)
      break

  c = cv2.waitKey(50)
  if c == 27:
    exit(0)

"""

Description: draws the shape seen by NAO.

"""
def robo_motion(shape_name, points, image):
  # NAO, what are we going to draw?
  voice.say("I will draw a " + shape_name);
  print "\nNAO Robot will draw this shape: " + shape_name + "."

  # Send NAO to Pose Init
  postureProxy.goToPosture("StandInit", 0.5)

  # Open nao right hand
  motionProxy.openHand('RHand')

  # open hand for a few seconds
  time.sleep(2)

  # grab marker
  motionProxy.closeHand('RHand')

  # say thank you for the pen Nao
  voice.say("Thank you for the marker! Let's begin!")

  # just sleep for 3 seconds
  time.sleep(3)

  #lets start at these coordinates
  start = [0,10,0,-1,0]

  # create grid plus stage 3 setup
  grid = lookup_table(points)# image, start)

  # debug
  print grid[0][0]
  print grid[3][0]

  # We will be moving the left arm
  effector   = "RArm"
  space      = motion.SPACE_TORSO
  axisMask   = almath.AXIS_MASK_VEL
  isAbsolute = False

  # lets set our starting position
  start_pos = transformation_matrices(grid[0][0])

  # draw specific shapes
  if shape_name is "line":
    path = [start_pos, transformation_matrices(grid[3][0]), start_pos]
    times = [4.0]
  elif shape_name is "square":
    path = [start_pos, transformation_matrices(grid[4][0]), transformation_matrices(grid[4,4]), transformation_matrics(grid[0][4]), start_pos]
    times = [2.0, 4.0, 4.0, 4.0, 2.0]
  elif shape_name is "triangle":
    path = [start_pos, transformation_matrics(grid[4][4]), transformation_matrics(grid[0][4]), start_pos]
    times = [2.0, 4.0, 4.0, 2.0]
  else:
    print shape_name + " was not programmed to be drawn."

  # draw the shape!
  motionProxy.transformInterpolation(effector, space, path, axisMask, times, isAbsolute)

  voice.say("Here is your " + shape_name)

"""
@function: transformation
@description: create a transformation matrix from a given joint angle.
@return: transformation matrix for given joint.
"""
def transformation(name_of_matrix, matrix, rows, columns, a, alpha, distance, theta):
  temp = 0.0
  for i in range(rows):
    for j in range(columns):
      if i == 0 and j == 0:
        temp = math.cos(theta*math.pi/180.0)
        if temp == -0:
         temp = 0.0
        matrix[i][j] = round(temp)
      elif i == 0 and j == 1:
         temp = (-(math.sin(theta*math.pi/180.0))*math.cos(alpha*math.pi/180.0))
         if temp == -0:
           temp = 0.0
         matrix[i][j] = round(temp)
      elif i == 0 and j == 2:
         temp = (math.sin(theta*math.pi/ 180.0) * math.sin(alpha*math.pi/ 180.0))
         if temp == -0:
           temp = 0.0
         matrix[i][j] = round(temp)
      elif i == 0 and j == 3:
         temp = (a * math.cos(theta*math.pi/180.0))
         if temp == -0:
           temp = 0.0
         matrix[i][j] = round(temp)
      elif i == 1 and j == 0:
         temp = math.sin(theta*math.pi/180.0)
         if temp == -0:
           temp = 0.0
         matrix[i][j] = round(temp)
      elif i == 1 and j == 1:
         temp = (math.cos(theta*math.pi/180.0)*math.cos(alpha*math.pi/180.0))
         if temp == -0:
           temp = 0.0
         matrix[i][j] = round(temp)
      elif i == 1 and j == 2:
         temp = (-(math.cos(theta*math.pi/180.0))*math.sin(alpha*math.pi/180.0))
         if temp == -0:
           temp = 0.0
         matrix[i][j] = round(temp)
      elif i == 1 and j == 3:
         temp = a*math.sin(theta*math.pi/180.0)
         if temp == -0:
           temp = 0.0
         matrix[i][j] = round(temp)
      elif i == 2 and j == 0:
         matrix[i][j] = 0.0
      elif i == 2 and j == 1:
         temp = math.sin(alpha * math.pi/180.0)
         if temp == -0:
            temp = 0.0
         matrix[i][j] = round(temp)
      elif i == 2 and j == 2:
         temp = math.cos(alpha * math.pi/180.0)
         if temp == -0:
            temp = 0.0
         matrix[i][j] = round(temp)
      elif i == 2 and j == 3:
         temp = distance
         if temp == -0:
            temp = 0.0
         matrix[i][j] = round(temp)
      elif i == 3 and j == 0:
         matrix[i][j] = 0.0
      elif i == 3 and j == 1:
         matrix[i][j] = 0.0
      elif i == 3 and j == 2:
         matrix[i][j] = 0.0
      elif i == 3 and j == 3:
         matrix[i][j] = 1.0
  return matrix

"""
@function: multiply_matrices
@description: Multiply the joints together to get end effector.
@return: end effector.
"""
def multiply_matrices(RShoulderPitch, RShoulderRoll, RElbowYaw, RElbowRoll, RWristYaw):
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
          m3[i][j] = round(m3[i][j]+m2[i][inner]*RWristYaw[inner][j])
    return m3

"""
@function: transformation_matrices
@description: Pass a list of thetas for each joint to create our end effector
@return: a final transformation matrix (end effector)
"""
def transformation_matrices(list_of_thetas):
  rows = 4
  columns = 4

  # get theta values from the list
  theta0 = list_of_thetas[0]
  theta1 = list_of_thetas[1]
  theta2 = list_of_thetas[2]
  theta3 = list_of_thetas[3]
  theta4 = list_of_thetas[4]

  # initialize transformation matrices
  RShoulderPitch = [[0 for x in range(rows)] for x in range(columns)]
  RShoulderRoll = [[0 for x in range(rows)] for x in range(columns)]
  RElbowYaw = [[0 for x in range(rows)] for x in range(columns)]
  RElbowRoll = [[0 for x in range(rows)] for x in range(columns)]
  RWristYaw = [[0 for x in range(rows)] for x in range(columns)]
  base_to_start = [[0 for x in range(rows)] for x in range(columns)]

  # bounds checking (to prevent overheating)
  #print "(Before check): Thetas are %d, %d, %d, %d, %d" % (float(theta0), float(theta1), float(theta2), float(theta3), float(theta4))

  if float(theta0) >= 119.5:
    theta0 = 110
  elif float(theta0) <= -119.5:
    theta0 = -110
  elif float(theta1) > 18.0:
    theta1 = 15
  elif float(theta1) < -76:
    theta1 = -70
  elif float(theta2) >= 119.5:
    theta2 = 110
  elif float(theta2) <= -119.5:
    theta2 = -110
  elif float(theta3) >= 88.5:
    theta3 = 80
  elif float(theta3) <= 2:
    theta3 = 5
  elif float(theta4) >= 104.5:
    theta4 = 100
  elif float(theta4) <= -104.5:
    theta4 = -100
  else:
    print "no issues!"

  #print "\n(After check): Thetas are %d, %d, %d, %d, %d" % (float(theta0), float(theta1), float(theta2), float(theta3), float(theta4))

  # transformation matrices
  RShoulderPitch = transformation("RShoulderPitch",RShoulderPitch,rows,columns,0,-(math.pi/2.0), 0, float(theta0))
  pretty_print("RShoulderPitch", RShoulderPitch)
  RShoulderRoll = transformation("RShoulderRoll",RShoulderRoll,rows,columns,0,math.pi/2.0, 0, float(theta1) + (math.pi/2.0))
  pretty_print("RShoulderRoll", RShoulderRoll)
  RElbowYaw = transformation("RElbowYaw",RElbowYaw,rows,columns,-ELBOW_OFFSET_Y, (math.pi / 2.0), UPPER_ARM_LENGTH, float(theta2))
  pretty_print("RElbowYaw", RElbowYaw)
  RElbowRoll = transformation("RElbowRoll",RElbowRoll,rows,columns,0, -(math.pi / 2.0), 0, float(theta3))
  pretty_print("RElbowRoll", RElbowRoll)
  RWristYaw = transformation("RWristYaw",RWristYaw,rows,columns,LOWER_ARM_LENGTH, (math.pi/ 2.0), 0, float(theta4))
  pretty_print("RWristRoll", RWristRoll)
  base_to_start = multiply_matrices(RShoulderPitch,RShoulderRoll,RElbowYaw,RElbowRoll,RWristYaw)
  pretty_print("base_to_start", base_to_start)

  return base_to_start

if __name__ == '__main__':
  print("\nWelcome to the CannyBot Program!\n")

  while (True):
    # have Nao Robot look at shapes!
    #robo_vision()

    #debugging => get joint angles
    get_joint_angles()

    # run again?
    input = raw_input("\nWould you like to run this program again? (y or n)")
    if input == "n" or input == "no" or input == "0":
      break
    else:
      robo_vision()

  # tron reference!
  voice.say("End of line.")

  # smooth transition ftw!
  time.sleep(3)

  # stiffness off
  stiffness_off(motionProxy)

  # end of program
  print("End of Program.")
