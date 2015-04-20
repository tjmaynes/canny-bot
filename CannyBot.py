"""
@file: CannyBot.py
@authors: Tommy Lin, TJ Maynes
@subject: getting the NAO Robot to draw the shapes it "sees" using image processing.
"""

import os, sys, math, motion, almath, time
import numpy as np
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
ip = "169.254.242.79"
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
  pNames = "Body"
  pStiffnessLists = 1.0
  pTimeLists = 1.0
  proxy.stiffnessInterpolation(pNames, pStiffnessLists, pTimeLists)

def stiffness_off(proxy):
  pNames = "Body"
  pStiffnessLists = 0.0
  pTimeLists = 1.0
  proxy.stiffnessInterpolation(pNames, pStiffnessLists, pTimeLists)

"""
@function: bilinear_interpolation
@description: http://en.wikipedia.org/wiki/Bilinear_interpolation
"""
def bilinear_interpolation(x, y, values):
  # sort values first!
  values = sorted(values)

  # get x and y values from values
  x1  = values[0][0]
  y1  = values[0][1]
  _x1 = values[1][0]
  y2  = values[1][1]
  x2  = values[2][0]
  _y1 = values[2][1]
  _x2 = values[3][0]
  _y2 = values[3][1]

  # check that the values form a rectangle
  if x1 != _x1 or x2 != _x2 or y1 != _y1 or y2 != _y2:
    raise ValueError('points do not form a rectangle')
  if not x1 <= x <= x2 or not y1 <= y <= y2:
    raise ValueError('(x, y) not within the rectangle')

  # get joint angles from each defined defined
  q11 = values[0][2]
  q12 = values[1][2]
  q21 = values[2][2]
  q22 = values[3][2]

  # calculations
  temp1 = [i * (x2 - x) * (y2 - y) for i in q11]
  temp2 = [i * (x - x1) * (y2 - y) for i in q21]
  temp3 = [i * (x2 - x) * (y - y1) for i in q12]
  temp4 = [i * (x - x1) * (y - y1) for i in q22]
  add12 = [x + y for x,y in zip(temp1, temp2)]
  add34 = [x + y for x,y in zip(temp3, temp4)]
  final_add = [x + y for x,y in zip(add12, add34)]
  temp = ((x2 - x1) * (y2 - y1) + 0.0)
  final_temp = [i / temp for i in final_add]

  # return final calculation
  return final_temp

"""
@function: get_times
@description: manually add in a time for each set of theta values per point (lots of points)
"""
def get_times(path):
  times = []
  print len(path)
  print path[0]
  for i in range(len(path)):
    times.append([1.0,2.0,3.0,4.0,5.0,6.0])

  return times

"""
@function: get_joint_angles()
@description: manually get joint angles within NAO's workspace
@return: a list of joint angles =>  [rshoulderpitch, rshoulderroll, relbowyaw, rwristyaw, rhand]
"""
def get_joint_angles():
  # which coordinate location to record
  input_value = raw_input("\Which coordinate? ")

  # read input file
  f = open('debug/input.txt', 'a')
  f.write("\n\nCoordinate: " + input_value)
  
  #We use the "Body" name to signify the collection of all joints
  pNames = "RArm"
  pStiffnessLists = 0.0
  pTimeLists = 1.0
  motionProxy.stiffnessInterpolation(pNames, pStiffnessLists, pTimeLists)

  # Example that finds the difference between the command and sensed angles.
  names       = "RArm"
  useSensors    = False
  commandAngles = motionProxy.getAngles(names, useSensors)

  f.write("\Command Angles:\n" + str(commandAngles) )

  useSensors  = True
  sensorAngles = motionProxy.getAngles(names, useSensors)

  f.write("\nSensor Angles:\n" + str(sensorAngles) )

  # close input file
  f.close()

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
@function: lookup_table
@description: create lookup table of size 5 by 5 with each coordinate
containing the joint angles (theta values) to get to that position in
NAO's workspace.
@return: a table of joint angles
"""
def lookup_table(points):
  pixel_rows = 640
  pixel_columns = 460
  grid = [[0 for x in range(pixel_columns+1)] for x in range(pixel_rows+1)]

  for i in range(0,pixel_rows+1):
    for j in range(0,pixel_columns+1):
      if i == 0 and j == 0:
        grid[i][j] = [0.5200679898262024, 0.3141592741012573, 0.5982180833816528, 0.6611959934234619, 0.621228039264679, 0.7531999945640564]
      elif i == 640 and j == 0:
        grid[i][j] = [0.7256239652633667, -0.1871899664402008, 0.7746280431747437, 1.0339579582214355, 0.7071320414543152, 0.7531999945640564]
      elif i == 0 and j == 460:
        grid[i][j] = [0.8038579821586609, 0.2668740451335907, 0.32670003175735474, 1.2671259641647339, 0.4463520646095276, 0.7547999620437622]
      elif i == 640 and j == 460:
        grid[i][j] = [0.8836259841918945, -0.44029998779296875, 0.5706060528755188, 1.5446163415908813, 0.8298520445823669, 0.753600001335144]

  defined_grid_points = [[0, 0, grid[0][0]],
                         [640, 0, grid[640][0]],
                         [0, 460, grid[0][460]],
                         [640, 460, grid[640][460]]]

  print points
  print points[0][0].item(0)

  path = []

  # perform bilinear interpolation on all the points
  # to get theta values of each point
  for i in range(len(points)):
    for j in range(len(points[i])):
      path.append(bilinear_interpolation(points[i][j].item(0), points[i][j].item(1), defined_grid_points))

  return path

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

  # get rid of pink garbage for final canny
  w, h = im.size
  im.crop((0, 10, w, h-10)).save("debug/test.png")

  # use opencv to read from image
  frame = cv2.imread("debug/test.png")

  normal = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

  # Convert to greyscale
  gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)

  # Gaussian blur to remove noise
  blur = cv2.GaussianBlur(gray, (3,3), 0)

  # And do Canny edge detection
  canny = cv2.Canny(blur, 10, 100)

  # debugging -- write canny to file
  cv2.imwrite("debug/NAOVISION.png", canny)

  # contour detection
  contours,h = cv2.findContours(canny,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

  # http://opencvpython.blogspot.com/2012/06/hi-this-article-is-tutorial-which-try.html

  # draw a specific shape
  # http://docs.opencv.org/modules/imgproc/doc/structural_analysis_and_shape_descriptors.html?highlight=findcontours#findcontours
  # http://stackoverflow.com/questions/9413216/simple-digit-recognition-ocr-in-opencv-python
  for cnt in contours:
    approx = cv2.approxPolyDP(cnt,0.01*cv2.arcLength(cnt,True),True)
    robo_motion(approx)

  c = cv2.waitKey(50)
  if c == 27:
    exit(0)

"""
@function: robo_motion
@description: draws the shape seen by NAO.
"""
def robo_motion(points):
  # NAO, what are we going to draw?
  voice.say("I will draw your shape!");

  #stiffness_off(motionProxy)

  # Send NAO to Pose Init
  #postureProxy.goToPosture("StandInit", 0.5)

  # Open nao right hand
  #motionProxy.openHand('RHand')

  # open hand for a few seconds
  #time.sleep(2)

  # grab marker
  #motionProxy.closeHand('RHand')

  # say thank you for the pen Nao
  #voice.say("Thank you for the marker! Let's begin!")

  # just sleep for 3 seconds
  #time.sleep(3)

  # We will be moving the left arm
  motionProxy.setStiffnesses("RArm", 0.0)
  effector   = "RArm"
  space      = motion.SPACE_TORSO
  axisMask   = almath.AXIS_MASK_VEL
  isAbsolute = True

  # create grid with stage 3 setup
  path = lookup_table(points)

  times = get_times(path)

  # draw the shape!
  motionProxy.positionInterpolation(effector, space, path, axisMask, times, isAbsolute)

  # raise your hand!
  motionProxy.positionInterpolation(effector, space, end, axisMask, [2.0], isAbsolute)

  # Done
  voice.say("Here is your shape!");

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
  print "(Before check): Thetas are %d, %d, %d, %d, %d" % (float(theta0), float(theta1), float(theta2), float(theta3), float(theta4))

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

  print "\n(After check): Thetas are %d, %d, %d, %d, %d" % (float(theta0), float(theta1), float(theta2), float(theta3), float(theta4))

  # transformation matrices
  RShoulderPitch = transformation("RShoulderPitch",RShoulderPitch,rows,columns,0,-(math.pi/2.0), 0, float(theta0))
  RShoulderRoll = transformation("RShoulderRoll",RShoulderRoll,rows,columns,0,math.pi/2.0, 0, float(theta1) + (math.pi/2.0))
  RElbowYaw = transformation("RElbowYaw",RElbowYaw,rows,columns,-ELBOW_OFFSET_Y, (math.pi / 2.0), UPPER_ARM_LENGTH, float(theta2))
  RElbowRoll = transformation("RElbowRoll",RElbowRoll,rows,columns,0, -(math.pi / 2.0), 0, float(theta3))
  RWristYaw = transformation("RWristYaw",RWristYaw,rows,columns,LOWER_ARM_LENGTH, (math.pi/ 2.0), 0, float(theta4))
  RHand = transformation("RHand",RHand,rows,columns,LOWER_ARM_LENGTH, (math.pi/ 2.0), 0, float(theta4))
  base_to_start = multiply_matrices(RShoulderPitch,RShoulderRoll,RElbowYaw,RElbowRoll,RWristYaw,RHand)

  # pretty print each transformation
  pretty_print("RShoulderPitch", RShoulderPitch)
  pretty_print("RShoulderRoll", RShoulderRoll)
  pretty_print("RElbowYaw", RElbowYaw)
  pretty_print("RElbowRoll", RElbowRoll)
  pretty_print("RWristRoll", RWristRoll)
  pretty_print("RWristRoll", RWristRoll)
  pretty_print("base_to_start", base_to_start)

  return base_to_start

if __name__ == '__main__':
  print("\nWelcome to the CannyBot Program!\n")

  # debug?
  decision = raw_input("\nWould you like to debug? (y or n) ")
  if decision == "n" or decision == "no" or decision == "0":
    robo_vision()
  else:
    get_joint_angles()

  # tron reference!
  voice.say("End of line.")

  # smooth transition ftw!
  time.sleep(3)

  # stiffness off
  #stiffness_off(motionProxy)

  # end of program
  print("End of Program.")
