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

# defaults
rows = 4
columns = 4

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

  """
  # uncomment area to see what the NAO "sees"
  # debugging -- write canny to file
  cv2.imwrite("debug/NAOVISION.png", canny)

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
        robo_motion("line")
        break
      if len(approx)==5:
        robo_motion("pentagon")
        break
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

def create_grid(start):
  rows = 7
  columns = 7
  grid = [[0 for x in range(rows)] for x in range(columns)]

  # traverse through matrix and add theta values based on measurements of space in "invisible" grid
  for i in range(rows):
    for j in range(columns):
      if i == 0 and j == 0:
        grid[i][j] = start
      elif i == 0 and j == 1:
        grid[i][j] = [1,2,1,2]
      elif i == 0 and j == 2:
        break
      elif i == 0 and j == 3:
        break
      elif i == 0 and j == 4:
        break
      elif i == 0 and j == 5:
        break
      elif i == 0 and j == 6:
        break
      elif i == 0 and j == 7:
        break
      elif i == 1 and j == 0:
        break
      elif i == 1 and j == 1:
        break
      elif i == 1 and j == 2:
        break
      elif i == 1 and j == 3:
        break
      elif i == 1 and j == 4:
        break
      elif i == 1 and j == 5:
        break
      elif i == 1 and j == 6:
        break
      elif i == 1 and j == 7:
        break
      elif i == 2 and j == 0:
        break
      elif i == 2 and j == 1:
        break
      elif i == 2 and j == 2:
        break
      elif i == 2 and j == 3:
        break
      elif i == 2 and j == 4:
        break
      elif i == 2 and j == 5:
        break
      elif i == 2 and j == 6:
        break
      elif i == 2 and j == 7:
        break
      elif i == 3 and j == 0:
        break
      elif i == 3 and j == 1:
        break
      elif i == 3 and j == 2:
        break
      elif i == 3 and j == 3:
        break
      elif i == 3 and j == 4:
        break
      elif i == 3 and j == 5:
        break
      elif i == 3 and j == 6:
        break
      elif i == 3 and j == 7:
        break
      elif i == 4 and j == 0:
        break
      elif i == 4 and j == 1:
        break
      elif i == 4 and j == 2:
        break
      elif i == 4 and j == 3:
        break
      elif i == 4 and j == 4:
        break
      elif i == 4 and j == 5:
        break
      elif i == 4 and j == 6:
        break
      elif i == 4 and j == 7:
        break
      elif i == 5 and j == 0:
        break
      elif i == 5 and j == 1:
        break
      elif i == 5 and j == 2:
        break
      elif i == 5 and j == 3:
        break
      elif i == 5 and j == 4:
        break
      elif i == 5 and j == 5:
        break
      elif i == 5 and j == 6:
        break
      elif i == 5 and j == 7:
        break
      elif i == 6 and j == 0:
        break
      elif i == 6 and j == 1:
        break
      elif i == 6 and j == 2:
        break
      elif i == 6 and j == 3:
        break
      elif i == 6 and j == 4:
        break
      elif i == 6 and j == 5:
        break
      elif i == 6 and j == 6:
        break
      elif i == 6 and j == 7:
        break
      elif i == 7 and j == 0:
        break
      elif i == 7 and j == 1:
        break
      elif i == 7 and j == 2:
        break
      elif i == 7 and j == 3:
        break
      elif i == 7 and j == 4:
        break
      elif i == 7 and j == 5:
        break
      elif i == 7 and j == 6:
        break
      elif i == 7 and j == 7:
        break
      else:
        # should never reach here!
        break
  return grid

def robo_motion(shape_name):
  voice.say("I will draw a " + shape);

    print "\nNAO Robot will draw this shape: " + shape + "."

    stiffness_on(motionProxy)

    start = [0,10,0,-1]

    grid = create_grid(start)

    # Send NAO to Pose Init
    postureProxy.goToPosture("StandInit", 0.5)

    effector   = "RArm"
    space      = motion.FRAME_ROBOT
    axisMask   = almath.AXIS_MASK_VEL    # just control position
    isAbsolute = False

   # Since we are in relative, the current position is zero
   current_pos = transformation_matrices(grid[0][0])

   if shape_name is "line":
     path = [current_pos, transformation_matrices(grid[5,0])]
     times = [2.0, 6.0]
   elif shape_name is "square":
     path = [current_pos, transformation_matrices(grid[5,0]), transformation_matrices(grid[5,5]), transformation_matrics(grid[0,5]), current_pos]
     times = [2.0, 4.0, 4.0, 4.0, 2.0]
   elif shape_name is "triangle":
     path = [current_pos, transformation_matrics(grid[5,5]), transformation_matrics(grid[0,5]), current_pos]
     times = [2.0, 4.0, 4.0, 2.0]
   else:
     print shape_name + " was not programmed to be drawn."

   motionProxy.transferInterpolation(effector, space, path, axisMask, times, isAbsolute)

def transformation_matrix(name_of_matrix, matrix, rows, columns, a, alpha, distance, theta):
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
def transformation_matrices(thelist):
   # get theta values from the list
   theta0 = thelist[0]
   theta1 = thelist[1]
   theta2 = thelist[2]
   theta3 = thelist[3]
  
    # initialize matrices
    RShoulderPitch = [[0 for x in range(4)] for x in range(4)]
    RShoulderRoll = [[0 for x in range(4)] for x in range(4)]
    RElbowYaw = [[0 for x in range(4)] for x in range(4)]
    RElbowRoll = [[0 for x in range(4)] for x in range(4)]
    RWristRoll = [[0 for x in range(4)] for x in range(4)]
    base_to_start = [[0 for x in range(4)] for x in range(4)]

    # bounds checking (to prevent overheating)
    #print "(Before check): Thetas are %d, %d, %d, %d, %d" % (float(theta0), float(theta1), float(theta2), float(theta3), float(theta4))

    if float(theta0) >= 119.5:
      theta0 = 110
    elif float(theta0) <= -119.5:
      theta0 = -110
    if float(theta1) > 18.0:
      theta1 = 15
    elif float(theta1) < -76:
      theta1 = -70
    if float(theta2) >= 119.5:
      theta2 = 110
    elif float(theta2) <= -119.5:
      theta2 = -110
    if float(theta3) >= 88.5:
      theta3 = 80
    elif float(theta3) <= 2:
      theta3 = 5
    if float(theta4) >= 104.5:
      theta4 = 100
    elif float(theta4) <= -104.5:
      theta4 = -100

    #print "\n(After check): Thetas are %d, %d, %d, %d, %d" % (float(theta0), float(theta1), float(theta2), float(theta3), float(theta4))

    # transformation matrices
    RShoulderPitch = transformation_matrix("RShoulderPitch",RShoulderPitch,rows,columns,0,-(math.pi/2.0), 0, float(theta0))
    RShoulderRoll = transformation_matrix("RShoulderRoll",RShoulderRoll,rows,columns,0,math.pi/2.0, 0, float(theta1) + (math.pi/2.0))
    RElbowYaw = transformation_matrix("RElbowYaw",RElbowYaw,rows,columns,-ELBOW_OFFSET_Y, (math.pi / 2.0), UPPER_ARM_LENGTH, float(theta2))
    RElbowRoll = transformation_matrix("RElbowRoll",RElbowRoll,rows,columns,0, -(math.pi / 2.0), 0, float(theta3))
    RWristRoll = transformation_matrix("RWristRoll",RWristRoll,rows,columns,LOWER_ARM_LENGTH, (math.pi/ 2.0), 0, float(theta4))
    base_to_start = multiply_matrices(RShoulderPitch,RShoulderRoll,RElbowYaw,RElbowRoll,RWristRoll)

    return base_to_start

if __name__ == '__main__':
  print("\nWelcome to the CannyBot Program!\n")

    while (true):
      # have nao look at shapes!
      robo_vision()

      # debugging
      #test_transformation_matrices()

      # run again?
      input = raw_input("\nWould you like to run this program again?")
      if input == "n" or input == "no" or input == "0":
        break
      else:
        robo_vision()

    # end of line
    print("End of Program.")
