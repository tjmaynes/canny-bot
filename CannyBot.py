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

helper functions

"""
def stiffness_on(proxy):
  pNames = "RArm"
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
@function: get_joint_angles()
@description: manually get joint angles within NAO's workspace
@return: a list of joint angles =>  [rshoulderpitch, rshoulderroll, relbowyaw, rwristyaw, rhand]
"""
def record_joint_angles():
  # which coordinate location to record
  input_value = raw_input("\nWhich coordinate? ")

  # read input file
  f = open('debug/input.txt', 'a')
  f.write("\n\nCoordinate: " + input_value)

  #We use the "Body" name to signify the collection of all joints
  pNames = "LArm"
  pStiffnessLists = 0.0
  pTimeLists = 1.0
  motionProxy.stiffnessInterpolation(pNames, pStiffnessLists, pTimeLists)

  #We use the "Body" name to signify the collection of all joints
  pNames = "RArm"
  pStiffnessLists = 0.0
  pTimeLists = 1.0
  motionProxy.stiffnessInterpolation(pNames, pStiffnessLists, pTimeLists)

  # Example that finds the difference between the command and sensed angles.
  names       = "LArm"
  useSensors    = False
  commandAngles = motionProxy.getAngles(names, useSensors)

  f.write("\nLArm:")

  f.write("\nCommand Angles:\n" + str(commandAngles) )

  useSensors  = True
  sensorAngles = motionProxy.getAngles(names, useSensors)

  f.write("\nSensor Angles:\n" + str(sensorAngles) )

  f.write("\n\nRArm:")

  # Example that finds the difference between the command and sensed angles.
  names       = "RArm"
  useSensors    = False
  commandAngles = motionProxy.getAngles(names, useSensors)

  f.write("\nCommand Angles:\n" + str(commandAngles) )

  useSensors  = True
  sensorAngles = motionProxy.getAngles(names, useSensors)

  f.write("\nSensor Angles:\n" + str(sensorAngles) )

  # close input file
  f.close()

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
        grid[i][j] = [0.4188239574432373, 0.3141592741012573, 0.7577540874481201, 0.5660879611968994, 0.5997520685195923, 0.7547999620437622]
      elif i == 640 and j == 0:
        grid[i][j] = [0.4970579743385315, -0.3497939705848694, 0.5491300821304321, 1.055433988571167, 0.9909220933914185, 0.7547999620437622]
      elif i == 0 and j == 460:
        grid[i][j] = [0.4157559871673584, 0.18710604310035706, 0.10273604094982147, 1.152076005935669, 1.043078064918518, 0.7547999620437622]
      elif i == 640 and j == 460:
        grid[i][j] = [0.5216019749641418, -0.6289819478988647, 0.2530680298805237, 1.5446163415908813, 1.0599520206451416, 0.7547999620437622]

  defined_grid_points = [[0, 0, grid[0][0]],
                         [640, 0, grid[640][0]],
                         [0, 460, grid[0][460]],
                         [640, 460, grid[640][460]]]

  # read input file
  f = open('debug/input.txt', 'a')

  f.write("\n\nTesting")

  # perform bilinear interpolation on all the points
  # to get theta values of each point
  path = []
  go_back_to_start = bilinear_interpolation(points[0][0].item(0), points[0][0].item(1), defined_grid_points)
  for i in range(len(points)):
    for j in range(len(points[i])):
      path.append(bilinear_interpolation(points[i][j].item(0), points[i][j].item(1), defined_grid_points))
      f.write("\n\n " + str(bilinear_interpolation(points[i][j].item(0), points[i][j].item(1), defined_grid_points)))
  path.append(go_back_to_start)

  f.close()

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
  im.save("debug/noognagnook_square.png")

  # get rid of pink garbage for final canny
  w, h = im.size
  im.crop((0, 10, w, h-10)).save("debug/noognagnook_square.png")

  # use opencv to read from image
  frame = cv2.imread("debug/noognagnook_square.png")

  # Convert to greyscale
  gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)

  # Gaussian blur to remove noise
  blur = cv2.GaussianBlur(gray, (3,3), 0)

  # And do Canny edge detection
  canny = cv2.Canny(blur, 10, 100)

  # debugging -- write canny to file
  cv2.imwrite("debug/NAOVISION_square.png", canny)

  # contour detection
  contours,h = cv2.findContours(canny, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

  # Send NAO to Pose Init
  #postureProxy.goToPosture("StandInit", 0.5)

  # NAO, what are we going to draw?
  voice.say("I will draw your shape!");

  # debug => turn stiffness off
  #stiffness_off(motionProxy)

  # We will be moving the left arm and right arm
  motionProxy.setStiffnesses("LArm", 1.0)
  motionProxy.setStiffnesses("RArm", 1.0)

  # draw a specific shape
  # http://opencvpython.blogspot.com/2012/06/hi-this-article-is-tutorial-which-try.html
  # http://docs.opencv.org/modules/imgproc/doc/structural_analysis_and_shape_descriptors.html?highlight=findcontours#findcontours
  # http://stackoverflow.com/questions/9413216/simple-digit-recognition-ocr-in-opencv-python
  for cnt in contours:
    approx = cv2.approxPolyDP(cnt,0.1*cv2.arcLength(cnt,True),True)
    robo_motion(approx)
    break

  # cleaner exit
  effector = ["RArm"]
  path = [0.39121198654174805, -0.03839196264743805, 0.7132680416107178, 0.9603259563446045, 0.7884340286254883, 0.7547999620437622]

  motionProxy.setAngles(effector, path, 0.3)

  # raise your hand!
  effectors = ["LArm","RArm"]
  path = [[-1.716588020324707, 0.11500804126262665, -0.1150919571518898, -0.03490658476948738, -1.4542739391326904, 0.7563999891281128],
          [-1.6628141403198242, 0.09506604075431824, 1.1704000234603882, 0.07367396354675293, 0.48470205068588257, 0.7555999755859375]]

  # draw the shape!
  for i in range(len(path)):
    motionProxy.setAngles(effectors[i], path[i], 0.2)
    time.sleep(1)

  # Done
  voice.say("Here is your shape!")

  """
  c = cv2.waitKey(50)
  if c == 27:
    exit(0)
  """

"""
@function: robo_motion
@description: draws the shape seen by NAO.
"""
def robo_motion(points):
  # specify the effector to use
  effector   = "RArm"

  # create grid with stage 3 setup
  path = lookup_table(points)

  # draw the shape!
  for i in range(len(path)):
    motionProxy.setAngles(effector, path[i], 0.2)
    time.sleep(2)

if __name__ == '__main__':
  print("\nWelcome to the CannyBot Program!\n")

  # debug?
  decision = raw_input("\nWould you like to debug? (0 or 1)\n> ")
  if decision == "0":
    robo_vision()
  else:
    record_joint_angles()

  # tron reference!
  voice.say("End of line.")

  # smooth transition ftw!
  time.sleep(3)

  # end of program
  print("End of Program.")
