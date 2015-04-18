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
ip = "169.254.43.193"
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
  pStiffnessLists = 1.0
  pTimeLists = 1.0
  proxy.stiffnessInterpolation(pNames, pStiffnessLists, pTimeLists)
def stiffness_off(proxy):
  #We use the "Body" name to signify the collection of all joints
  pNames = "Body"
  pStiffnessLists = 0.0
  pTimeLists = 1.0
  proxy.stiffnessInterpolation(pNames, pStiffnessLists, pTimeLists)

def bilinear_interpolation(x, y, points):
    '''
    StackOverflow: http://stackoverflow.com/a/8662355
    Interpolate (x,y) from values associated with four points.

    The four points are a list of four triplets:  (x, y, value).
    The four points can be in any order.  They should form a rectangle.

        >>> bilinear_interpolation(12, 5.5,
        ...                        [(10, 4, 100),
        ...                         (20, 4, 200),
        ...                         (10, 6, 150),
        ...                         (20, 6, 300)])
        165.0

    '''

    points = sorted(points)              # order points by x, then by y
    (x1, y1, q11), (_x1, y2, q12), (x2, _y1, q21), (_x2, _y2, q22) = points

    if x1 != _x1 or x2 != _x2 or y1 != _y1 or y2 != _y2:
        raise ValueError('points do not form a rectangle')
    if not x1 <= x <= x2 or not y1 <= y <= y2:
        raise ValueError('(x, y) not within the rectangle')

    return (q11 * (x2 - x) * (y2 - y) +
            q21 * (x - x1) * (y2 - y) +
            q12 * (x2 - x) * (y - y1) +
            q22 * (x - x1) * (y - y1)) / ((x2 - x1) * (y2 - y1) + 0.0)
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
    input_value = raw_input("\Which coordinate? ")
    print input_value

   # read input file
    f = open('debug/input.txt', 'a')

    f.write("\n\nCoordinate: " + input_value)

    # posture
    #postureProxy.goToPosture("StandInit", 0.5)

    #We use the "Body" name to signify the collection of all joints
    pNames = "Body"
    pStiffnessLists = 0.0
    pTimeLists = 1.0
    motionProxy.stiffnessInterpolation(pNames, pStiffnessLists, pTimeLists)

    # Example that finds the difference between the command and sensed angles.
    names       = "RArm"
    useSensors    = False
    commandAngles = motionProxy.getAngles(names, useSensors)
    print "Command angles:"
    print str(commandAngles) + "\n"
    f.write("\nCommandAngles:\n" + str(commandAngles))

    useSensors  = True
    sensorAngles = motionProxy.getAngles(names, useSensors)
    print "Sensor angles:"
    print str(sensorAngles)
    print ""

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
  rows = 5
  columns = 5
  pixel_rows = 640
  pixel_columns = 480
  scale_x = pixel_rows/rows
  scale_y = pixel_columns/columns

  stiffness_off(motionProxy)

  grid = [[0 for x in range(pixel_columns+1)] for x in range(pixel_rows+1)]

  # TODO: get 36 positions (6 thetas per position)! Just get two for drawing a line
  # TODO: in stage 3, use bilinear interpolation for drawing exact shapes!
  # http://stackoverflow.com/questions/18044485/how-to-write-lines-and-grid-on-image-in-python
  # traverse through matrix and add theta values based on measurements of space in "invisible" grid

  # do range check to send values to bilinear-interoplation

  for i in range(0,pixel_rows+1):
    for j in range(0,pixel_columns+1):
      if i == 0*scale_x and j == 0*scale_y:
        grid[i][j] = [1,2,2,3,1]
      elif i == 0*scale_x and j == 1*scale_y:
        grid[i][j] = [1,2,1,2,1]
      elif i == 0*scale_x and j == 2*scale_y:
        grid[i][j] = [1,2,1,2,1]
      elif i == 0*scale_x and j == 3*scale_y:
        grid[i][j] = [1,2,1,2,1]
      elif i == 0*scale_x and j == 4*scale_y:
        grid[i][j] = [-0.0383080393075943, 0.04137603938579559, -0.08287796378135681, 0.533873975276947, 1.4526560306549072, 0.25200000405311584]
      elif i == 0*scale_x and j == 5*scale_y:
        grid[i][j] = [1,2,1,2,1]

      elif i == 1*scale_x and j == 0*scale_y:
        grid[i][j] = [1,2,1,2,1]
      elif i == 1*scale_x and j == 1*scale_y:
        grid[i][j] = [1,2,1,2,1]
      elif i == 1*scale_x and j == 2*scale_y:
        grid[i][j] = [1,2,1,2,1]
      elif i == 1*scale_x and j == 3*scale_y:
        grid[i][j] = [1,2,1,2,1]
      elif i == 1*scale_x and j == 4*scale_y:
        grid[i][j] = [1,2,1,2,1]
      elif i == 1*scale_x and j == 5*scale_y:
        grid[i][j] = [1,2,1,2,1]

      elif i == 2*scale_x and j == 0*scale_y:
        grid[i][j] = [1,2,1,2,1]
      elif i == 2*scale_x and j == 1*scale_y:
        grid[i][j] = [1,2,1,2,1]
      elif i == 2*scale_x and j == 2*scale_y:
        grid[i][j] = [1,2,1,2,1]
      elif i == 2*scale_x and j == 3*scale_y:
        grid[i][j] = [1,2,1,2,1]
      elif i == 2*scale_x and j == 4*scale_y:
        grid[i][j] = [1,2,1,2,1]
      elif i == 2*scale_x and j == 5*scale_y:
        grid[i][j] = [1,2,1,2,1]

      elif i == 3*scale_x and j == 0*scale_y:
        grid[i][j] = [1,2,1,2,1]
      elif i == 3*scale_x and j == 1*scale_y:
        grid[i][j] = [1,2,1,2,1]
      elif i == 3*scale_x and j == 2*scale_y:
        grid[i][j] = [1,2,1,2,1]
      elif i == 3*scale_x and j == 3*scale_y:
        grid[i][j] = [1,2,1,2,1]
      elif i == 3*scale_x and j == 4*scale_y:
        grid[i][j] = [1,2,1,2,1]
      elif i == 3*scale_x and j == 5*scale_y:
        grid[i][j] = [1,2,1,2,1]

      elif i == 4*scale_x and j == 0*scale_y:
        grid[i][j] = [1,2,1,2,1]
      elif i == 4*scale_x and j == 1*scale_y:
        grid[i][j] = [1,2,1,2,1]
      elif i == 4*scale_x and j == 2*scale_y:
        grid[i][j] = [1,2,1,2,1]
      elif i == 4*scale_x and j == 3*scale_y:
        grid[i][j] = [1,2,1,2,1]
      elif i == 4*scale_x and j == 4*scale_y:
        grid[i][j] = [1,2,1,2,1]
      elif i == 4*scale_x and j == 5*scale_y:
        grid[i][j] = [1,2,1,2,1]

      elif i == 5*scale_x and j == 0*scale_y:
        grid[i][j] = [1,2,1,2,1]
      elif i == 5*scale_x and j == 1*scale_y:
        grid[i][j] = [1,2,1,2,1]
      elif i == 5*scale_x and j == 2*scale_y:
        grid[i][j] = [1,2,1,2,1]
      elif i == 5*scale_x and j == 3*scale_y:
        grid[i][j] = [1,2,1,2,1]
      elif i == 5*scale_x and j == 4*scale_y:
        grid[i][j] = [1,2,1,2,1]
      elif i == 5*scale_x and j == 5*scale_y:
        grid[i][j] = [1,2,1,2,1]

  defined_grid_points = [[0,0], [128,0], [256,0], [384,0], [512,0], [640,0],
                         [0,96], [128,96], [256,96], [384,96], [512,96], [640,96],
                         [0,192], [128,192], [256,192], [384,192], [512,192], [640,192],
                         [0,288], [128,288], [256,288], [384,288], [512,288], [640,288],
                         [0,384], [128,384], [256,384], [384,384], [512,384], [640,384],
                         [0,480], [128,480], [256,480], [384,480], [512,480], [640,480]]

  print len(points[0])
  print points[0][0]
  print  points[0].item(0)
  print heyl

  # list of nearest neighbors to each point
  nearest_neighbors = []

  # compare the points found to the grid point
  for i in range(0, pixel_rows+1):
    for j in range(0, pixel_columns+1):
      for k in range(len(grid)):
        if points[[k][i]] <= defined_grid_points[i]:
          nearest_neighbors = []

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

  # use opencv to read from image
  frame = cv2.imread("debug/noognagnook.png")

  # Convert to greyscale
  gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)

  # Gaussian blur to remove noise
  blur = cv2.GaussianBlur(gray, (3,3), 0)

  # And do Canny edge detection
  canny = cv2.Canny(blur, 10, 100)

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

  # draw a specific shape
  # http://docs.opencv.org/modules/imgproc/doc/structural_analysis_and_shape_descriptors.html?highlight=findcontours#findcontours
  # http://stackoverflow.com/questions/9413216/simple-digit-recognition-ocr-in-opencv-python
  for cnt in contours:
    approx = cv2.approxPolyDP(cnt,0.01*cv2.arcLength(cnt,True),True)
    if len(approx)==2:
      robo_motion("line", contours)
      break
    elif len(approx)==5:
      robo_motion("pentagon", contours)
      break
    elif len(approx)==3:
      robo_motion("triangle", contours)
      break
    elif len(approx)==4:
      robo_motion("square", contours)
      break
    elif len(approx) == 9:
      robo_motion("half-circle", contours)
      break
    elif len(approx) > 15:
      robo_motion("circle", contours)
      break

  c = cv2.waitKey(50)
  if c == 27:
    exit(0)

"""
@function: robo_motion
@description: draws the shape seen by NAO.
"""
def robo_motion(shape_name, points):
  # NAO, what are we going to draw?
  #voice.say("I will draw a " + shape_name);
  print "\nNAO Robot will draw this shape: " + shape_name + "."

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
  #motionProxy.positionInterpolation(effector, space, end, axisMask, [2.0], isAbsolute)

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

  # TODO: switch to radians
  # http://doc.aldebaran.com/1-14/family/nao_h25/joints_h25.html#h25-joints

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
  RHand = transformation("RHand",RHand,rows,columns,LOWER_ARM_LENGTH, (math.pi/ 2.0), 0, float(theta4))
  pretty_print("RWristRoll", RWristRoll)
  base_to_start = multiply_matrices(RShoulderPitch,RShoulderRoll,RElbowYaw,RElbowRoll,RWristYaw,RHand)
  pretty_print("base_to_start", base_to_start)

  return base_to_start

if __name__ == '__main__':
  print("\nWelcome to the CannyBot Program!\n")

  while (True):
    # debug?
    decision = raw_input("\nWould you like to debug? (y or n) ")
    if decision == "n" or decision == "no" or decision == "0":
      robo_vision()
    else:
      get_joint_angles()

    # run again?
    input = raw_input("\nWould you like to run this program again? (y or n) ")
    if input == "n" or input == "no" or input == "0":
      break
    else:
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
  stiffness_off(motionProxy)

  # end of program
  print("End of Program.")
