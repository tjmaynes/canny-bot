"""
@file: CannyBot.py
@authors: Tommy Lin, TJ Maynes
@subject: getting the NAO Robot to draw the shapes it "sees" using image processing.
"""

import sys

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
  print "\nWelcome to the Matrix Testing Program!\n"

  transformation_matrices([120,12,45,-12,0])

  print "End of program!"
