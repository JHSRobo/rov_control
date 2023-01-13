#!/usr/bin/env python3
# author Adon Sharp | v0.2 | 1-15-2022
# author Alex Bertran | v1.0 | 10-29-2022
# author Alex Bertran | v1.1 | 1-12-2023

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32 # for depth hold
from math import copysign
from copilot_interface.msg import controlData
from copilot_interface.msg import autoControlData
from rov_control.msg import thrusterPercents

rospy.init_node("drive_control")

sensitivity = {"linear": 0.5, "angular": 0.25, "vertical": 0.5} # Holds a percent multiplier for ROV sensitivity

thrustEN = False  # thrusters enabled (True = yes, False = default = no
dhEnable = False
depthHoldThrust = 0.0

#The vector that gets edited by the callbacks and then published
joy_vector = Twist()

# Multiplies the value of the axis by an exponent (1.4)
# Uses copysign to make sure that raising axis to an even power can still return a negative number
def expDrive(axis):
  axis = copysign(abs(axis) ** 1.1, axis)
  return axis

# Takes in vectors and translates them to thrusterPercents
  # linearX is the left-right joystick axis
  # linearY is the front-back joystick axis
  # linearZ is the throttle
  # angularX is the rotational joystick axis
def translate_vectors(vector):
  # Read twist values into variables
  linearX = vector.linear.x
  linearY = vector.linear.y
  linearZ = vector.linear.z
  angularX = vector.angular.x

  #Check to make sure values are appropriate
  if abs(linearX) > 1 or abs(linearY) > 1 or abs(linearZ) > 1 or abs(angularX) > 1: # The values for max are subject to change
    rospy.logerr("Vectors outside of range")
  
  # Motor Calculations
  # IMPORTANT TO UNDERSTAND THIS: https://drive.google.com/file/d/11VF0o0OYVaFGKFvbYtnrmOS0e6yM6IxH/view
  # TOTALLY SUBJECT TO CHANGE IN EVENT OF THRUSTER REARRANGEMENT
  # T is the name of the dictionary where we store our temp thruster vals. This is a dict and not a list so we can start at index 1.
  # This is advantageous because it means T[1] lines up with Thruster #1 on the ROV.
  T = { 1 : linearX + linearY + angularX, 2 : -linearX + linearY - angularX, 3 : -linearX - linearY + angularX,
        4 : linearX - linearY - angularX, 5 : linearZ, 6 : linearZ }
  T[1] = linearX + linearY + angularX
  T[2] = -linearX + linearY - angularX
  T[3] = -linearX - linearY + angularX
  T[4] = linearX - linearY - angularX
  T[5] = linearZ
  T[6] = linearZ
  
  # Do a little math to normalize the values
  max_motor = max(abs(T[1]), abs(T[2]), abs(T[3]), abs(T[4]), abs(T[5]), abs(T[6]))
  max_input = max(abs(linearX), abs(linearY), abs(linearZ), abs(angularX))
  if max_motor == 0:
    max_motor = 1

  for key in T:
    T[key] *= max_input / max_motor
  
  # Load up the thruster_vals message with our calculated values
  thruster_vals = thrusterPercents() # The amount we multiply each value by is just what we had in the original code. Subject to change

  thruster_vals.t1 = (int)(T[1] * 1000)
  thruster_vals.t2 = (int)(T[2] * 1000)
  thruster_vals.t3 = (int)(T[3] * 1000)
  thruster_vals.t4 = (int)(T[4] * 1000)
  thruster_vals.t5 = (int)(T[5] * 1000)
  thruster_vals.t6 = (int)(T[6] * 1000)
  
  return thruster_vals


def joystick_callback(joy):
  global joy_vector, sensitivity, thrustEN

  # If thrusters enabled, map the joystick inputs to the joy_vector
  if thrustEN:
    # Multiply LR axis by -1 in base position (front-front, etc.)to make right positive
    # NOTE: right and rotate right are negative on the joystick's LR axis
    l_axisLR = joy.axes[0] * sensitivity['linear'] * -1
    l_axisFB = joy.axes[1] * sensitivity['linear']
    a_axis = joy.axes[2] * sensitivity['angular'] * -1 

    # Apply the exponential ratio on all axis
    a_axis = expDrive(a_axis)
    l_axisLR = expDrive(l_axisLR)
    l_axisFB = expDrive(l_axisFB)

  else:
    a_axis = 0
    l_axisLR = 0
    l_axisFB = 0

  # Add the created vectors to the joy_vector
  joy_vector.linear.x = l_axisLR
  joy_vector.linear.y = l_axisFB
  joy_vector.angular.x = a_axis

  # Translate to thrusterPercents msg and publish
  thrust_percents = translate_vectors(joy_vector)
  vel_pub.publish(thrust_percents)

# Callback that runs whenever the throttle sends an update
def throttle_callback(joy):
  global thrustEN, joy_vector, sensitivity, dhEnable, depthHoldThrust

  # check if thrusters disabled
  if thrustEN:
    if dhEnable:
      v_axis = depthHoldThrust / 10
    
    else:
      v_axis = joy.axes[2] * sensitivity['vertical'] * -1

      v_axis = expDrive(v_axis)
  else:
    v_axis = 0

  joy_vector.linear.z = v_axis
  
  # Translate to thrusterPercents msg and publish
  thrust_percents = translate_vectors(joy_vector)
  vel_pub.publish(thrust_percents)


# Handles copilot input: updates thrusters, edits sensitivity
# Callback to anything published by the dynamic reconfigure copilot page
def control_callback(control):
  global thrustEN, sensitivity

  thrustEN = control.thruster_status
  sensitivity['linear'] = control.linear_sense
  sensitivity['angular'] = control.angular_sense
  sensitivity['vertical'] = control.vertical_sense

def auto_control_callback(auto_control):
  global dhEnable
  dhEnable = auto_control.dh_status

def depth_hold_callback(pid_effort):
  global depthHoldThrust
  depthHoldThrust = pid_effort.data

if __name__  == "__main__":
  global horizJoySub, vertJoySub, vel_pub, control_sub, auto_control_sub, depth_hold_sub
  horiz_joy_sub = rospy.Subscriber('joystick', Joy, joystick_callback)
  vert_joy_sub = rospy.Subscriber('throttle', Joy, throttle_callback)
  control_sub = rospy.Subscriber('control', controlData, control_callback)
  auto_control_sub = rospy.Subscriber('auto_control', autoControlData, auto_control_callback)
  depth_hold_sub = rospy.Subscriber('pid_effort', Float32, depth_hold_callback)

  vel_pub = rospy.Publisher('thrusters', thrusterPercents, queue_size=1)

    # Enter the event loop
    rospy.spin()
