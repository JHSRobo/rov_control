#!/usr/bin/env python3
# author Alex Bertran '24 | v0.0 | 11-11-2022 | prototyping autonomous infrastructure
# author Alex Bertran '24 | v1.0 | 1-12-2023 | functional depth hold fully integrated
#import sys
#import os
#sys.path.remove(os.path.dirname(__file__))

import rospy
from geometry_msgs.msg import Twist # may be necessary in the future, delete after 3/1/2023
from simple_pid import PID  # For pids
from std_msgs.msg import Float32  # For depth sensor
from copilot_interface.msg import autoControlData, controlData# /auto_control custom message

rospy.init_node("autonomous_control")

# defaults for variables
thrustEN = False  # Thruster toggle status
dhEnable = False  # Depth hold toggle status
targetDepth = 7.5  # Target depth for depth hold
p_scalar = 1.0  # PID: proportional
i_scalar = 0.1  # PID: integral
d_scalar = 0.05  # PID: derivative

# Create new pid object with scalars and target point
pid = PID(p_scalar, i_scalar, d_scalar, setpoint=targetDepth)
pid.output_limits = (-10, 10)

# update variables with data from "/control" topic
def rovDataCallback(data):
  global thrustEN
  thrustEN = data.thruster_status

# update variables with data from "/auto_control" topic
def autoDataCallback(data):
  global dhEnable, targetDepth, p_scalar, i_scalar, d_scalar, pid
  dhEnable = data.dh_status
  targetDepth = data.target_depth
  p_scalar = data.p_scalar
  i_scalar = data.i_scalar
  d_scalar = data.d_scalar
  
  # Slightly redundant code but other variables should stay in case of future use
  pid.setpoint = targetDepth
  pid.tunings = (p_scalar, i_scalar, d_scalar)
  
def change_depth_callback(depth):
  global dhEnable, thrustEN, targetDepth, pid, pid_pub
 
  if thrustEN and dhEnable:
    # calibration of pressure sensor | REMEMBER TO KEEP UP TO DATE
    #currentDepth = abs((depth.data - 198.3) / (893.04 / 149))
    currentDepth = abs((depth.data * 3.281) + 1.94) # Convert to feet and calibrate
    calculation = pid(currentDepth)
    pid_pub.publish(calculation)
  
def main():
  global depth_sub, control_status_sub, auto_control_status_sub, pid_pub
  
  depth_sub = rospy.Subscriber('rov/depth_sensor', Float32, change_depth_callback)
  control_status_sub = rospy.Subscriber('control', controlData, rovDataCallback)
  auto_control_status_sub = rospy.Subscriber('auto_control', autoControlData, autoDataCallback)
  
  # Depth hold PID value publisher
  pid_pub = rospy.Publisher('pid_effort', Float32, queue_size=1)
  
  rospy.spin()

if __name__  == "__main__":
    main()
