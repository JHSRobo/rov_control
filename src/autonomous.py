#!/usr/bin/env python3
# author Alex Bertran '24 | v0.0 | 11-11-2022 | prototyping autonomous infrastructure
# author Alex Bertran '24 | v1.0 | 1-12-2023 | functional depth hold fully integrated
# author Alex Bertran '24 | v1.1 | 5-6-2023 | automonous docking for the 22-23 season
#import sys
#import os
#sys.path.remove(os.path.dirname(__file__))

import rospy
from geometry_msgs.msg import Twist # may be necessary in the future, delete after 3/1/2023
from simple_pid import PID  # for pids
from std_msgs.msg import Float32  # for depth sensor
from copilot_interface.msg import autoControlData, controlData # /auto_control custom message
from rov_control.msg import autoDock
from rov_control.msg import autoDockPID

rospy.init_node("autonomous_control")

# defaults for variables
thrustEN = False  # Thruster toggle status
dhEnable = False  # Depth hold toggle status
adEnable = False  # Auto dock toggle status
targetDepth = 7.5  # Target depth for depth hold
dh_p_scalar = 1.0  # PID: proportional
dh_i_scalar = 0.1  # PID: integral
dh_d_scalar = 0.05  # PID: derivative

# auto docking variables
adTarget = 0.0
adEffort = autoDockPID()
ad_x_p_scalar = 1.0
ad_x_i_scalar = 0.1
ad_x_d_scalar = 0.05

ad_y_p_scalar = 1.0
ad_y_i_scalar = 0.1
ad_y_d_scalar = 0.05

# create new pid object with scalars and target point
dh_pid = PID(dh_p_scalar, dh_i_scalar, dh_d_scalar, setpoint=targetDepth)
dh_pid.output_limits = (-10, 10)
ad_x_pid = PID(ad_x_p_scalar, ad_x_i_scalar, ad_x_d_scalar, setpoint=adTarget)
ad_y_pid = PID(ad_y_p_scalar, ad_y_i_scalar, ad_y_d_scalar, setpoint=adTarget)
ad_x_pid.output_limits = (-10, 10)
ad_y_pid.output_limits = (-10, 10)

# update variables with data from "/control" topic
def rovDataCallback(data):
  global thrustEN
  thrustEN = data.thruster_status

# update variables with data from "/auto_control" topic
def autoDataCallback(data):
  global dhEnable, adEnable, targetDepth, dh_p_scalar, dh_i_scalar, dh_d_scalar, dh_pid
  dhEnable = data.dh_status
  adEnable = data.auto_dock
  targetDepth = data.target_depth
  dh_p_scalar = data.p_scalar
  dh_i_scalar = data.i_scalar
  dh_d_scalar = data.d_scalar
  
  # slightly redundant code but other variables should stay in case of future use
  dh_pid.setpoint = targetDepth
  dh_pid.tunings = (dh_p_scalar, dh_i_scalar, dh_d_scalar)
  
def change_depth_callback(depth):
  global dhEnable, thrustEN, targetDepth, dh_pid, dh_pid_pub
 
  if thrustEN and dhEnable:
    # calibration of pressure sensor | REMEMBER TO KEEP UP TO DATE
    #currentDepth = abs((depth.data - 198.3) / (893.04 / 149))
    currentDepth = abs(depth.data * 3.281) # Convert to feet and calibrate
    calculation = dh_pid(currentDepth)
    dh_pid_pub.publish(calculation)

def autoDockCallback(data):
  global thrustEN, adTarget, ad_x_pid, ad_y_pid, ad_pid_pub
  
  if thrustEN and adEnable:
        adEffort.x_eff = ad_x_pid(data.ad_x_error)
        adEffort.y_eff = ad_y_pid(data.ad_y_error)
        ad_pid_pub.publish(adEffort)

def main():
  global depth_sub, control_status_sub, auto_control_status_sub, auto_dock_sub, dh_pid_pub, ad_pid_pub
  
  depth_sub = rospy.Subscriber('rov/depth_sensor', Float32, change_depth_callback)
  control_status_sub = rospy.Subscriber('control', controlData, rovDataCallback)
  auto_control_status_sub = rospy.Subscriber('auto_control', autoControlData, autoDataCallback)
  auto_dock_sub = rospy.Subscriber('auto_dock_data', autoDock, autoDockCallback)
  
  # PID value publishers
  dh_pid_pub = rospy.Publisher('dh_pid_effort', Float32, queue_size=1)
  ad_pid_pub = rospy.Publisher('ad_pid_effort', autoDockPID, queue_size=1)
  
  rospy.spin()

if __name__  == "__main__":
    main()
