#!/usr/bin/env python3
# author Alex Bertran | v0.0 | 11-11-2022
# Current imports are all those from drive_control.py and some may not be necessary
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

from std_msgs.msg import UInt8  # For camera  pub
from std_msgs.msg import Bool  # For TCU relay and solenoid controller pub and for pids
from std_msgs.msg import Float64  # For pids
from simple_pid import PID  # Also for pids
from std_msgs.msg import Float32
from std_msgs.msg import Int32
from nav_msgs.msg import Odometry
from copilot_interface.msg import controlData # control custom message

from math import copysign
from dynamic_reconfigure.server import Server
from copilot_interface.cfg import copilotControlParamsConfig

rospy.init_node("autonomous_control")

# defaults for variables
thrustEN = False
dhEnable = False
targetDepth = 0.0
pid = PID(1, 0.1, 0.05, setpoint=1)

#def controlCallback(config, level):
  #global thrustEN, dhEnable, p_scalar, i_scalar, d_scalar
  
  #thrustEN = config.thrusters
  #p_scalar = config.p_scalar
  #i_scalar = config.i_scalar
  #d_scalar = config.d_scalar
  #dhEnable = config.dh_enable
  
  #return config

def ROS_INFO_STREAM(thrustEN):
  pass

# update variables with data from "/control" topic
def rovDataCallback(data):
  global thrustEN, dhEnable, targetDepth
  thrustEN = data.thruster_status
  ROS_INFO_STREAM(thrustEN)
  dhEnable = data.dh_status
  targetDepth = data.target_depth

def depthHoldCallback(data):
  global p_scalar, i_scalar, d_scalar
    
  p_scalar = data.p_scalar
  i_scalar = data.i_scalar
  d_scalar = data.d_scalar

# ???
def dhStateCallback(data):
  global dhMostRecentDepth
  
  if dhEnable: # only update depth if depth hold is disabled {dhEnable == False}
    dhMostRecentDepth = data.pose.pose.position.z * -1
    depth = Float64()
    depth.data = dhMostRecentDepth
    dh_setpoint_pub.publish(depth)
  
def dhControlEffortCallback(data): # no need for dhEnable check since PIDs won't publish control effort when disabled
  global dh_eff
  
  dh_eff = data.data
  
def change_depth_callback(depth):
  global dhEnable, thrustEN, targetDepth, test_pub, pid
 
  if thrustEN and dhEnable:
    # calibration of pressure sensor
    #currentDepth = abs((depth.data - 198.3) / (893.04 / 149))
    currentDepth = abs(depth.data * 5)
    depthError = currentDepth - targetDepth
    calculation = pid(depthError)
    rospy.loginfo(calculation)
  
def main():
  global thruster_status_sub, depth_hold_sub, dh_state_sub, dh_ctrl_eff_sub, dh_toggle_sub, depth_sub, test_pub, control_status_sub
  
  test_pub = rospy.Publisher('/rov/thruster_testing', Int32, queue_size=1)
  #depth_hold_sub = rospy.Subscriber('depth_hold/pid_enable', PID, depthHoldCallback)
  #dh_state_sub = rospy.Subscriber('odometry/filtered', Odometry, dhStateCallback)
  dh_ctrl_eff_sub = rospy.Subscriber('depth_hold/control_effort', Float64, dhControlEffortCallback)
  depth_sub = rospy.Subscriber('rov/depth_sensor', Float32, change_depth_callback)
  control_status_sub = rospy.Subscriber('control', controlData, rovDataCallback)
  
  # creates another gui, use different one for possible depth hold gui or just delete
  #server = Server(copilotControlParamsConfig, controlCallback)
  
  rospy.spin()

if __name__  == "__main__":
    main()
