#!/usr/bin/env python
# BEGIN ALL
import rospy, numpy as np, math
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, Imu


global wall_follow
wall_follow = False
global turn_command
turn_command = 0

def scan_callback(msg):
  global wall_follow, turn_command
  sight = len(msg.ranges)
  right_end = msg.ranges[1]
  left_end = msg.ranges[-1]

  if not wall_follow:
      turn_command = 0
      if np.nanmin(msg.ranges)<2.0:
          if (not np.isnan(left_end)) or np.isnan(right_end):
              turn_command = 1
          else:
            wall_follow = True
            turn_command = -1
  else:
      if not np.isnan(right_end):
          if msg.ranges[1] < 1.2:
              turn_command = 1
          else:
              if msg.ranges[1] > 1.5:
                  turn_command = -1
              else:
                  turn_command = 0
      else:
          turn_command = -2

def imu_callback(msg):
    global orient
    imu_msg = msg.orientation
    orient = np.arctan2(imu_msg.z,imu_msg.w)*2

scan_sub = rospy.Subscriber('scan', LaserScan, scan_callback)
imu_sub = rospy.Subscriber('/mobile_base/sensors/imu_data', Imu, imu_callback)
cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
rospy.init_node('wander')
state_change_time = rospy.Time.now()
driving_forward = True

rate = rospy.Rate(10)


while not rospy.is_shutdown():
  if driving_forward:
    # BEGIN FORWARD
    if (abs(turn_command)>0.5 or rospy.Time.now() > state_change_time):
      driving_forward = False
      state_change_time = rospy.Time.now() + rospy.Duration(2)
    # END FORWARD
    
  else: # we're not driving_forward
    # BEGIN TURNING
    if rospy.Time.now() > state_change_time:
      driving_forward = True # we're done spinning, time to go forwards!
      state_change_time = rospy.Time.now() + rospy.Duration(10)
    # END TURNING
  twist = Twist()
  if driving_forward:
    twist.linear.x = 0.8
  else:
    if abs(turn_command)<1.5:
        twist.linear.x = .25
        twist.angular.z = 0.5*turn_command
    else:
        twist.linear.x = 0.25
        twist.angular.z = 0.075*turn_command

  cmd_vel_pub.publish(twist)

  rate.sleep()
# END ALL
