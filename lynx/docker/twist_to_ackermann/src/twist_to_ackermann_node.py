#!/usr/bin/env python

import rospy, math
import numpy
from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDriveStamped

def convert_trans_rot_vel_to_steering_angle(v, omega, wheelbase):
  if omega == 0 or v == 0:
    return 0

  radius = v / omega
  return math.atan(wheelbase / radius)


def cmd_callback(data):
  global wheelbase
  global ackermann_cmd_topic
  global frame_id
  global pub

  v = data.linear.x
  steering = convert_trans_rot_vel_to_steering_angle(v, data.angular.z, wheelbase) * 3

  msg = AckermannDriveStamped()
  msg.header.stamp = rospy.Time.now()
  msg.header.frame_id = frame_id

  msg.drive.steering_angle = steering
  msg.drive.speed = v

  msg.drive.steering_angle = data.angular.z*1.55
  msg.drive.speed = v

  data.angular.z = numpy.clip(data.angular.z, -0.36, 0.36)

  msg.drive.steering_angle = data.angular.z * (1.0 + (0.36 - abs(data.angular.z)))
  msg.drive.speed = v

  pub.publish(msg)


if __name__ == '__main__':
  try:

    rospy.init_node('twist_to_ackermann_node')

    twist_cmd_topic = '/twist_cmd'
    ackermann_cmd_topic = '/ackermann_cmd'
    wheelbase = rospy.get_param('~wheelbase', 0.16)
    frame_id = rospy.get_param('~frame_id', 'odom')

    rospy.Subscriber(twist_cmd_topic, Twist, cmd_callback, queue_size=1)
    pub = rospy.Publisher(ackermann_cmd_topic, AckermannDriveStamped, queue_size=1)


    rospy.spin()

  except rospy.ROSInterruptException:
    pass
