#!/usr/bin/python

import rospy
import sensor_msgs.msg
import serial
import struct

RECOGNIZED_ARM = ['hip', 'shoulder', 'elbow', 'lower_elbow']
RECOGNIZED_GRIPPER = ['wrist', 'grip']
ALL_RECOGNIZED_JOINTS = []
ALL_RECOGNIZED_JOINTS.extend(RECOGNIZED_ARM)
ALL_RECOGNIZED_JOINTS.extend(RECOGNIZED_GRIPPER)


def on_new_joint(data):
  # For better readability and robustness, convert ROS joint message
  # to dict so that joint values can be access by name instead of by index.
  joint_position = {}
  for idx, name in enumerate(data.name):
    joint_position[name] = data.position[idx]

  # Execute arm position
  if data.name[0:4] == RECOGNIZED_ARM:
    encoded_position = struct.pack("<Bffff", 1,
                                    joint_position['shoulder'],
                                    joint_position['elbow'],
                                    joint_position['lower_elbow'],
                                    joint_position['hip'])
    theSerial.write(encoded_position)
    rospy.loginfo('Executed arm position %s'% data)

  # Execute gripper position
  if data.name[4:6] == RECOGNIZED_GRIPPER:
    encoded_position = struct.pack("<Bff", 3,
                                     joint_position['wrist'],
                                     joint_position['grip'])
    theSerial.write(encoded_position)
    rospy.loginfo('Executed gripper position %s'% data)

  unknown_joints = [joint for joint in data.name if joint not in ALL_RECOGNIZED_JOINTS]
  if unknown_joints:
    rospy.logwarn('Ignoring unknown joints: %s' % unknown_joints)

theSerial = serial.Serial()
theSerial.port = "/dev/ttyACM0" # SET SERIAL DEVICE
theSerial.open()

rospy.init_node("joint_sender")
subscriber = rospy.Subscriber("/joint_states", sensor_msgs.msg.JointState, on_new_joint, queue_size=10)
rospy.spin()


