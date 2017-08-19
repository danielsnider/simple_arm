#!/usr/bin/python
import rospy
import serial
import struct
import numpy as np

from sensor_msgs.msg import Joy

class JoyArmSerial:
    def __init__(self):
        self.velocities = {
            'grip': 0,
            'wrist_roll': 0,
            'wrist_pitch': 0,
            'upper_elbow': 0,
            'lower_elbow': 0,
            'base_yaw': 0
        }
        self.positions = {
            'camera': 0
        }
        baudrate = rospy.get_param('~baudrate', 9600)
        self.serialDev = serial.Serial(baudrate=baudrate)
        self.serialDev.port = rospy.get_param("~serial_dev")
        self.serialDev.open()
        self.arm_sub = rospy.Subscriber("/joy_arm", Joy, controller.arm_joy_callback)

    def write_serial(self):
      # Execute arm position
        rospy.loginfo('velocities:%s\n' % self.velocities)
        rospy.loginfo('positions:%s\n' % self.positions)
        encoded_position = struct.pack("<fffffffff",
                                        self.velocities['grip'],
                                        self.velocities['wrist_roll'],
                                        self.velocities['wrist_pitch'],
                                        self.velocities['upper_elbow'],
                                        self.velocities['lower_elbow'],
                                        self.velocities['base_yaw'],
                                        self.positions['camera']
                                        )
        self.serialDev.write(encoded_position)

    def arm_joy_callback(self, data):
        self.velocities['grip'] = 0
        self.velocities['wrist_roll'] = 0
        self.velocities['wrist_pitch'] = 0
        self.velocities['upper_elbow'] = 0
        self.velocities['lower_elbow'] = 0
        self.velocities['base_yaw'] = 0

        # button 12 = double speed
        SPEED_MULTIPLIER = 1
        if data.buttons[11]:
            SPEED_MULTIPLIER = 2
        # button 10 = half speed
        if data.buttons[9]:
            SPEED_MULTIPLIER = 0.5

        # +big stick twist = base clockwise
        # -big stick twist = base counter-clockwise
        self.velocities['base_yaw'] = data.axes[2] / 5.0 * SPEED_MULTIPLIER

        # +big stick forward = big arm up
        # -big stick back = big arm down
        self.velocities['lower_elbow'] = data.axes[1] / 3.0 * SPEED_MULTIPLIER

        # thumb stick forward = small arm down
        # thumb stick back = small arm up
        self.velocities['upper_elbow'] = data.axes[5] / 5.0 * SPEED_MULTIPLIER

        # paddle forward = arm camera up
        # paddle back = arm camera down
        self.positions['camera'] = np.interp(data.axes[3],[-1,1],[7,77]) # this value is angular postion in degrees

        # button 9 = wrist clockwise
        # button 11 = wrist counter-clockwise
        if data.buttons[8]:
            self.velocities['wrist_roll'] = 0.35 * SPEED_MULTIPLIER
        if data.buttons[10]:
            self.velocities['wrist_roll'] = -0.35 * SPEED_MULTIPLIER

        # button 6 = wrist down
        # button 4 = wrist up
        if data.buttons[4]:
            self.velocities['wrist_pitch'] = 0.15 * SPEED_MULTIPLIER
        if data.buttons[2]:
            self.velocities['wrist_pitch'] = -0.15 * SPEED_MULTIPLIER

        # trigger = close gripper
        # thumb button = open gripper
        if data.buttons[0]:
            self.velocities['grip'] = 1
        if data.buttons[1]:
            self.velocities['grip'] = -1

        # MOVE ARM
        self.write_serial()


def main():
    rospy.init_node("joystick_teleoperation_arm")
    controller = JoyArmSerial()
    rospy.spin()
