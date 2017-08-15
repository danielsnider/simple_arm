#!/usr/bin/python
import time
import rospy
import serial
import struct
import numpy as np

from sensor_msgs.msg import Joy
from std_msgs.msg import Float32

class JoyArmSerial:
    PI = 3.14159265359

    def __init__(self,r = 0,i = 0):
        self.serialDev = serial.Serial()
        self.serialDev.port = "/dev/ttyACM0" # SET SERIAL DEVICE
        self.serialDev.open()

        self.velocities = {
            'wrist_roll': 0,
            'wrist_pitch': 0,
            'small_arm': 0,
            'big_arm': 0,
            'base_yaw': 0,
            'grip': 0,
            'winch': 0,
            'zed_camera': 1500
        }
        self.positions = {
            'arm_camera': 0,
            'zed_camera': 0
        }

    def write_serial(self):
        # MOTOR_VEL_OFFSET = -0.1 # use this when the motors don't respond to this value
        # for key in ['wrist_roll', 'wrist_pitch', 'small_arm', 'big_arm', 'base_yaw']:
        #     self.velocities[key] = self.velocities[key] + MOTOR_VEL_OFFSET

        rospy.loginfo('velocities:%s\n' % self.velocities)
        rospy.loginfo('positions:%s\n' % self.positions)

      # Execute arm position
        encoded_position = struct.pack("<fffffffff",
                                        self.velocities['wrist_roll'],
                                        self.velocities['wrist_pitch'],
                                        self.velocities['small_arm'],
                                        self.velocities['big_arm'],
                                        self.velocities['base_yaw'],
                                        self.velocities['grip'],
                                        self.velocities['winch'],
                                        self.velocities['zed_camera'],
                                        # self.positions['zed_camera'],
                                        self.positions['arm_camera']
                                        )
        self.serialDev.write(encoded_position)

    def zed_servo_callback(self, data):
        # Zed servo panning control
        # pan leftward (left bumper)
        # pan rightward (right bumper)

        self.velocities['zed_camera'] = 1500
        if data.buttons[5]: # pan leftward (left bumper)
            self.velocities['zed_camera'] += 1500
        if data.buttons[4]: # pan rightward (right bumper)
            self.velocities['zed_camera'] -= 1500

        # MOVE
        self.write_serial()

    def zed_servo_180_callback(self, data):
        # Zed servo panning control
        # pan leftward (left bumper)
        # pan rightward (right bumper)
        self.positions['zed_camera'] = data.data # this value is angular postion in degrees

        # MOVE
        self.write_serial()


    def arm_joy_callback(self, data):
        self.velocities['wrist_roll'] = 0
        self.velocities['wrist_pitch'] = 0
        self.velocities['small_arm'] = 0
        self.velocities['big_arm'] = 0
        self.velocities['base_yaw'] = 0
        self.velocities['grip'] = 0
        self.velocities['winch'] = 0

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
        self.velocities['big_arm'] = data.axes[1] / 3.0 * SPEED_MULTIPLIER

        # thumb stick forward = small arm down
        # thumb stick back = small arm up
        self.velocities['small_arm'] = data.axes[5] / 5.0 * SPEED_MULTIPLIER

        # paddle forward = arm arm_camera up
        # paddle back = arm arm_camera down
        self.positions['arm_camera'] = np.interp(data.axes[3],[-1,1],[7,77]) # this value is angular postion in degrees

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

        # button 7 & 8 = winch
        if data.buttons[6]:
            self.velocities['winch'] = 1
        if data.buttons[7]:
            self.velocities['winch'] = -1
        if data.buttons[6] and data.buttons[7]:
            self.velocities['winch'] = 1337 # special flag to free-wheel

        # MOVE ARM
        self.write_serial()


controller = JoyArmSerial()
rospy.init_node("joystick_teleoperation_arm")
arm_sub = rospy.Subscriber("/joy_arm", Joy, controller.arm_joy_callback)
zed_servo_sub = rospy.Subscriber("/joy", Joy, controller.zed_servo_callback) # continuous
# zed_servo_sub = rospy.Subscriber("/zed_servo", Float32, controller.zed_servo_180_callback) # 180

rospy.spin()
