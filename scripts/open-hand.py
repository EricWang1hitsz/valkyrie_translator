#!/usr/bin/env python

import lcm
import bot_core as lcmbotcore
import time
import sys

myLCM = lcm.LCM()

msg = lcmbotcore.joint_angles_t()
msg.joint_name = ["rightIndexFingerMotorPitch1", "rightMiddleFingerMotorPitch1", "rightPinkyMotorPitch1", "rightThumbMotorPitch1", "rightThumbMotorPitch2", "rightThumbMotorRoll"]
msg.num_joints = len(msg.joint_name)
msg.joint_position = [0, 0, 0, 0, 0, 0] #1]

myLCM.publish("DESIRED_HAND_ANGLES", msg.encode())
