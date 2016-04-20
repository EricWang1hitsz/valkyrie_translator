#!/usr/bin/env python

import lcm
import bot_core as lcmbotcore
import time
import sys

myLCM = lcm.LCM()

msg = lcmbotcore.joint_angles_t()
msg.joint_name = ["rightForearmYaw", "rightWristRoll", "rightWristPitch"]

msg.num_joints = len(msg.joint_name)
msg.joint_position = [0] * len(msg.joint_name)

msg.joint_position[0] = float(sys.argv[1])
msg.joint_position[1] = float(sys.argv[2])
msg.joint_position[2] = float(sys.argv[3])

print msg.joint_name[0] + " going to " + str(msg.joint_position[0])
myLCM.publish("DESIRED_FOREARM_ANGLES", msg.encode())
