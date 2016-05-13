#!/usr/bin/env python

import lcm
import bot_core as lcmbotcore
import time
import sys
import numpy as np

myLCM = lcm.LCM()

percentage_closed = float(sys.argv[1])

msg = lcmbotcore.joint_angles_t()
msg.joint_name = ["leftIndexFingerMotorPitch1", "leftMiddleFingerMotorPitch1", "leftPinkyMotorPitch1", "leftThumbMotorPitch1", "leftThumbMotorPitch2"] #, "leftThumbMotorRoll"]
msg.num_joints = len(msg.joint_name)
grasp_map = np.array([ 1.0, 1.0, 1.0, 1.0, 1.0 ]) #, 1.0 ])
msg.joint_position = grasp_map * percentage_closed
#msg.joint_position[5] = 1.0

myLCM.publish("DESIRED_HAND_ANGLES", msg.encode())
