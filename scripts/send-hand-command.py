#!/usr/bin/env python

import lcm
import bot_core as lcmbotcore
import time
import sys

myLCM = lcm.LCM()

joint_to_update_id = int(sys.argv[1])

msg = lcmbotcore.joint_angles_t()
msg.joint_name = ["rightIndexFingerMotorPitch1", "rightMiddleFingerMotorPitch1", "rightPinkyMotorPitch1", "rightThumbMotorPitch1", "rightThumbMotorPitch2", "rightThumbMotorRoll"]
# msg.joint_name = ["rightMiddleFingerMotorPitch1"] #, "rightPinkyMotorPitch1", "rightThumbMotorPitch1", "rightThumbMotorPitch2"] #, "rightThumbMotorRoll"]
# msg.joint_name = ["rightThumbMotorRoll"]
msg.num_joints = len(msg.joint_name)
msg.joint_position = [0] * len(msg.joint_name)

msg.joint_position[joint_to_update_id] = float(sys.argv[2])
print msg.joint_name[joint_to_update_id] + " going to " + str(msg.joint_position[joint_to_update_id])
myLCM.publish("DESIRED_HAND_ANGLES", msg.encode())
