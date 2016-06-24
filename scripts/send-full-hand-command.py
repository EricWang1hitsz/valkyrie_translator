#!/usr/bin/env python

import lcm
import bot_core as lcmbotcore
import time
import sys

def print_usage_and_exit():
	print "Usage: "
	print "\t send-full-hand-command <side> <Index> <Middle> <Pinky> <Thumb1> <Thumb2> <ThumbRoll>"
	print "\t <side> = 'left' or 'right'"
	exit(0)

myLCM = lcm.LCM()

joints_of_interest = ["IndexFingerMotorPitch1", "MiddleFingerMotorPitch1", "PinkyMotorPitch1", "ThumbMotorPitch1", "ThumbMotorPitch2", "ThumbMotorRoll"]
# msg.joint_name = ["rightMiddleFingerMotorPitch1"] #, "rightPinkyMotorPitch1", "rightThumbMotorPitch1", "rightThumbMotorPitch2"] #, "rightThumbMotorRoll"]
# msg.joint_name = ["rightThumbMotorRoll"]

if (len(sys.argv) != 2 + len(joints_of_interest)):
	print "Argument error: Incorrect # of args."
	print_usage_and_exit() 
if (sys.argv[1] != "left" and sys.argv[1] != "right"):
	print "Argument error: <side> must be 'left' or 'right'"
	print_usage_and_exit()

side = sys.argv[1]

msg = lcmbotcore.joint_angles_t()
msg.joint_name = [side + jointname for jointname in joints_of_interest]
msg.num_joints = len(msg.joint_name)
msg.joint_position = [0] * len(joints_of_interest)

for i, jointname in enumerate(joints_of_interest):
	msg.joint_position[i] = float(sys.argv[2+i])
	print "Joint %s going to %f" % (jointname, msg.joint_position[i])

myLCM.publish("DESIRED_HAND_ANGLES", msg.encode())
