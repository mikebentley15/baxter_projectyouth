import sys
import select
import re
import rospy
import baxter_interface
import time

from util import connectToBaxter, waitForButtonPress, saveJointAngles

def main():
    ''
    # Connect to Baxter
    leftArm, rightArm = connectToBaxter()

    # Record arm positions
    print 'Press <CTRL>-C to quit'
    while not rospy.is_shutdown():
        # Read button presses and see if we should record
        try:
            whichArm = waitForButtonPress()
        except rospy.ROSException:
            print 'exiting'
            break
        currentArm = leftArm if whichArm == 'left' else rightArm
        jointAngles = currentArm.joint_angles()
        print 'Saving ' + whichArm + ' joint angles'
        saveJointAngles(whichArm + '_saved_joints.py', jointAngles)

if __name__ == '__main__':
    sys.exit(main())

