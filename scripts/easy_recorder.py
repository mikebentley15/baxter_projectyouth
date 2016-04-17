#!/usr/bin/env python
'''
Records joint positions of the two Baxter arms to files.  The recorded joint
positions are saved in a python-style file in an array called data.  These
joint positions can be used later for playback.
'''


# System includes
import rospy

# Local includes
from baxter_projectyouth import util


def main():
    'Main entry point.  Records joint angles on button press until CTRL-C.'

    # Connect to Baxter
    util.connect_to_baxter('easy_recorder')

    left_arm_filename = 'left_saved_joints.py'
    right_arm_filename = 'right_saved_joints.py'

    # Output some instructions
    print
    print 'This script records joint positions of the baxter arms into a python'
    print 'file.'
    print
    print 'Move the arms where you want them, then press the big round gray'
    print 'button on the arm to record those joint angles.'
    print
    print 'Joints will be saved to:'
    print '  Left arm: ', left_arm_filename
    print '  Right arm:', right_arm_filename
    print
    print 'When you are finished, press <CTRL>-C to quit'
    print

    while util.is_baxter_running():
        # Read button presses and see if we should record
        try:
            which_arm = util.wait_for_button_press()
        except rospy.ROSException:
            print 'exiting'
            break

        # See which arm to query and to which file to add
        if which_arm == 'left':
            joint_names = util.left_arm_joint_names()
            joint_angles = util.left_arm_joint_angles()
            current_filename = left_arm_filename
        elif which_arm == 'right':
            joint_names = util.right_arm_joint_names()
            joint_angles = util.right_arm_joint_angles()
            current_filename = right_arm_filename

        # Save the joint angles to file
        print 'Saving ' + which_arm + ' joint angles'
        util.save_joint_angles(current_filename, joint_names, joint_angles)


# This calls the main() function if this script is called instead of imported
if __name__ == '__main__':
    main()

