#!/usr/bin/env python
'''
Plays back recorded joint positions of one of the two Baxter arms.  The
recorded joint positions are from a python-style file in an array called data.
'''


# System includes
import os
import rospy
import sys

# Local includes
from baxter_projectyouth import util


def print_usage():
    'Prints usage information to the console'
    print 'Usage:'
    print '    {0} <which arm>'.format(__file__)
    print '    {0} --help'
    print
    print 'Description:'
    print '    This script will playback joints saved in the file from the'
    print '    easy_recorder.py script, which is'
    print '        left_save_joints.py (for saving from the left arm)'
    print '        right_save_joints.py (for saving from the rightarm)'
    print
    print '    It will play this in a loop until CTRL-C is pressed by the user.'
    print
    print 'Arguments:'
    print '    <which arm> - This is either "left" or "right" indicating'
    print '                  which arm to control.'
    print
    print '    --help - This option prints this information and exits.'
    print

def main(arguments):
    'Main entry point.  Plays recorded joint angles in a loop until CTRL-C.'

    # Argument parsing
    if '-h' in arguments or '--help' in arguments:
        print_usage()
        return 0

    which_arm = arguments[0]

    # Connect to Baxter
    util.connect_to_baxter('easy_playback_' + which_arm)

    sys.path.append(os.path.realpath(os.curdir))
    if which_arm == 'left':
        import left_saved_joints as saved_joints
    elif which_arm == 'right':
        import right_saved_joints as saved_joints
    else:
        sys.stderr.write('Error: Not a valid first argument: ' + which_arm \
                + '\n')
        print_usage()
        return 1

    # Output some instructions
    print
    print 'This script plays recorded joint positions recorded from the'
    print 'easy_recorder.py script.  These joint positions are of the baxter'
    print 'arms and store in a python file.  The playback will continue in a'
    print 'loop until you press CTRL-C.'
    print
    print 'You have chosen to playback the ' + \
            '{0} arm from {0}_saved_joints.py.'.format(which_arm)
    print
    print 'When you are finished, press <CTRL>-C to quit'
    print

    while not rospy.is_shutdown():
        for positions in saved_joints.path:
            if which_arm == 'left':
                util.move_left_arm_to_positions(positions)
            elif which_arm == 'right':
                util.move_right_arm_to_positions(positions)

    return 0


# This calls the main() function if this script is called instead of imported
if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))

