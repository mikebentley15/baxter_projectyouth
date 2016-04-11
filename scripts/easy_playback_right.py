#!/usr/bin/env python
'''
Plays back recorded joint positions of the right of the two Baxter arms.  The
recorded joint positions are from a python-style file in an array called data.
'''


# System includes
import os
import rospy
import sys

# Local includes
from baxter_projectyouth import util

def main(arguments):
    'Main entry point.  Plays recorded joint angles in a loop until CTRL-C.'

    # Connect to Baxter
    _, right_arm = util.connect_to_baxter('easy_playback_right')

    sys.path.append(os.path.realpath(os.curdir))
    import right_saved_joints

    # Output some instructions
    print
    print 'This script plays recorded joint positions recorded from the'
    print 'easy_recorder.py script.  These joint positions are of the baxter'
    print 'arms and store in a python file.  The playback will continue in a'
    print 'loop until you press CTRL-C.'
    print
    print 'You have chosen to playback the right arm from right_saved_joints.py.'
    print
    print 'When you are finished, press <CTRL>-C to quit'
    print

    while not rospy.is_shutdown():
        util.play_path(right_arm, right_saved_joints.data)


# This calls the main() function if this script is called instead of imported
if __name__ == '__main__':
    main(sys.argv[1:])
