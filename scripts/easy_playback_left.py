#!/usr/bin/env python
'''
Plays back recorded joint positions of the left of the two Baxter arms.  The
recorded joint positions are from a python-style file in an array called data.
'''


# System includes
import os
import sys

# Local includes
from baxter_projectyouth import util

def main():
    'Main entry point.  Plays recorded joint angles in a loop until CTRL-C.'

    # Connect to Baxter
    util.connect_to_baxter('easy_playback_left')

    sys.path.append(os.path.realpath(os.curdir))
    import left_saved_joints

    # Output some instructions
    print
    print 'This script plays recorded joint positions recorded from the'
    print 'easy_recorder.py script.  These joint positions are of the baxter'
    print 'arms and store in a python file.  The playback will continue in a'
    print 'loop until you press CTRL-C.'
    print
    print 'You have chosen to playback the left arm from left_saved_joints.py.'
    print
    print 'When you are finished, press <CTRL>-C to quit'
    print

    while util.is_baxter_running():
        for positions in left_saved_joints.path:
            util.move_left_arm_to_positions(positions)


# This calls the main() function if this script is called instead of imported
if __name__ == '__main__':
    main()
