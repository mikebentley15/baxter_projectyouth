import sys
import select
import re
import rospy
import baxter_interface
from baxter_interface import CHECK_VERSION

import Queue

_buttonPresses = Queue.Queue(1)

_lButton = None
_rButton = None

_L_BUTTON = 'left'
_R_BUTTON = 'right'
_ROS_SHUTDOWN = 'shutdown'

def tryGetLine(inStream):
    'Returns a line if there is one, else an empty string'
    line = ''
    fd = inStream.fileno()
    timeout = 0.01 # seconds
    readyToRead, _, _ = select.select([fd], [], [], timeout)
    if fd in readyToRead:
        line = inStream.readline()
    return line[:-1]  # Remove the newline

def connectToBaxter():
    global _lButton, _rButton
    rospy.init_node('my_node')
    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    rs.enable()
    rospy.on_shutdown(_onShutdown)
    _lButton = baxter_interface.DigitalIO('left_itb_button0')
    _rButton = baxter_interface.DigitalIO('right_itb_button0')
    _lButton.state_changed.connect(_storeLeftButtonPress)
    _rButton.state_changed.connect(_storeRightButtonPress)
    rArm = baxter_interface.Limb('right')
    lArm = baxter_interface.Limb('left')
    return (lArm, rArm)

def _storeLeftButtonPress(state):
    global _buttonPresses
    if state:
        try:
            _buttonPresses.put_nowait(_L_BUTTON)
        except Queue.Full:
            pass # ignore that we haven't processed the last one yet

def _storeRightButtonPress(state):
    global _buttonPresses
    if state:
        try:
            _buttonPresses.put_nowait(_R_BUTTON)
        except Queue.Full:
            pass # ignore that we haven't processed the last one yet

def _onShutdown():
    global _buttonPresses
    try:
        _buttonPresses.get(block=False)
    except Queue.Empty:
        pass # ignore
    _buttonPresses.put(_ROS_SHUTDOWN)

def waitForButtonPress():
    global _buttonPresses
    # Wait for one to be pushed
    ONEYEAR = 365 * 24 * 60 * 60
    whichButton = _buttonPresses.get(timeout=ONEYEAR)
    if whichButton == _ROS_SHUTDOWN:
        raise rospy.ROSException()
    return whichButton

def saveJointAngles(filename, angles):
    '''
    Saves the angles dictionary to filename in python notation

    Will save it into an array of dictionaries called data.
    For example

    saveJointAngles('out.py', {'a': 1, 'b': 2})

    will write to 'out.py':
    data = []
    data.append({
        'a': 1,
        'b': 2,
        })
    '''
    string = str(angles)
    string = string.replace('{', '')
    string = string.replace('}', '')
    split = string.split(', ')
    split.append('})')
    isDataDefined = False
    try:
        with open(filename, 'r') as infile:
            for line in infile:
                if re.match('data =', line):
                    isDataDefined = True
                    break
    except IOError:
        pass # Ignore the problem that the file doesn't yet exist
    with open(filename, 'a') as outfile:
        if not isDataDefined:
            outfile.write('data = []\n')
        outfile.write('\n')
        outfile.write('data.append({\n    ')
        outfile.write(',\n    '.join(split))
        outfile.write('\n')

