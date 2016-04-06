'Utility functions for the main scripts.'

#
# System includes
#
from baxter_interface import CHECK_VERSION
import Queue
import baxter_interface
import re
import rospy
import select


#
# File-local variables
#

_button_presses = Queue.Queue(1)  # thread-safe place for button-press events

_L_BUTTON = 'left'         # Element to add to _button_presses for left  button
_R_BUTTON = 'right'        # Element to add to _button_presses for right button
_ROS_SHUTDOWN = 'shutdown' # Element to add to _button_presses for quit

def try_get_line(in_stream):
    'Returns a line if there is one, else an empty string'
    line = ''
    file_number = in_stream.fileno()
    timeout = 0.01 # seconds
    ready_to_read, _, _ = select.select([file_number], [], [], timeout)
    if file_number in ready_to_read:
        line = in_stream.readline()
    return line[:-1]  # Remove the newline

def connect_to_baxter():
    '''
    Connects to baxter, initializes this process as a node, and returns the two
    Limb objects for accessing and controlling the two Baxter arms as
    (left_arm, right_arm).
    '''
    rospy.init_node('my_node')
    baxter_enabler = baxter_interface.RobotEnable(CHECK_VERSION)
    baxter_enabler.enable()

    rospy.on_shutdown(_cleanup)

    left_button = baxter_interface.DigitalIO('left_itb_button0')
    left_button.state_changed.connect(_store_left_button_press)

    right_button = baxter_interface.DigitalIO('right_itb_button0')
    right_button.state_changed.connect(_store_right_button_press)

    left_arm = baxter_interface.Limb('left')
    right_arm = baxter_interface.Limb('right')

    return (left_arm, right_arm)

def _store_left_button_press(state):
    'Callback for left button press.  Stores _L_BUTTON into _button_presses.'
    if state:
        try:
            _button_presses.put_nowait(_L_BUTTON)
        except Queue.Full:
            pass # ignore that we haven't processed the last one yet

def _store_right_button_press(state):
    'Callback for right button press.  Stores _R_BUTTON into _button_presses.'
    if state:
        try:
            _button_presses.put_nowait(_R_BUTTON)
        except Queue.Full:
            pass # ignore that we haven't processed the last one yet

def _cleanup():
    'Callback for ROS shutdown.  Stores _ROS_SHUTDOWN into _button_presses'
    try:
        _button_presses.get(block=False)
    except Queue.Empty:
        pass # ignore
    _button_presses.put(_ROS_SHUTDOWN)

def wait_for_button_press():
    '''
    Waits for a button press signal or a ROS shutdown signal.  It will return
    'left' for the left button being pressed and 'right' for the right arm
    being pressed.  If there was a ROS shutdown signal, then this method will
    raise a rospy.ROSException.

    Note, this is a blocking call until one of the three things happen, so be
    sure you called connect_to_baxter().
    '''
    # Wait for one to be pushed
    one_year = 365 * 24 * 60 * 60
    which_button = _button_presses.get(timeout=one_year)
    if which_button == _ROS_SHUTDOWN:
        raise rospy.ROSException()
    return which_button

def save_joint_angles(filename, angles):
    '''
    Saves the angles dictionary to filename in python notation

    Will save it into an array of dictionaries called data.
    For example

    save_joint_angles('out.py', {'a': 1, 'b': 2})

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
    is_data_defined = False
    try:
        with open(filename, 'r') as infile:
            for line in infile:
                if re.match('data =', line):
                    is_data_defined = True
                    break
    except IOError:
        pass # Ignore the problem that the file doesn't yet exist
    with open(filename, 'a') as outfile:
        if not is_data_defined:
            outfile.write('data = []\n')
        outfile.write('\n')
        outfile.write('data.append({\n    ')
        outfile.write(',\n    '.join(split))
        outfile.write('\n')

