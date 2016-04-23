'Utility functions for the main scripts.'

#
# System includes
#
from baxter_interface import CHECK_VERSION
import Queue
import baxter_dataflow
import baxter_interface
import math
import re
import rospy
import select
import time


#
# File-local variables
#

_button_presses = Queue.Queue(1)  # thread-safe place for button-press events

_L_BUTTON = 'left'         # Element to add to _button_presses for left  button
_R_BUTTON = 'right'        # Element to add to _button_presses for right button
_ROS_SHUTDOWN = 'shutdown' # Element to add to _button_presses for quit

_left_arm = None
_left_joints = None
_right_arm = None
_right_joints = None

def connect_to_baxter(nodename):
    '''
    Connects to baxter, and initializes this process as a node.

    This function should be called before other functions that control Baxter.

    Try to think of a unique nodename to give this connection.
    '''
    global _left_arm
    global _right_arm
    rospy.init_node(nodename)
    baxter_enabler = baxter_interface.RobotEnable(CHECK_VERSION)
    baxter_enabler.enable()

    rospy.on_shutdown(_cleanup)

    left_button = baxter_interface.DigitalIO('left_itb_button0')
    left_button.state_changed.connect(_store_left_button_press)

    right_button = baxter_interface.DigitalIO('right_itb_button0')
    right_button.state_changed.connect(_store_right_button_press)

    _left_arm = baxter_interface.Limb('left')
    _right_arm = baxter_interface.Limb('right')

def is_baxter_running():
    return not rospy.is_shutdown()

def set_left_arm_speed(ratio):
    '''
    ratio is a number between 0 and 1.  0 means stop, 1 means maximum speed,
    0.5 means half speed.
    '''
    assert _left_arm is not None, 'You need to call connect_to_baxter() first'
    _left_arm.set_joint_position_speed(ratio)

def set_right_arm_speed(ratio):
    '''
    ratio is a number between 0 and 1.  0 means stop, 1 means maximum speed,
    0.5 means half speed
    '''
    assert _right_arm is not None, 'You need to call connect_to_baxter() first'
    _right_arm.set_joint_position_speed(ratio)

def left_arm_joint_angles():
    '''
    Returns a list of the joint angles in the same order as
    left_arm_joint_names() in degrees
    '''
    assert _left_arm is not None, 'You need to call connect_to_baxter() first'
    return _arm_joint_angles(_left_arm)

def right_arm_joint_angles():
    '''
    Returns a list of the joint angles in the same order as
    right_arm_joint_names()
    '''
    assert _right_arm is not None, 'You need to call connect_to_baxter() first'
    return _arm_joint_angles(_right_arm)

def left_arm_joint_names():
    '''
    Returns a list of the joint names starting from the shoulder to the wrist
    of the left Baxter arm
    '''
    assert _left_arm is not None, 'You need to call connect_to_baxter() first'
    return _left_arm.joint_names()

def right_arm_joint_names():
    '''
    Returns a list of the joint names starting from the shoulder to the wrist
    of the right Baxter arm
    '''
    assert _right_arm is not None, 'You need to call connect_to_baxter() first'
    return _right_arm.joint_names()

def move_left_arm_joint(joint, angle):
    '''
    Move one joint from Baxter's left arm.

    @param joint: the name of the joint to move
    @param angle: The angle to move it to in degrees
    '''
    assert _left_arm is not None, 'You need to call connect_to_baxter() first'
    _move_to_positions(_left_arm, {joint: angle})

def move_right_arm_joint(joint, angle):
    '''
    Move one joint from Baxter's left arm.

    @param joint: the name of the joint to move
    @param angle: The angle to move it to
    '''
    assert _right_arm is not None, 'You need to call connect_to_baxter() first'
    _move_to_positions(_right_arm, {joint: angle})

def move_left_arm_to_positions(positions):
    '''
    Move Baxter's left arm to the given positions.

    @param positions: A list of joint angles for the joints starting at the
                      shoulder and going to the wrist (same order of joint
                      names)
    '''
    assert _left_arm is not None, 'You need to call connect_to_baxter() first'
    positions_dictionary = dict(zip(_left_arm.joint_names(), positions))
    _move_to_positions(_left_arm, positions_dictionary)

def move_right_arm_to_positions(positions):
    '''
    Move Baxter's right arm to the given positions.

    @param positions: A list of joint angles for the joints starting at the
                      shoulder and going to the wrist (same order of joint
                      names)
    '''
    assert _right_arm is not None, 'You need to call connect_to_baxter() first'
    positions_dictionary = dict(zip(_right_arm.joint_names(), positions))
    _move_to_positions(_right_arm, positions_dictionary)

def try_get_line(in_stream):
    'Returns a line if there is one, else an empty string'
    line = ''
    file_number = in_stream.fileno()
    timeout = 0.01 # seconds
    ready_to_read, _, _ = select.select([file_number], [], [], timeout)
    if file_number in ready_to_read:
        line = in_stream.readline()
    return line[:-1]  # Remove the newline

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

def save_joint_angles(filename, names, angles, digits=1):
    '''
    Saves the angles dictionary to filename in python notation

    Will save it into a list of lists called path.
    For example

    save_joint_angles('out.py', ['a', 'b'], [1, 2])

    will write to 'out.py' something like:
    path = []
    joint_names = [
        'a',
        'b',
        ]
    positions = [
        1,
        2,
        ]
    path.append(positions)
    '''
    # Check for first-time initialization
    is_path_defined = False
    is_joint_names_defined = False
    try:
        with open(filename, 'r') as infile:
            for line in infile:
                if re.match('path =', line):
                    is_path_defined = True
                if re.match('joint_names =', line):
                    is_joint_names_defined = True
    except IOError:
        pass # Ignore the problem that the file doesn't yet exist

    with open(filename, 'a') as outfile:
        if not is_path_defined:
            outfile.write('path = []\n')
        if not is_joint_names_defined:
            outfile.write('joint_names = [\n    "')
            outfile.write('",\n    "'.join(names))
            outfile.write('"\n    ]\n')
        outfile.write('\n')
        outfile.write('positions = [\n    ')
        format_str = '{0:' + str(digits+5) + '.0' + str(digits) + 'f}'
        outfile.write(',\n    '.join([format_str.format(x) for x in angles]))
        outfile.write('\n    ]\n')
        outfile.write('path.append(positions)\n')

def register_note_right_arm(note, above_position, down_position):
    _register_note('right', note, above_position, down_position)

def register_note_left_arm(note, above_position, down_position):
    _register_note('left', note, above_position, down_position)

def play_notes_right_arm(notes):
    _play_notes('right', notes)

def play_notes_left_arm(notes):
    _play_notes('left', notes)

## Private functions

def _rad_to_deg(angle):
    'Convert radians to degrees'
    return angle * 180 / math.pi

def _deg_to_rad(angle):
    'Convert radians to degrees'
    return angle * math.pi / 180

def _arm_joint_angles(arm):
    '''
    Returns a list of the joint angles in the same order as
    arm.joint_names() in degrees
    '''
    positions_dict = arm.joint_angles()
    joint_names = arm.joint_names()
    angles = [_rad_to_deg(positions_dict[joint]) for joint in joint_names]
    return angles

_left_note_above_positions = {}
_left_note_down_positions = {}
_right_note_above_positions = {}
_right_note_down_positions = {}
def _register_note(arm, note, above_position, down_position):
    if arm == 'left':
        above_dict = _left_note_above_positions
        down_dict = _left_note_down_positions
    else:
        above_dict = _right_note_above_positions
        down_dict = _right_note_down_positions
    above_dict[note] = above_position
    down_dict[note] = down_position

def _play_notes(arm, notes):
    if arm == 'left':
        above_dict = _left_note_above_positions
        down_dict = _left_note_down_positions
        mover = move_left_arm_to_positions
    else:
        above_dict = _right_note_above_positions
        down_dict = _right_note_down_positions
        mover = move_right_arm_to_positions
    for note in notes:
        if note == '-':
            time.sleep(0.5)
            continue
        mover(above_dict[note])
        time.sleep(0.1)
        mover(down_dict[note])
        time.sleep(0.1)
        mover(above_dict[note])
        

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

def _move_to_positions(limb, pos_dict):
    '''
    Moves the limb to the desired positions.

    @param limb - The baxter_interface.Limb object to move
    @param pos_dict - Dictionary of joint -> angle positions in degrees

    Note, to change how fast it goes, you can call

    >>> limb.set_joint_position_speed(ratio)

    where ratio is a number between 0 and 1 with 1 being pretty fast and 0
    being stopped.
    '''
    # Convert to radians first thing
    for key in pos_dict:
        pos_dict[key] = _deg_to_rad(pos_dict[key])

    cmd = limb.joint_angles()
    def filtered_cmd():
        # First Order Filter - 0.2 Hz Cutoff
        for joint, value in pos_dict.iteritems():
            cmd[joint] = 0.07 * value + 0.93 * cmd[joint]
        return cmd

    diff = lambda joint, angle: abs(angle - limb.joint_angle(joint))
    #threshold = baxter_interface.settings.JOINT_ANGLE_TOLERANCE
    # We want a bigger threshold so that it doesn't wiggle at the end, but
    # says "good enough" and goes to the next point
    threshold = 0.02
    timeout = 15.0 # seconds

    # Otherwise, go there without the filter
    #limb.set_joint_positions(filtered_cmd())
    limb.set_joint_positions(pos_dict)
    baxter_dataflow.wait_for(
        lambda: (all(diff(j, a) < threshold for j, a in pos_dict.iteritems())),
        timeout=timeout,
        rate=100,
        raise_on_error=False,
        body=lambda: limb.set_joint_positions(pos_dict)
        )
