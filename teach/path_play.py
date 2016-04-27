from baxter_projectyouth import util
from right_saved_joints import path

util.connect_to_baxter('path_play')

util.set_right_arm_speed(1.0)

for i in range(0, 5):
    # Green
    util.move_right_arm_to_positions(path[3])
    util.move_right_arm_to_positions(path[4])

    # Red
    util.move_right_arm_to_positions(path[3])
    util.move_right_arm_to_positions(path[5])

