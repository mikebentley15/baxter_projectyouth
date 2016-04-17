from baxter_projectyouth import util

util.connect_to_baxter('baxter_learning_node_02')

# With teaching Baxter to play the drums, we want an easy way to play the drums
# at a certain speed.  Let's make a FUNCTION!
def play_drum(speed, drum_count):
    up_position = [
        1.0,
        1.25,
        0.0543,
        2.189,
        0.25,
        0.987,
        ]
    down_position = [
        1.0,
        1.25,
        0.0543,
        2.189,
        0.25,
        0.987,
        ]
    util.set_left_arm_speed(speed)
    for i in range(drum_count):
        util.move_left_arm_to_positions(up_position)
        util.move_left_arm_to_positions(down_position)


# Let's now use our new function!
play_drum(0.6, 5)
play_drum(0.4, 7)
play_drum(1.0, 2)
play_drum(0.7, 5)
