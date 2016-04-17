from baxter_projectyouth import util

util.connect_to_baxter('baxter_learning_node_02')

# Let's now put these places in a list
# because we see the function util.move_left_arm_to_positions()
straight_out = [
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    ]
# TODO Find the actual joint to move.
wave_position = [
    1.0,
    1.25,
    0.0543,
    2.189,
    0.25,
    0.987,
    ]

# Baxter, put your arm straight out
util.move_left_arm_to_positions(straight_out)

# Baxter, go back to position to get ready to wave
util.move_left_arm_to_positions(wave_position)

# Baxter, wave to us!
# TODO Find the actual joint to move.
util.move_left_arm_joint('left_e0', 1.3546)
util.move_left_arm_joint('left_e0', 1.0000)
