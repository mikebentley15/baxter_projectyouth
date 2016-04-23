from baxter_projectyouth import util

util.connect_to_baxter('baxter_learning_node_03')

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
wave_position = [
    -0.87,
     0.82,
     3.01,
     2.46,
    -1.59,
     0.00,
    -1.94
    ]

# Baxter, put your arm straight out
util.move_left_arm_to_positions(straight_out)

# Baxter, go back to position to get ready to wave
util.move_left_arm_to_positions(wave_position)

# Baxter, wave to us!
util.move_left_arm_joint('left_w1', -0.4)
util.move_left_arm_joint('left_w1',  0.4)
