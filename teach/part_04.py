from baxter_projectyouth import util

util.connect_to_baxter('baxter_learning_node_04')

# We want to now teach Baxter to play the drum with his left hand
# TODO: Record placements of his hand and copy and paste from that file
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

# We want it to play the drum many times, and we don't want to have to put
# in a lot of copies of the same commands over and over again.
# How: LOOPS!
for i in range(10):
    util.move_left_arm_to_positions(up_position)
    util.move_left_arm_to_positions(down_position)

# Let's do it again, but faster
util.set_left_arm_speed(0.6)
for i in range(10):
    util.move_left_arm_to_positions(up_position)
    util.move_left_arm_to_positions(down_position)

# Faster still
util.set_left_arm_speed(0.8)
for i in range(10):
    util.move_left_arm_to_positions(up_position)
    util.move_left_arm_to_positions(down_position)

# Fastest
util.set_left_arm_speed(1.0)
for i in range(10):
    util.move_left_arm_to_positions(up_position)
    util.move_left_arm_to_positions(down_position)
