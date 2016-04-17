from baxter_projectyouth import util

util.connect_to_baxter('baxter_learning_node_02')

# Maybe play the drum here, have it be painful...


# Baxter, put your arm straight out
# TODO Find the actual joint to move.
util.move_left_arm_joint('left_e0', 0.0)
util.move_left_arm_joint('left_e1', 0.0)
util.move_left_arm_joint('left_e2', 0.0)
util.move_left_arm_joint('left_e3', 0.0)
util.move_left_arm_joint('left_e4', 0.0)
util.move_left_arm_joint('left_e5', 0.0)

# Baxter, go back to position to get ready to wave
# TODO Find the actual joint to move.
util.move_left_arm_joint('left_e0', 1.0)
util.move_left_arm_joint('left_e1', 1.25)
util.move_left_arm_joint('left_e2', 0.0543)
util.move_left_arm_joint('left_e3', 2.189)
util.move_left_arm_joint('left_e4', 0.25)
util.move_left_arm_joint('left_e5', 0.987)

# Baxter, wave to us!
# TODO Find the actual joint to move.
util.move_left_arm_joint('left_e0', 1.3546)
util.move_left_arm_joint('left_e0', 1.0000)
