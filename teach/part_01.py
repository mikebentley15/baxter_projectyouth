from baxter_projectyouth import util

util.connect_to_baxter('baxter_learning_node_01')

# Baxter, wave to us!
# TODO Find the actual joint to move.
util.move_left_arm_joint('left_e0', 1.3546)
util.move_left_arm_joint('left_e0', 1.0000)

# wave again!
util.move_left_arm_joint('left_e0', 1.3546)
util.move_left_arm_joint('left_e0', 1.0000)

