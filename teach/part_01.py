from baxter_projectyouth import util

util.connect_to_baxter('baxter_learning_node_01')

# Baxter, wave to us!
util.move_left_arm_joint('left_w1', -0.4)
util.move_left_arm_joint('left_w1',  0.4)

# wave again!
util.move_left_arm_joint('left_w1', -0.4)
util.move_left_arm_joint('left_w1',  0.4)

