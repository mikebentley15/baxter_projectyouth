from baxter_projectyouth import util

util.connect_to_baxter('baxter_learning_node_02')

# Maybe play the drum here, have it be painful...


# Baxter, put your arm straight out
util.move_left_arm_joint('left_s0', 0.0)
util.move_left_arm_joint('left_s1', 0.0)
util.move_left_arm_joint('left_e0', 0.0)
util.move_left_arm_joint('left_e1', 0.0)
util.move_left_arm_joint('left_w0', 0.0)
util.move_left_arm_joint('left_w1', 0.0)
util.move_left_arm_joint('left_w2', 0.0)

# Baxter, go back to position to get ready to wave
util.move_left_arm_joint('left_s0', -0.87)
util.move_left_arm_joint('left_s1',  0.82)
util.move_left_arm_joint('left_e0',  3.01)
util.move_left_arm_joint('left_e1',  2.46)
util.move_left_arm_joint('left_w0', -1.58)
util.move_left_arm_joint('left_w1',  0.00)
util.move_left_arm_joint('left_w2', -1.94)

# Baxter, wave to us!
util.move_left_arm_joint('left_w1', -0.4)
util.move_left_arm_joint('left_w1',  0.4)
util.move_left_arm_joint('left_w1', -0.4)
util.move_left_arm_joint('left_w1',  0.4)
