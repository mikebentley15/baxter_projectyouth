from baxter_projectyouth import util, common_positions

util.connect_to_baxter('baxter_learning_node_01')

util.set_left_arm_speed(0.7)

util.move_left_arm_to_positions(common_positions.left_positions['wave'])


# Baxter, wave to us!
while util.is_baxter_running():
    util.move_left_arm_joint('left_w1', -30.0)
    util.move_left_arm_joint('left_w1',  30.0)

