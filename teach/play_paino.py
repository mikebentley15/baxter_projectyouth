from baxter_projectyouth import util
import time

util.connect_to_baxter('baxter_learning_node_piano')

green_above  = [ 39.6, 20.0, -18.0, 25.1, -150.5, 45.0, 87.8 ]
yellow_above = [ 37.7, 20.0, -18.0, 25.6, -150.5, 45.0, 87.8 ]
red_above    = [ 35.9, 20.0, -18.0, 25.9, -150.5, 45.0, 87.8 ]
purple_above = [ 34.0, 20.0, -18.0, 26.2, -150.5, 45.0, 87.8 ]

green_down     = list(green_above)
yellow_down    = list(yellow_above)
red_down       = list(red_above)
purple_down    = list(purple_above)
green_down[5]  = 39.3
yellow_down[5] = 39.3
red_down[5]    = 39.3
purple_down[5] = 39.3

util.move_right_arm_to_positions(yellow_above)
util.set_right_arm_speed(0.6)
for i in range(5):
    #util.move_right_arm_joint('right_w1', 39.3)
    #time.sleep(0.2)
    #util.move_right_arm_joint('right_w1', 45.0)
    util.move_right_arm_to_positions(yellow_down)
    time.sleep(0.2)
    util.move_right_arm_to_positions(yellow_above)

util.register_note_right_arm('c', green_above, green_down)
util.register_note_right_arm('e', yellow_above, yellow_down)
util.register_note_right_arm('g', red_above, red_down)
util.register_note_right_arm('C', purple_above, purple_down)

util.play_notes_right_arm('cegC--cegC')
