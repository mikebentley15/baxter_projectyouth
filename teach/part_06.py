from baxter_projectyouth import util

util.connect_to_baxter('baxter_learning_node_06')

# We need to teach the other arm to play the triangle.  Let's do the same thing
# we did here!
def play_triangle(speed, drum_count):
    left_position = [
        1.0,
        1.25,
        0.0543,
        2.189,
        0.25,
        0.987,
        ]
    right_position = [
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
    util.set_right_arm_speed(speed)
    for i in range(drum_count):
        util.move_right_arm_to_positions(left_position)
        util.move_right_arm_to_positions(right_position)
        util.move_right_arm_to_positions(down_position)

# Let's now use our new function!
play_triangle(0.6, 5)
play_triangle(0.4, 7)
play_triangle(1.0, 2)
play_triangle(0.7, 5)
