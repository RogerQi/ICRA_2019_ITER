# all parameters are measured in mm
# robot params according to rules
robot_width = 600
robot_length = 600
robot_height = 500

# arena params according to rules
arena_width = 2625 + 2375 # y
arena_length = 3500 + 4500 # x

# cell width (to scale)
cell_width = 10 # 10mm
cell_x_cnt = arena_length / cell_width
cell_y_cnt = arena_width / cell_width

obstacle_width = 2625-2375
obstacle_length = 2400-1400
