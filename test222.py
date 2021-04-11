def transformToMaze(arm, goals, obstacles, window, granularity):
    """This function transforms the given 2D map to the maze in MP1.
    
        Args:
            arm (Arm): arm instance
            goals (list): [(x, y, r)] of goals
            obstacles (list): [(x, y, r)] of obstacles
            window (tuple): (width, height) of the window
            granularity (int): unit of increasing/decreasing degree for angles

        Return:
            Maze: the maze instance generated based on input arguments.

    """

    # row = alpha, col = beta
    # num of position = max - min / grannularity + 1

    # initial alpha, beta = starting point
    # alpha and beta at which the arm tip touches the goals would be the goals in the maze.
    # alpha and beta at which the arm cannot reach would be the walls in the maze.
    # construct walls and goals, you need to use geometry functions defined in Part 1
    #print("start transform")
    limit = arm.getArmLimit()
    
    alpha_min = limit[0][0]
    alpha_max = limit[0][1]

    beta_min = limit[1][0]
    beta_max = limit[1][1]

    alpha_diff = alpha_max - alpha_min
    beta_diff = beta_max - beta_min
    row = int(alpha_diff / granularity) + 1
    col = int(beta_diff / granularity) + 1
    #print(row, col)
    
    map = []

    map = [[SPACE_CHAR for x in range(col)] for y in range(row)]
    
    # not work idk why since greeksforgreeks says samething to initilize
    #map = [[SPACE_CHAR]*col]*row
    
    start_point = angleToIdx(arm.getArmAngle(), [alpha_min, beta_min], granularity)
    map[start_point[0]][start_point[1]] = START_CHAR

    angle_idx_x = alpha_min
    angle_idx_y = beta_min
    #test = 0
    while angle_idx_x <= alpha_max:
        while angle_idx_y <= beta_max:
            #print(angle_idx_x, angle_idx_y)
            current_angle = (angle_idx_x, angle_idx_y)
            arm.setArmAngle(current_angle)
            current_idx = angleToIdx(current_angle, (alpha_min, beta_min), granularity)

            if doesArmTouchObjects(arm.getArmPosDist(), obstacles, False):
                map[current_idx[0]][current_idx[1]] = WALL_CHAR
            elif doesArmTipTouchGoals(arm.getEnd(), goals):
                #test += 1
                #print("first", test)
                map[current_idx[0]][current_idx[1]] = OBJECTIVE_CHAR
            elif doesArmTouchObjects(arm.getArmPosDist(), goals, True):
                #print("found objective")
                #test += 1
                #print("second", test)
                map[current_idx[0]][current_idx[1]] = OBJECTIVE_CHAR
            elif not isArmWithinWindow(arm.getArmPos(), window):
                map[current_idx[0]][current_idx[1]] = WALL_CHAR
            angle_idx_y += granularity
        
        angle_idx_x += granularity
        angle_idx_y = beta_min
    