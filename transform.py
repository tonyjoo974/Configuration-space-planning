
# transform.py
# ---------------
# Licensing Information:  You are free to use or extend this projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to the University of Illinois at Urbana-Champaign
# 
# Created by Jongdeog Lee (jlee700@illinois.edu) on 09/12/2018

"""
This file contains the transform function that converts the robot arm map
to the maze.
"""
import copy
from arm import Arm
from maze import Maze
from search import *
from geometry import *
from const import *
from util import *

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
    angle_range = arm.getArmLimit()
    alpha_range = angle_range[0]
    beta_range = angle_range[1]
    start_angle = arm.getArmAngle()
    start_x = int((arm.getArmAngle()[0]-alpha_range[0])/granularity)
    start_y = int((arm.getArmAngle()[1]-beta_range[0])/granularity)
    if len(angle_range) > 3:
        gamma_range = angle_range[2]
    rows = int((alpha_range[1]-alpha_range[0])/granularity) + 1
    cols = int((beta_range[1]-beta_range[0])/granularity) + 1
    input_map = [[' ' for x in range(abs(cols))] for y in range(abs(rows))]
    offsets = []
    for x in angle_range:
        offsets.append(x[0])
    for i in range(rows):
        for j in range(cols):
            # transform each pair into angles
            angles = idxToAngle([i, j], offsets, granularity)
            arm.setArmAngle(angles)
            armEnd = arm.getEnd()
            armPostDist = arm.getArmPosDist()
            armPos = arm.getArmPos()
            if not isArmWithinWindow(armPos, window):
                input_map[i][j] = '%'
            elif doesArmTouchObjects(armPostDist, obstacles, False):
                input_map[i][j] = '%'
            elif doesArmTouchObjects(armPostDist, goals, True) and not doesArmTipTouchGoals(armEnd, goals):
                input_map[i][j] = '%'
            elif doesArmTipTouchGoals(armEnd, goals) and not doesArmTouchObjects(armPostDist, obstacles, False):
                input_map[i][j] = '.'

    input_map[start_x][start_y] = 'P'
    maze = Maze(input_map, offsets, granularity)
    return maze
