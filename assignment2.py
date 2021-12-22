#!/usr/bin/env python
import numpy as np
import rospy
import Manager as mn

GOOD_LINE = 5
TH = 7
WALL = 80
SPIRAL_SIZE = 30
RADIUS = 10  # to check if in center of sphere area
COVER_RADIUS = 13  # to mark sphere around it
SPHERE_SIZE = 20
LINE_INDICATE = 10
SPHERE_INDICATE = 80
WALL_DIST = 5  # same as manager param!
SHIFT_INDICATE = 12  # for indicate shifted circle
DIFF = 3
CLOSE_CENTER = 8

BIGGER_SQUARE = 6


class Cleaning:
    def __init__(self):
        self.controller = mn.Manager()
        self.MyMap = self.controller.ms.map_arr
        self.m_big_room_list = []
        self.MaxMove = max(self.controller.ms.shape[0], self.controller.ms.shape[1])

    def step(self, i, j):
        dis_from_wall = self.GetDistance(i, j, self.controller.cur_pos[0], self.controller.cur_pos[1])
        estimate_dest = np.array(
            [self.controller.cur_pos[0] + i * SPIRAL_SIZE, self.controller.cur_pos[1] + j * SPIRAL_SIZE])
        NonCoverDist = dis_from_wall - SPIRAL_SIZE
        if NonCoverDist <= 0 or NonCoverDist < SPHERE_SIZE:
            return [self.MaxMove, [estimate_dest[0], estimate_dest[1]]]
        return [dis_from_wall, [estimate_dest[0], estimate_dest[1]]]

    def GetDistance(self, i, j, row, col):
        distance = 0
        if self.MyMap[row][col] < 0:
            chck = 0
            while chck < TH:
                if self.controller.Valid(row + i * chck, col + j * chck) and self.MyMap[row + i * chck][
                    col + j * chck] == 0:
                    row, col = row + i * chck, col + j * chck
                    distance = distance + chck
                    break
                chck = chck + 1
            if chck >= TH:
                return distance
        while self.controller.Valid(row, col) and 0 <= self.MyMap[row][col] < WALL:
            distance = distance + 1
            col = col + j
            row = row + i
        return distance

    def Mark(self, x_dest, y_dest):
        current_square = SPIRAL_SIZE
        while current_square > 0:
            if self.controller.InMyArea(x_dest, y_dest, current_square, self.controller.cur_pos) == True:
                current_square = current_square - 1
                continue
            break
        if current_square <= WALL_DIST:
            return
        current_square = current_square - WALL_DIST
        i = -current_square
        while i < current_square:
            i = i + 1
            j = -current_square
            while j < current_square:
                j = j + 1
                row = self.controller.cur_pos[0] + i
                column = self.controller.cur_pos[1] + j
                if self.controller.Valid(row, column) and self.MyMap[row][column] == 0:
                    self.MyMap[row][column] = -1

    def GetBestDist(self, direction, dest):
        i, j = -direction[0], -direction[1]
        cur_row, cur_col = dest[0], dest[1]
        wall_other_side = SPIRAL_SIZE * 2
        for step in range(SPIRAL_SIZE - WALL_DIST):
            if self.controller.Valid(cur_row + i * step, cur_col + j * step) and self.MyMap[cur_row + i * step][
                cur_col + j * step] != 0:
                wall_other_side = step
                break
        ahead = min(SPIRAL_SIZE - WALL_DIST, int(wall_other_side // 2))
        if ahead < WALL_DIST:
            self.Mark(cur_row, cur_col)
            return [self.controller.cur_pos[0], self.controller.cur_pos[1]]
        return [cur_row + i * ahead, cur_col + j * ahead]

    def ZerosWallCheck(self, row, col):
        directions = [(1, 0), (0, 1), (-1, 0), (0, -1)]
        for direction in directions:
            for i in range(2 * WALL_DIST):
                if self.controller.Valid(row + direction[0] * i, col + direction[1] * i) and \
                        self.MyMap[row + direction[0] * i][col + direction[1] * i] != 0:
                    return [True, direction]
        return [False, (0, 0)]

    def FindBestZero(self):
        for row in range(self.controller.ms.shape[0]):
            for col in range(self.controller.ms.shape[1]):
                if self.controller.Valid(col, row) == True and self.MyMap[col][row] == 0 and self.controller.InMyArea(col, row,
                                                                                        SPIRAL_SIZE,
                                                                                        self.controller.cur_pos) == False and \
                        self.ZerosWallCheck(col, row)[0] == False:
                    return [col, row]
        return [self.controller.cur_pos[0], self.controller.cur_pos[1]]
    def Move(self):
        left = mn.Direction('left', self.step(0, -1), (0, -1))
        right = mn.Direction('right', self.step(0, 1), (0, 1))
        up = mn.Direction('up', self.step(1, 0), (1, 0))
        down = mn.Direction('down', self.step(-1, 0), (-1, 0))
        DirectionList = [left, right, up, down]
        DirectionList.sort(key=mn.Direction.getVal)
        if DirectionList[0].value == self.MaxMove:
            next_move = self.FindBestZero()


def vacuum_cleaning():
    print('start vacuum_cleaning')
    raise NotImplementedError


def inspection():
    print('start inspection')
    raise NotImplementedError


def inspection_advanced():
    print('start inspection_advanced')
    raise NotImplementedError


# If the python node is executed as main process (sourced directly)
if __name__ == '__main__':
    print('start inspection')
    check = Cleaning()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        first = check.firstMove()
        rate.sleep()
    rospy.spin()
