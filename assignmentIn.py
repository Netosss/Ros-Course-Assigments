#!/usr/bin/env python

import rospy
import numpy as np
from nav_msgs.msg import OccupancyGrid, MapMetaData, Odometry
import Manager as mn
import matplotlib.pyplot as plt
from copy import deepcopy
from nav_msgs.msg import OccupancyGrid
from map_msgs.msg import OccupancyGridUpdate
from math import dist
from datetime import datetime
import threading
import time

GOOD_LINE = 5
TH = 7
WALL = 80
SEEN_SIZE = 30
RADIUS = 10  # to check if in center of sphere area
COVER_RADIUS = 13  # to mark sphere around it
SPHERE_SIZE = 20
LINE_INDICATE = 10
SPHERE_INDICATE = 80
WALL_DIST = 5  # same as manager param!
SHIFT_INDICATE = 12  # for indicate shifted circle
DIFF = 3
CLOSE_CENTER = 8


# mohammad code
class CostmapUpdater:
    def __init__(self):
        self.cost_map = None
        self.shape = None
        rospy.Subscriber('/move_base/global_costmap/costmap', OccupancyGrid, self.init_costmap_callback)
        rospy.Subscriber('/move_base/global_costmap/costmap_updates', OccupancyGridUpdate, self.costmap_callback_update)

    def init_costmap_callback(self, msg):
        self.shape = msg.info.height, msg.info.width
        self.cost_map = np.array(msg.data).reshape(self.shape)

    def costmap_callback_update(self, msg):
        shape = msg.height, msg.width
        data = np.array(msg.data).reshape(shape)
        self.cost_map[msg.y:msg.y + shape[0], msg.x: msg.x + shape[1]] = data
        # self.show_map()  # For the student to see that it works

    def show_map(self):
        if not self.cost_map is None:
            plt.imshow(self.cost_map)
            plt.show()


class SphereHandler:
    def __init__(self, manager: mn.Manager):
        self.cmu = CostmapUpdater()
        self.ms = mn.MapService()
        self.m_founds_spheres = []  # center of exposed spheres
        self.m_manager = manager


    def Count(self, row, col, direction):
        counter = 0
        temp_row_top = row
        temp_col_top = col
        # first we want to get the far point at the row\column that have sphere indicate
        while self.m_manager.Valid(temp_row_top, temp_col_top) == True and self.cmu.cost_map[temp_row_top][
            temp_col_top] > SPHERE_INDICATE and self.m_manager.WallCheck(temp_row_top, temp_col_top)[0] == False:
            temp_row_top = temp_row_top + direction[0]
            temp_col_top = temp_col_top + direction[1]
        temp_row_down = temp_row_top - direction[0]
        temp_col_down = temp_col_top - direction[1]
        while self.m_manager.Valid(temp_row_down, temp_col_down) == True and self.cmu.cost_map[temp_row_down][
            temp_col_down] > SPHERE_INDICATE and self.m_manager.WallCheck(temp_row_down, temp_col_down)[0] == False:
            counter = counter + 1
            temp_row_down = temp_row_down - direction[0]
            temp_col_down = temp_col_down - direction[1]
        center = [int((temp_row_top + temp_row_down) // 2), int((temp_col_down + temp_col_down) // 2)]
        return counter, center

    def AddSphere(self, _row, _col):
        directions = [[1, 0], [0, 1]]
        left = False
        for direction in directions:
            opposite_direction = self.m_manager.OppositeDirection(direction)
            cur_counter, cur_center = self.Count(_row, _col, direction)
            if cur_counter < GOOD_LINE:
                continue
            left_counter, _ = self.Count(cur_center[0] + opposite_direction[0],
                                         cur_center[1] + opposite_direction[1], direction)
            right_counter, _ = self.Count(cur_center[0] - opposite_direction[0],
                                          cur_center[1] - opposite_direction[1], direction)
            if 1 < left_counter - cur_counter <= DIFF and 1 < cur_counter - right_counter <= DIFF:
                while self.m_manager.Valid(cur_center[0], cur_center[1]) and self.cmu.cost_map[cur_center[0]][
                    cur_center[1]] > SPHERE_INDICATE:
                    cur_center = (cur_center[0] - opposite_direction[0], cur_center[1] - opposite_direction[1])
                    left = True
            elif 1 < right_counter - cur_counter <= DIFF and 1 < cur_counter - left_counter <= DIFF:
                while self.m_manager.Valid(cur_center[0], cur_center[1]) and self.cmu.cost_map[cur_center[0]][
                    cur_center[1]] > SPHERE_INDICATE:
                    cur_center = (cur_center[0] + opposite_direction[0], cur_center[1] + opposite_direction[1])
            else:
                return
            if left == True:
                center = [
                    cur_center[0] + RADIUS * (opposite_direction[0]),
                    cur_center[1] + RADIUS * (opposite_direction[1])]
            else:
                center = [
                    cur_center[0] - RADIUS * (opposite_direction[0]),
                    cur_center[1] - RADIUS * (opposite_direction[1])]
            self.m_founds_spheres.append(center)

    def Optimize(self, sphere_center1, sphere_center2):
        if sphere_center1 in self.m_founds_spheres and sphere_center2 in self.m_founds_spheres:
            if dist([sphere_center1[0], sphere_center1[1]],
                    [sphere_center2[0], sphere_center2[1]]) <= DIFF:
                new_center = [int((sphere_center1[0] + sphere_center2[0]) // 2),
                              int((sphere_center1[1] + sphere_center2[1]) // 2)]
                self.m_founds_spheres.remove(sphere_center1)
                self.m_founds_spheres.remove(sphere_center2)
                self.m_founds_spheres.append(new_center)
            elif self.m_manager.InMyArea(sphere_center1[0], sphere_center1[1], COVER_RADIUS, sphere_center2):
                sphere1_counter = 0
                sphere2_counter = 0
                x_1, x_2, y_1, y_2 = 0, 0, 0, 0
                for direct in [[0, 1], [1, 0], [0, -1], [-1, 0]]:
                    for i in range(RADIUS):
                        x_1 = sphere_center1[0] + i * direct[0]
                        y_1 = sphere_center1[1] + i * direct[1]
                        if self.m_manager.Valid(x_1, y_1) and self.cmu.cost_map[x_1][y_1] > SPHERE_INDICATE:
                            if self.m_manager.WallCheck(x_1, y_1)[0] == False:
                                sphere1_counter = sphere1_counter + 1
                        x_2 = sphere_center2[0] + i * direct[0]
                        y_2 = sphere_center2[1] + i * direct[1]
                        if self.m_manager.Valid(x_2, y_2) and self.cmu.cost_map[x_2][y_2] > SPHERE_INDICATE:
                            if self.m_manager.WallCheck(x_2, y_2)[0] == False:
                                sphere2_counter = sphere2_counter + 1
                for direct in [[1, 1], [-1, 1], [-1, -1], [1, -1]]:
                    for i in range(COVER_RADIUS // 2):
                        x_1 = sphere_center1[0] + i * direct[0]
                        y_1 = sphere_center1[1] + i * direct[1]
                        if self.m_manager.Valid(x_1, y_1) and self.cmu.cost_map[x_1][y_1] > SPHERE_INDICATE:
                            if self.m_manager.WallCheck(x_1, y_1)[0] == False:
                                sphere1_counter = sphere1_counter + 1
                        x_2 = sphere_center2[0] + i * direct[0]
                        y_2 = sphere_center2[1] + i * direct[1]
                        if self.m_manager.Valid(x_2, y_2) and self.cmu.cost_map[x_2][y_2] > SPHERE_INDICATE:
                            if self.m_manager.WallCheck(x_2, y_2)[0] == False:
                                sphere2_counter = sphere2_counter + 1
                if sphere1_counter > sphere2_counter:
                    self.m_founds_spheres.remove(sphere_center2)
                else:
                    self.m_founds_spheres.remove(sphere_center1)

    # if they far and on the same circle, we take the one with more data(more inidicates near her)
    def UpdateWholeMap(self):
        cur_map = np.subtract(self.cmu.cost_map, self.ms.map_arr)
        # changed SEEN_SIZE TO WHOLE MAP IF ZERO MOVE WAS USED.
        for i in range(self.m_manager.ms.shape[0]):
            row = i
            for j in range(self.m_manager.ms.shape[1]):
                column = j
                if self.m_manager.Valid(row, column) and cur_map[row][column] > SPHERE_INDICATE:
                    if self.m_manager.WallCheck(row, column)[0] == True:
                        self.WallHandler(row, column, cur_map)
                    elif self.OldSphere(row,
                                        column) == False:  # if it is new sphere we add it, otherwise shift and cont
                        self.AddSphere(row, column)
        [self.Optimize(p1, p2) for p1 in self.m_founds_spheres for p2 in self.m_founds_spheres if p1 != p2]

    def UpdateMap(self):
        # we can look just at local or 60X60 map at the global from cur pos.
        # current_global_cm = deepcopy(self.cmu.cost_map) #maybe need to copy for un-update
        cur_map = np.subtract(self.cmu.cost_map, self.ms.map_arr)
        # changed SEEN_SIZE TO WHOLE MAP IF ZERO MOVE WAS USED.
        i = -SEEN_SIZE - SPHERE_SIZE
        while i < SEEN_SIZE + SPHERE_SIZE:
            i = i + 1
            j = -SEEN_SIZE - SPHERE_SIZE
            while j < SEEN_SIZE + SPHERE_SIZE:
                j = j + 1
                row = self.m_manager.cur_pos[0] + i
                column = self.m_manager.cur_pos[1] + j
                if self.m_manager.Valid(row, column) and cur_map[row][column] > SPHERE_INDICATE:
                    if self.m_manager.WallCheck(row, column)[0] == True:
                        self.WallHandler(row, column, cur_map)
                    elif self.OldSphere(row,
                                        column) == False:  # if it is new sphere we add it, otherwise shift and cont
                        self.AddSphere(row, column)
        [self.Optimize(p1, p2) for p1 in self.m_founds_spheres for p2 in self.m_founds_spheres if p1 != p2]

    def WallHandler(self, _row, _col, cur_map):
        directions = [(1, 0), (0, 1), (-1, 0), (0, -1)]
        for direction in directions:
            cur_row = _row
            cur_col = _col
            while self.m_manager.Valid(cur_row, cur_col) and cur_map[cur_row][cur_col] > SPHERE_INDICATE:
                cur_map[cur_row][cur_col] = 0
                cur_row = _row + direction[0]
                cur_col = _col + direction[1]

    def PointShifter(self, source_x, shifed_x):
        if source_x < shifed_x:
            return 1
        elif source_x == shifed_x:
            return 0
        else:
            return -1

    # if the new point is close to found src we should shift the old center closer to the point
    def OldSphere(self, _row, _col):
        for sphere in self.m_founds_spheres:
            if self.m_manager.InMyArea(_row, _col, RADIUS, sphere):
                return True
            if self.m_manager.InMyArea(_row, _col, SPHERE_SIZE, sphere):
                eDistance = dist([_row, _col], [sphere[0], sphere[1]])
                if RADIUS + 1 < eDistance <= SHIFT_INDICATE:
                    shifted_src = [
                        sphere[0] + self.PointShifter(sphere[0], _row), sphere[1] + self.PointShifter(sphere[1], _col)]
                    self.m_founds_spheres.remove(sphere)
                    self.m_founds_spheres.append(shifted_src)
                    return True
        return False


class Inspect:

    def __init__(self):
        self.controller = mn.Manager()
        self.MyMap = self.controller.ms.map_arr
        self.lastCoord = None
        self.MaxMove = max(self.controller.ms.shape[0], self.controller.ms.shape[1])
        self.notAvailAble = []
        self.m_sph_handler = SphereHandler(self.controller)
        self.first = True

    def ZerosWallCheck(self, row, col):
        directions = [(1, 0), (0, 1), (-1, 0), (0, -1)]
        for direction in directions:
            for i in range(2 * WALL_DIST):
                if self.controller.Valid(row + direction[0] * i, col + direction[1] * i) and \
                        self.MyMap[row + direction[0] * i][col + direction[1] * i] != 0:
                    return [True, direction]
        return [False, (0, 0)]

    # calculate the closest wall with given direction and given position
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

    def printSpheres(self):
        while (True):
            spheres = len(self.m_sph_handler.m_founds_spheres)
            ct = datetime.now()
            ts = ct.timestamp()
            print(spheres, "spheres detected at time", ts)
            time.sleep(30)

    def printit(self):
        threading.Timer(0.0, self.printSpheres).start()

    def step(self, i, j):
        dis_from_wall = self.GetDistance(i, j, self.controller.cur_pos[0], self.controller.cur_pos[1])
        estimate_dest = np.array(
            [self.controller.cur_pos[0] + i * SEEN_SIZE, self.controller.cur_pos[1] + j * SEEN_SIZE])
        NonCoverDist = dis_from_wall - SEEN_SIZE
        if NonCoverDist <= 0 or NonCoverDist < SPHERE_SIZE:
            return [self.MaxMove, [estimate_dest[0], estimate_dest[1]]]
        return [dis_from_wall, [estimate_dest[0], estimate_dest[1]]]

    def Mark(self, x_dest, y_dest):
        current_square = SEEN_SIZE
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

    def FindBestZero(self):
        for row in range(self.controller.ms.shape[0]):
            for col in range(self.controller.ms.shape[1]):
                if self.controller.Valid(col, row) == True and self.MyMap[col][row] == 0 and (
                        col, row) not in self.notAvailAble and self.controller.InMyArea(col, row,
                                                                                        SEEN_SIZE,
                                                                                        self.controller.cur_pos) == False and \
                        self.ZerosWallCheck(col, row)[0] == False and self.m_sph_handler.cmu.cost_map[row][
                    col] < SPHERE_INDICATE:
                    return [col, row]
        return [self.controller.cur_pos[0], self.controller.cur_pos[1]]

    def GetBestDist(self, direction, dest):
        i, j = -direction[0], -direction[1]
        cur_row, cur_col = dest[0], dest[1]
        wall_other_side = SEEN_SIZE * 2
        for step in range(SEEN_SIZE - WALL_DIST):
            if self.controller.Valid(cur_row + i * step, cur_col + j * step) and self.MyMap[cur_row + i * step][
                cur_col + j * step] != 0:
                wall_other_side = step
                break
        ahead = min(SEEN_SIZE - WALL_DIST, int(wall_other_side // 2))
        if ahead < WALL_DIST:
            self.notAvailAble.append((cur_row, cur_col))
            self.Mark(cur_row, cur_col)
            return [self.controller.cur_pos[0], self.controller.cur_pos[1]]
        return [cur_row + i * ahead, cur_col + j * ahead]

    # check also spheres
    def SphereCheck(self, coord):
        for sphere in self.m_sph_handler.m_founds_spheres:
            if self.controller.InMyArea(coord[0], coord[1], COVER_RADIUS, sphere):
                return [True, sphere]
        return [False, None]

    def SphereShifter(self, coord, direction, sphere_center):
        opp_direction = self.controller.OppositeDirection(direction)
        temp_row_top, temp_row_down = coord[0], coord[0]
        temp_col_top, temp_col_down = coord[1], coord[1]
        counter_top, row_top, col_top, counter_down, row_down, col_down = 0, 0, 0, 0, 0, 0
        # first we want to get the far point at the row\column that have sphere indicate
        while self.controller.Valid(temp_row_top, temp_col_top) == True and self.controller.InMyArea(temp_row_top,
                                                                                                     temp_col_top,
                                                                                                     COVER_RADIUS,
                                                                                                     sphere_center) == True and \
                self.controller.WallCheck(
                    temp_row_top, temp_col_top)[0] == False:
            temp_row_top = temp_row_top + opp_direction[0]
            temp_col_top = temp_col_top + opp_direction[1]
        counter_top = 0
        row_top = temp_row_top
        col_top = temp_col_top
        while self.controller.Valid(temp_row_top, temp_col_top) == True and \
                self.m_sph_handler.cmu.cost_map[temp_row_top][
                    temp_col_top] < SPHERE_INDICATE and self.controller.WallCheck(temp_row_top, temp_col_top)[
            0] == False:
            counter_top = counter_top + 1
            temp_row_top = temp_row_top + opp_direction[0]
            temp_col_top = temp_col_top + opp_direction[1]
        temp_row_down = coord[0]
        temp_col_down = coord[1]
        while self.controller.Valid(temp_row_down, temp_col_down) == True and self.controller.InMyArea(temp_row_down,
                                                                                                       temp_col_down,
                                                                                                       COVER_RADIUS,
                                                                                                       sphere_center) == True and \
                self.controller.WallCheck(
                    temp_row_down, temp_col_down)[0] == False:
            temp_row_down = temp_row_down - opp_direction[0]
            temp_col_down = temp_col_down - opp_direction[1]
        counter_down = 0
        row_down = temp_row_down
        col_down = temp_col_down
        while self.controller.Valid(temp_row_down, temp_col_down) == True and \
                self.m_sph_handler.cmu.cost_map[temp_row_down][
                    temp_col_down] < SPHERE_INDICATE and self.controller.WallCheck(temp_row_down,
                                                                                   temp_col_down)[0] == False:
            counter_down = counter_down + 1
            temp_row_down = temp_row_down - opp_direction[0]
            temp_col_down = temp_col_down - opp_direction[1]
        if counter_top > counter_down:
            step = min(COVER_RADIUS // 2, int(counter_top // 2))
            return np.array([row_top + step * opp_direction[0], col_top + step * opp_direction[1]])
        else:
            step = min(COVER_RADIUS // 2, int(counter_down // 2))
            return np.array([row_down + step * (-opp_direction[0]), col_down + step * (-opp_direction[1])])

    def chooseDirection(self):
        if self.controller.cur_pos is not None:
            if self.first == True:
                self.printit()
                self.m_sph_handler.UpdateMap()
                self.first = False
            left = mn.Direction('left', self.step(0, -1), (0, -1))
            right = mn.Direction('right', self.step(0, 1), (0, 1))
            up = mn.Direction('up', self.step(1, 0), (1, 0))
            down =mn. Direction('down', self.step(-1, 0), (-1, 0))
            DirectionList = [left, right, up, down]
            DirectionList.sort(key=mn.Direction.getVal)
            if DirectionList[0].value == self.MaxMove:
                next_move = self.FindBestZero()
                DirectionList[0] = mn.Direction('zero', [0, [next_move[0], next_move[1]]], (0, 0))
            x_y = np.array(
                [DirectionList[0].Pos[0], DirectionList[0].Pos[1]])
            wall_check = self.controller.WallCheck(x_y[0], x_y[1])
            if wall_check[0] == True:
                # need to check if next step is sphere and valid place(not -1 or wall)
                row, col = self.GetBestDist(wall_check[1], x_y)  # get away from wall in -direct move
                x_y[0] = row
                x_y[1] = col
            check_sphere = self.SphereCheck(x_y)
            if check_sphere[0]:
                x_y = self.SphereShifter(x_y, DirectionList[0].direc_, check_sphere[1])
            xy = self.controller.ms.map_to_position(x_y)
            x_y = self.controller.ms.position_to_map(xy)
            xy = xy[1], xy[0]
            self.Mark(x_y[0], x_y[1])
            self.controller.Move(xy[0], xy[1])
            if dist([xy[0], xy[1]], [self.controller.cur_pos[0], self.controller.cur_pos[1]]) >= DIFF:
                if DirectionList[0].name == 'zero':
                    self.m_sph_handler.UpdateWholeMap()
                else:
                    self.m_sph_handler.UpdateMap()


def vacuum_cleaning():
    print('start vacuum_cleaning')
    raise NotImplementedError


def inspection():
    print('start inspection')
    check = Inspect()
    rate = rospy.Rate(10)
    rospy.Subscriber("/odom", Odometry, check.controller.GetPos)
    while not rospy.is_shutdown():
        check.chooseDirection()
        rate.sleep()


def inspection_advanced():
    print('start inspection_advanced')
    raise NotImplementedError


# If the python node is executed as main process (sourced directly)
if __name__ == '__main__':
    inspection()
