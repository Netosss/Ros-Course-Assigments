#!/usr/bin/env python
from datetime import datetime
from nav_msgs.srv import GetMap
from std_msgs.msg import String
import ast
import numpy as np
import matplotlib.pyplot as plt
from geometry_msgs.msg import PoseStamped
import nav_msgs.srv
import nav_msgs
from math import sqrt
import sys
from numpy.lib.function_base import copy
import rospy
from nav_msgs.srv import GetMap
import copy
import dynamic_reconfigure.client
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid
from map_msgs.msg import OccupancyGridUpdate
import threading
import time
import tf
from geometry_msgs.msg import Point
import multiprocessing as mp
from std_msgs.msg import Empty
from threading import Lock

GOOD_LINE = 5
ESTIMATE_EACH = 3  # linear speed should be here
time_limit = float(2 * 60)  # [sec]
close_to_dirt = 4  # take radius from mohammad
BETA_VALUE_INIT = 0.0
ALPHA_VALUE_INIT = 0.0
AWAY = 4
w1 = 0.7
w2 = 1.0
w3 = 0.9
w4 = 3.0

###
DIST_FROM_PARTNER = 2.3
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import signal

TH = 8
WALL = 80
SEEN_SIZE = 30
RADIUS_IN_M = 0.5  # change the radius of sphere here![m]
RADIUS = round(14.6154 * RADIUS_IN_M + 2.76923)  # here we assign the size of the radius in pixels.
COVER_RADIUS = 1.3 * RADIUS  # to mark sphere around it
SPHERE_SIZE = 2 * RADIUS
SPHERE_INDICATE = 80
WALL_DIST = 10  # same as manager param!
SHIFT_INDICATE = 1.2 * RADIUS  # for indicate shifted circle
DIFF = 3
STEP = 1.75
WALL_DIST_C = 11
SEEN_SIZE_C = 25
RADIUS_EXTENSION = 0.1
SPEED = 0.20  # 0.375
A = 0.03
B = 0.04  # 0.04


# trying mohamad code for odom, ask for help...

def signal_handler(signum, frame):
    print("Timed out!")
    raise Exception


class Direction:
    def __init__(self, name, value, direction):
        self.name = name
        self.value = value[0]
        self.Pos = np.array(value[1])
        self.direc_ = np.array(direction)

    def getVal(self):
        return self.value


class MapService(object):

    def __init__(self, player):
        """
        Class constructor
        """
        rospy.wait_for_service('tb3_%d/static_map' % player)
        static_map = rospy.ServiceProxy('tb3_%d/static_map' % player, GetMap)
        self.map_data = static_map().map
        self.id = player
        self.map_org = np.array([self.map_data.info.origin.position.x, self.map_data.info.origin.position.y])
        self.shape = self.map_data.info.height, self.map_data.info.width
        self.map_arr = np.array(self.map_data.data, dtype='float32').reshape(self.shape)
        self.resolution = self.map_data.info.resolution

    def show_map(self, point=None):
        plt.imshow(self.map_arr)
        if point is not None:
            plt.scatter([point[0]], [point[1]])
        plt.show()

    def position_to_map(self, pos):
        return (pos - self.map_org) // self.resolution

    def map_to_position(self, indices):
        return indices * self.resolution + self.map_org


class Manager:
    def __init__(self, player, map_):
        self.id = player
        self.ms = MapService(self.id)
        self.my_map_ = map_
        self.initial_pose = PositionHandler(self.id, self.ms)
        self.partner_pose = PositionHandler(1 - int(self.id), self.ms)  # assume same ms is ok...
        self.rc_DWA_client = dynamic_reconfigure.client.Client('tb3_%d/move_base/DWAPlannerROS/' % self.id)
        self.rc_DWA_client.update_configuration({"max_vel_x": 2.5})
        self.rc_DWA_client.update_configuration({"min_vel_x": -2.5})

    def WallCheck(self, row, col, wall=WALL_DIST, clean=False):
        directions = [(1, 0), (0, 1), (-1, 0), (0, -1)]
        for direction in directions:
            for i in range(wall):
                if self.Valid(row + direction[0] * i, col + direction[1] * i) and \
                        self.my_map_[row + direction[0] * i][col + direction[1] * i] == 100:
                    return [True, direction]
                # check diagonal also
        ra = int(wall // 2) + 1
        if clean == True:
            ra = int(wall * 1.5)
        for direction in [(-1, -1), (1, 1), (-1, 1), (1, -1)]:
            for j in range(ra):
                if self.Valid(row + direction[0] * j, col + direction[1] * j) and \
                        self.my_map_[int(row + direction[0] * j)][int(col + direction[1] * j)] == 100:
                    return [True, direction]
        return [False, (0, 0)]

    def OppositeDirection(self, direction):
        if abs(direction[0]) == 1:
            opp = [0, 1]
        else:
            opp = [1, 0]
        return opp

    def Valid(self, i, j):
        info = self.ms.map_data.info
        if i >= 0 and i < info.height and j >= 0 and j < info.width:
            return True
        else:
            return False

    def InMyArea(self, row, col, square_size, center):
        x_dis = abs(row - center[0])
        y_dis = abs(col - center[1])
        if x_dis < square_size and y_dis < square_size:
            return True
        return False


class MovementHandler:
    def __init__(self, manager, mutex_):
        self.controller = manager
        self.mutex = mutex_
        self.MyMap = self.controller.my_map_
        self.MaxMove = max(self.controller.ms.shape[0], self.controller.ms.shape[1])
        self.notAvailAble = []

    def ZerosWallCheck(self, row, col, wall=WALL_DIST):
        directions = [(1, 0), (0, 1), (-1, 0), (0, -1)]
        for direction in directions:
            for i in range(2 * wall):
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

    def Mark(self, x_dest, y_dest, seen=SEEN_SIZE, wall=WALL_DIST, cur_pos=None):
        if cur_pos is None:
            cur_pos = self.controller.initial_pose.m_cur_pos
        current_square = seen
        while current_square > 0:
            if self.controller.InMyArea(x_dest, y_dest, current_square, cur_pos) == True:
                current_square = current_square - 1
                continue
            break
        if current_square <= wall:
            return
        current_square = current_square - wall
        i = -current_square
        while i < current_square:
            i = i + 1
            j = -current_square
            while j < current_square:
                j = j + 1
                row = cur_pos[0] + i
                column = cur_pos[1] + j
                if self.controller.Valid(row, column) and self.MyMap[row][column] == 0:
                    self.mutex.acquire()
                    try:
                        self.MyMap[row][column] = -1
                    finally:
                        self.mutex.release()

    def FindBestZero(self, cost_map=None, seen=SEEN_SIZE, wall=WALL_DIST, partner_zero=False):
        if cost_map is None:
            cost_map = self.MyMap
        for row in range(self.controller.ms.shape[0]):
            if partner_zero == True:
                row = self.controller.ms.shape[0] - row
            for col in range(self.controller.ms.shape[1]):
                if partner_zero == True:
                    col = self.controller.ms.shape[1] - col
                if self.controller.Valid(col, row) == True and self.MyMap[col][row] == 0 and (
                        col, row) not in self.notAvailAble and self.controller.InMyArea(col, row,
                                                                                        seen,
                                                                                        self.controller.initial_pose.m_cur_pos) == False and \
                        self.ZerosWallCheck(col, row, wall)[0] == False and cost_map[row][
                    col] < SPHERE_INDICATE and GetDistToGoal(self.controller.id,
                                                             self.controller.ms.map_to_position(np.array([col, row])),
                                                             self.controller.ms.map_to_position(
                                                                 self.controller.partner_pose.m_cur_pos)) > 1.3 * DIST_FROM_PARTNER:
                    return [col, row]
        return [self.controller.initial_pose.m_cur_pos[0], self.controller.initial_pose.m_cur_pos[1]]

    def GetBestDist(self, direction, dest, wall=WALL_DIST, seen=SEEN_SIZE):
        i, j = -direction[0], -direction[1]
        cur_row, cur_col = dest[0], dest[1]
        wall_other_side = seen * 2
        for step in range(seen - wall):
            if self.controller.Valid(cur_row + i * step, cur_col + j * step) and self.MyMap[cur_row + i * step][
                cur_col + j * step] != 0:
                wall_other_side = step
                break
        ahead = min(seen - wall, int(wall_other_side // 2))
        if ahead < wall:
            self.notAvailAble.append((cur_row, cur_col))
            self.Mark(cur_row, cur_col)
            return [self.controller.initial_pose.m_cur_pos[0], self.controller.initial_pose.m_cur_pos[1]]
        return [cur_row + i * ahead, cur_col + j * ahead]

    def step(self, i, j, seen=SEEN_SIZE, xy_pos=None):
        x = self.controller.initial_pose.m_cur_pos[0]
        y = self.controller.initial_pose.m_cur_pos[1]
        size = SPHERE_SIZE
        if xy_pos is not None:
            x = xy_pos[0]
            y = xy_pos[1]
            size = SPHERE_SIZE - WALL_DIST
        dis_from_wall = self.GetDistance(i, j, x, y)
        estimate_dest = np.array([x + i * seen, y + j * seen])
        NonCoverDist = dis_from_wall - seen
        if NonCoverDist <= 0 or NonCoverDist < size:
            return [self.MaxMove, [estimate_dest[0], estimate_dest[1]]]
        return [dis_from_wall, [estimate_dest[0], estimate_dest[1]]]


class SphereHandler:
    def __init__(self, manager, player):
        self.cmu = CostmapUpdater(player)
        self.ms = MapService(player)
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
            if center not in self.m_founds_spheres:
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
                    for i in range(int(COVER_RADIUS // 2)):
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
                row = self.m_manager.initial_pose.m_cur_pos[0] + i
                column = self.m_manager.initial_pose.m_cur_pos[1] + j
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

    def __init__(self, player, map_, mutex_):
        self.id = player
        self.controller = Manager(self.id, map_)
        self.m_sph_handler = SphereHandler(self.controller, self.id)
        self.first = True
        self.last_partner_direction = (-1, -1)
        self.move_alg = MovementHandler(self.controller, mutex_)
        self.publisher = rospy.Publisher('inspection_report', String, queue_size=10)
        self.direction_publisher = rospy.Publisher('tb3_%d/last_move_direc' % self.id, Point, queue_size=10)
        self.partner_direction_subscriber = rospy.Subscriber('tb3_%d/last_move_direc' % (1 - self.id), Point,
                                                             self.UpdatePartnerDirection)
        self.partner_zero = False
        self.first_move = True

    def UpdatePartnerDirection(self, data):
        self.last_partner_direction = [int(data.x), int(data.y)]

    def PublishSpheres(self):
        while True:
            self.m_sph_handler.UpdateWholeMap()
            spheres = len(self.m_sph_handler.m_founds_spheres)
            ct = datetime.now()
            ts = ct.timestamp()
            str = '{} spheres detected at time {} by agent {}'.format(spheres, ts, self.id)
            self.publisher.publish(str)
            time.sleep(10)

    def printit(self):
        threading.Timer(0.0, self.PublishSpheres).start()

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

    def chooseDirection(self, msg):
        if self.controller.initial_pose.m_cur_pos[0] is not None:
            if self.first == True:
                self.printit()
                self.m_sph_handler.UpdateMap()
                self.first = False
            left = Direction('left', self.move_alg.step(0, -1), (0, -1))
            right = Direction('right', self.move_alg.step(0, 1), (0, 1))
            up = Direction('up', self.move_alg.step(1, 0), (1, 0))
            down = Direction('down', self.move_alg.step(-1, 0), (-1, 0))
            DirectionList = [left, right, up, down]
            DirectionList.sort(key=Direction.getVal)
            # if we close to partner and make the same move we will make another direct
            # if self.last_partner_direction != (-1, -1) and GetDistToGoal(self.id, self.controller.ms.map_to_position(
            #         self.controller.initial_pose.m_cur_pos),
            #                                                              self.controller.ms.map_to_position(
            #                                                                  self.controller.partner_pose.m_cur_pos)) < DIST_FROM_PARTNER and (
            #         any(DirectionList[0].direc_ == self.last_partner_direction) or (
            #         DirectionList[0].value == self.move_alg.MaxMove and self.last_partner_direction == (
            #         0, 0))):  # if we are close and (go same direction or we both need find best zero)
            #     if self.last_partner_direction == (0, 0):  # both best zero move+
            #         self.partner_zero = True
            #     # elif self.last_partner_direction == (-1, -1):#stay at current position move
            #     else:
            #         DirectionList[0] = DirectionList[1]
            if DirectionList[0].value != self.move_alg.MaxMove and GetDistToGoal(self.id,
                                                                                 self.controller.ms.map_to_position(
                                                                                         self.controller.initial_pose.m_cur_pos),
                                                                                 self.controller.ms.map_to_position(
                                                                                     self.controller.partner_pose.m_cur_pos)) < DIST_FROM_PARTNER:
                cur_max = [0, DirectionList[0]]
                for i in range(4):
                    estimate_goal = self.controller.ms.map_to_position(self.controller.initial_pose.m_cur_pos + \
                                                                       DirectionList[i].direc_[0] * SEEN_SIZE +
                                                                       DirectionList[i].direc_[1] * SEEN_SIZE)

                    cur_dist = GetDistToGoal(self.id, self.controller.ms.map_to_position(
                        self.controller.partner_pose.m_cur_pos), estimate_goal)
                    if cur_dist > cur_max[0]:
                        cur_max = [cur_dist, DirectionList[i]]
                DirectionList[0] = cur_max[1]
            if DirectionList[0].value == self.move_alg.MaxMove:
                next_move = self.move_alg.FindBestZero(self.m_sph_handler.cmu.cost_map, partner_zero=self.partner_zero)
                DirectionList[0] = Direction('zero', [0, [next_move[0], next_move[1]]], (0, 0))
                self.partner_zero = False
            x_y = np.array([DirectionList[0].Pos[0], DirectionList[0].Pos[1]])
            wall_check = self.controller.WallCheck(x_y[0], x_y[1])
            if wall_check[0] == True:
                # need to check if next step is sphere and valid place(not -1 or wall)
                row, col = self.move_alg.GetBestDist(wall_check[1], x_y)  # get away from wall in -direct move
                x_y[0] = row
                x_y[1] = col
            check_sphere = self.SphereCheck(x_y)
            if check_sphere[0]:
                x_y = self.SphereShifter(x_y, DirectionList[0].direc_, check_sphere[1])
            xy = self.controller.ms.map_to_position(x_y)
            x_y = self.controller.ms.position_to_map(xy)
            xy = xy[1], xy[0]
            self.move_alg.Mark(x_y[0], x_y[1])
            Move(self.id, xy[0], xy[1])
            print(self.id, xy)
            if dist([xy[0], xy[1]],
                    [self.controller.initial_pose.m_cur_pos[0], self.controller.initial_pose.m_cur_pos[1]]) >= DIFF:
                str1 = ''.join(str(e) for e in DirectionList[0].direc_)
                point = Point()
                point.x = DirectionList[0].direc_[0]
                point.y = DirectionList[0].direc_[1]
                self.direction_publisher.publish(point)
                if DirectionList[0].name == 'zero':
                    self.m_sph_handler.UpdateWholeMap()
                else:
                    self.m_sph_handler.UpdateMap()
            else:
                point = Point()
                point.x = -1
                point.y = -1
                self.direction_publisher.publish(point)


###//////////////////////////////////////////////////////////////////////////////////////////////////////////////###

# mohammad code
class CostmapUpdater:
    def __init__(self, player):
        self.cost_map = None
        self.shape = None

        rospy.Subscriber('/tb3_%d/move_base/global_costmap/costmap' % player, OccupancyGrid, self.init_costmap_callback)
        rospy.Subscriber('tb3_%d/move_base/global_costmap/costmap_updates' % player, OccupancyGridUpdate,
                         self.costmap_callback_update)

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


def Move(agent_id, x, y, w=1.0, use=True):
    # if use == True:
    #     max_time = 45  # 45 secs before starting a new move instead this one
    #     signal.signal(signal.SIGALRM, signal_handler)
    #     signal.alarm(max_time)
    try:
        client = actionlib.SimpleActionClient('tb3_%d/move_base' % agent_id, MoveBaseAction)
        client.wait_for_server()
        # Creates a new goal with the MoveBaseGoal constructor
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        # Move to position 0.5 on the x axis of the "map" coordinate frame
        goal.target_pose.pose.position.x = x
        # Move to position 0.5 on the y axis of the "map" coordinate frame
        goal.target_pose.pose.position.y = y
        # No rotation of the mobile base frame w.r.t. map frame
        goal.target_pose.pose.orientation.w = w
        # Sends the goal to the action server.
        client.send_goal(goal)
        rospy.loginfo("New goal command received!")
        # Waits for the server to finish performing the action.
        wait = client.wait_for_result()
        # If the result doesn't arrive, assume the Server is not available
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
            # Result of executing the action
            return client.get_result()
    except Exception as msg:
        print('move time done,calculate next move')


def GetDistToGoal(agent_id, cur_pos, goal_pos):
    if cur_pos[0] == None:
        return 2
    start = PoseStamped()
    start.header.seq = 0
    start.header.frame_id = "map"
    start.header.stamp = rospy.Time(0)
    start.pose.position.x = float(cur_pos[0])
    start.pose.position.y = float(cur_pos[1])
    start.pose.position.z = float(0)
    Goal = PoseStamped()
    Goal.header.seq = 0
    Goal.header.frame_id = "map"
    Goal.header.stamp = rospy.Time(0)
    Goal.pose.position.x = float(goal_pos[0])
    Goal.pose.position.y = float(goal_pos[1])
    Goal.pose.position.z = float(0)
    rospy.wait_for_service('/tb3_%d/move_base/NavfnROS/make_plan' % agent_id)
    try:
        get_plan = rospy.ServiceProxy('/tb3_%d/move_base/NavfnROS/make_plan' % agent_id, nav_msgs.srv.GetPlan)
        req = nav_msgs.srv.GetPlan()
        req.start = start
        req.goal = Goal
        req.tolerance = .5
        get_plan.wait_for_service()
        resp = get_plan(req.start, req.goal, req.tolerance)
        data = resp.plan
        sum_ = 0
        for i in range(len(data.poses) - 1):
            sum_ += sqrt(pow((data.poses[i + 1].pose.position.x - data.poses[i].pose.position.x), 2) + pow(
                (data.poses[i + 1].pose.position.y - data.poses[i].pose.position.y), 2))
        # rospy.loginfo(len(resp.plan.poses))
        # return sum_
        if sum_ == float(0):  # in case make path failed we take euclidean dist
            return dist(cur_pos, goal_pos)
        else:
            return sum_
    except rospy.ServiceException as e:
        print("service failed")
        return 20


# handle the game status, positions, total_dirt, and dirt each player
class PerformState:
    def __init__(self, position, time_, amount, adversary_position, adversary_amount, current_dirt, goal_dirt,
                 last_pose):
        self.position_ = position
        self.time_left = time_
        self.cur_amount = amount
        self.adversary_amount_ = adversary_amount
        self.adversary_position_ = adversary_position
        self.dirt_list = current_dirt
        self.cur_dirt = goal_dirt
        self.last_pose = last_pose
    ##########################################################################################################


class SearchAlgos:
    def __init__(self, utility, succ, perform_move, player):
        """The constructor for all the search algos.
        You can code these functions as you like to,
        and use them in MiniMax and AlphaBeta algos as learned in class
        :param utility: The utility function. #heuristic, return number
        :param succ: The succesor function. #actually return all dirt points left
        :param perform_move: The perform move function. #perform move by given state(position,time_left,cur_amount),goal(dirt point),player,ms return new state
        """
        self.utility = utility
        self.succ = succ
        self.perform_move = perform_move
        self.player = player

    def search(self, state, depth, maximizing_player):
        pass


class AlphaBeta(SearchAlgos):
    def search(self, state, depth, maximizing_player, alpha=ALPHA_VALUE_INIT, beta=BETA_VALUE_INIT):
        """Start the AlphaBeta algorithm.
        :param state: The state to start from.
        :param depth: The maximum allowed depth for the algorithm.
        :param maximizing_player: Whether this is a max node (True) or a min node (False).
        :param alpha: alpha value
        :param beta: beta value
        :return: A tuple: (The min max algorithm value, The direction in case of max node or None in min mode)
        """
        # if adversary is closer than me to goal we need to think about another strategy.
        # current prog fail- especially when all dirt closer to adversary...
        # in first think, we should skip just on the one who the closest to the adversary, or give it weight,
        # as more as close to adversary than us is the more is bad point!
        Options = state.dirt_list
        move = None
        if depth == 0 or not Options:
            return self.utility(state), None
        if maximizing_player == True:
            cur_max = float('-inf')
            for child in Options:
                new_state = self.perform_move(state, child, maximizing_player)
                if new_state.time_left < 0:
                    continue
                best_val = self.search(new_state, depth - 1, not maximizing_player, alpha, beta)
                value = best_val[0]
                if value > cur_max:
                    cur_max = value
                    move = child
                alpha = max(value, alpha)
                if beta <= alpha:
                    break
                # if cur_max >= beta:
                #     return float('inf'), move
            return cur_max, move
        else:
            cur_min = float('inf')
            for child in Options:
                new_state = self.perform_move(state, child, maximizing_player)
                if new_state.time_left < 0:
                    continue
                best_val = self.search(new_state, depth - 1, not maximizing_player, alpha, beta)
                value = best_val[0]
                cur_min = min(value, cur_min)
                beta = min(cur_min, beta)
                if beta <= alpha:
                    break
                # if cur_min <= alpha:
                #     return float('-inf'), move
            return cur_min, move


##################################################################################################
def dist(point1, point2):
    return sqrt((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2)


def Valid(i, j, shape):
    if 0 <= i < shape[0] and 0 <= j < shape[1]:
        return True
    else:
        return False


class PositionHandler:
    def __init__(self, player_num, ms):
        self.odom_sub = rospy.Subscriber('tb3_%d/odom' % int(player_num), Odometry, self.pos_callback)
        self.m_cur_pos = [None, None]
        self.map_service = ms
        self.id = player_num

    def pos_callback(self, data):
        cur = np.array([data.pose.pose.position.x, data.pose.pose.position.y])
        pose_ = self.map_service.position_to_map(cur)
        self.m_cur_pos = np.array([int(pose_[1]), int(pose_[0])])

    def TryingMohamadCode(self):
        # rospy.init_node('tf_listener')
        rospy.loginfo('started listener node!')
        listener = tf.TransformListener()
        rate = rospy.Rate(1.0)
        while not rospy.is_shutdown():
            try:
                (trans, rot) = listener.lookupTransform('tb3_%d/map' % int(self.id), 'tb3_%d/base_link' % int(self.id),
                                                        rospy.Time(0))

            except(tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
            rate.sleep()


class AlgorithmHandler:
    def __init__(self, agent, ms, dirt):
        self.id = agent
        self.map_service = ms
        self.dirt = dirt
        self.alpha_beta = AlphaBeta(self.heuristic, self.dirt, self.perform_move, self.id)

    def CalculateTime(self, position, goal):
        # calculate estimate time
        dist_to_goal = GetDistToGoal(self.id, position, goal)
        return ESTIMATE_EACH * dist_to_goal

    def perform_move(self, state: PerformState, goal, player):
        new_position = goal
        new_list = state.dirt_list.copy()
        new_list.remove(goal)
        if player == True:
            new_time = state.time_left - self.CalculateTime(state.position_, goal)
            return PerformState(new_position, new_time, state.cur_amount + 1, state.adversary_position_,
                                state.adversary_amount_, new_list, goal, state.position_)
        else:
            new_time = state.time_left - self.CalculateTime(state.adversary_position_,
                                                            goal)
            return PerformState(state.position_, new_time, state.cur_amount, new_position, state.adversary_amount_ + 1,
                                new_list, goal, state.adversary_position_)

    def CloseToMore(self, state):
        my_score = 0
        adversary_score = 0
        for point in self.dirt:
            my_score = my_score + GetDistToGoal(self.id, state.position_, point)
            adversary_score = adversary_score + GetDistToGoal((1 - self.id), state.adversary_position_, point)
        return sqrt((1 / my_score) / (1 / adversary_score))

    def Winning(self, state):
        return sqrt((state.cur_amount + 1) / (state.adversary_amount_ + 1))

    def CloserToClosest(self, state):
        if len(state.dirt_list) == 0:
            return 1
        min_dist = float('inf')
        closest = None
        for dirt in state.dirt_list:
            cur_dist = GetDistToGoal(int(1 - self.id), state.adversary_position_,
                                     dirt)
            if cur_dist < min_dist:
                min_dist = cur_dist
                closest = dirt
        my_dist = GetDistToGoal(self.id, state.position_, closest)
        return sqrt(min_dist / (my_dist + 1))

    def TookClosest(self, state: PerformState):
        if state.cur_dirt == None:
            return 1
        else:
            dist_ = sqrt(1 / (GetDistToGoal(self.id, state.last_pose, state.cur_dirt) + 1))
            return dist_

    def heuristic(self, state):
        if len(state.dirt_list) == 0:
            return self.Winning(state)
            # if state.cur_amount > state.adversary_amount_:
            #     return float('inf')
            # else:
            #     return float('-inf')
        if state.time_left <= 0:
            return 0
        closer = self.CloseToMore(state)
        winning = self.Winning(state)
        closest = self.CloserToClosest(state)
        took = self.TookClosest(state)
        print(closer, winning, closest, took)
        return w1 * closer + w2 * winning + w3 * closest + w4 * took


class GameManager:
    def __init__(self, player):
        self.dirt = []  # list of points(float) for dirt
        self.player = player
        self.m_total_amount = 0
        self.ms = MapService(player)
        self.start_time = time.time()
        self.player_pos_wrapper = PositionHandler(player, self.ms)  # updating actual player position
        self.adversary_pos_wrapper = PositionHandler(1 - player, self.ms)  # updating actual adversary position
        self.sub = rospy.Subscriber('dirt', String, self.dirt_callback)  # updating actual dirt list
        self.cur_amount = 0
        self.adversary_amount = 0
        self.dirt_updated = False
        self.my_map_ = copy.deepcopy(self.ms.map_arr)
        self.player_pos_wrapper.TryingMohamadCode()

        while self.dirt_updated == False or self.player_pos_wrapper.m_cur_pos[0] == None or \
                self.adversary_pos_wrapper.m_cur_pos[0] == None:
            continue

    def dirt_callback(self, msg):
        last = 0
        cur_list = []
        for i, v in enumerate(msg.data):
            if v == "]":
                x = ast.literal_eval(msg.data[last:i + 1])
                last = i + 1
                cur_list.append(x)
        if self.dirt_updated == False or len(cur_list) != self.m_total_amount:
            diff = [i for i in cur_list if not any(i[0] == k for k, _ in self.dirt)]
            if self.dirt_updated == True:  # just if we have old list and diffrent
                for var in diff:
                    var = self.ms.position_to_map(var)
                    if dist(var, self.player_pos_wrapper.m_cur_pos) < close_to_dirt:
                        self.cur_amount = self.cur_amount + 1
                    else:
                        self.adversary_amount = self.adversary_amount + 1
            self.dirt = cur_list
            self.m_total_amount = len(cur_list)
            self.dirt_updated = True

    def RemoveClosestFromList(self):
        closest = None
        min_dist = float('inf')
        for dirt in self.dirt:
            adversary_dist = GetDistToGoal(self.player,
                                           self.ms.map_to_position(self.adversary_pos_wrapper.m_cur_pos), dirt)
            player_dist = GetDistToGoal(self.player, self.ms.map_to_position(self.player_pos_wrapper.m_cur_pos), dirt)
            if adversary_dist < player_dist and adversary_dist < min_dist:
                closest = dirt
                min_dist = adversary_dist
        if closest == None:
            return self.dirt.copy()
        new_list = self.dirt.copy()
        new_list.remove(closest)
        return new_list

    def choose_move(self, total_time_for_turn):
        if len(self.dirt) == 1:
            return self.dirt[0]
        best_move = None
        cur_dirt = self.RemoveClosestFromList()
        algo = AlgorithmHandler(self.player, self.ms, cur_dirt)
        cur = PerformState(self.ms.map_to_position(self.player_pos_wrapper.m_cur_pos),
                           time_limit - (time.time() - self.start_time),
                           self.cur_amount, self.ms.map_to_position(self.adversary_pos_wrapper.m_cur_pos),
                           self.adversary_amount, cur_dirt, None, None)
        for i in range(len(cur_dirt)):
            if i < 1:
                continue
            iteration_start = time.time()
            best = algo.alpha_beta.search(cur, i, True, float('-inf'), float('inf'))
            if best[1] != None:
                best_move = best[1]
            cur_time = time.time()
            if (cur_time - iteration_start) * (i / 2) > total_time_for_turn:  # factorial i instead 4.1 for
                # current depth calculate don't take too much...
                break
        if best_move is None and len(cur_dirt) > 0:
            best_move = cur_dirt[0]
        return best_move

    def Valid(self, i, j):
        info = self.ms.map_data.info
        if i >= 0 and i < info.height and j >= 0 and j < info.width:
            return True
        else:
            return False

    def WallCheck(self, row, col, wall=WALL_DIST, clean=False):
        directions = [(1, 0), (0, 1), (-1, 0), (0, -1)]
        for direction in directions:
            for i in range(wall):
                if self.Valid(row + direction[0] * i, col + direction[1] * i) and \
                        self.my_map_[row + direction[0] * i][col + direction[1] * i] == 100:
                    return [True, direction]
                # check diagonal also
        ra = int(wall // 2) + 1
        if clean == True:
            ra = int(wall * 1.5)
        for direction in [(-1, -1), (1, 1), (-1, 1), (1, -1)]:
            for j in range(ra):
                if self.Valid(row + direction[0] * j, col + direction[1] * j) and \
                        self.my_map_[int(row + direction[0] * j)][int(col + direction[1] * j)] == 100:
                    return [True, direction]
        return [False, (0, 0)]

    def make_move(self, move):
        move_to = move
        if move_to is None:
            return self.ms.map_to_position(self.player_pos_wrapper.m_cur_pos)
        wall_check_move = self.ms.position_to_map(move_to)
        is_close = self.WallCheck(int(wall_check_move[1]), int(wall_check_move[0]))
        if is_close[0] == True:
            move_to = np.array([wall_check_move[0] - is_close[1][1] * AWAY,
                                wall_check_move[1] - is_close[1][0] * AWAY])
            move_to = self.ms.map_to_position(move_to)
        Move(self.player, move_to[0], move_to[1], True)
        # result = multi_move_base.move(agent_id=int(self.player), x=move_to[0], y=move_to[1])
        # assume amount updating in call back == assume dirt taken updated in topic /dirt


def vacuum_cleaning(id_, agent_max_vel):
    print('start vacuum_cleaning')
    check = GameManager(int(id_))
    cur_turn = 1
    while not rospy.is_shutdown():
        cur_time = (0.210556 - 0.0203333 * cur_turn) * (
                time_limit - (time.time() - check.start_time))  # giving first turns more calculate time
        if cur_turn >= 10 or cur_time < 0.02:
            cur_time = 0.08
        move = check.choose_move(cur_time)
        check.make_move(move)
        cur_turn = cur_turn + 1
    rospy.spin()


def inspection(agent_1_max_vel, agent_2_max_vel):
    print('start inspection')
    map_ = copy.deepcopy(MapService(0).map_arr)
    mutex = Lock()

    check_0 = Inspect(0, map_, mutex)
    check_1 = Inspect(1, map_, mutex)
    sub = rospy.Subscriber('start_foo%d' % 0, Empty, check_0.chooseDirection, queue_size=10)
    sub1 = rospy.Subscriber('start_foo%d' % 1, Empty, check_1.chooseDirection, queue_size=10)

    while rospy.is_shutdown():
        continue
    pub0 = rospy.Publisher('/start_foo0', Empty, queue_size=10)
    pub1 = rospy.Publisher('/start_foo1', Empty, queue_size=10)
    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        pub0.publish()
        rate.sleep()
        pub1.publish()
        continue
    # check.m_sph_handler.cmu.show_map()


# If the python node is executed as main process (sourced directly)
if __name__ == '__main__':
    # Initializes a rospy node to let the SimpleActionClient publish and subscribe
    rospy.init_node('assignment3')
    agent_max_vel = 0.22
    exec_mode = sys.argv[1]
    print('exec_mode:' + exec_mode)

    if exec_mode == 'cleaning':
        agent_id = sys.argv[2]
        print('agent id:' + agent_id)
        vacuum_cleaning(agent_id, agent_max_vel)


    elif exec_mode == 'inspection':
        agent_1_max_vel = sys.argv[2]
        agent_2_max_vel = sys.argv[3]
        inspection(agent_1_max_vel, agent_2_max_vel)

    else:
        print("Code not found")
        raise NotImplementedError
