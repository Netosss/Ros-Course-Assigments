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

GOOD_LINE = 5
TH = 7
WALL = 80
SEEN_SIZE = 50
RADIUS = 10
SPHERE_SIZE = 20
LINE_INDICATE = 10
SPHERE_INDICATE = 80
WALL_DIST = 5  # same as manager param!
SHIFT_INDICATE = 12  # for indicate shifted circle
DIFF = 2


class Direction:
    def __init__(self, name, value, direction):
        self.name = name
        self.value = value[0]
        self.Pos = np.array(value[1])
        self.direc_ = np.array(direction)

    def getVal(self):
        return self.value


# mohammad code
class CostmapUpdater:
    def __init__(self):
        self.cost_map = None
        self.shape = None
        rospy.Subscriber('/move_base/global_costmap/costmap', OccupancyGrid, self.init_costmap_callback)
        rospy.Subscriber('/move_base/global_costmap/costmap_updates', OccupancyGridUpdate, self.costmap_callback_update)

    def init_costmap_callback(self, msg):
        print('only once')  # For the student to understand
        self.shape = msg.info.height, msg.info.width
        self.cost_map = np.array(msg.data).reshape(self.shape)

    def costmap_callback_update(self, msg):
        # print('periodically')  # For the student to understand
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

    # # need to rechange the function!!!first indicate if it is top or bottom point by the dis in the caller function,
    # # after you know that is bottom for example you take the bottom_point and take about until 10 steps down and check for
    # # a sphere indicate at row, you get the distance if it is legal(for now it is 4-10) return the (x,y) with the
    # # appropriate dist , than at caller you should make a symmetric with given x,y and the known point
    # def GetDistLine(self, x_top_point, y_top_point, opposite_direction):
    #     dist_from_top = 0
    #     x_top = x_top_point
    #     y_top = y_top_point
    #     current = 0
    #     if abs(opposite_direction[0]) == 1:
    #         direction = (0, 1)
    #     else:
    #         direction = (1, 0)
    #     for direct in [direction, (-direction[0], -direction[1])]:
    #         while self.m_manager.Valid(x_top, y_top) and self.cmu.cost_map[x_top][y_top] > SPHERE_INDICATE:
    #             is_circle = False
    #             temp_x = x_top
    #             temp_y = y_top
    #             if is_circle:
    #                 current = current + 1
    #             for i in range(int(SPHERE_SIZE + 2 // 2)):
    #                 if self.m_manager.Valid(temp_x, temp_y) and self.cmu.cost_map[temp_x][temp_y] > SPHERE_INDICATE:
    #                     is_circle = True
    #                     break
    #                 temp_x = temp_x + direct[0]
    #                 temp_y = temp_y + direct[1]
    #             x_top = x_top + opposite_direction[0]
    #             y_top = y_top + opposite_direction[1]
    #             dist_from_top = dist_from_top + 1

    def OppositeDirection(self, direction):
        if abs(direction[0]) == 1:
            opp = (0, 1)
        else:
            opp = (1, 0)
        return opp

    def Count(self, row, col, direction):
        counter = 0
        temp_row_top = row
        temp_col_top = col
        # first we want to get the far point at the row\column that have sphere indicate
        while self.m_manager.Valid(temp_row_top, temp_col_top) and self.cmu.cost_map[temp_row_top][
            temp_col_top] > SPHERE_INDICATE:
            temp_row_top = temp_row_top + direction[0]
            temp_col_top = temp_col_top + direction[1]
        direction[0], direction[1] = -direction[0], -direction[1]
        temp_row_down = temp_row_top + direction[0]
        temp_col_down = temp_col_top + direction[0]
        while self.m_manager.Valid(temp_row_down, temp_col_down) and self.cmu.cost_map[temp_row_down][
            temp_col_down] > SPHERE_INDICATE:
            counter = counter + 1
            temp_row_down = temp_row_down + direction[0]
            temp_col_down = temp_col_down + direction[1]
        center = (int((temp_row_top + temp_row_down) // 2), int((temp_col_down + temp_col_down) // 2))
        return counter, center

    def AddSphere(self, _row, _col):
        directions = [(1, 0), (0, 1)]
        left = False
        for direction in directions:
            opposite_direction = self.OppositeDirection(direction)
            cur_counter, cur_center = self.Count(_row, _col, direction)
            if cur_counter < GOOD_LINE:
                continue
            left_counter, _ = self.Count(cur_counter[0] + opposite_direction[0],
                                                   cur_center[1] + opposite_direction[1], direction)
            right_counter, _ = self.Count(cur_counter[0] - opposite_direction[0],
                                                     cur_center[1] - opposite_direction[1], direction)
            if left_counter - DIFF <= cur_counter <= DIFF + right_counter:  # we would like to get the center of the smallest line
                while self.m_manager.Valid(cur_center[0], cur_center[1]) and self.cmu.cost_map[cur_center[0]][
                    cur_center[1]] > SPHERE_INDICATE:
                    cur_center = (cur_counter[0] + opposite_direction[0], cur_center[1] + opposite_direction[1])
                    left = True
            elif left_counter + DIFF <= cur_counter <= DIFF - right_counter:
                while self.m_manager.Valid(cur_center[0], cur_center[1]) and self.cmu.cost_map[cur_center[0]][
                    cur_center[1]] > SPHERE_INDICATE:
                    cur_center = (cur_counter[0] - opposite_direction[0], cur_center[1] - opposite_direction[1])
            else:
                return
            if left:
                center = (
                    cur_center[0] + RADIUS * (-opposite_direction[0]),
                    cur_center[0] + RADIUS * (-opposite_direction[1]))
            else:
                center = (
                    cur_center[0] + RADIUS * (opposite_direction[0]), cur_center[0] + RADIUS * (opposite_direction[1]))
            self.m_founds_spheres.append(center)

    def UpdateMap(self):
        # we can look just at local or 60X60 map at the global from cur pos.
        # current_global_cm = deepcopy(self.cmu.cost_map) #maybe need to copy for un-update
        cur_map = np.subtract(self.cmu.cost_map - self.ms.map_arr)
        _row = self.m_manager.cur_pos[0] - SEEN_SIZE
        _col = self.m_manager.cur_pos[1] - SEEN_SIZE
        for j in range(2 * SEEN_SIZE):
            for i in range(2 * SEEN_SIZE):
                if self.m_manager.Valid(_row, _col) and cur_map[_row][_col] > SPHERE_INDICATE:
                    if self.m_manager.WallCheck(_row, _col)[0] == True:
                        self.WallHandler(_row, _col, cur_map)
                    elif self.OldSphere(_row, _col) == False:  # if it is new sphere we add it, otherwise shift and cont
                        self.AddSphere(_row, _col)
                    _row = _row + 1
                    _col = _col + 1

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
            if self.m_manager.InMyArea(_row, _col, SPHERE_SIZE, sphere):
                return True
            eDistance = dist([_row, _col], [sphere[0], sphere[1]])
            if eDistance < SHIFT_INDICATE:
                shifted_src = (
                    sphere[0] + self.PointShifter(sphere[0], _row), sphere[1] + self.PointShifter(sphere[1], _col))
                i = 0
                while i < len(self.m_founds_spheres):
                    self.m_founds_spheres[i].replace(sphere, shifted_src)
                    i = i + 1
                return True
        return False


class Inspect:

    def __init__(self):
        # self.pose = 0, 0
        self.controller = mn.Manager()
        self.MyMap = self.controller.ms.map_arr
        self.cmu = CostmapUpdater()
        self.lastCoord = None
        self.MaxMove = max(self.controller.ms.shape[0], self.controller.ms.shape[1])
        self.notAvailAble = []

    def ZerosWallCheck(self, row, col):
        directions = [(1, 0), (0, 1), (-1, 0), (0, -1)]
        for direction in directions:
            for i in range(2 * WALL_DIST):
                if self.controller.Valid(row + direction[0] * i, col + direction[1] * i) and \
                        self.MyMap[row + direction[0] * i][col + direction[1] * i] != 0:
                    return [True, direction]
        return [False, (0, 0)]

    # indicate if there is wall in WALL_DIST far from the given coordinates
    # def WallCheck(self, row, col):
    #     directions = [(1, 0), (0, 1), (-1, 0), (0, -1)]
    #     for direction in directions:
    #         for i in range(WALL_DIST):
    #             if self.controller.Valid(row + direction[0] * i, col + direction[1] * i) and \
    #                     self.MyMap[row + direction[0] * i][col + direction[1] * i] == 100:
    #                 return [True, direction]
    #     return [False, (0, 0)]

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

    # get the closest wall distance, bigger than WALL_DIST and not in the last direction
    def ClosestWallOtherSides(self, i, j, row, col, zeros_indicate):
        if i == 0 and j == 1:
            dis1 = self.GetDistance(1, 0, row, col)
            # dis2 = self.GetDistance(0, -1, row, col)
            dis3 = self.GetDistance(-1, 0, row, col)
            dis = sorted([dis1, dis3])
            for distance in dis:
                if distance > SPHERE_SIZE:
                    if distance == dis1:
                        return 1, 0, distance
                    # elif distance == dis2:
                    #     return 0, -1, distance
                    else:
                        return -1, 0, distance
            return 0, 0, 0  # when all walls are too close
        elif i == 0 and j == -1:
            dis1 = self.GetDistance(1, 0, row, col)
            # dis2 = self.GetDistance(0, 1, row, col)
            dis3 = self.GetDistance(-1, 0, row, col)
            dis = sorted([dis1, dis3])
            for distance in dis:
                if distance > SPHERE_SIZE:
                    if distance == dis1:
                        return 1, 0, distance
                    # elif distance == dis2:
                    #     return 0, 1, distance
                    else:
                        return -1, 0, distance
            return 0, 0, 0  # when all walls are too close

        elif i == 1 and j == 0:
            dis1 = self.GetDistance(0, -1, row, col)
            dis2 = self.GetDistance(0, 1, row, col)
            # dis3 = self.GetDistance(-1, 0, row, col)
            dis = sorted([dis1, dis2])
            for distance in dis:
                if distance > SPHERE_SIZE:
                    if distance == dis1:
                        return 0, -1, distance
                    elif distance == dis2:
                        return 0, 1, distance
                    # else:
                    #     return -1, 0, distance
            return 0, 0, 0  # when all walls are too close
        elif i == -1 and j == 0:  # -1,0
            dis1 = self.GetDistance(0, -1, row, col)
            dis2 = self.GetDistance(0, 1, row, col)
            # dis3 = self.GetDistance(1, 0, row, col)
            dis = sorted([dis1, dis2])
            for distance in dis:
                if distance > SPHERE_SIZE:
                    if distance == dis1:
                        return 0, -1, distance
                    elif distance == dis2:
                        return 0, 1, distance
                    # else:
                    #     return 1, 0, distance
            return 0, 0, 0  # when all walls are too close
        else:  # 0,0 for random place purpose
            dis1 = self.GetDistance(0, -1, row, col)
            dis2 = self.GetDistance(0, 1, row, col)
            dis3 = self.GetDistance(1, 0, row, col)
            dis4 = self.GetDistance(-1, 0, row, col)
            dis = [dis1, dis2, dis3, dis4]
            dis.sort()
            for distance in dis:
                if zeros_indicate == True and distance < WALL_DIST:
                    continue
                if distance > SPHERE_SIZE - WALL_DIST:
                    if distance == dis1:
                        return 0, -1, distance
                    elif distance == dis2:
                        return 0, 1, distance
                    elif distance == dis3:
                        return 1, 0, distance
                    else:
                        return -1, 0, distance
            return 0, 0, 0  # when all walls are too close

    def step(self, i, j):
        dis_from_wall = self.GetDistance(i, j, self.controller.cur_pos[0], self.controller.cur_pos[1])
        estimate_dest = np.array(
            [self.controller.cur_pos[0] + i * SEEN_SIZE, self.controller.cur_pos[1] + j * SEEN_SIZE])
        NonCoverDist = dis_from_wall - SEEN_SIZE
        if NonCoverDist <= 0 or NonCoverDist < SPHERE_SIZE:
            return [self.MaxMove, [estimate_dest[0], estimate_dest[1]]]
            # if self.controller.Valid(row, col) and self.cmu.cost_map[row][col] > SPHERE_INDICATE:
            # if not self.WallCheck(estimate_dist[0], estimate_dist[1]):
            # we know this is sphere for sure!
            # row, col = SphereHandle(row, col)
        return [dis_from_wall, [estimate_dest[0], estimate_dest[1]]]

    def Mark(self, x_dest, y_dest, zero_indicate):
        current_square = SEEN_SIZE
        while current_square > 0:
            if self.controller.InMyArea(x_dest, y_dest, current_square, self.controller.cur_pos) == True:
                current_square = current_square - 1
                continue
            break
        if current_square <= WALL_DIST:
            return
        current_square = current_square - WALL_DIST
        print("my marking square size is: ", current_square)
        i = -current_square
        if zero_indicate == True:
            current_square = WALL_DIST
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
                        self.ZerosWallCheck(col, row)[0] == False:
                    return [col, row]

                # and self.cmu.cost_map[row][col] < SPHERE_INDICATE
                # i, j, second_dis = self.ClosestWallOtherSides(0, 0, col, row,
                #                                               True)  # need to add sphere handler function
                # if second_dis == 0:
                #     self.notAvailAble.append((col, row))
                #     continue
                # elif second_dis < 30:
                #     return [col, row]
                # else:
                #     return [row + j * (second_dis - SEEN_SIZE), col + i * (second_dis - SEEN_SIZE)]
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
            self.Mark(cur_row, cur_col, WALL_DIST)
            return [self.controller.cur_pos[0], self.controller.cur_pos[1]]
        return [cur_row + i * ahead, cur_col + j * ahead]

    # check also spheres

    def chooseDirection(self):
        if self.controller.cur_pos is not None:
            print("my current position: ", self.controller.cur_pos)
            left = Direction('left', self.step(0, -1), (0, -1))
            right = Direction('right', self.step(0, 1), (0, 1))
            up = Direction('up', self.step(1, 0), (1, 0))
            down = Direction('down', self.step(-1, 0), (-1, 0))
            DirectionList = [left, right, up, down]
            DirectionList.sort(key=Direction.getVal)
            if DirectionList[0].value == self.MaxMove:
                next_move = self.FindBestZero()
                DirectionList[0] = Direction('zero', [0, [next_move[0], next_move[1]]], (0, 0))
            # print(DirectionList[0].name)
            x_y = np.array(
                [DirectionList[0].Pos[0], DirectionList[0].Pos[1]])
            wall_check = self.controller.WallCheck(x_y[0], x_y[1])
            if wall_check[0] == True:
                # need to check if next step is sphere and valid place(not -1 or wall)
                row, col = self.GetBestDist(wall_check[1], x_y)  # get away from wall in -direct move
                x_y[0] = row
                x_y[1] = col
                # map_move = self.controller.cur_pos + x_y
            # self.controller.ms.show_map(map_move)
            xy = self.controller.ms.map_to_position(x_y)
            x_y = self.controller.ms.position_to_map(xy)
            print("my real position: ", x_y)
            xy = xy[1], xy[0]
            # xy = convert(xy[0], xy[1])
            # self.controller.Move(0, 0)
            print("moving to: ", x_y)
            # self.MarkVisited(DirectionList[1], DirectionList[2], DirectionList[3])
            self.Mark(x_y[0], x_y[1], False)
            self.controller.Move(xy[0], xy[1])
            # self.cmu.show_map()
            # print(self.controller.ms.map_to_position(x_y))
            # print(DirectionList[0].Pos[0] * DirectionList[0].value, DirectionList[0].Pos[1] * DirectionList[0].value)
            # self.controller.Move(self.controller.cur_pos[0] + DirectionList[0].Pos[0] * DirectionList[0].value, self.controller.cur_pos[1] + DirectionList[0].Pos[1] * DirectionList[0].value)


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
    # msg_to_cf = "'move':'1', 'psi':'{}, 'x':'{}', 'y':'{}', 'z':'{}'".format(0, 0, 0, 0.5)
    # print(msg_to_cf)
    # exit()
    check = Inspect()
    rate = rospy.Rate(10)
    rospy.Subscriber("/odom", Odometry, check.controller.GetPos)
    # rospy.Subscriber("/move_base/global_costmap/costmap", OccupancyGrid, check.callbackB)

    while not rospy.is_shutdown():
        # Inspect.controller.ms.show_map()
        # rospy.Subscriber("/move_base/global_costmap/costmap", OccupancyGrid, check.callbackB)
        # rospy.Subscriber("/odom", Odometry, check.GetPos)
        # check.Move(0,0) #need to make suitable change to the coordinatae
        check.chooseDirection()
        # rospy.spin()
        # check.controller.ms.show_map(check.controller.cur_pos)
        # print('now:!')
        # print(check.MyMap[199][240])
        # found = np.array([199, 240])
        # check.controller.ms.show_map(found)

        # plt.show()
        # np.set_printoptions()
        # original = sys.stdout
        # with open('ben', 'w') as f:
        #     sys.stdout = f
        #     print(check.controller.ms.map_arr - check.MyMap)
        #     f.close()
        # sys.stdout = original
        print(check.controller.cur_pos)
        print(check.MyMap[252][199])
        if check.controller.cur_pos is not None:
            print(check.MyMap[check.controller.cur_pos[0], check.controller.cur_pos[1]])
            print(check.MyMap[check.controller.cur_pos[1], check.controller.cur_pos[0]])
        rate.sleep()

    # rospy.spin()
    # exec_mode = sys.argv[1]
    # print('exec_mode:' + exec_mode)
    # if exec_mode == 'cleaning':
    #     vacuum_cleaning()
    # elif exec_mode == 'inspection':
    #     inspection()
    # elif exec_mode == 'inspection_advanced':
    #     inspection_advanced()
    # else:
    #     print("Code not found")
    #     raise NotImplementedError
