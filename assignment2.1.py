#!/usr/bin/env python

import rospy
import numpy as np
from nav_msgs.msg import OccupancyGrid, MapMetaData, Odometry
import Manager as mn
import matplotlib.pyplot as plt

from nav_msgs.msg import OccupancyGrid
from map_msgs.msg import OccupancyGridUpdate

TH = 7
WALL = 80
STEP_SIZE = 60
SEEN_SIZE = 29
SPHERE_SIZE = 20
SPHERE_INDICATE = 45
WALL_DIST = 5


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
            if self.InMyArea(x_dest, y_dest, current_square) == True:
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

    def InMyArea(self, row, col, square_size):
        x_dis = abs(row - self.controller.cur_pos[0])
        y_dis = abs(col - self.controller.cur_pos[1])
        if x_dis < square_size and y_dis < square_size:
            return True
        return False

    def FindBestZero(self):
        for row in range(self.controller.ms.shape[0]):
            for col in range(self.controller.ms.shape[1]):
                if self.controller.Valid(col, row) == True and self.MyMap[col][row] == 0 and (
                        col, row) not in self.notAvailAble and self.InMyArea(col, row, SEEN_SIZE) == False and \
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
