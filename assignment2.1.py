#!/usr/bin/env python

import move_base
import rospy
import sys
import rospy
from map_msgs.msg import OccupancyGridUpdate
import numpy as np
from nav_msgs.msg import OccupancyGrid, MapMetaData, Odometry
import Manager as mn
import matplotlib.pyplot as plt
# Brings in the SimpleActionClient
import actionlib
# Brings in the .action file and messages used by the move base action
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

WALL = 60
STEP_SIZE = 60
SEEN_SIZE = 30
SPHERE_SIZE = 20


class Direction:
    def __init__(self, name, value, Pos):
        self.name = name
        self.value = value
        self.Pos = Pos

    def getVal(self):
        return self.value


from nav_msgs.msg import OccupancyGrid
from map_msgs.msg import OccupancyGridUpdate


    #mohammad code
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
        print('periodically')  # For the student to understand
        shape = msg.height, msg.width
        data = np.array(msg.data).reshape(shape)
        self.cost_map[msg.y:msg.y + shape[0], msg.x: msg.x + shape[1]] = data
        self.show_map()  # For the student to see that it works

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

    def callbackB(self, data: OccupancyGrid):
        arr = np.reshape(data.data, (384, 384))
        # arr = np.reshape(data.data, (60, 60))
        i = 0
        while i < 300:
            arr[7][i] = 100
            i = i + 1
        # plt.imshow(arr, interpolation='nearest')
        # plt.show()
        np.set_printoptions(threshold=sys.maxsize)

    def step(self, i, j):
        currPos = self.controller.cur_pos
        width = self.controller.ms.map_data.info.width
        row = currPos[0]
        col = currPos[1]
        distance = 0
        while self.controller.Valid(row, col) and 0 <= self.MyMap[row][col] < WALL:
            x = self.MyMap[row][col]
            distance = distance + 1
            col = col + j
            row = row + i
        # if self.controller.Valid(currPos[0] + row, currPos[1] + col) and self.MyMap[currPos[0] + row][
        #    currPos[1] + col] == -1:
        #   return width  # big number! -1 is a place that we already visited
        # x = self.MyMap[row][col]
        NonCoverDist = distance - SEEN_SIZE
        if NonCoverDist <= 0 or NonCoverDist < SPHERE_SIZE:
            return width  # big number!
        if NonCoverDist < STEP_SIZE:
            return SEEN_SIZE + int(NonCoverDist / 2)
        return STEP_SIZE

    def chooseDirection(self):
        if self.controller.cur_pos is not None:
            left = Direction('left', self.step(0, -1), (0, -1))
            right = Direction('right', self.step(0, 1), (0, 1))
            up = Direction('up', self.step(1, 0), (1, 0))
            down = Direction('down', self.step(-1, 0), (-1, 0))
            DirectionList = [left, right, up, down]
            DirectionList.sort(key=Direction.getVal)
            print(DirectionList[0].name)
            x_y = np.array(
                [DirectionList[0].Pos[0] * DirectionList[0].value, DirectionList[0].Pos[1] * DirectionList[0].value])
            map_move = self.controller.cur_pos + x_y
            # self.controller.ms.show_map(map_move)
            xy = self.controller.ms.map_to_position(map_move)
            xy = xy[1], -xy[0]
            # xy = convert(xy[0], xy[1])
            # self.controller.Move(0, 0)
            print(xy)
            self.controller.Move(xy[0], xy[1])
            print(self.controller.ms.map_to_position(x_y))
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
    rospy.Subscriber("/move_base/global_costmap/costmap", OccupancyGrid, check.callbackB)

    while not rospy.is_shutdown():
        # Inspect.controller.ms.show_map()
        # rospy.Subscriber("/move_base/global_costmap/costmap", OccupancyGrid, check.callbackB)
        # rospy.Subscriber("/odom", Odometry, check.GetPos)
        # check.Move(0,0) #need to make suitable change to the coordinatae
        check.chooseDirection()
        # rospy.spin()
        check.controller.ms.show_map(check.controller.cur_pos)
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
