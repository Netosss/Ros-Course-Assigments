#!/usr/bin/env python
import numpy as np
import rospy
from nav_msgs.msg import OccupancyGrid, MapMetaData, Odometry
from matplotlib import pyplot as plt

WALL = 50
STEP_SIZE = 60


class Direction:
    def __init__(self, name, value):
        self.name = name
        self.value = value

    def getVal(self):
        return self.value


class Ben:
    def __init__(self):
        self.map = None
        self.width = 0
        self.height = 0
        self.origin = None
        self.pose = [0, 0]

    def callbackA(self, data: OccupancyGrid):
        #if (self.width != 0 or self.height != 0) and self.map is None:
         #   self.map = np.reshape(data.data, (self.width, self.height))
        plt.imshow(self.map, interpolation='nearest')
        plt.show()

    def callbackB(self, data: OccupancyGrid):
        if self.map is not None:
            arr = np.reshape(data.data, (384, 384))
            # arr = np.reshape(data.data, (60, 60))
            i = 0
            while i < 300:
                arr[7][i] = 100
                i = i + 1
            # plt.imshow(arr, interpolation='nearest')
            # plt.show()

    def callbackC(self, data: MapMetaData):
        self.width = data.width
        self.height = data.height
        self.origin = data.origin

    def GetPos(self, data: Odometry):
        self.pose[0] = data.pose.pose.position.x
        self.pose[1] = data.pose.pose.position.y

    def step(self, i, j):
        row = i
        col = j
        distance = 0
        while self.x + row >= 0 and self.y + col >= 0 and self.map[self.x + row][self.y + col] < WALL:
            distance = distance + 1
            col = col + j
            row = row + i
        if self.x + row >= 0 and self.y + col >= 0 and self.map[self.x + row][self.y + col] == -1:
            return self.width  # big number! -1 is a place that we already visited
        if distance - STEP_SIZE <= 0:
            return self.width  # big number!
        return distance - STEP_SIZE

    def chooseDirection(self):
        left = Direction('left', self.step(0, -1))
        right = Direction('right', self.step(0, 1))
        up = Direction('up', self.step(1, 0))
        down = Direction('down', self.step(-1, 0))
        DirectionList = [left, right, up, down]
        DirectionList.sort(key=Direction.getVal)
        print(DirectionList[0].name)


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
    ben = Ben()
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/move_base/local_costmap/costmap", OccupancyGrid, ben.callbackA)
    rospy.Subscriber("/odom", Odometry, ben.GetPos)
    rospy.Subscriber("/map", OccupancyGrid, ben.callbackA)
    rospy.Subscriber("/map_metadata", MapMetaData, ben.callbackC)
    #ben.chooseDirection()
    rospy.spin()

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
