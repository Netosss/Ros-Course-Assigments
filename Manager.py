#!/usr/bin/env python

from numpy.lib.financial import rate
from numpy.lib.function_base import copy
import move_base
import rospy
import sys
from nav_msgs.srv import GetMap
import numpy as np
from nav_msgs.msg import OccupancyGrid, MapMetaData, Odometry
import matplotlib.pyplot as plt
# Brings in the SimpleActionClient
import actionlib
# Brings in the .action file and messages used by the move base action
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import copy

WALL = 50
STEP_SIZE = 60
WALL_DIST = 5


class Direction:
    def __init__(self, name, value):
        self.name = name
        self.value = value


class MapService(object):

    def __init__(self):
        """
        Class constructor
        """
        rospy.wait_for_service('static_map')
        static_map = rospy.ServiceProxy('static_map', GetMap)
        self.map_data = static_map().map
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
    def __init__(self):
        rospy.init_node('movebase_client_py')
        self.move_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        # rospy.init_node('get_map_example')
        self.ms = MapService()
        self.cur_pos = None
        self.my_map_ = copy.deepcopy(self.ms.map_arr)

    def WallCheck(self, row, col):
        directions = [(1, 0), (0, 1), (-1, 0), (0, -1)]
        for direction in directions:
            for i in range(WALL_DIST):
                if self.Valid(row + direction[0] * i, col + direction[1] * i) and \
                        self.my_map_[row + direction[0] * i][col + direction[1] * i] == 100:
                    return [True, direction]
        return [False, (0, 0)]

    def Move(self, x, y, w=1.0):
        self.move_client.wait_for_server()
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
        self.move_client.send_goal(goal)
        rospy.loginfo("New goal command received!")
        # Waits for the server to finish performing the action.
        wait = self.move_client.wait_for_result()
        # If the result doesn't arrive, assume the Server is not available
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
            # Result of executing the action
            return self.move_client.get_result()

    def GetPos(self, data):
        cur = np.array([data.pose.pose.position.x, data.pose.pose.position.y])
        pose_ = self.ms.position_to_map(cur)
        self.cur_pos = np.array([int(pose_[1]), int(pose_[0])])
        # print(self.cur_pos)
        # print(self.ms.position_to_map(cur))

    def Valid(self, i, j):
        info = self.ms.map_data.info
        return i >= 0 and i < info.height and j >= 0 and j < info.width

# if __name__ == '__main__':
#     check=Manager()
#     check.ms.show_map()
#     rate=rospy.Rate(10)
#     # rospy.Subscriber("/move_base/global_costmap/costmap", OccupancyGrid, check.callbackB)
#     rospy.Subscriber("/odom", Odometry, check.GetPos)
#     check.Move(0,0) #need to make suitable change to the coordinatae
#     rospy.spin()
#     rate.sleep
