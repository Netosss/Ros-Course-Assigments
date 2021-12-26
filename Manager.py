#!/usr/bin/env python

from numpy.lib.function_base import copy
import rospy
from nav_msgs.srv import GetMap
import numpy as np
import matplotlib.pyplot as plt
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped
import copy
import signal
import dynamic_reconfigure.client
from nav_msgs.msg import Odometry

STEP_SIZE = 60
WALL_DIST = 5


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
        rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, self.initial_pose)
        self.ms = MapService()
        self.cur_pos = None
        self.my_map_ = copy.deepcopy(self.ms.map_arr)
        rospy.Subscriber("/odom", Odometry, self.GetPos)
        self.initial_pose = None  # check if need to use instead of odom?
        self.rc_DWA_client = dynamic_reconfigure.client.Client('/move_base/DWAPlannerROS/')
        self.rc_DWA_client.update_configuration({"max_vel_trans": 2.5})

    def initial_pose(self, msg):
        self.initialpose = msg.pose.pose

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

    def Move(self, x, y, w=1.0, use=True):
        if use == True:
            max_time = 25  # 45 secs before starting a new move instead this one
            signal.signal(signal.SIGALRM, signal_handler)
            signal.alarm(max_time)
        try:
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
        except Exception as msg:
            print('move time done,calculate next move')

    def OppositeDirection(self, direction):
        if abs(direction[0]) == 1:
            opp = [0, 1]
        else:
            opp = [1, 0]
        return opp

    def GetPos(self, data):
        cur = np.array([data.pose.pose.position.x, data.pose.pose.position.y])
        pose_ = self.ms.position_to_map(cur)
        self.cur_pos = np.array([int(pose_[1]), int(pose_[0])])

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

