#!/usr/bin/env python
import rospy
import sys
from nav_msgs.srv import GetMap
import multi_move_base
from std_msgs.msg import String
import ast
from nav_msgs.msg import Odometry, Path
import numpy as np
import matplotlib.pyplot as plt
import time
from math import sqrt

time_limit = float(2 * 60)  # [sec]
close_to_dirt = 4  # take radius from mohammad
BETA_VALUE_INIT = 0.0
ALPHA_VALUE_INIT = 0.0


# handle the game status, positions, total_dirt, and dirt each player
class PerformState:
    def __init__(self, position, time_, amount, adversary_position, adversary_amount, current_dirt):
        self.position_ = position
        self.time_left = time_
        self.cur_amount = amount
        self.adversary_amount_ = adversary_amount
        self.adversary_position_ = adversary_position
        self.dirt_list = current_dirt
    ##########################################################################################################


class SearchAlgos:
    def __init__(self, utility, succ, perform_move):
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
        Options = state.dirt_list
        move = None
        if depth == 0 or not Options:
            return self.utility(state), move
        if maximizing_player:
            cur_max = float('-inf')
            for child in Options:
                new_state = self.perform_move(state, child, maximizing_player)
                best_val = self.search(new_state, depth - 1, not maximizing_player, alpha, beta)
                value = best_val[0]
                if value > cur_max:
                    cur_max = value
                    move = child
                alpha = max(value, alpha)
                if cur_max >= beta:
                    return float('inf'), move
            return cur_max, move
        else:
            cur_min = float('inf')
            for child in Options:
                new_state = self.perform_move(state, child, maximizing_player)
                best_val = self.search(new_state, depth - 1, not maximizing_player, alpha, beta)
                value = best_val[0]
                cur_min = min(value, cur_min)
                beta = min(cur_min, beta)
                if cur_min <= alpha:
                    return float('-inf'), move
            return cur_min, None


class MapService(object):
    def __init__(self, agent_num):
        """
        Class constructor
        """
        rospy.wait_for_service('tb3_%d/static_map' % int(agent_num))
        static_map = rospy.ServiceProxy('tb3_%d/static_map' % int(agent_num), GetMap)
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


##################################################################################################
def dist(point1, point2):
    return sqrt((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2)


def Valid(i, j, shape):
    if 0 <= i < shape[0] and 0 <= j < shape[1]:
        return True
    else:
        return False


def map_dist(position, goal, ms: MapService):
    shape = [ms.map_data.info.height, ms.map_data.info.width]
    map_a = ms.map_arr


class PositionHandler:
    def __init__(self, player_num, ms):
        self.odom_sub = rospy.Subscriber('tb3_%d/odom' % int(player_num), Odometry, self.pos_callback)
        self.m_cur_pos = None
        self.map_service = ms

    def pos_callback(self, data):
        cur = np.array([data.pose.pose.position.x, data.pose.pose.position.y])
        pose_ = self.map_service.position_to_map(cur)
        self.m_cur_pos = np.array([int(pose_[1]), int(pose_[0])])


class AlgorithmHandler:
    def __init__(self, ms, dirt):
        self.map_service = ms
        self.dirt = dirt
        self.alpha_beta = AlphaBeta(self.heuristic, self.dirt, self.perform_move)

    def CalculateTime(self, position, goal):
        # calculate estimate time
        dist_to_goal = map_dist(position, goal, self.map_service)
        print("to pass")
        return 10.0

    def perform_move(self, state, goal, player):
        new_position = self.map_service.position_to_map(goal)
        state.dirt_list.remove(goal)
        if player == True:
            new_time = state.time_left - self.CalculateTime(state.position_, new_position)
            return PerformState(new_position, new_time, state.cur_amount + 1, state.adversary_position_,
                                state.adversary_amount_, state.dirt_list)
        else:
            new_time = state.time_left - self.CalculateTime(state.adversary_position_, new_position)
            return PerformState(state.position_, new_time, state.cur_amount, new_position,
                                state.adversary_amount_ + 1,
                                state.dirt_list)

    def heuristic(self, state):
        print("nothing")
        # if state.position_
        # we need to take in account : this dirt is closer to me (consider full path and not euclidean),
        # Is this dirt is close to many another dirt
        # we can make it with the time left
        # I already won
        return 1


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
        while self.dirt_updated == False:
            continue
        self.nav_sub = rospy.Subscriber("tb3_%d/move_base/NavfnROS/plan" % int(player), Path, self.path_callback)
        self.just_check_move = False

    def path_callback(self, data):
        if self.just_check_move == False:
            return
        sum_ = 0
        for i in range(len(data.poses) - 1):
            sum_ += sqrt(pow((data.poses[i + 1].pose.position.x - data.poses[i].pose.position.x), 2) + pow(
                (data.poses[i + 1].pose.position.y - data.poses[i].pose.position.y), 2))
        print(sum_)  # here we would like to get the calculate for time estimate purpose
        back_to = self.ms.map_to_position(np.array(self.player_pos_wrapper.m_cur_pos))
        self.make_move(back_to)
        self.just_check_move = False

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

    def TurnTime(self, time_for_turn):
        return 4  # should return depth

    def choose_move(self, total_time_for_turn):
        start = time.time()
        best_move = None
        algo = AlgorithmHandler(self.ms, self.dirt)
        cur = PerformState(self.player_pos_wrapper.m_cur_pos, time_limit - (time.time() - self.start_time),
                           self.cur_amount, self.adversary_pos_wrapper.m_cur_pos, self.adversary_amount,
                           self.dirt.copy())
        depth = self.TurnTime(total_time_for_turn)  # calculate max possible depth by given time_limit , depth is int
        for i in range(depth):
            if i == 0:
                continue
            iteration_start = time.time()
            best = algo.alpha_beta.search(cur, i, True, float('-inf'), float('inf'))
            if best[1] != None:
                best_move = best[1]
            cur_time = time.time()
            if (cur_time - iteration_start) * 4.1 > total_time_for_turn - (cur_time - start):
                break
        if best_move is None and len(self.dirt) > 0:
            best_move = self.dirt[0]
        return best_move

    def make_move(self, move):
        move_to = move
        if move_to is None:
            move_to = self.player_pos_wrapper.m_cur_pos
        result = multi_move_base.move(agent_id=int(self.player), x=move_to[0], y=move_to[1])
        # assume amount updating in call back == assume dirt taken updated in topic /dirt


def vacuum_cleaning(id_, agent_max_vel):
    # sub = GameManager(0)
    # move = sub.choose_move(time_limit)
    # sub.make_move(move)

    x = 0.4
    y = 0.3
    print('cleaning (%d,%d)' % (x, y))
    result = multi_move_base.move(agent_id=1-int(id_), x=x, y=y)

    print('moving agent %d' % int(id_))
    x = 0
    y = 1
    print('cleaning (%d,%d)' % (x, y))
    result = multi_move_base.move(agent_id=int(1 - int(id_)), x=x, y=y)


def inspection(agent_1_max_vel, agent_2_max_vel):
    print('start inspection')
    raise NotImplementedError


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
