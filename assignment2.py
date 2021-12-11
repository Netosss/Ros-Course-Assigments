#!/usr/bin/env python

import Manager
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
    check=Manager()
    check.ms.show_map()
    rate=rospy.Rate(10)
    # rospy.Subscriber("/move_base/global_costmap/costmap", OccupancyGrid, check.callbackB)
    rospy.Subscriber("/odom", Odometry, check.GetPos)
    check.Move(0,0) #need to make suitable change to the coordinatae
    rospy.spin()
    rate.sleep
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
