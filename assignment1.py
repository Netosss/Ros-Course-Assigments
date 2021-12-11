#! /usr/bin/env python
import time
import rospy
from math import sqrt,pi
from geometry_msgs.msg import Twist
from rospy.rostime import Time
from rospy.timer import sleep

###robot params
speed = float(0.1) #you can choose linear speed [0.05,1]

##circle params
radius = float(0.4) #you can choose radius [0.25,0.5]
ang_speed = float(speed/radius)
T = 2*pi/ang_speed 
circle_treshold = sqrt(T)/2.65
circle_duration = T + circle_treshold

##square params
square_side = 0.8
degree_treshold = pi/12
angular_speed = 0.1 
turn_degree = pi/2 

def stop_move():
    rospy.init_node('twist_publisher')
    pub = rospy.Publisher('/cmd_vel',Twist, queue_size=1)
    pub.publish(Twist())
    rate = rospy.Rate(10)
    rate.sleep()
    
def move_line():
    rospy.init_node('twist_publisher')
    twist = Twist()
    twist.angular.z = 0 #0.5
    twist.linear.x = 0.5
    pub = rospy.Publisher('/cmd_vel',Twist , queue_size=1)
    r = rospy.Rate(2)
    while not rospy.is_shutdown(): 
        pub.publish(twist)
        r.sleep()


def move_circle():
    rospy.init_node('twist_publisher')
    twist=Twist()
    rate = rospy.Rate(10)
    twist.linear.x = speed 
    twist.angular.z = ang_speed
    start = time.time()
    pub = rospy.Publisher('/cmd_vel',Twist, queue_size=1)
    while time.time()-start < circle_duration and not rospy.is_shutdown():
        pub.publish(twist)
        rate.sleep()

    stop_move()
    print('COMPLETE')

def turn(turn_degree):
    rospy.init_node('twist_publisher')
    pub = rospy.Publisher('/cmd_vel',Twist, queue_size=1)
    msg = Twist()
    rate = rospy.Rate(10)
    start = time.time()
    current_angle = 0
    while current_angle< turn_degree and not rospy.is_shutdown():
        msg.angular.z = angular_speed
        pub.publish(msg)
        rate.sleep()
        current_angle =angular_speed*(time.time()-start)
    stop_move()

def move_side(side):
    rospy.init_node('twist_publisher')
    pub = rospy.Publisher('/cmd_vel',Twist, queue_size=1)
    msg = Twist()
    rate = rospy.Rate(2)
    start = time.time()
    current_dist = 0
    while current_dist<side and not rospy.is_shutdown():
        msg.linear.x=speed
        if side-current_dist<0.15 or current_dist<0.1:
            msg.linear.x=speed/3
        if side-current_dist<0.05:
            msg.linear.x=speed/6
        pub.publish(msg)
        current_dist=speed*(time.time()-start)
        rate.sleep()
    #stop_move()
def move_square():
    for i in range(4):
        move_side(square_side)
        turn(turn_degree + 1*degree_treshold)
        i+=1
    print('COMPLETE')

if __name__ == "__main__":
    #move_circle()
    stop_move()
    #sleep(5)
    move_square()
    # move_line()
