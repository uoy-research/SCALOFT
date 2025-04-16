#!/usr/bin/env python3
import numpy as np
import rospy
import time

from std_srvs.srv import Empty
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from gazebo_msgs.srv import GetModelState
from gazebo_msgs.msg import ModelState 
from gazebo_msgs.srv import SetModelState
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry

position=Odometry()

def start(loop_rate):
    while not rospy.is_shutdown():
        loop_rate.sleep()

def smsClient(model_name, pos):
    state_msg = ModelState()
    state_msg.model_name = model_name
    state_msg.pose.position.x = pos[0]
    state_msg.pose.position.y = pos[1]
    state_msg.pose.position.z = pos[2]
    state_msg.pose.orientation.x = 0
    state_msg.pose.orientation.y = 0
    state_msg.pose.orientation.z = 0
    state_msg.pose.orientation.w = 0

    rospy.wait_for_service('/gazebo/set_model_state')
    try:
        set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        resp = set_state( state_msg )

    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)

def updatePose(data):
    position.pose.pose=data.pose.pose

def checkGoal(goal):
    eps=0.1
    x_diff=(position.pose.pose.position.x-goal[0])
    y_diff=(position.pose.pose.position.y-goal[1])
    z_diff=0##(position.pose.pose.position.z-goal[2])
    if (np.sqrt(x_diff**2+y_diff**2+z_diff**2))<eps:
        return True
    else:
        return False

if __name__=='__main__':
    rospy.init_node("drone_sim", anonymous=True)
    move_base_pub=rospy.Publisher('/move_base_simple/goal',PoseStamped, queue_size=1)
    pos_val=rospy.Subscriber('/mavros/global_position/local', Odometry, updatePose, queue_size=1)
    vel_update_pub=rospy.Publisher('/drone_update_vel', Float32, queue_size=1)
    vel=Float32()
    vel.data=0.5
    loop_rate=rospy.Rate(20)
    for i in np.arange(10):
        loop_rate.sleep()
        vel_update_pub.publish(vel)
    goal=PoseStamped()
    for i in np.arange(10):
        loop_rate.sleep()
        move_base_pub.publish(goal)

    loop_rate.sleep()

    waypoints=np.array([[0,0,1.65],[0,7,1.65],[-3,8.0,1.65]])
    waypoint=0
    while not rospy.is_shutdown():
        goal.pose.position.x=waypoints[waypoint,0]
        goal.pose.position.y=waypoints[waypoint,1]
        goal.pose.position.z=waypoints[waypoint,2]
        move_base_pub.publish(goal)
        if checkGoal(waypoints[waypoint]):
            print("Travelling to waypoint "+str(waypoint)+"/"+str(len(waypoints)))
            if waypoint+1<len(waypoints):
                waypoint+=1
            else:
                break
        loop_rate.sleep()
    print("Finished!")