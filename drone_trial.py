#!/usr/bin/env python3


import numpy as np
import rospy
import sys
import time
import matplotlib.pyplot as plt

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
from darknet_ros_msgs.msg import BoundingBoxes
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose, Point, Quaternion
from gazebo_msgs.msg import ContactsState
from std_msgs.msg import Bool
import tf

person_detected = Bool()
position = Odometry()
vel = Float32()
vel_update_pub = rospy.Publisher('/drone_update_vel', Float32, queue_size=1)
collision = False

# Lists to store drone positions and collisions
drone_positions = []
collision_points = []
log_filename = "drone_mission_log.txt"

def log_data(message):
    with open(log_filename, "a") as log_file:
        log_file.write(message + "\n")

def updatePose(data):
    global position, drone_positions
    position.pose.pose = data.pose.pose
    drone_positions.append((data.pose.pose.position.x, data.pose.pose.position.y, rospy.get_time()))
    log_data(f"Drone position: ({data.pose.pose.position.x}, {data.pose.pose.position.y}) at time {rospy.get_time()}")
    
def checkGoal(goal):
    eps=0.15
    x_diff=(position.pose.pose.position.x-goal[0])
    y_diff=(position.pose.pose.position.y-goal[1])
    z_diff=0##(position.pose.pose.position.z-goal[2])
    if (np.sqrt(x_diff**2+y_diff**2+z_diff**2))<eps:
        return True
    else:
        return False
        
def collisionDetector(data):
    global collision, collision_points
    if len(data.states) > 0:
        collision = True
        collision_points.append((position.pose.pose.position.x, position.pose.pose.position.y, rospy.get_time()))
        log_data(f"Collision detected at ({position.pose.pose.position.x}, {position.pose.pose.position.y}) at time {rospy.get_time()}")

def detectedPerson(data):
    global person_detected, vel
    for box in data.bounding_boxes:
        if box.Class == "person":
            vel.data = 0.2
            vel_update_pub.publish(vel)
            person_detected.data = True
            log_data("Person detected, reducing speed to 0.2")
            break

def plot_drone_journey():
    plt.figure(figsize=(10, 6))
    if drone_positions:
        x, y, _ = zip(*drone_positions)
        plt.plot(x, y, linestyle='-', label='Drone Path')
    if collision_points:
        cx, cy, _ = zip(*collision_points)
        plt.scatter(cx, cy, color='red', label='Collisions', zorder=3)
    
    plt.xlabel('X Position')
    plt.ylabel('Y Position')
    plt.title('Drone Journey')
    plt.legend()
    plt.grid()
    plt.savefig("drone_journey.png")
    plt.show()


if __name__ == '__main__':
    human_present=int(sys.argv[1])
    state_msg=ModelState()
    state_msg.model_name='person'
    if human_present==1:
        state_msg.pose.position.x=-0.5
        state_msg.pose.position.y=3.25
        state_msg.pose.position.z=0.0
        state_msg.pose.orientation.x=0
        state_msg.pose.orientation.y=0
        state_msg.pose.orientation.z=0.0
        state_msg.pose.orientation.w=1.0
    else:
        state_msg.pose.position.x=10.0
        state_msg.pose.position.y=10.0
        state_msg.pose.position.z=0.0
        state_msg.pose.orientation.x=0
        state_msg.pose.orientation.y=0
        state_msg.pose.orientation.z=0.0
        state_msg.pose.orientation.w=1.0
    rospy.wait_for_service('/gazebo/set_model_state')
    set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
    resp = set_state(state_msg)

    rospy.init_node("drone_sim", anonymous=True)
    move_base_pub=rospy.Publisher('/move_base_simple/goal',PoseStamped, queue_size=1)
    pos_val=rospy.Subscriber('/mavros/global_position/local', Odometry, updatePose, queue_size=1)
    land_command_pub=rospy.Publisher('land_command', Float32, queue_size=1)

    vel.data=0.5
    vel_update_pub.publish(vel)
    detect_time=-1
    person_detected.data=False
    human_detector_sub=rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, detectedPerson, queue_size=1)
    collision_detector=rospy.Subscriber('/gazebo_bumper', ContactsState, collisionDetector, queue_size=1)

    loop_rate=rospy.Rate(20)
    for i in np.arange(10):
        loop_rate.sleep()
        vel_update_pub.publish(vel)
    goal=PoseStamped()
    for i in np.arange(10):
        loop_rate.sleep()
        move_base_pub.publish(goal)

    loop_rate.sleep()

    waypoints=np.array([[-3,8.0,1.65], [0,7,1.65], [0,0,1.65]])
    waypoint=0
    start_time=rospy.get_time()
    start_detect_time=rospy.get_time()
    while not rospy.is_shutdown():
        if collision:
            print("Collision detected, aborting mission.")
            log_data("Mission aborted due to collision.")
            plot_drone_journey()
            break
        
        if person_detected.data and detect_time<0:
            detect_time=rospy.get_time()-start_detect_time
        goal.pose.position.x=waypoints[waypoint,0]
        goal.pose.position.y=waypoints[waypoint,1]
        goal.pose.position.z=waypoints[waypoint,2]
        move_base_pub.publish(goal)
        if checkGoal(waypoints[waypoint]):
            if waypoint+1<len(waypoints):
                waypoint+=1
            else:
                land=Float32()
                if collision:
                    print("Collision during mission")
                else:
                    print("No collision during mission")
                if detect_time==-1:
                    print("No human presence detected during mission")
                else:
                    print("Human presence detected at time  ", detect_time)
                land_command_pub.publish(land)
                end_time=rospy.get_time()
                final_time=end_time-start_time
                print("Final time to complete mission:  ", final_time)
                plot_drone_journey()
                break
        loop_rate.sleep()
    
      

