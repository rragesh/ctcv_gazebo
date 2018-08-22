#!/usr/bin/env python
import rospy
import math
import actionlib
import tf
from std_msgs.msg import Int8
from sensor_msgs.msg import LaserScan
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from ctcv_gazebo.srv import dock
from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid
#Global declarations
global check_map
global cmd_vel_pub
global bat_level
global x_off
global y_off
global quat
global dock_msg

#Initialization
cmd_vel_pub   = rospy.Publisher('cmd_vel', Twist, queue_size=1)
dock_response_pub = rospy.Publisher("dock_response",Int8)
goal = MoveBaseGoal()
dock_msg = Int8()

#Constant values
bat_level     =  12.0
bat_thresh    =  11.5
stop_distance =  0.45
offset        =  0.60
x_origin      = -29.675 
y_origin      = -7.4
robot_width   = 0.50
win_off       = int(robot_width*20)
count_max     = (2*win_off)**2 


dock_msg = -1

# function that handles the robot when the docking is missed(WORST CASE SCENARIO) 
def docking_missed():
    rospy.loginfo("Second attempt to dock robot")
    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    client.wait_for_server()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x_off
    goal.target_pose.pose.position.y = y_off
    goal.target_pose.pose.orientation.x = quat[0]
    goal.target_pose.pose.orientation.y = quat[1]
    goal.target_pose.pose.orientation.z = quat[2]
    goal.target_pose.pose.orientation.w = quat[3]
    client.send_goal(goal)
    wait = client.wait_for_result()
    if not wait:
        rospy.logerr("Action server not available!")
    else:
        result = client.get_result()
    if result:
        rospy.loginfo("Docking begins")
        rospy.Subscriber("/scan", LaserScan, laser_callback)
        rate = rospy.Rate(10)
        return True 
    else:
        return False    

#Callback for battery node to check for successful docking
def battery_callback(battery_data):
    bat_level = battery_data

#Callback for laser scanner to locate the docking station and dock robot
def laser_callback(scan_data):
    if dock_msg == 1:
        #Create 3 sectors of 30 degrees each
        range_left = scan_data.ranges[14:44]
        range_mid1 = scan_data.ranges[0:14]
        range_mid2 = scan_data.ranges[345:359]
        range_mid = range_mid1+range_mid2
        range_right = scan_data.ranges[299:329]
        #Take the closest distance to the wall
        sensor_left = min(range_left)
        sensor_mid = min(range_mid)
        sensor_right = min(range_right)
        
        cmd_vel_msg = Twist()

        if (sensor_left - 0.05  > sensor_right) and (sensor_left > stop_distance) :
            rospy.loginfo(sensor_left)
            cmd_vel_msg.linear.x = 0.0
            cmd_vel_msg.angular.z = -0.1
            rospy.loginfo("right_turn")
            cmd_vel_pub.publish(cmd_vel_msg)

            if sensor_mid > stop_distance:
                rospy.loginfo(sensor_mid)
                cmd_vel_msg.linear.x = -0.09 
                cmd_vel_msg.angular.z = 0.0
                rospy.loginfo("reverse") 
                cmd_vel_pub.publish(cmd_vel_msg)

            elif sensor_mid < stop_distance:
                rospy.loginfo(sensor_mid)
                cmd_vel_msg.linear.x = 0.0
                cmd_vel_msg.angular.z = 0.0
                cmd_vel_pub.publish(cmd_vel_msg)
                rospy.loginfo("Stopping")
                #Checking for sudden battery voltage spike (which means docked properly)
                if bat_level > bat_thresh:
                    cmd_vel_msg.linear.x = 0.0
                    cmd_vel_msg.angular.z = 0.0
                    cmd_vel_pub.publish(cmd_vel_msg)
                    rospy.loginfo("Charging Started")
                    
                else: 
                    docking_missed()
        
        elif (sensor_right - 0.05 > sensor_left) and (sensor_right > stop_distance) :
            rospy.loginfo(sensor_right)
            cmd_vel_msg.linear.x = 0.0
            cmd_vel_msg.angular.z = 0.1
            rospy.loginfo("left_turn")
            cmd_vel_pub.publish(cmd_vel_msg)
            if sensor_mid > stop_distance:
                rospy.loginfo(sensor_mid)
                cmd_vel_msg.linear.x = -0.09 
                cmd_vel_msg.angular.z = 0.0
                rospy.loginfo("reverse") 
                cmd_vel_pub.publish(cmd_vel_msg)

            elif sensor_mid < stop_distance:
                rospy.loginfo(sensor_mid)
                cmd_vel_msg.linear.x = 0.0
                cmd_vel_msg.angular.z = 0.0
                rospy.loginfo("Stopping")
                #Checking for sudden battery voltage spike (which means docked properly)
                if bat_level > bat_thresh:
                    cmd_vel_msg.linear.x = 0.0
                    cmd_vel_msg.angular.z = 0.0
                    cmd_vel_pub.publish(cmd_vel_msg)
                    rospy.loginfo("Charging Started")
                    
                else: 
                    docking_missed()
        else:
            cmd_vel_msg.linear.x = 0.0
            cmd_vel_msg.angular.z = 0.0
            cmd_vel_pub.publish(cmd_vel_msg)


def dock_service_CB(request):
    #The request is receives as a service call from terminal as -- rosservice call dock_service ...pose...
    x = request.pose.x
    y = request.pose.y
    theta = request.pose.theta
    quat = tf.transformations.quaternion_from_euler(0, 0,theta)
    x_off = x + offset*math.cos(theta)
    y_off = y + offset*math.sin(theta)
    # Checking if the offset position is not an obstacle or close to it - Error check 1   
    def map_callback(map_data):
        global dock_msg
        w = map_data.info.width
        h = map_data.info.height
        # Origin of map is not (0,0)
        x_map_off = int((x_off*20)+ 593.5)
        y_map_off = int((y_off*20)-148)

        count = 0
        for i in range( x_map_off - win_off, x_map_off + win_off):
            for j in range( y_map_off - win_off, y_map_off + win_off):
                idx = i + w*j
                if map_data.data[idx] == 0:
                    count+=1

        if count == count_max:
            check_map = True 
            rospy.loginfo("..GOAL IS IN THE MAP..")
            dock_msg = -1
        else:
            check_map = False
            rospy.loginfo("Try a different docking spot")

        if check_map == True:
            client.send_goal(goal)
            wait = client.wait_for_result()
            if not wait:
                rospy.loginfo("Action server not available!")
            else:
                result = client.get_result()

            if result:
                dock_msg = 1                     #Published a topic to access the response outside the scope of this function
                dock_response_pub.publish(dock_msg)
                rospy.loginfo("Docking begins")
                rate = rospy.Rate(10)
            else:
                dock_msg = 0
                dock_response_pub.publish(dock_msg)
        else:
            rospy.logerr("THE LOCATION CAN'T BE REACHED!")
            rospy.loginfo("Try different docking position")
        
    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    client.wait_for_server()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x_off
    goal.target_pose.pose.position.y = y_off
    goal.target_pose.pose.orientation.x = quat[0]
    goal.target_pose.pose.orientation.y = quat[1]
    goal.target_pose.pose.orientation.z = quat[2]
    goal.target_pose.pose.orientation.w = quat[3]

    rospy.Subscriber("/map", OccupancyGrid, map_callback)
    rospy.Subscriber("/scan", LaserScan, laser_callback)
    rate = rospy.Rate(10)
    #To create a block
    while dock_msg == -1:
        a = 1
    #Response to service call
    if dock_msg == 1:
        return True
    else:
        return False   

def main():
    rospy.init_node('recharge')
    try: 
        reply = rospy.Service('dock_service', dock, dock_service_CB )
        rospy.loginfo(reply)
    except rospy.ROSInterruptException:
        rospy.loginfo("Docking Failed!!")
    rospy.spin()

if __name__ == '__main__':
    main()             
    