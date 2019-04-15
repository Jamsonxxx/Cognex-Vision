#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import rospy
import tf
import socket

from tf.transformations import * 

import moveit_commander
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

from geometry_msgs.msg import PoseStamped, Pose
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class MoveItIkDemo:
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('moveit_ik_demo',anonymous=True)
        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()
        arm = moveit_commander.MoveGroupCommander('arm')

        end_effector_link = arm.get_end_effector_link()
        reference_frame = 'base_link'
        arm.set_pose_reference_frame(reference_frame)

        arm.allow_replanning(True)

        arm.set_goal_position_tolerance(0.001)
        arm.set_goal_orientation_tolerance(0.005)

        #Socket
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        print("create socket succ!")
        s.connect(("192.168.1.110", 3000))
        data = 0.5
        s.send("101010101010101".decode('gbk').encode('utf-8'))

        #arm.set_named_target('all-zeros')
        #arm.go()
        #rospy.sleep(2)

        while True:
            if not data:
                break

            recv_data = s.recv(128)
            data = recv_data.decode('gbk').encode('utf-8')
            print 'omg!!',data
            temp = data.split(',')

            rpy = [float(temp[3]),float(temp[4]),float(temp[5])]
            point = tf.transformations.quaternion_from_euler(rpy[0],rpy[1],rpy[2])
            target_pose = PoseStamped()
            target_pose.header.frame_id = reference_frame
            target_pose.header.stamp = rospy.Time.now()
            target_pose.pose.position.x =float(temp[0])
            target_pose.pose.position.y =float(temp[1])
            target_pose.pose.position.z =float(temp[2])
            target_pose.pose.orientation.x = point[0]
            target_pose.pose.orientation.y = point[1]
            target_pose.pose.orientation.z = point[2]
            target_pose.pose.orientation.w = point[3]

            arm.set_start_state_to_current_state()
            arm.set_pose_target(target_pose, 'tool_0')

            traj = arm.plan()

            arm.execute(traj)
                
            print target_pose.pose.position
            print target_pose.pose.orientation
            #arm.set_named_target('all-zeros')
            arm.go()

            rospy.sleep(0)
        

        # 关闭并退出moveit
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)

if __name__ == "__main__":
    MoveItIkDemo()
