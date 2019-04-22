# !/usr/bin/env python
# -*- coding: utf-8 -*-

# 导入ROS_moveit!库
from tf.transformations import * 
import moveit_commander
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from geometry_msgs.msg import PoseStamped, Pose
from tf.transformations import euler_from_quaternion, quaternion_from_euler

# 导入opencv库
import cv2
import numpy as np
from ftplib import FTP
import os
import time
from dateutil import parser

# 导入socket库
import socket


def cv_pic( name1, name2):
  # 焊缝查找部分
  # 读取两帧图像
	Ori_img1 = cv2.imread("pic/" + name1, 0) 
	Ori_img2 = cv2.imread("pic/" + name2, 0)

  # 选取opi区域
	img1 = Ori_img1[200:1000, 400:500]
	img2 = Ori_img2[200:1000, 400:500]

   # 对两帧图像高斯模糊
	img1 = cv2.GaussianBlur(img1, (3, 3), 0)
	img2 = cv2.GaussianBlur(img2, (3, 3), 0)

  # 自适应阈值分割
	img1 = cv2.adaptiveThreshold(img1, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY_INV, 11, 10)
	img2 = cv2.adaptiveThreshold(img2, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY_INV, 11, 10) 

  # 两帧图像and操作
	img = cv2.bitwise_and(img1, img2) 

  # canny边缘检测
	canny = cv2.Canny(img, 100, 200) 

  # 霍夫直线 得到距原点距离和弧度
	# minLineLength=50  
	# maxLineGap=20
	lines = cv2.HoughLines(canny, 1.0, np.pi / 180, 118) 

	result = Ori_img1.copy()

  # 绘制白色直线并计算offset和rev
	for line in lines[0]:
		rho = line[0] 
		theta = line[1]  
		print (rho)
		print (theta)
		if  (theta < (np.pi/4.0)) or (theta > (3.*np.pi/4.0)): 
      # 计算直线与最上方水平线的交点
			pt1 = (int(rho/np.cos(theta)),0)             
			pt2 = (int((rho-result.shape[0]*np.sin(theta))/np.cos(theta)),result.shape[0])
			cv2.line( result, pt1, pt2, (255))             
		else: 
			pt1 = (0,int(rho/np.sin(theta)))             
			pt2 = (result.shape[1], int((rho-result.shape[1]*np.cos(theta))/np.sin(theta)))
			cv2.line(result, pt1, pt2, (255), 1)          
			a = (pt1[1]+pt2[1])/2 - 600
            b = theta

  # 显示图像
	cv2.imshow("img1", Ori_img1)
	cv2.imshow("img_AND", img)
	cv2.imshow("Hough", result)

	cv2.waitKey(0)
	cv2.destroyAllWindows() 

  # 返回偏移与扭转
  return(a, b)

def cognex_ftp():
  # ftp客户端部分
  # 获得目录列表
	names = ftp.nlst() 
  # 列出目录内容      
	ftp.retrlines('LIST')  
    
  # 获取图片文件名
	final_names= [line for line in names if ('bmp' in line) and ('image' in line)]

  # 显示图片文件名
	print final_names[-2], final_names[-1]

  # 下载图片
	file1 = open(final_names[-2], 'wb')
	ftp.retrbinary('RETR '+ final_names[-1], file1.write)
  file2 = open(final_names[-1], 'wb')
	ftp.retrbinary('RETR '+ final_names[-1], file2.write)
    
  # 返回两帧图片的文件名
  return(final_names[-2], final_names[-1])

def cognex_tcp():
    #TCP部分
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM) #第一个参数是满足IP地址协议，第一个参数创建的socket完成TCP协议
    # 建立连接:
    s.connect(("192.168.56.1", 8888))
    print("create socket succ!\n")
    # 接收欢迎消息:
    s.send("101010101010101".decode('gbk').encode('utf-8'))  #客户端向服务器以gbk的编码格式发送
    #发送字符串的时候需要注意，如果是windows上运行的服务器编码格式是gbk，而linux,mac都是utf-8
    recv_data = s.recv(128)  # 客户端接收服务器发来的信息最大长度为1K
    data = recv_data.decode('gbk').encode('utf-8')    #解码
    print 'Input data:',data[5:]
    #处理字符串
        data = data[5:-1]
        temp = data.split(',')
        x=float(temp[0])
        x=(500-x)*0.001
        y=float(temp[1])
        y=((180-y)/180)*3.14

# 初始化
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('moveit_ik_demo',anonymous=True)
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()

# 运动组设定
arm = moveit_commander.MoveGroupCommander('arm')

end_effector_link = arm.get_end_effector_link()
reference_frame = 'base_link'
arm.set_pose_reference_frame(reference_frame)

arm.allow_replanning(True)

# 设置宽容度
arm.set_goal_position_tolerance(0.001)
arm.set_goal_orientation_tolerance(0.005)

arm.set_named_target('all-zeros')
arm.go()
# rospy.sleep(2)

# 初始化窗口
cv2.namedWindow("img1")
cv2.namedWindow("img2") 
cv2.namedWindow("img_AND") 
cv2.namedWindow("Hough") 

# 初始化FTP
ftp = FTP()
timeout = 30
port = 21
# 连接FTP服务器
ftp.connect('192.168.1.110',port,timeout) 
# 登录
ftp.login('admin','admin') 
# 获得欢迎信息
print ftp.getwelcome()  
# 设置FTP路径
ftp.cwd('/')    

# 循环识别并控制机器人运动
while True:
    # 调用ftp和cv部分，返回偏移和扭转
    po = cv_pic(cognex_ftp())
    offset = po[0]
    rev = po[1]

    # 赋值rev
    rpy = [0,1.57,rev]
    print 'rpy:',rpy

    # 赋值偏移到目标位姿
    target_pose = PoseStamped()
    target_pose.header.frame_id = reference_frame
    target_pose.header.stamp = rospy.Time.now()
    target_pose.pose = arm.get_current_pose().pose
    # target_pose.pose.position.x =0.5
    target_pose.pose.position.y = offset
    # target_pose.pose.position.z =0.5
    print 'target_pose.pose.position:\n',target_pose.pose.position

    # 欧拉角换算为四元数
    point = tf.transformations.quaternion_from_euler(rpy[0],rpy[1],rpy[2])
    print 'point:',point

    # 赋值扭转偏差到目标位姿
    target_pose.pose.orientation.x = point[0]
    target_pose.pose.orientation.y = point[1]
    target_pose.pose.orientation.z = point[2]
    target_pose.pose.orientation.w = point[3]

    # 设置机械臂终端运动的目标位姿
    arm.set_start_state_to_current_state()
    arm.set_pose_target(target_pose, 'tool_0')

    # 规划运动路径
    traj = arm.plan()

    # 按照规划的运动路径控制机械臂运动
    arm.execute(traj)
    
    # 打印位姿
    print target_pose.pose.position
    print target_pose.pose.orientation
    arm.go()

    # rospy.sleep(2)


# 关闭并退出
moveit_commander.roscpp_shutdown()
moveit_commander.os._exit(0)
