#!/usr/bin/env python
# coding=utf-8

import numpy as np
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from math import copysign
from apriltag_ros.msg import AprilTagDetectionArray
from dynamic_reconfigure.server import Server
from apriltag_detection.cfg import arPIDConfig

class ArFollower():
	def __init__(self):
		rospy.init_node("ar_follower")
		rate = rospy.Rate(1)

		global PID_param

		self.targetDist_y= rospy.get_param('~targetDist_y') # 获取中距值
		self.targetDist_x= rospy.get_param('~targetDist_x')# 将中距值赋值为动态调参后的值
		self.targetDist_z= rospy.get_param('~targetDist_z')
		self.max_speed_y = rospy.get_param("~max_speed_y")
		self.max_speed_x = rospy.get_param("~max_speed_x")
		self.max_speed_z = rospy.get_param("~max_speed_z")

		PID_param = rospy.get_param('~PID_controller') # 获取PID值 [y轴,x轴,z轴]

		#rqt_srv
		self.srv = Server(arPIDConfig,self.config_callback)
		
		#订阅AR标签位姿信息，发布速度话题
		self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=5)
		self.ar_pose_sub = rospy.Subscriber("/tag_detections", AprilTagDetectionArray, self.set_cmd_vel)

		#先设定初始化时未检测到AR标签
		self.target_visible = False
		self.move_cmd = Twist()

		#未检测到进程退出时持续发布速度信息
		while not rospy.is_shutdown():
			self.cmd_vel_pub.publish(self.move_cmd)
			rate.sleep()

	def config_callback(self, config, level):
		self.targetDist_y = config.targetDist_y # 将中距值赋值为动态调参后的值
		self.targetDist_x = config.targetDist_x
		self.targetDist_z = config.targetDist_z
		PID_param['P'] = [config.P_y,config.P_x,config.P_z]
		PID_param['I'] = [config.I_y,config.P_x,config.I_z]
		PID_param['D'] = [config.D_y,config.P_x,config.D_z]
		# 创建simplePID对象				[y轴，x轴，z轴姿态]		P 				I 				D
		self.PID_controller = simplePID([self.targetDist_y,self.targetDist_x,self.targetDist_z], PID_param['P'], PID_param['I'], PID_param['D']) # 实例化simplePID以实现PID控制更新
		return config

	def set_cmd_vel(self, msg):
		try:
			marker = msg.detections[0] #设定以检测到的第一个AR标签为目标
			if not self.target_visible:
				rospy.loginfo("the robot is Tracking Target!")
			self.target_visible = True #检测到AR标签后将检测标志位置为ture
		except:
			if self.target_visible:
				rospy.loginfo("Robot Lost Target!")
			self.target_visible = False
			self.move_cmd.linear.x = 0
			self.move_cmd.linear.y = 0
			return

		offset = 0 #小车中心与摄像头检测到的AR标签中心的偏差0.06,可以根据相机在小车的位置进行调整
		#apriltag横向是X,纵向距离是z
		offset_y = marker.pose.pose.pose.position.x + offset #AR标签位置信息y方向（已校正）
		offset_x = marker.pose.pose.pose.position.z #AR标签位置信息x方向
		orientation = marker.pose.pose.pose.orientation.z #AR标签姿态信息方向
		#当AR标签和小车的距离与设定距离存在偏差时
		rospy.loginfo("targetDist_y %f", self.targetDist_y)
		rospy.loginfo("targetDist_x %f", self.targetDist_x)
		rospy.loginfo("targetDist_z %f", self.targetDist_z)
		rospy.loginfo("PID_param['P'] %f", PID_param['P'][0])
		rospy.loginfo("PID_param['P'] %f", PID_param['P'][1])
		rospy.loginfo("PID_param['P'] %f", PID_param['P'][2])
		rospy.loginfo("PID_param['I'] %f", PID_param['I'][0])
		rospy.loginfo("PID_param['I'] %f", PID_param['I'][1])
		rospy.loginfo("PID_param['I'] %f", PID_param['I'][2])

		[speed_y, speed_x,speed_z] = self.PID_controller.update([offset_y, offset_x, orientation])
		#限速
		speed_y = np.clip(-speed_y, -self.max_speed_y, self.max_speed_y)
		speed_x  = np.clip(-speed_x, -self.max_speed_x, self.max_speed_x)
		speed_z  = np.clip(-speed_z, -self.max_speed_z, self.max_speed_z)

		self.move_cmd.linear.y=speed_y
		self.move_cmd.linear.y=speed_x
		self.move_cmd.angular.z=speed_z

		
class simplePID:
	def __init__(self, target, P, I, D):
		# check if parameter shapes are compatabile. 
		# 检查参数形状是否兼容
		if(not(np.size(P)==np.size(I)==np.size(D)) or ((np.size(target)<=2))):
			raise TypeError('input parameters shape is not compatable')
		rospy.loginfo('PID initialised with P:{}, I:{}, D:{}'.format(P,I,D))

		self.Kp		=np.array(P)
		self.Ki		=np.array(I)
		self.Kd		=np.array(D)
		self.setPoint   =np.array(target)
		
		self.last_error=0
		self.integrator = 0
		self.integrator_max = float('inf')
		self.timeOfLastCall = None 

	def update(self, current_value):
		return [2,2,1]
	
if __name__ == '__main__':
	try:
		ArFollower()
		rospy.spin()
	except rospy.ROSInterruptException:
		print('exception')

