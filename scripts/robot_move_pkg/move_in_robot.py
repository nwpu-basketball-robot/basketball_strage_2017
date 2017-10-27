#!/usr/bin/env python
#-*- coding: UTF-8 -*-
#基于机器人坐标系下的机器人移动策略，只针对机器人坐标系下的x，y方向的移动，不做斜线运动

import rospy
import geometry_msgs.msg as g_msgs
import  math
import config
import  sys
sys.path.append(config.robot_state_pkg_path)
import robot_state_pkg.get_robot_position as robot_state_pkg_path
import turn_an_angular
class growth_curve(object):
    def __init__(self):
        # 插值起始速度等于 gamma/(1+beta）
        self.start_beta  = 22
        # alpha 影响达到最大速度的自变量的值 alpha越大,越快达到最大速度
        self.start_alpha = 27
        self.end_beta = 15
        self.end_alpha = 10
    
    def set_goal(self,goal_distance,max_velocity):
        #设置总的移动距离
        self.goal = goal_distance
        #设置速度最大值
        self.velocity = max_velocity
    
    def cal(self,dis):
        f1 = lambda x: self.velocity / (1.0 + self.start_beta*math.exp(-self.start_alpha * x))
        f2 = lambda x: self.velocity / (1.0 + self.end_beta*math.exp(-self.end_alpha *(self.goal - x)))
        if dis <= self.goal/2.5 :
            return f1(dis)
        else :
            return f2(dis)

class linear_move(object):
    def __init__(self):
        #通过这个模块获取机器人当前姿态
        self.robot_state = robot_state_pkg_path.robot_position_state()
        self.cmd_move_pub = rospy.Publisher('/cmd_move_robot', g_msgs.Twist, queue_size = 100)
        self.rate = rospy.Rate(10)
        #设置机器人直线移动阈值
        self.stop_tolerance = 0.015
        #通过这个模块修正最终姿态角
        self.accurate_turn_an_angular = turn_an_angular.turn_an_angular()
        self.x_speed = 0.0
        self.y_speed = 0.0
        self.w_speed = 0.0
        #进行线速度插值
        self.linear_sp = growth_curve()
        #amend_speed是单一方向运动时矫正速度
        self.amend_speed = 0.06
        self.x_y_amend_speed = 0.05

    def brake(self):#停止时的回调函数
        move_velocity = g_msgs.Twist()
        move_velocity.linear.x = 0
        move_velocity.linear.y = 0
        move_velocity.angular.z = 0
        self.cmd_move_pub.publish(move_velocity)


    def move_to(self,x=0.0,y=0.0,max_velocity=0.0):
        rospy.logerr('Robot Move to x=%s y=%s'%(x,y))
        #设置停止回调函数
        rospy.on_shutdown(self.brake)
        move_velocity = g_msgs.Twist()
        # 计算目标移动欧拉距离
        goal_distance = math.sqrt(math.pow(x, 2)+math.pow(y, 2))
        # 算出方向角,便于接下来的分解
        direction_angle = math.atan2(abs(y) , abs(x))
        # 获取启动前的x，y，yaw
        start_x, start_y, start_w = self.robot_state.get_robot_current_x_y_w()
        rospy.logerr('start_x=%s start_y=%s'%(start_x,start_y))
        # 设置目标插值距离，确定最终插值曲线
        self.linear_sp.set_goal(abs(goal_distance),max_velocity)
        # 确定目标位置
        angular_has_moved = 0.0
        is_in_x = False
        is_in_y = False
        #用于判断是否为单一方向上的运动(针对机器人坐标系)
        if x == 0 :
            is_in_y = True
        if y == 0:
            is_in_x = True

        while not rospy.is_shutdown() and goal_distance != 0:
            current_x, current_y, current_w = self.robot_state.get_robot_current_x_y_w()
            #x 方向已经移动的距离
            x_has_moved = current_x - start_x
            #y 方向已经移动的距离
            y_has_moved = current_y - start_y
            #已经走过的距离
            dis_has_moved = math.sqrt(math.pow(x_has_moved, 2)+math.pow(y_has_moved, 2))
            #角度的偏移量
            angular_has_moved = abs(current_w - start_w)
            #计算速度
            #如果只走x
            if is_in_x == True:
                #当x与y方向都到达目标以后退出循环,机器人的速度赋0
                #防止机器人到达目标点后无法停止
                if abs(dis_has_moved) >= abs(x):
                    self.brake()
                    rospy.logerr('We have arrived the goal!!!!')
                    break
                else:    
                    #进行速度赋值
                    self.x_speed = math.copysign(self.linear_sp.cal(abs(dis_has_moved)),x)
                    self.y_speed = 0.0
            #只走y方向
            elif is_in_y == True:
                #当x与y方向都到达目标以后退出循环,机器人的速度赋0
                #防止机器人到达目标点后无法停止
                if abs(dis_has_moved) >= abs(y):
                    self.brake()
                    rospy.logerr('We have arrived the goal!!!!')
                    break
                else: 
                    #进行速度赋值   
                    self.x_speed = 0.0
                    self.y_speed = math.copysign(self.linear_sp.cal(abs(dis_has_moved)),y)
            #机器人角速度为0
            self.w_speed = 0.0     
            move_velocity.linear.x = self.x_speed
            move_velocity.linear.y = self.y_speed
            move_velocity.angular.z = self.w_speed
            self.cmd_move_pub.publish(move_velocity)
            self.rate.sleep()
        self.brake()
    
            
if __name__ == '__main__'   :
    rospy.init_node('linear_move')
    move_cmd = linear_move()
    move_cmd.move_to(0,3,1)

sys.path.remove(config.robot_state_pkg_path)
