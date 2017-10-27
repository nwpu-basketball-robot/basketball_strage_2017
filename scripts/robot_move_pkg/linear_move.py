#!/usr/bin/env python
#-*- coding: UTF-8 -*-
#采用慢速启动，慢速停止的策略，由于机器人底盘在启动时只能同时启动三个轮子，所以暂时采用这种方法

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
        self.cmd_move_pub = rospy.Publisher('/cmd_move', g_msgs.Twist, queue_size = 100)
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
        #矫正速度，参数可调
        self.amend_speed = 0.1
        self.x_y_amend_speed = 0.1

    def brake(self):#停止时的回调函数
        move_velocity = g_msgs.Twist()
        move_velocity.linear.x = 0
        move_velocity.linear.y = 0
        move_velocity.angular.z = 0
        self.cmd_move_pub.publish(move_velocity)


    def move_to(self,x=0.0,y=0.0,max_velocity=0.0):
        rospy.logerr('Move to x=%s y=%s'%(x,y))
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
        x_goal = start_x+x
        y_goal = start_y+y
        
        angular_has_moved = 0.0
        y_has_moved = 0.0
        x_has_moved =0.0
        x_amend_speed = 0.0
        y_amend_speed = 0.0
        x_arrive_goal = False
        y_arrive_goal = False
        move_in_x = False
        move_in_y = False
        #用于判断是否为单一方向上的运动
        if x == 0 :
            move_in_y = True
            x_arrive_goal = True
        if y == 0:
            move_in_x = True
            y_arrive_goal = True

        while not rospy.is_shutdown() and goal_distance != 0:
            current_x, current_y, current_w = self.robot_state.get_robot_current_x_y_w()
            #x 方向已经移动的距离
            x_has_moved = current_x - start_x
            #y 方向已经移动的距离
            y_has_moved = current_y - start_y
            #角度的偏移量
            angular_has_moved = abs(current_w - start_w)
            #计算速度
            #如果只走x
            if move_in_x == True:
                #当x与y方向都到达目标以后退出循环,机器人的速度赋0
                if x_arrive_goal == True and y_arrive_goal == True:
                    self.brake()
                    break
                #y方向的速度矫正   
                if abs(y_has_moved) < self.stop_tolerance :
                    self.y_speed = 0.0
                else: 
                    self.y_speed = math.copysign(self.x_y_amend_speed, -y_has_moved)
                #防止机器人到达目标点后无法停止
                if abs(x_has_moved) >= abs(x):
                    self.brake()
                    rospy.logerr('We have arrived the goal!!!!')
                    break
                else:    
                    #x方向的速度进行赋值
                    self.x_speed = math.copysign(self.linear_sp.cal(abs(x_has_moved)),x)
                #角度矫正   
                if angular_has_moved < math.pi/180:
                    self.w_speed = 0.0
                else:
                    self.w_speed = math.copysign(1,current_w)
                    
            #只走y方向
            elif move_in_y == True:
                #当x与y方向都到达目标以后退出循环,机器人的速度赋0
                if x_arrive_goal == True and y_arrive_goal == True:
                    self.brake()
                    break
                #y方向的角度矫正
                if abs(x_has_moved) >= self.stop_tolerance:
                    self.x_speed = math.copysign(self.x_y_amend_speed , -x_has_moved )
                else :
                    self.x_speed = 0.0
                #防止机器人到达目标点后无法停止
                if abs(y_has_moved) >= abs(y):
                    self.brake()
                    break
                else:    
                    #y方向的速度进行赋值
                    self.y_speed = math.copysign(self.linear_sp.cal(abs(y_has_moved)),y)
                #角度矫正
                if angular_has_moved < math.pi/180:
                    self.w_speed = 0.0
                else:
                    self.w_speed = math.copysign(1,current_w)    
            else:
                #同时向x与y方向进行运动
                #如果x与y方向均到达目标则机器人停止运动
                #设置末端矫正
                if x_arrive_goal == True and y_arrive_goal == True:
                    if abs(current_x-x_goal)>0.02:
                        linear_move().move_to(x_goal-current_x,0,1)
                    if abs(y-y_has_moved)>0.02:
                        linear_move().move_to(0,y_goal-current_y,1)
                    break                         
                else:
                    #设置基于目标角度的误差矫正，需要配合末端矫正才可以使机器人到达准确的位置
                    current_direction_angle = math.atan2(abs(current_y) , abs(current_x))
                    dis_has_moved = math.sqrt(x_has_moved**2 + y_has_moved**2)
                    linear_speed = self.linear_sp.cal(dis_has_moved)
                    bisu = direction_angle - current_direction_angle
                    if abs(bisu) > 0.05:
                        # Y 方向跑的慢
                        if bisu > 0:
                            x_amend_speed = 0.0
                            y_amend_speed = self.amend_speed
                        # X 方向跑的慢
                        else:
                            x_amend_speed = self.amend_speed
                            y_amend_speed = 0.0
                    else:
                        x_amend_speed = 0.0
                        y_amend_speed = 0.0
                    #判断x方向是否到目标距离
                    if abs(x_has_moved) >= abs(x):
                        self.x_speed = 0.0
                        x_arrive_goal = True
                    else:
                        self.x_speed = math.copysign((linear_speed+x_amend_speed) * math.cos(direction_angle),x)
                    #判断y方向是否到目标距离
                    if abs(y_has_moved) >= abs(y):
                        self.y_speed = 0.0
                        y_arrive_goal = True
                    else:
                        self.y_speed = math.copysign((linear_speed+y_amend_speed) * math.sin(direction_angle),y)
                    #角度矫正   
                    if angular_has_moved < math.pi/180:
                        self.w_speed = 0.0
                    else:
                        self.w_speed = math.copysign(1,current_w)
            #将计算好的速度赋值并发下去
            move_velocity.linear.x = self.x_speed
            move_velocity.linear.y = self.y_speed
            move_velocity.angular.z = self.w_speed
            self.cmd_move_pub.publish(move_velocity)
            self.rate.sleep()
        self.brake()
    
            
if __name__ == '__main__'   :
    rospy.init_node('linear_move')
    move_cmd = linear_move()
    move_cmd.move_to(-0.9,-6,1)

sys.path.remove(config.robot_state_pkg_path)
