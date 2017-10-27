#!/usr/bin/env python
# -*- coding: UTF-8 -*-
#流程：机器前往传球区并放下铲子 -> 传球 -> 前进到找球的位置 -> 开始检测球（以原地自转的方式） -> 检测到球后接近球到球前方0.8米处
#      -> 再次检测球并调整铲子方向后前进  -> 铲球 -> 调整机器角度朝向传球区 -> 投球 -> 升起铲子 -> 回家

import rospy
import smach
import math
import smach_ros
from robot_move_pkg import turn_an_angular
from robot_move_pkg import linear_move
from robot_move_pkg import move_in_robot
from robot_shovel_srv import control_srv
from robot_find_pkg import find_basketball
from robot_find_pkg import find_volleyball
from robot_find_pkg import find_ball
from robot_state_pkg import get_robot_position

#传球
class Shoot(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['successed', 'failed'])
        self.cmd_shoot = control_srv.shootControlSrv()
        rospy.loginfo('The shoot is initial ok!')

    def execute(self, ud):
        rospy.logwarn("Start Shoot!!!!!")
        if self.preempt_requested():
            self.service_preempt()
            return 'failed'
        self.cmd_shoot.shoot_ball()
        return 'successed'

#抬起铲子
class Shovel_UP(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['successed','failed'])
        self.cmd_shovel = control_srv.shovelControlSrv()

    def execute(self, ud):
        rospy.logwarn('The shovel is up!!!!')
        if self.preempt_requested():
            self.service_preempt()
            return 'failed'
        #将铲子抬起来
        self.cmd_shovel.control_shovel(control_type = 3)
        return 'successed'

#把铲子降下来
class Shovel_Down(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['successed','failed'])
        self.cmd_shovel = control_srv.shovelControlSrv()

    def execute(self,ud):
        rospy.logwarn('The shovel down')
        if self.preempt_requested():
            self.service_preempt()
            return 'failed'
        #将铲子降下来
        self.cmd_shovel.control_shovel(control_type = 4)
        return 'successed'

#传球第一回合开始时机器人移动到指定的位置准备进行传球,同时调整机器人的姿态
class Move_Point_To_Shoot(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['successed', 'failed'])
        self.move_cmd = linear_move.linear_move()
        self.turn_cmd = turn_an_angular.turn_an_angular()

    def execute(self, ud):
        rospy.logwarn('Move Point To Shoot')
        if self.preempt_requested():
            self.service_preempt()
            return 'failed'
        self.move_cmd.move_to(x = 1.1,y = 7.1 ,max_velocity= 1.0)
        rospy.sleep(0.5)
        self.turn_cmd.turn_to(math.pi/2.5)
        return 'successed'

#投球调整
class Move_To_Find_Ball(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['successed', 'failed'])
        self.turn_cmd = turn_an_angular.turn_an_angular()
        self.move_cmd = linear_move.linear_move()
        self.cmd_position = get_robot_position.robot_position_state()

    def execute(self,ud):
        rospy.logwarn('Move_To_Find_Ball')
        if self.preempt_requested():
            self.service_preempt()
            return 'failed'
        current_y = self.cmd_position.get_robot_current_y()
        self.move_cmd.move_to(x = 0,y = 5-current_y,max_velocity= 1.0)
        current_x = self.cmd_position.get_robot_current_x()
        self.move_cmd.move_to(x = 1.1-current_x,y = 0,max_velocity= 1.0)
        current_w = self.cmd_position.get_robot_current_w()
        self.turn_cmd.turn_to(-current_w+math.pi/12)
        return 'successed'

#进行找球，从图像那里得到球的距离和角度信息
class Search_Ball(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['successed', 'failed'])
        self.move_cmd = move_in_robot.linear_move()
        self.turn_cmd = turn_an_angular.turn_an_angular()
        self.cmd_shovel = control_srv.shovelControlSrv()
        self.find_ball = find_basketball.find_basketball()

    def execute(self, ud):
        rospy.logwarn('Start search the ball!!')
        if self.preempt_requested():
            self.service_preempt()
            return 'failed'
        (ball_dis,ball_theta) = self.find_ball.findball_ni()
        self.move_cmd.move_to(math.tan(ball_theta)*ball_dis+0.03,0,0.5)
        (ball_dis,ball_theta) = self.find_ball.findball_shun()
        if abs(ball_dis)>1.3:
            #先移动到球前面1m处然后准备再次检测球的位置并且放下铲子
            self.move_cmd.move_to(0,ball_dis-0.9,0.5)
            (ball_dis,ball_theta) = self.find_ball.findball_shun()
            self.move_cmd.move_to(math.tan(ball_theta)*ball_dis+0.02,0,0.5)
            self.cmd_shovel.control_shovel(control_type = 4)
            self.move_cmd.move_to(0,ball_dis-0.22,0.5)
        else:
            self.cmd_shovel.control_shovel(control_type = 4)
            self.move_cmd.move_to(0,ball_dis-0.22,0.5)
        self.cmd_shovel.control_shovel(control_type = 3)
        rospy.sleep(0.5)
        self.cmd_shovel.control_shovel(control_type = 4)
        return 'successed'

#铲球后调整机器姿态，使其正对传球目标区域中心
class Shoot_Adjust(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['successed','failed'])
        self.move_cmd = linear_move.linear_move()
        self.turn_cmd = turn_an_angular.turn_an_angular()
        self.cmd_position = get_robot_position.robot_position_state()
        rospy.loginfo('The Shoot_Adjust is initial ok!!!!')

    def execute(self, ud):
        rospy.logwarn('Start Shoot Adjust')
        if self.preempt_requested():
            self.service_preempt()
            return 'failed'
        current_x = self.cmd_position.get_robot_current_x()
        self.move_cmd.move_to(x = 1.1-current_x,y = 0,max_velocity= 1.0)
        current_y = self.cmd_position.get_robot_current_y()
        self.move_cmd.move_to(x = 0,y = 7.1-current_y,max_velocity= 1.0)
        current_w = self.cmd_position.get_robot_current_w()
        self.turn_cmd.turn_to(math.pi/2.5-current_w)
        rospy.sleep(0.1)
        return 'successed'

#回家
class Return(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['successed','failed'])
        self.move_cmd = linear_move.linear_move()
        self.cmd_position = get_robot_position.robot_position_state()
        rospy.loginfo('We are ready to go home!!!')

    def execute(self,ud):
        rospy.logwarn('Return!!!!')
        if self.preempt_requested():
            self.service_preempt()
            return 'failed'
        (current_x,current_y) = self.cmd_position.get_robot_current_x_y()
        print current_x,current_y
        self.move_cmd.move_to(x = -current_x, y = 0, max_velocity = 1.0)
        self.move_cmd.move_to(x = 0, y = -current_y-0.05, max_velocity = 1.0)
        return 'successed'
#回家调整
class Return_Adjust(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['successed'])
        self.turn_cmd = turn_an_angular.turn_an_angular()
        self.cmd_position = get_robot_position.robot_position_state() 

    def execute(self,ud):
        current_w = self.cmd_position.get_robot_current_w()
        self.turn_cmd.turn_to(-current_w)
        return 'successed'
