#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import rospy
import smach
import math
import smach_ros
from robot_move_pkg import turn_an_angular
from robot_move_pkg import linear_move
from robot_move_pkg import move_in_robot
from robot_shovel_srv import control_srv
from robot_find_pkg import find_cylinder
from robot_find_pkg import find_volleyball
from robot_find_pkg import find_ball
from robot_state_pkg import get_robot_position

#投球
class Shoot(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['successed', 'failed'])
        self.move_cmd = move_in_robot.linear_move()
        self.cmd_shoot = control_srv.shootControlSrv()
        self.cmd_shovel = control_srv.shovelControlSrv()
        self.find_cylinder = find_cylinder.find_cylinder()
        self.cmd_position = get_robot_position.robot_position_state()
        rospy.loginfo('The shoot is initial ok!')

    def execute(self, ud):
        rospy.logwarn("Start Shoot!!!!!")
        if self.preempt_requested():
            self.service_preempt()
            return 'failed'
        (cylinder_dis,cylinder_theta) = self.find_cylinder.findcylinder_ni() 
        self.move_cmd.move_to(math.tan(cylinder_theta)*cylinder_dis +0.17,0,0.5)
        (cylinder_dis,cylinder_theta) = self.find_cylinder.findcylinder_ni() 
        self.move_cmd.move_to(0,-(2.5-cylinder_dis),0.5)
        self.cmd_shovel.control_shovel(4)
        (current_x,current_y) = self.cmd_position.get_robot_current_x_y()
        rospy.logerr('current_x=%s current_y=%s'%(current_x,current_y))
        rospy.sleep(0.3)
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
        self.cmd_position = get_robot_position.robot_position_state()
        self.turn_cmd = turn_an_angular.turn_an_angular()

    def execute(self, ud):
        rospy.logwarn('Move Point To Shoot')
        if self.preempt_requested():
            self.service_preempt()
            return 'failed'
        self.move_cmd.move_to(x = 0,y = 2, max_velocity = 1.2)
        (current_x,current_y) = self.cmd_position.get_robot_current_x_y()
        self.move_cmd.move_to(x = 7.8-current_x,y = 7.25-current_y ,max_velocity = 1.2)
        current_w = self.cmd_position.get_robot_current_w()
        self.turn_cmd.turn_to(math.pi/2.2-current_w)
        rospy.sleep(0.5)
        return 'successed'


#找球调整
class Shoot_Adjust1(smach.State):
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
        (current_x,current_y) = self.cmd_position.get_robot_current_x_y()
        self.move_cmd.move_to(x=1.4-current_x, y=4.6-current_y,max_velocity = 1.2)
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
        self.find_ball = find_ball.find_ball()
        self.find_volleyball = find_volleyball.find_volleyball()

    def execute(self, ud):
        rospy.logwarn('Start search the ball!!')
        if self.preempt_requested():
            self.service_preempt()
            return 'failed'
        rospy.sleep(0.2)
        (ball_dis,ball_theta) = self.find_volleyball.findball_ni()
        self.move_cmd.move_to(math.tan(ball_theta)*ball_dis+0.02,0,0.5)
        (ball_dis,ball_theta) = self.find_volleyball.findball_ni()
        if abs(ball_dis)>1.3:
            #先移动到球前面0.8m处然后准备再次检测球的位置并且放下铲子
            self.move_cmd.move_to(0,ball_dis-0.8,0.5)
            (ball_dis,ball_theta) = self.find_volleyball.findball_ni()
            self.move_cmd.move_to(math.tan(ball_theta)*ball_dis+0.01,0,0.5)
            self.cmd_shovel.control_shovel(control_type = 4)
            self.move_cmd.move_to(0,ball_dis-0.22,0.5)
        else:
            self.cmd_shovel.control_shovel(control_type = 4)
            self.move_cmd.move_to(0,ball_dis-0.22,0.5)
        self.cmd_shovel.control_shovel(control_type = 3)
        return 'successed'

#铲球后调整机器姿态，使其正对传球目标区域中心
class Shoot_Adjust2(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['successed','failed'])
        self.move_cmd = linear_move.linear_move()
        self.turn_cmd = turn_an_angular.turn_an_angular()
        self.cmd_position = get_robot_position.robot_position_state()
        self.cmd_shovel = control_srv.shovelControlSrv()
        rospy.loginfo('The Shoot_Adjust is initial ok!!!!')

    def execute(self, ud):
        rospy.logwarn('Start Shoot Adjust')
        if self.preempt_requested():
            self.service_preempt()
            return 'failed'
        current_y = self.cmd_position.get_robot_current_y()     
        self.move_cmd.move_to(x=0, y=4.6-current_y, max_velocity=1)
        (current_x,current_y) = self.cmd_position.get_robot_current_x_y()
        self.move_cmd.move_to(x = 7.8-current_x,y = 7.25-current_y,max_velocity= 1.2)
        current_w = self.cmd_position.get_robot_current_w()
        self.turn_cmd.turn_to(math.pi/2.2-current_w)
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
        self.move_cmd.move_to(x = -current_x,y = -current_y ,max_velocity = 1.2)
        return 'successed'
        
#回家调整
class Return_Adjust(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['successed'])
        self.turn_cmd = turn_an_angular.turn_an_angular()
        self.cmd_position = get_robot_position.robot_position_state() 

    def execute(self,ud):
        rospy.sleep(0.5)
        current_w = self.cmd_position.get_robot_current_w()
        self.turn_cmd.turn_to(-current_w)
        return 'successed'


