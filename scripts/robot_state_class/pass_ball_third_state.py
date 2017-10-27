#!/usr/bin/env python
# -*- coding: UTF-8 -*-


import rospy
import smach
import math
import smach_ros
from robot_move_pkg import turn_an_angular
from robot_move_pkg import linear_move
from robot_shovel_srv import control_srv
from robot_find_pkg import find_basketball
from robot_find_pkg import find_ball
from robot_move_pkg import move_in_robot
from robot_state_pkg import get_robot_position

#传球
class Shoot(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['successed', 'failed'])
        self.cmd_shoot = control_srv.shootControlSrv()
        rospy.loginfo('the shoot is initial ok!')

    def execute(self, ud):
        rospy.loginfo("Start Shoot!!!!!")
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
        if self.preempt_requested():
            self.service_preempt()
            return 'failed'
        #将铲子降下来
        self.cmd_shovel.control_shovel(control_type = 4)
        return 'successed'


#移动到三分线附近
class Move_To_Three_Point_Line(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['successed','failed'])
        self.cmd_position = get_robot_position.robot_position_state()
        self.move_cmd = linear_move.linear_move()
        self.turn_cmd = turn_an_angular.turn_an_angular()
        rospy.loginfo("the Move_To_Three_Point_Line is initial OK!")

    def execute(self, ud):
        rospy.loginfo("Start Move_To_Three_Point_Line!!!!")
        if self.preempt_requested():
            self.service_preempt()
            return 'failed'
        current_y = self.cmd_position.get_robot_current_y()
        self.move_cmd.move_to(x = 0, y = 2.5, max_velocity = 1)
        current_x = self.cmd_position.get_robot_current_x()
        self.move_cmd.move_to(x = 4.5,y = 0.0 ,max_velocity = 1)
        return 'successed'

#第一次找球，从图像那里得到球的距离和角度信息
class Search_Ball_Shun(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['successed', 'failed'])
        self.move_cmd = move_in_robot.linear_move()
        self.turn_cmd = turn_an_angular.turn_an_angular()
        self.cmd_shovel = control_srv.shovelControlSrv()
        self.find_ball = find_basketball.find_basketball()
        self.cmd_position = get_robot_position.robot_position_state()

    def execute(self, ud):
        rospy.logwarn('Start search the ball!!')
        if self.preempt_requested():
            self.service_preempt()
            return 'failed'
        (ball_dis,ball_theta) = self.find_ball.findball_shun()
        self.move_cmd.move_to(math.tan(ball_theta)*ball_dis+0.03,0,0.5)
        (ball_dis,ball_theta) = self.find_ball.findball_shun()
        if abs(ball_dis)>1.3:
            #先移动到球前面0.8m处然后准备再次检测球的位置并且放下铲子
            self.move_cmd.move_to(0,ball_dis-0.9,0.5)
            (ball_dis,ball_theta) = self.find_ball.findball_shun()
            self.move_cmd.move_to(0,math.tan(ball_theta)*ball_dis+0.02,0.5)
            self.cmd_shovel.control_shovel(control_type = 4)
            self.move_cmd.move_to(0,ball_dis-0.22,0.5)
        else:
            self.cmd_shovel.control_shovel(control_type = 4)
            self.move_cmd.move_to(0,ball_dis-0.22,0.5)
        self.cmd_shovel.control_shovel(control_type = 3)
        return 'successed'

#第一次找球，从图像那里得到球的距离和角度信息
class Search_Ball_Ni(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['successed', 'failed'])
        self.move_cmd = move_in_robot.linear_move()
        self.turn_cmd = turn_an_angular.turn_an_angular()
        self.cmd_shovel = control_srv.shovelControlSrv()
        #self.find_ball = find_ball.find_ball()
        self.find_ball = find_basketball.find_basketball()
        self.cmd_position = get_robot_position.robot_position_state()

    def execute(self, ud):
        rospy.logwarn('Start search the ball!!')
        if self.preempt_requested():
            self.service_preempt()
            return 'failed'
        (ball_dis,ball_theta) = self.find_ball.findball_ni()
        self.move_cmd.move_to(math.tan(ball_theta)*ball_dis+0.03,0,0.5)
        (ball_dis,ball_theta) = self.find_ball.findball_ni()
        if abs(ball_dis)>1.3:
            #先移动到球前面0.8m处然后准备再次检测球的位置并且放下铲子
            self.move_cmd.move_to(0,ball_dis-0.9,0.5)
            (ball_dis,ball_theta) = self.find_ball.findball_ni()
            self.move_cmd.move_to(math.tan(ball_theta)*ball_dis+0.02,0,0.5)
            self.cmd_shovel.control_shovel(control_type = 4)
            self.move_cmd.move_to(0,ball_dis-0.22,0.5)
        else:
            self.cmd_shovel.control_shovel(control_type = 4)
            self.move_cmd.move_to(0,ball_dis-0.22,0.5)
        self.cmd_shovel.control_shovel(control_type = 3)
        return 'successed'


#进行第一次姿态调整
class Shoot_Adjust1(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['successed','failed'])
        self.cmd_shovel = control_srv.shovelControlSrv()
        self.turn_cmd = turn_an_angular.turn_an_angular()
        self.move_cmd = linear_move.linear_move()
        self.cmd_position = get_robot_position.robot_position_state()

    def execute(self,ud):
        rospy.logwarn('We are ready to adjust to shoot first time!!!')
        if self.preempt_requested():
            self.service_preempt()
            return 'failed'
        current_y = self.cmd_position.get_robot_current_y()
        self.move_cmd.move_to(x=0, y=1.2-current_y,max_velocity = 1)
        current_x = self.cmd_position.get_robot_current_x()
        self.move_cmd.move_to(x=6.5-current_x, y=0,max_velocity = 1)
        current_w = self.cmd_position.get_robot_current_w()
        self.turn_cmd.turn_to(-current_w+math.pi/30)
        self.cmd_shovel.control_shovel(control_type = 4)
        return 'successed'


#第一个球传球结束后移动到中心置球区域
class Find_Another_Ball(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['successed','failed'])
        self.turn_cmd = turn_an_angular.turn_an_angular()
        self.move_cmd = linear_move.linear_move()
        self.cmd_position = get_robot_position.robot_position_state()

    def execute(self, ud):
        if self.preempt_requested():
            self.service_preempt()
            return 'failed'
        #移动到目标位置同时准备下一回合的找球
        #移动到底脚置球球附近准备进行找球
        current_y = self.cmd_position.get_robot_current_y()
        self.move_cmd.move_to(x=0.0,y=1.0-current_y,max_velocity=1)
        current_x = self.cmd_position.get_robot_current_x()     
        self.move_cmd.move_to(x=8.9-current_x, y=0, max_velocity=1)
        current_w = self.cmd_position.get_robot_current_w()
        self.turn_cmd.turn_to(-current_w+math.pi/2.1)
        return 'successed'


#进行射击的第二次姿态调整
class Shoot_Adjust2(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['successed','failed'])
        self.turn_cmd = turn_an_angular.turn_an_angular()
        self.move_cmd = linear_move.linear_move()
        self.cmd_shovel = control_srv.shovelControlSrv()
        self.cmd_position = get_robot_position.robot_position_state()

    def execute(self,ud):
        rospy.logwarn('We are ready to adjust to shoot first time!!!')
        if self.preempt_requested():
            self.service_preempt()
            return 'failed'
        #移动到传球目标位置
        current_y = self.cmd_position.get_robot_current_y()
        self.move_cmd.move_to(x=0, y=1.0-current_y,max_velocity = 1)
        current_x = self.cmd_position.get_robot_current_x()
        self.move_cmd.move_to(x=6.5-current_x, y=0,max_velocity = 1)
        current_w = self.cmd_position.get_robot_current_w()
        self.turn_cmd.turn_to(-current_w+math.pi/25)
        self.cmd_shovel.control_shovel(control_type = 4)
        rospy.sleep(0.2)
        return 'successed'
 

#回家
class Return(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['successed','failed'])
        self.move_cmd = linear_move.linear_move()
        self.cmd_position = get_robot_position.robot_position_state()
        rospy.loginfo('We are ready to go home!!!')

    def execute(self,ud):
        if self.preempt_requested():
            self.service_preempt()
            return 'failed'    
        #比赛的时候弃用
        current_y = self.cmd_position.get_robot_current_y()
        self.move_cmd.move_to(x=0.0, y=2.55-current_y,max_velocity = 1)
        current_x = self.cmd_position.get_robot_current_x()
        self.move_cmd.move_to(x = -current_x, y = 0,max_velocity = 1)
        current_y = self.cmd_position.get_robot_current_y()
        self.move_cmd.move_to(x = 0, y = -current_y-0.1 ,max_velocity = 1)
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
