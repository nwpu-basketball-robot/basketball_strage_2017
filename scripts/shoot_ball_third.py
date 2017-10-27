#!/usr/bin/env python
# -*- coding: UTF-8 -*-

#投篮项目3的状态机
#流程： 前进到三分线附近 -> 自转并检测篮球 -> 检测到球后接近球 -> 二次检测球并调整角度（记录球所在位置，世界坐标系）-> 铲球 -> 前进到定位柱附近 -> 检测定位柱 -> 调整角度对准定位柱  -> 投篮
#      前进到三分线捡球处（防止撞到排球）-> 移动到底脚置球区附近 ->在自转并检测球 -> 检测到球后接近球 -> 二次检测并调整 -> 铲球 -> 前进到三分线捡球处（防止撞到排球） -> 前进到定位柱附近
#      -> 检测定位柱 -> 调整角度对准定位柱  -> 投篮

import rospy
import smach
import math
import smach_ros
from robot_state_class.shoot_ball_third_state import *


def shoot_third():
    shoot_ball_third = smach.StateMachine(outcomes=['successed', 'failed'])
    rospy.logwarn("Shoot_ball_third STARTING!!!")
    with shoot_ball_third:

        smach.StateMachine.add('MOVE_TO_THREE_POINT_LINE',Move_To_Three_Point_Line(),
                                transitions={'successed':'FindBall1',
                                             'failed':'failed'})
                                                        
        smach.StateMachine.add('FindBall1', Search_Ball_Shun(),
                               transitions={'successed':'SHOOT_ADJUST1',
                                            'failed':'failed'})
        

        smach.StateMachine.add('SHOOT_ADJUST1',Shoot_Adjust1(),
                                transitions={'successed':'SHOOT1',
                                             'failed':'failed'})

        smach.StateMachine.add('SHOOT1',Shoot(),
                                transitions={'successed':'FIND_ANOTHER_BALL',
                                            'failed':'failed'})


        smach.StateMachine.add('FIND_ANOTHER_BALL',Find_Another_Ball(),
                                transitions={'successed':'FindBall2',
                                           'failed':'failed'})
        

        smach.StateMachine.add('FindBall2', Search_Ball_Ni(),
                               transitions={'successed': 'SHOOT_ADJUST2',
                                            'failed': 'failed'},)
        
                                        
        smach.StateMachine.add('SHOOT_ADJUST2',Shoot_Adjust2(),
                               transitions={'successed':'SHOOT2',
                                            'failed':'failed'})
        
        smach.StateMachine.add('SHOOT2',Shoot(),
                               transitions={'successed':'Shovel_Control_Up2',
                                            'failed':'failed'})
        #比赛去掉
        smach.StateMachine.add('Shovel_Control_Up2',Shovel_UP(),
                               transitions={'successed':'Return',
                                            'failed':'failed'})


        smach.StateMachine.add('Return',Return(),transitions={'successed':'successed',
                                                                    'failed':'failed'})
        
        '''
        Auir = smach.Concurrence(outcomes=['successed','failed'],
                                default_outcome='failed',
                                outcome_map={'successed':{'RETURN':'successed',
                                                          'Return_Adjust':'successed'}})
        with Auir:
            smach.Concurrence.add('RETURN',Return())
            smach.Concurrence.add('Return_Adjust',Return_Adjust())

        smach.StateMachine.add('My Life For Auir',Auir,transitions={'successed':'successed',
                                                                    'failed':'failed'})
        '''
    shoot_ball_third.execute()
    rospy.logerr('Shoot_Ball_Third has finished!!!!')

if __name__ == '__main__':
    rospy.init_node('ShootBall_third')
    shoot_third()
