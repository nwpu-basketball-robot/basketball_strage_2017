#!/usr/bin/env python
# -*- coding: UTF-8 -*-
#传球项目3的状态机
#流程： 前进到三分线附近 -> 自转并检测篮球 -> 检测到球后接近球 -> 二次检测球并调整角度 -> 铲球 -> 调整传球方向 -> 传球
#      -> 移动到底脚置球区附近 ->在自转并检测球 -> 检测到球后接近球 -> 二次检测并调整 -> 铲球 -> 调整传球位置和方向 -> 传球 —> 回家

import rospy
import smach
import math
import smach_ros
from robot_state_class.pass_ball_third_state import *


def pass_third():
    pass_ball_third = smach.StateMachine(outcomes=['successed', 'failed'])

    rospy.logwarn("pass_ball_third STARTING!!!")
    with pass_ball_third:
        
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
                                transitions={'successed':'SHOVEL_UP',
                                            'failed':'failed'})

        smach.StateMachine.add('SHOVEL_UP',Shovel_UP(),
                                transitions={'successed':'FIND_ANOTHER_BALL',
                                             'failed':'failed'})

        smach.StateMachine.add('FIND_ANOTHER_BALL',Find_Another_Ball(),
                                transitions={'successed':'FindBall2',
                                           'failed':'failed'})
        
        smach.StateMachine.add('FindBall2', Search_Ball_Ni(),
                               transitions={'successed': 'SHOOT_ADJUST2',
                                            'failed': 'failed'})
        
                                        
        smach.StateMachine.add('SHOOT_ADJUST2',Shoot_Adjust2(),
                               transitions={'successed':'SHOOT2',
                                            'failed':'failed'})
        
        smach.StateMachine.add('SHOOT2',Shoot(),
                               transitions={'successed':'Shovel_Control_Up2',
                                            'failed':'failed'})
        
        
        smach.StateMachine.add('Shovel_Control_Up2',Shovel_UP(),
                               transitions={'successed':'My Life For Auir',
                                            'failed':'failed'})

        Auir = smach.Concurrence(outcomes=['successed','failed'],
                                default_outcome='failed',
                                outcome_map={'successed':{'RETURN':'successed',
                                                          'Return_Adjust':'successed'}})
        with Auir:
            smach.Concurrence.add('RETURN',Return())
            smach.Concurrence.add('Return_Adjust',Return_Adjust())

        smach.StateMachine.add('My Life For Auir',Auir,transitions={'successed':'successed',
                                                                    'failed':'failed'})
        
    pass_ball_third.execute()
    rospy.logerr('Pass_Ball_Third has finished!!!!')

if __name__ == '__main__':
    rospy.init_node('PassBall_third')
    pass_third()
