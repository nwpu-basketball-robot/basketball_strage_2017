#!/usr/bin/env python
# -*- coding: UTF-8 -*-

#传球项目1状态机：
#流程：机器前往传球区并放下铲子 -> 传球 -> 前进到找球的位置 -> 开始检测球（以原地自转的方式） -> 检测到球后接近球到球前方0.8米处
#      -> 再次检测球并调整铲子方向后前进  -> 铲球 -> 调整机器角度朝向传球区 -> 投球 -> 升起铲子 -> 回家

import rospy
import smach
import math
import smach_ros
from robot_state_class.pass_ball_first_state import *


def pass_first():
    pass_ball_first = smach.StateMachine(outcomes=['successed', 'failed'])
    rospy.logwarn("pass_ball_first STARTING!!!")
    
    with pass_ball_first:
        
        start = smach.Concurrence(outcomes=['successed','failed'],
                                       default_outcome='successed',
                                       outcome_map={'successed':{'SHOVEL_DOWN':'successed',
                                                                 'MOVE_POINT_TO_SHOOT':'successed'}})
        with start:
            smach.Concurrence.add('SHOVEL_DOWN',Shovel_Down())
            smach.Concurrence.add('MOVE_POINT_TO_SHOOT',Move_Point_To_Shoot())

        smach.StateMachine.add('Start',start,transitions={'successed':'SHOOT1',
                                                          'failed':'failed' })
        
        smach.StateMachine.add('SHOOT1',Shoot(),
                                transitions={'successed':'Shovel_Control_Up1',
                                            'failed':'failed'})

        smach.StateMachine.add('Shovel_Control_Up1',Shovel_UP(),
                                transitions={'successed':'ADJUST1',
                                            'failed':'failed'})
        
        smach.StateMachine.add('ADJUST1',Move_To_Find_Ball(),
                               transitions={'successed':'FindBall',
                                            'failed':'failed'})

        
        smach.StateMachine.add('FindBall', Search_Ball(),
                               transitions={'successed': 'ADJUST2',
                                            'failed': 'failed'})
        
        smach.StateMachine.add('ADJUST2',Shoot_Adjust(),
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
        
                
    pass_ball_first.execute()
    rospy.logerr('Pass_Ball_First has finished!!!!')

if __name__ == '__main__':
    rospy.init_node('passBall_first')
    pass_first()
