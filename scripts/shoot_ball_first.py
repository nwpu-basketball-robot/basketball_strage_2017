#!/usr/bin/env python
# -*- coding: UTF-8 -*-


#投篮项目1状态机：
#流程：机器前往投篮区域（并放下铲子）-> 检测定位柱 -> 对准定位柱 -> 投篮 -> 前进到中间置球区附近 -> 开始检测球（以原地自转的方式） -> 检测到球后接近球到球前方1米处
#      -> 再次检测球并调整铲子方向后前进  -> 铲球 -> 前进到投篮区域（定位柱附近） -> 检测定位柱 -> 对准定位柱 -> 投篮


import rospy
import smach
import math
import smach_ros
from robot_state_class.shoot_ball_first_state import *


def Shoot_First():
    Shoot_Ball_First = smach.StateMachine(outcomes=['successed', 'failed'])
    rospy.logwarn("shoot_ball_first STARTING!!!")
    with Shoot_Ball_First:
        
        smach.StateMachine.add('MOVE_POINT_TO_SHOOT',Move_Point_To_Shoot(),
                                transitions={'successed':'SHOOT1',
                                            'failed':'failed'})  
                                                                                        
        smach.StateMachine.add('SHOOT1',Shoot(),
                                transitions={'successed':'Shovel_Control_Up1',
                                            'failed':'failed'})
        
        smach.StateMachine.add('Shovel_Control_Up1',Shovel_UP(),
                                transitions={'successed':'ADJUST1',
                                            'failed':'failed'})
        
        smach.StateMachine.add('ADJUST1',Shoot_Adjust1(),
                               transitions={'successed':'FindBall',
                                            'failed':'failed'})

        smach.StateMachine.add('FindBall', Search_Ball(),
                               transitions={'successed': 'ADJUST2',
                                            'failed': 'failed'})
        
        smach.StateMachine.add('ADJUST2',Shoot_Adjust2(),
                               transitions={'successed':'SHOOT2',
                                            'failed':'failed'})
        
        smach.StateMachine.add('SHOOT2',Shoot(),
                               transitions={'successed':'Shovel_Control_Up2',
                                            'failed':'failed'})

        smach.StateMachine.add('Shovel_Control_Up2',Shovel_UP(),
                               transitions={'successed':'Return',
                                            'failed':'failed'})

        smach.StateMachine.add('Return',Return(),
                               transitions={'successed':'successed',
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
         
    Shoot_Ball_First.execute()
    rospy.logerr('Shoot_Ball_First has finished!!!!')

if __name__ == '__main__':
    rospy.init_node('ShootBall_First')
    Shoot_First()
