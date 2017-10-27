#!/usr/bin/python
#coding=utf-8

#为状态机模块提供找定位柱的接口
#寻找柱子的函数分为直接找，顺时针找，逆时针找和直接找。主要使用顺逆时针找柱子（直接找很多情况下找不到）
import math
import rospy
from basketball_msgs.srv import *
import geometry_msgs.msg as g_msgs


class find_cylinder(object):
    def __init__(self):
        self.cmd_angular_pub = rospy.Publisher('/cmd_move',g_msgs.Twist,queue_size=100)
        self.find_cylinder_client = rospy.ServiceProxy('visionDate',visionDate)

    #发送急停速度，使机器人转动停止
    def brake(self):
        move_velocity = g_msgs.Twist()
        move_velocity.linear.x = 0
        move_velocity.linear.y = 0
        move_velocity.angular.z = 0
        self.cmd_angular_pub.publish(move_velocity)

    #直接获取当前检测到的柱子的数据
    def findcylinder_directly(self): 
        rospy.logwarn('[visionDate]->waiting CylinderDate service')
        self.find_cylinder_client.wait_for_service()
        rospy.logwarn('[visionDate] -> connected to cylinder service')
        res = self.find_cylinder_client(2)
        #判断是否找到了柱子
        length = len(res.cylinders)
        if len(res.cylinders) >= 1:
            has_cylinder = True
        else:
            has_cylinder = False
        if not rospy.is_shutdown() and has_cylinder == True:    
            #判断是否找到了柱子
            while not rospy.is_shutdown():
                res = self.find_cylinder_client(2)
                theta = -res.cylinders[0].theta
                dis = res.cylinders[0].distance
                #判断柱子是否太远无法检测
                if dis < 0:
                    rospy.logerr('The cylinder is too far!!!')
                else:
                    rospy.logwarn('The cylinder is located in dis=%s theta=%s'%(dis,theta))
                    return (dis, theta)
    
    #逆时针边旋转边找柱子
    def findcylinder_ni(self):
        rospy.logwarn('[visionDate]->waiting CylinderDate service')
        self.find_cylinder_client.wait_for_service()
        rospy.logwarn('[visionDate] -> connected to Cylinder service')
        move_velocity = g_msgs.Twist()
        #设置轮子的转速为3
        
        while not rospy.is_shutdown() :
            res = self.find_cylinder_client(2)
            length = len(res.cylinders)
            if len(res.cylinders) >= 1:
                break
            else:
                has_cylinder = False
            move_velocity.angular.z = 3
            self.cmd_angular_pub.publish(move_velocity)
        self.brake()       
        if not rospy.is_shutdown():    
            #判断是否找到了柱子
            while not rospy.is_shutdown():
                res = self.find_cylinder_client(2)
                length = len(res.cylinders)
                if length >= 1:
                    has_cylinder = True
                else:
                    has_cylinder = False

                if has_cylinder == True:
                    theta = -res.cylinders[0].theta
                    dis = res.cylinders[0].distance
                    #判断柱子是否太远无法检测
                    if dis < 0:
                        rospy.logerr('The cylinder is too far!!!')   
                    else:
                        rospy.logwarn('The cylinder is located in dis=%s theta=%s'%(dis,theta))
                        return (dis, theta)

    #顺时针边旋转边找柱子
    def findcylinder_shun(self):
        rospy.logwarn('[visionDate]->waiting CylinderDate service')
        self.find_cylinder_client.wait_for_service()
        rospy.logwarn('[visionDate] -> connected to Cylinder service')
        move_velocity = g_msgs.Twist()
        #设置轮子的转速为3
        
        while not rospy.is_shutdown() :
            res = self.find_cylinder_client(2)
            length = len(res.cylinders)
            if len(res.cylinders) >= 1:
                if res.cylinders[0].distance>0:
                    break
            else:
                has_cylinder = False
            move_velocity.angular.z = -3
            self.cmd_angular_pub.publish(move_velocity)
        self.brake()       
        if not rospy.is_shutdown():    
            #判断是否找到了柱子
            while not rospy.is_shutdown():
                res = self.find_cylinder_client(2)
                length = len(res.cylinders)
                if length >= 1:
                    has_cylinder = True
                else:
                    has_cylinder = False

                if has_cylinder == True:
                    theta = -res.cylinders[0].theta
                    dis = res.cylinders[0].distance
                    #判断柱子是否太远无法检测
                    if dis < 0:
                        rospy.logerr('The cylinder is too far!!!')   
                    else:
                        rospy.logwarn('The cylinder is located in dis=%s theta=%s'%(dis,theta))
                        return (dis, theta)


if __name__ == '__main__':
    rospy.init_node('find_cylinder')
    test = find_cylinder()
    test.findcylinder_shun()
