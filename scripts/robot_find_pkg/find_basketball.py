#!/usr/bin/env python
#coding:utf-8

# 检测篮球的相关接口
import math
import rospy
import geometry_msgs.msg as g_msgs
from basketball_msgs.srv import *

class find_basketball(object):
    def __init__(self):
        self.rate = rospy.Rate(10)
        self.cmd_angular_pub = rospy.Publisher('/cmd_move',g_msgs.Twist,queue_size=100)
        self.find_ball_client = rospy.ServiceProxy('visionDate',visionDate)
        self.rate = rospy.Rate(30)

    
    #发送急停速度，使机器人转动停止
    def brake(self):#停止时的回调函数
        move_velocity = g_msgs.Twist()
        move_velocity.linear.x = 0
        move_velocity.linear.y = 0
        move_velocity.angular.z = 0
        self.cmd_angular_pub.publish(move_velocity)

    #直接获取当前检测到的球的数据
    #会有队列直接炸掉和玄学找不到球的问题，所以基本弃用
    def findball_directly(self):
        #设置停止回调函数
        rospy.on_shutdown(self.brake)
        rospy.logwarn('[visionDate]->waiting BallDate service')
        self.find_ball_client.wait_for_service()
        rospy.logwarn('[visionDate] -> connected to ball service')
        #判断是否找到了球
        index = 0
        while not rospy.is_shutdown():
            res = self.find_ball_client(1)
            length = len(res.balls)
            if length >= 1:
                has_ball = True
            else:
                has_ball = False

            if has_ball == True:
                res = self.find_ball_client(1)
                Type = res.balls[index].type
                theta = -res.balls[index].theta
                dis = res.balls[index].distance
                index += 1
                if index >= len(res.balls):
                    index = 0
                if Type == 1:
                    return (dis, theta)
                else :
                    rospy.loginfo('We can not find the ball!!!')
            else:
                rospy.loginfo('We can not find the ball!!!')

    #顺时针边旋转边找球
    def findball_shun(self):
        rospy.logwarn('[visionDate]->waiting BallDate service')
        self.find_ball_client.wait_for_service()
        rospy.logwarn('[visionDate] -> connected to ball service')
        move_velocity = g_msgs.Twist()
        has_ball = False
        #设置轮子的转速为3
        index=0
        while not rospy.is_shutdown() :
            res = self.find_ball_client(1)
            length = len(res.balls)
            if len(res.balls) >= 1:
                if res.balls[0].type == 1 and res.balls[0].distance>0:
                    has_ball = True
                    break
            else:
                has_ball = False
            move_velocity.angular.z = -3
            self.cmd_angular_pub.publish(move_velocity)

        self.brake()
        #判断是否找到了球
        while not rospy.is_shutdown() and has_ball==True:
            res = self.find_ball_client(1)
            length = len(res.balls)
            if length >= 1:
                self.brake()
                Type = res.balls[0].type
                theta = -res.balls[0].theta
                dis = res.balls[0].distance
                #距离为-1时表示kinect摄像机可以看到球但是深度摄像机看不到球(两个摄像头的位置不同)
                if Type == 1 and dis > 0:
                    rospy.logwarn('The basketball is located in dis=%s theta=%s'%(dis,theta*180/math.pi))
                    return (dis, theta)
                else :
                    rospy.loginfo('We can not find the ball!!!')
            else:
                rospy.loginfo('We can not find the ball!!!')
                move_velocity.angular.z = -3
                self.cmd_angular_pub.publish(move_velocity)

    #逆时针边旋转边找球
    def findball_ni(self):
        rospy.logwarn('[visionDate]->waiting BallDate service')
        self.find_ball_client.wait_for_service()
        rospy.logwarn('[visionDate] -> connected to ball service')
        move_velocity = g_msgs.Twist()
        has_ball = False
        #设置轮子的转速为3
        while not rospy.is_shutdown() :
            res = self.find_ball_client(1)
            length = len(res.balls)
            if len(res.balls) >= 1:
                if res.balls[0].type == 1 and res.balls[0].distance>0:
                    has_ball = True
                    break
            else:
                has_ball = False
            move_velocity.angular.z = 3
            self.cmd_angular_pub.publish(move_velocity)
        self.brake()
        #判断是否找到了球
        while not rospy.is_shutdown() and has_ball==True:
            res = self.find_ball_client(1)
            length = len(res.balls)
            if length >= 1:
                self.brake()
                Type = res.balls[0].type
                theta = -res.balls[0].theta
                dis = res.balls[0].distance
                #距离为-1时表示kinect摄像机可以看到球但是深度摄像机看不到球(两个摄像头的位置不同)
                if Type == 1 and dis > 0:
                    rospy.logwarn('The basketball is located in dis=%s theta=%s'%(dis,theta*180/math.pi))
                    return (dis, theta)
                else :
                    rospy.loginfo('We can not find the ball!!!')
            else:
                rospy.loginfo('We can not find the ball!!!')
                move_velocity.angular.z = 3
                self.cmd_angular_pub.publish(move_velocity)
    

if __name__ == '__main__':
    rospy.init_node('find_basketball')
    test = find_basketball()
    test.findball_shun()

