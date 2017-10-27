所有状态机文件均在scripts文件夹下

目录列表：
scripts:
    -robot_find_pkg(摄像头的相关接口的目录)
        find_basketball.py  (检测篮球的接口)
        find_volleyball.py （检测排球的接口）
        find_cylinder.py    (检测柱子的接口)
    -robot_move_pkg（机器人移动的相关接口的目录）
        linear_move.py （机器人以世界坐标系移动的接口）
        move_in_robot.py （机器人以机器人坐标系移动的接口））
        turn_an_angular.py （机器人转一定角度的接口）

    -robot_shovel_srv（铲子控制的接口的目录）
        control_srv.py （铲子服务的接口）

    -robot_state_class（状态类的目录）
        pass_ball_first_state.py
        pass_ball_second_state.py
        pass_nall_third_state.py
        shoot_ball_first_state.py
        shoot_ball_second_state.py
        shoot_ball_third_state.py

    -robot_state_pkg （和机器位置有关的接口的目录）
        get_robot_position.py （获取机器人位置信息的接口）

    control_state.py （总的运行文件，通关接收参数不同来运行不同的状态机文件）
    pass_ball_first.py
    pass_ball_second.py
    pass_ball_third.py
    shoot_ball_first.py
    shoot_ball_second.py
    shoot_ball_third.py



运行方法：
  传球项目：
	sudo chmod 0777 /dev/ttyUSB0 (打开串口)
    roslaunch basketball_bringup start_robot.launch （打开相关节点）	
	rosrun vision vision (启动图像节点)
	rosrun basketball_strage pass_ball_>>> （最后为具体的状态机文件，运行状态机）

  投篮项目：
    sudo chmod 0777 /dev/ttyUSB0 (打开串口)
	roslaunch basketball_bringup start_robot.launch （打开相关节点）
	rosrun vision vision (启动图像节点)
	rosrun basketball_strage shoot_ball_>>> （最后为具体的状态机文件，运行状态机）

