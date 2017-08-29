#!/usr/bin/python2
# -*- coding: utf-8 -*-
# license removed for brevity
import rospy
from multirobot_detect_iarc.msg import RobotCamPos

def talker():
    pub = rospy.Publisher('/robot_cam_position', RobotCamPos, queue_size=10)
    rospy.init_node('pub_campos_test_py', anonymous=False)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        pubmsg = RobotCamPos()
        pubmsg.exist_rob_flag = True
        pubmsg.exist_obs_flag = True
        pubmsg.rob_num = 2
        pubmsg.obs_num = 1
        pubmsg.rob_cam_pos_x = [1., 2., 0., 0., 0.]
        pubmsg.rob_cam_pos_y = [3., 4., 0., 0., 0.]
        pubmsg.rob_cam_vel_x = [1., -1., 0., 0., 0.]
        pubmsg.rob_cam_vel_y = [1., 1., 0., 0., 0.]
        pubmsg.obs_cam_pos_x = [2., 0., 0., 0.]
        pubmsg.obs_cam_pos_y = [1., 0., 0., 0.]
        pubmsg.obs_cam_vel_x = [-1., 0., 0., 0.]
        pubmsg.obs_cam_vel_y = [0.5, 0., 0., 0.]

        rospy.loginfo(pubmsg)
        pub.publish(pubmsg)
        rate.sleep()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
