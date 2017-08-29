#!/usr/bin/python2
# -*- coding: utf-8 -*-
import rospy
import cv2
import numpy as np
import os
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image


class MyVideoWriter(object):
    # node
    # image_sub_ = rospy.Subscriber()
    # video
    VIDEO_WINDOW_NAME = ''
    show_video_flag = True
    save_video_flag = True
    video_rate = -1
    image_hight = -1
    image_width = -1
    video = cv2.VideoWriter()
    video_file_name = ''
    # frame
    frame_num = -1
    src_3 = np.array([])
    cvi = CvBridge()

    def __init__(self):
        # node
        self.subscribed_topic = rospy.get_param('~subscribed_topic', '/dji_sdk/image_raw')
        self.image_sub_ = rospy.Subscriber(self.subscribed_topic, Image, self.image_callback, queue_size=1)
        # video
        self.show_video_flag = rospy.get_param('~show_video_flag', True)
        if self.show_video_flag:
            self.VIDEO_WINDOW_NAME = 'video'
            cv2.namedWindow(self.VIDEO_WINDOW_NAME)
        self.save_video_flag = rospy.get_param('~save_video_flag', False)
        self.video_rate = rospy.get_param('~rate', 30.0)
        if not os.path.exists('/home/zmart/2017/src/video_writer_py/video/'):
            os.mkdir('/home/zmart/2017/src/video_writer_py/video/')
            print('mkdir /home/zmart/2017/src/video_writer_py/video/')
        self.video_file_name = rospy.get_param('~video_file_name',
                                               '/home/zmart/2017/src/video_writer_py/video/out.avi')
        # frame
        self.frame_num = 1

    def __del__(self):
        cv2.destroyAllWindows()

    def image_callback(self, msg):
        try:
            self.src_3 = self.cvi.imgmsg_to_cv2(msg, 'bgr8')
        except CvBridgeError as e:
            print(e)
            return
        if self.frame_num == 1:
            self.image_hight, self.image_width, channels = self.src_3.shape
            self.video = cv2.VideoWriter(self.video_file_name, cv2.cv.CV_FOURCC('M', 'J', 'P', 'G'), self.video_rate,
                                         (self.image_width, self.image_hight))
        self.frame_num += 1
        # save and show video
        if self.save_video_flag:
            self.video.write(self.src_3)
        if self.show_video_flag:
            cv2.imshow(self.VIDEO_WINDOW_NAME, self.src_3)
            cv2.waitKey(1)


if __name__ == '__main__':
    rospy.init_node('video_writer_py', anonymous=True)
    mrd = MyVideoWriter()
    rospy.spin()
