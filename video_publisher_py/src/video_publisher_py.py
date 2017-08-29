#!/usr/bin/python2
# -*- coding: utf-8 -*-
import rospy
import cv2
import numpy as np
import math
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


class VideoPublisher(object):
    # node
    # image_pub = rospy.Publisher()
    # video
    WINDOW_NAME = ''
    published_video_file_name = ''
    show_video_flag = True
    my_video = cv2.VideoCapture()
    video_open_flag = False
    video_rate = float('nan')
    video_hight = -1
    video_width = -1
    video_delay = -1
    # trans
    cvi = CvBridge()
    im = Image()
    src = np.array([])

    def __init__(self):
        self.image_pub = rospy.Publisher('/my_video', Image, queue_size=1)
        self.published_video_file_name = rospy.get_param('~published_video_file_name',
                                                         '/home/zj/ros_workspace/src/video_publisher_py/video/bh.avi')
        self.show_video_flag = rospy.get_param('~show_video_flag', True)
        if self.show_video_flag:
            self.WINDOW_NAME = 'Image window'
            cv2.namedWindow(self.WINDOW_NAME)
        self.my_video = cv2.VideoCapture(self.published_video_file_name)
        if not self.my_video.isOpened():
            self.video_open_flag = False
            print('video open error')
        else:
            self.video_open_flag = True
            print('video is open')
            # print self.my_video.set(cv2.cv.CV_CAP_PROP_BRIGHTNESS, 0.1)
            self.video_rate = self.my_video.get(cv2.cv.CV_CAP_PROP_FPS)
            if math.isnan(self.video_rate):
                self.video_rate = rospy.get_param('~given_video_rate', 30.0)
                print('video_rate is nan, using given_video_rate: % s' % self.video_rate)
            else:
                print('video_rate: %s' % self.video_rate)
            self.video_width = self.my_video.get(cv2.cv.CV_CAP_PROP_FRAME_WIDTH)
            print('video_width: %s' % self.video_width)
            self.video_hight = self.my_video.get(cv2.cv.CV_CAP_PROP_FRAME_HEIGHT)
            print('video_hight: %s' % self.video_hight)
            self.video_delay = int(1000.0 / self.video_rate)

    def __del__(self):
        cv2.destroyAllWindows()

    def run_publish(self):
        rate = rospy.Rate(self.video_rate)
        while not rospy.is_shutdown():
            success, self.src = self.my_video.read()
            if not success:
                break
            self.im = self.cvi.cv2_to_imgmsg(self.src, 'bgr8')
            if self.show_video_flag:
                cv2.imshow(self.WINDOW_NAME, self.src)
                cv2.waitKey(1)
            self.image_pub.publish(self.im)
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node('video_publisher_py', anonymous=True)
    vp = VideoPublisher()
    if vp.video_open_flag:
        vp.run_publish()
    rospy.spin()
