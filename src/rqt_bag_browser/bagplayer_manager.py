#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import numpy as np
from operator import add
import cv2

import rospy
import rosbag
import rospkg

from cv_bridge import CvBridge, CvBridgeError

from rqt_gui_py.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, QTimer, Signal, Slot
from python_qt_binding.QtGui import QIcon
from python_qt_binding.QtWidgets import QWidget, QFileDialog


class BagPlayer(object):
    def __init__(self, bagpath):
        self._bag = rosbag.Bag(bagpath)
        self._img_topicnames = self.get_img_topicname_list(self._bag)
        self._imgts_list = self.get_img_timestamp_list(self._bag, self._img_topicnames)
        self._imgs_dict = {tn: self.get_img_list(self._bag, tn,
                                                 self._imgts_list[0],
                                                 self._imgts_list[-1]) for tn in self._img_topicnames}

    def get_imgs(self, t):
        return [self._imgs_dict[tn][t] for tn in self._img_topicnames]

    def get_length_msec(self):
        return self._imgts_list[-1] - self._imgts_list[0] + 1

    @staticmethod
    def get_img_topicname_list(bag):
        return sorted(
            [td[0] for td in bag.get_type_and_topic_info()[1].items() if td[1].msg_type == 'sensor_msgs/Image'])

    @staticmethod
    def get_img_timestamp_list(bag, img_topicnames):
        return sorted((list(set(reduce(add,
                                       [[BagPlayer.rostime2msec(ts) for (_, msg, ts) in bag.read_messages(img_topic)]
                                        for img_topic in img_topicnames])))))

    @staticmethod
    def get_img_list(bag, img_topicname, start_ms, end_ms):
        l = [None] * (end_ms - start_ms + 1)
        for (_, msg, ts) in bag.read_messages(img_topicname):
            l[BagPlayer.rostime2msec(ts) - start_ms] = BagPlayer.rosimg2cvimg(msg)
        return l

    @staticmethod
    def rostime2msec(rostime):
        return int(rostime.to_sec() * 1000)

    @staticmethod
    def rosimg2cvimg(rosimg):
        try:
            return CvBridge().imgmsg_to_cv2(rosimg)
        except CvBridgeError as e:
            print e
            return None


class BagPlayerWidget(QWidget):
    def __init__(self):
        super(BagPlayerWidget, self).__init__()
        rospkg_pack = rospkg.RosPack()
        ui_file = os.path.join(rospkg_pack.get_path('bag_browser'),
                               'resource',
                               'BagPlayer.ui')
        loadUi(ui_file, self)

        self._play_icon = QIcon.fromTheme('media-playback-start')
        self._pause_icon = QIcon.fromTheme('media-playback-pause')
        self.qt_play_btn.setIcon(self._play_icon)

        self._playing = False
        self._current_msec = 0
        self._max_msec = 1000000
        self.qt_seekbar_slider.setTracking(False)

        self._play_timer = QTimer()
        self._play_timer.timeout.connect(self.on_image_update)
        self._play_timer.setInterval(1)  # [ms]

        self._gui_timer = QTimer()
        self._gui_timer.timeout.connect(self.on_gui_update)
        self._gui_timer.setInterval(100)  # [ms]
        self._gui_timer.start()

    def close(self):
        self._play_timer.stop()
        self._gui_timer.stop()
        super(BagPlayerWidget, self).close()

    def show(self, bagpath):
        self._bag_player = BagPlayer(bagpath)
        self.qt_filename_label.setText(os.path.basename(bagpath))

        self._max_msec = self._bag_player.get_length_msec()
        self.qt_seekbar_slider.setMinimum(0)
        self.qt_seekbar_slider.setMaximum(self._max_msec - 1)
        self.qt_seekbar_slider.setValue(0)
        self.qt_time_numer_label.setText(str(0))
        self.qt_time_denom_label.setText('/{0}[s]'.format(int(self._max_msec * 0.001)))

        self.update_images(self.qt_seekbar_slider.value())
        super(BagPlayerWidget, self).show()

    def on_image_update(self):
        self._current_msec = (self._current_msec + 1) % self._max_msec
        self.update_images(self._current_msec)

    def on_gui_update(self):
        t = self._current_msec
        self.qt_seekbar_slider.setValue(t)
        self.qt_time_numer_label.setText('{0:.1f}'.format(t * 0.001))

    def update_images(self, t):
        img = self._bag_player.get_imgs(t)[0]
        if img is not None:
            cv2.imshow('', img)

    @Slot()
    def on_qt_seekbar_slider_sliderPressed(self):
        self._play_timer.stop()
        self._gui_timer.stop()
        self.qt_seekbar_slider.setTracking(True)

    @Slot()
    def on_qt_seekbar_slider_sliderReleased(self):
        self._current_msec = self.qt_seekbar_slider.value()
        self.qt_seekbar_slider.setTracking(False)
        if self._playing:
            self._play_timer.start()
        else:
            self._play_timer.stop()
        self._gui_timer.start()

    @Slot()
    def on_qt_play_btn_clicked(self):
        if not self._playing:
            self.qt_play_btn.setIcon(self._pause_icon)
            self._play_timer.start()
        else:
            self.qt_play_btn.setIcon(self._play_icon)
            self._play_timer.stop()
        self._playing = not self._playing
