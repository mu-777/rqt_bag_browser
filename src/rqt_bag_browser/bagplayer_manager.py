#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import numpy as np

import rospy
import rosbag
import rospkg

from rqt_gui_py.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, QTimer, Signal, Slot
from python_qt_binding.QtGui import QIcon
from python_qt_binding.QtWidgets import QWidget, QFileDialog


class BagPlayer(object):
    def __init__(self, bagpath):
        pass


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

        self.qt_play_btn.clicked[bool].connect(self._on_qt_play_btn_clicked)

        self._play_timer = QTimer()
        self._play_timer.timeout.connect(self.on_idle)
        self._play_timer.setInterval(1)

    def on_idle(self):
        pos = self.qt_seekbar_slider.value() + 1
        if pos > self.qt_seekbar_slider.maximum():
            pos = 0
        self.qt_seekbar_slider.setValue(pos)

    def _on_qt_play_btn_clicked(self, checked):
        if checked:
            self.qt_play_btn.setIcon(self._pause_icon)
            self._play_timer.start()
        else:
            self.qt_play_btn.setIcon(self._play_icon)
            self._play_timer.stop()

    def update_images(self):
        pass

    def show(self, bagpath):
        self._bag_player = BagPlayer(bagpath)
        # self.qt_seekbar_slider.setMinimum(0)
        # self.qt_seekbar_slider.setMaximum(int(self._bag_player.get_length_msec()))
        # self.qt_seekbar_slider.setValue(0)
        self.qt_filename_label.setText(os.path.basename(bagpath))
        self.update_images()
        super(BagPlayerWidget, self).show()