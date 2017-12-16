#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
from operator import add
import cv2

import rosbag
import rospkg

from cv_bridge import CvBridge, CvBridgeError

from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, QTimer, Signal, Slot
from python_qt_binding.QtGui import QIcon, QImage, QPixmap
from python_qt_binding.QtWidgets import QWidget, QFileDialog


class BagPlayer(object):
    def __init__(self, bagpath, converter):
        self._bag = rosbag.Bag(bagpath)
        self._img_topicnames = self.get_img_topicname_list(self._bag)
        self._imgts_list = self.get_img_timestamp_list(self._bag, self._img_topicnames)
        self._imgs_dict = {tn: self.get_img_list(self._bag, tn,
                                                 self._imgts_list[0], self._imgts_list[-1],
                                                 converter) for tn in self._img_topicnames}

    def get_names(self):
        return self._img_topicnames

    def get_imgs(self, t):
        return [self._imgs_dict[tn][t] for tn in self._img_topicnames], self._img_topicnames

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
    def get_img_list(bag, img_topicname, start_ms, end_ms, converter):
        l = [None] * (end_ms - start_ms + 1)
        for (_, msg, ts) in bag.read_messages(img_topicname):
            l[BagPlayer.rostime2msec(ts) - start_ms] = converter(msg)
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


class ImageViewWidget(QWidget):
    def __init__(self, title=None, image=None):
        super(ImageViewWidget, self).__init__()
        rospkg_pack = rospkg.RosPack()
        ui_file = os.path.join(rospkg_pack.get_path('bag_browser'),
                               'resource',
                               'ImageView.ui')
        loadUi(ui_file, self)
        if title is not None:
            self.set_title(title)
        if image is not None:
            self.set_title(image)

    def set_title(self, title):
        self.qt_imgtitle_label.setText(title)

    def set_image(self, pixmap):
        self.qt_img_label.setPixmap(pixmap)


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
        def converter(rosimg):
            cvimg = BagPlayer.rosimg2cvimg(rosimg)
            height, width = cvimg.shape
            return QPixmap.fromImage(QImage(cvimg.data, width, height, QImage.Format_Grayscale8))

        self._bag_player = BagPlayer(bagpath, converter)
        self.qt_filename_label.setText(os.path.basename(bagpath))

        self._max_msec = self._bag_player.get_length_msec()
        self.qt_seekbar_slider.setMinimum(0)
        self.qt_seekbar_slider.setMaximum(self._max_msec - 1)
        self.qt_seekbar_slider.setValue(0)
        self.qt_time_numer_label.setText(str(0))
        self.qt_time_denom_label.setText('/{0}[s]'.format(int(self._max_msec * 0.001)))

        self.image_widgets = {}
        for i, name in enumerate(self._bag_player.get_names()):
            self.image_widgets[name] = ImageViewWidget(title=name)
            self.qt_imgs_gridlayout.addWidget(self.image_widgets[name], i/2, i%2)
        self.qt_imgs_gridlayout.setSpacing(3)

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
        imgs, names = self._bag_player.get_imgs(t)
        for img, name in zip(imgs, names):
            if img is None:
                continue
            self.image_widgets[name].set_image(img)

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
