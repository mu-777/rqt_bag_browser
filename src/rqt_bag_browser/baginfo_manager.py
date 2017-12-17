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
from python_qt_binding.QtWidgets import QWidget, QFileDialog


class BagInfoModel():
    def __init__(self, bagpath):
        self.bagpath = bagpath

    def get_info_iterator(self):
        info = ''
        if not os.path.exists(self.bagpath):
            rospy.loginfo("Not found bag file: " + self.bagpath)
            info = iter(["Not found bag file\n  " + self.bagpath])
            return info
        try:
            targetbag = rosbag.Bag(self.bagpath)
        except rosbag.bag.ROSBagUnindexedException as e:
            rospy.logerr(e)
            rospy.logerr("Broken bag file: " + self.bagpath)
            info = iter(["!! Broken bag file !!\n  " + self.bagpath])
        else:
            try:
                info = self._parse_baginfo(targetbag)
            except (rosbag.bag.ROSBagFormatException, ValueError) as e:
                rospy.logerr(e)
                info = iter([targetbag._get_yaml_info()])
        return info

    @staticmethod
    def _parse_baginfo(bag):
        def get_duration(bag):
            if bag._chunks:
                start = bag._chunks[0].start_time.to_sec()
                end = bag._chunks[-1].end_time.to_sec()
            else:
                start = min([index[0].time.to_sec()
                             for index in bag._connection_indexes.values()])
                end = max([index[-1].time.to_sec()
                           for index in bag._connection_indexes.values()])
            return end - start

        def get_stats(topic):
            stats = {"freq": -1, "msgnum": -1,
                     "difftime": {"ave": -1, "med": -1, "max": -1, "min": -1},
                     "diffseq": {"ave": -1, "med": -1, "max": -1, "min": -1}}
            msgs = [msg for (_, msg, _) in bag.read_messages(topic)]
            ms_periods = [int((m1.header.stamp.to_nsec() - m0.header.stamp.to_nsec())
                              * 0.001) * 0.001 for m1, m0 in zip(msgs[1:], msgs[:-1])]
            seq_periods = [m1.header.seq - m0.header.seq for m1,
                                                             m0 in zip(msgs[1:], msgs[:-1])]
            for key, periods in zip(["difftime", "diffseq"], [ms_periods, seq_periods]):
                stats[key]["ave"] = np.average(periods)
                stats[key]["med"] = np.median(periods)
                stats[key]["max"] = np.max(periods)
                stats[key]["min"] = np.min(periods)
            stats["freq"] = 1.0 / (stats["difftime"]["med"] * 0.001)
            stats["msgnum"] = len(msgs)
            return stats

        def format_diff(diffdict):
            s = ''
            for key, val in diffdict.items():
                s += key + ': ' + '{0:.3f} '.format(val)
            return s

        def format_size(size):
            multiple = 1024.0
            for suffix in ['KB', 'MB', 'GB', 'TB', 'PB', 'EB', 'ZB', 'YB']:
                size /= multiple
                if size < multiple:
                    return '{0:.2f} {1}'.format(size, suffix)
            return '-'

        indent = "    "
        separator = '------\n'

        info = ''
        info += 'path: {}\n'.format(bag._filename)
        info += 'size: {}\n'.format(format_size(bag.size))
        info += 'time: {0:.3f} sec\n'.format(get_duration(bag))
        info += separator
        yield info

        for topic in sorted(set([c.topic for c in bag._get_connections()])):
            info += 'topic: {0} ({1})\n'.format(topic,
                                                list(bag._get_connections(topic))[0].datatype)
            yield info
            try:
                stats = get_stats(topic)
            except AttributeError as e:
                rospy.logerr(e)
            else:
                info += indent + 'messages: {0:d}\n'.format(stats["msgnum"])
                info += indent + \
                        'frequency: {0:.3f} fps\n'.format(stats["freq"])
                info += indent + 'diff_time[ms]:\n'
                info += indent * 2 + \
                        '{}\n'.format(format_diff(stats["difftime"]))
                info += indent + 'diff_seq:\n'
                info += indent * 2 + \
                        '{}\n'.format(format_diff(stats["diffseq"]))
            info += separator
            yield info


class BagInfoViewWidget(QWidget):
    def __init__(self):
        super(BagInfoViewWidget, self).__init__()
        rospkg_pack = rospkg.RosPack()
        ui_file = os.path.join(rospkg_pack.get_path('bag_browser'),
                               'resource',
                               'BagInfoView.ui')
        loadUi(ui_file, self)

        self._default_msg = 'INFO'
        self._loading_msg = 'Loading...'

    def set_info(self, info):
        self.qt_info_label.setText(info)

    def set_loading(self):
        self.qt_info_label.setText(self._loading_msg)

    def set_default(self):
        self.qt_info_label.setText(self._default_msg)

    def clear(self):
        self.qt_info_label.setText('')

    def show(self, bagpath):
        super(BagInfoViewWidget, self).show()
        self.set_loading()
        self._bag_info = BagInfoModel(bagpath)

        for info in self._bag_info.get_info_iterator():
            print info
            self.set_info(info)
