#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os

from rqt_gui_py.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, QTimer, Signal, Slot
from python_qt_binding.QtWidgets import QWidget
import rospkg

class BagBrowserWidget(QWidget):
    def __init__(self, widget):
        super(BagBrowserWidget, self).__init__()
        rospkg_pack = rospkg.RosPack()
        ui_file = os.path.join(rospkg_pack.get_path('bag_browser'), 'resource', 'BagBrowser.ui')
        loadUi(ui_file, self)

        self._updateTimer = QTimer(self)
        self._updateTimer.timeout.connect(self.timeout_callback)

    def start(self):
        self._updateTimer.start(10)  # loop rate[ms]

    def stop(self):
        self._updateTimer.stop()

    def timeout_callback(self):
        pass

     # override
    def save_settings(self, plugin_settings, instance_settings):
        # self._frame_id = self._frame_id_widget.str
        # self._child_frame_id = self._child_frame_id_widget.str
        # instance_settings.set_value('frame_ids', (self._frame_id, self._child_frame_id))
        pass

    # override
    def restore_settings(self, plugin_settings, instance_settings):
        # frames = instance_settings.value('frame_ids')
        # try:
        #     self._frame_id, self._child_frame_id = frames
        #     self._frame_id_widget.update(self._frame_id)
        #     self._child_frame_id_widget.update(self._child_frame_id)
        # except Exception:
        #     self._frame_id, self._child_frame_id = FRAME_ID, CHILD_FRAME_ID
        pass

    # override
    def shutdown_plugin(self):
        self.stop()

class BagBrowser(Plugin):
    def __init__(self, context):
        super(BagBrowser, self).__init__(context)
        self.setObjectName('BagBrowser')
        self._widget = BagBrowserWidget(self)
        self._widget.start()
        context.add_widget(self._widget)

    def save_settings(self, plugin_settings, instance_settings):
        self._widget.save_settings(plugin_settings, instance_settings)

    def restore_settings(self, plugin_settings, instance_settings):
        self._widget.restore_settings(plugin_settings, instance_settings)

    def shutdown_plugin(self):
        self._widget.shutdown_plugin()
