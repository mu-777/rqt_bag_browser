#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import sys
import signal

from rqt_gui_py.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, QTimer, Signal, Slot, QDir
from python_qt_binding.QtGui import QIcon
from python_qt_binding.QtWidgets import QWidget, QFileDialog, QFileSystemModel, QMessageBox, QApplication, QMainWindow
import rospkg
import rosbag

try:
    from .baginfo_manager import BagInfoModel, BagInfoViewWidget
    from .bagplayer_manager import BagPlayerWidget
except ValueError:
    sys.path.append(os.path.abspath(os.path.dirname(__file__)))
    from baginfo_manager import BagInfoModel, BagInfoViewWidget
    from bagplayer_manager import BagPlayerWidget


class BagPathModel(object):
    def __init__(self, current_dir=None):
        self._current_dir = current_dir if current_dir is not None else os.environ['HOME']
        self._current_bagname = ''
        self._bag = None

    def set_cur_dir(self, dir):
        if os.path.exists(dir):
            self._current_dir = dir
            self._current_bagname = ''
            self._bag = None

    def set_cur_bag(self, bagname):
        if not os.path.exists(os.path.join(self.cur_dir, bagname)):
            return
        try:
            self._bag = rosbag.Bag(os.path.join(self._current_dir, bagname))
        except rosbag.bag.ROSBagException:
            self._bag = None
        self._current_bagname = bagname

    def delete_file(self):
        os.remove(self.cur_bagpath)

    @property
    def cur_dir(self):
        return self._current_dir

    @property
    def cur_dir(self):
        return self._current_dir

    @property
    def cur_bagpath(self):
        return os.path.join(self._current_dir, self._current_bagname)

    @property
    def cur_bagfilename(self):
        return self._current_bagname

    @property
    def has_imagetopics(self):
        if self._bag is None:
            return False
        for topicdata in self._bag.get_type_and_topic_info()[1].values():
            if topicdata.msg_type == 'sensor_msgs/Image':
                return True
        return False

    @property
    def is_validbag(self):
        return self._bag is not None


class BagBrowserWidget(QWidget):
    def __init__(self, widget):
        super(BagBrowserWidget, self).__init__()
        rospkg_pack = rospkg.RosPack()
        ui_file = os.path.join(rospkg_pack.get_path('bag_browser'), 'resource', 'BagBrowser.ui')
        loadUi(ui_file, self)

        self._baginfo_widget = BagInfoViewWidget()
        self._bagplay_widget = BagPlayerWidget()

        # self._updateTimer = QTimer(self)
        # self._updateTimer.timeout.connect(self.timeout_callback)

        self._path_model = BagPathModel()
        self.qt_file_listview.setModel(QFileSystemModel())
        self.qt_file_listview.model().setFilter(QDir.Files | QDir.NoSymLinks | QDir.NoDotAndDotDot)
        self.qt_file_listview.model().setNameFilters(['*.bag'])

        self.qt_file_listview.activated.connect(self.on_qt_file_listview_clicked)

        self._update_cur_dir(self._path_model.cur_dir)

    def start(self):
        # self._updateTimer.start(10)  # loop rate[ms]
        pass

    def stop(self):
        # self._updateTimer.stop()
        pass

    def timeout_callback(self):
        pass

        # override

    def save_settings(self, plugin_settings, instance_settings):
        instance_settings.set_value('data', self._path_model.cur_dir)

    # override
    def restore_settings(self, plugin_settings, instance_settings):
        try:
            data = instance_settings.value('data')
            self._update_cur_dir(data[0])
        except Exception as e:
            self._update_cur_dir('')

    # override
    def shutdown_plugin(self):
        self._baginfo_widget.close()
        self._bagplay_widget.close()

    def _update_cur_dir(self, dir):
        self._path_model.set_cur_dir(dir)
        self.qt_directory_lineedit.setText(self._path_model.cur_dir)
        index = self.qt_file_listview.model().setRootPath(self._path_model.cur_dir)
        self.qt_file_listview.setRootIndex(index)

    def _update_bagbtn_enable(self):
        self.qt_showinfo_btn.setEnabled(self._path_model.is_validbag)
        self.qt_delete_btn.setEnabled(self._path_model.is_validbag)
        self.qt_play_btn.setEnabled(self._path_model.has_imagetopics)

    @Slot()
    def on_qt_fileopen_btn_clicked(self):
        dir = QFileDialog.getExistingDirectory(self, 'Open a folder',
                                               self._path_model.cur_dir)
        self._update_cur_dir(dir)
        self._update_bagbtn_enable()

    @Slot()
    def on_qt_directory_lineedit_returnPressed(self):
        self._update_cur_dir(self.qt_directory_lineedit.text())
        self._update_bagbtn_enable()

    @Slot()
    def on_qt_delete_btn_clicked(self):
        msgbox = QMessageBox(QMessageBox.Warning,
                             'Do you want to delete this?',
                             'Name: {0}\nPath: {1}'.format(self._path_model.cur_bagfilename,
                                                           self._path_model.cur_bagpath))
        msgbox.addButton('DELETE', QMessageBox.AcceptRole)
        msgbox.addButton('cancel', QMessageBox.RejectRole)
        if msgbox.exec_() == 0:
            self._path_model.delete_file()

    @Slot()
    def on_qt_showinfo_btn_clicked(self):
        self._baginfo_widget.show(self._path_model.cur_bagpath)

    @Slot()
    def on_qt_play_btn_clicked(self):
        self._bagplay_widget.show(self._path_model.cur_bagpath)

    def on_qt_file_listview_clicked(self, index):
        filename = self.qt_file_listview.model().data(index)
        if os.path.splitext(filename)[1] == '.bag':
            self._path_model.set_cur_bag(filename)
            self._update_bagbtn_enable()


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


class BagBrowserMainWidget(QMainWindow):
    def __init__(self):
        super(BagBrowserMainWidget, self).__init__()
        self.setObjectName('BagBrowser')
        self._widget = BagBrowserWidget(self)
        self._widget.start()
        self.setCentralWidget(self._widget)


# --------------------------------------------
if __name__ == '__main__':
    def sigint_handler(*args):
        # if QMessageBox.question(None, '', "Are you sure you want to quit?",
        #                         QMessageBox.Yes | QMessageBox.No,
        #                         QMessageBox.No) == QMessageBox.Yes:
        #     QApplication.quit()
        QApplication.quit()


    signal.signal(signal.SIGINT, sigint_handler)

    app = QApplication(sys.argv)
    mw = BagBrowserMainWidget()
    mw.setWindowIcon(QIcon.fromTheme('system-search'))
    w, h = 700, 500
    mw.resize(w, h)
    mw.show()
    sys.exit(app.exec_())
