#!/usr/bin/python3
import glob
import os

import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path

import cv2
import yaml
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtCore import QObject, QThread, pyqtSignal, QDir
from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import (QApplication, QHBoxLayout, QVBoxLayout, QMessageBox)
from PyQt5.QtWidgets import QMainWindow
from mutagen.mp4 import MP4
from rosbag import Bag

from Videoplayer import VideoPlayer


def setNewTime(path, time):
    print("path: ", path)
    mod_output_file = MP4(path)
    mod_output_file["time"] = time
    mod_output_file.save(path)


def getStartTimeVideo(path):
    videofile = MP4(path)
    try:
        start_time = videofile.tags.get("time")[0]
    except TypeError:
        return -1
    return float(start_time)


def getDurationVideo(path):
    duration = subprocess.check_output(
        ["ffprobe", "-v", "error", "-show_entries",
         "format=duration", "-of",
         "default=noprint_wrappers=1:nokey=1", path])
    return float(duration)


def crop(start, end, input, output):
    command = "ffmpeg " + "-i " + input + " -ss  " + str(start) + " -to " + str(end) + " -map_chapters " \
                                                                                       "-1 " + output
    print("Command: ", command)
    os.system(command)


def showWarning(text):
    msg = QMessageBox()
    msg.setIcon(QMessageBox.Warning)
    msg.setText(text)
    msg.setWindowTitle("Предупреждение")
    msg.show()
    return_value = msg.exec()
    if return_value == QMessageBox.Ok:
        return


class WorkerIpCamera(QObject):
    finished_recording = False
    finished_thread = pyqtSignal()
    progress = QtCore.pyqtSignal(str, object)

    def run(self):
        user = "admin"
        password = ""
        server = "10.135.4.238/trackID=2"
        address = f"rtsp://{user}:{password}@{server}"
        print(address)
        os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = "rtsp_transport;udp"

        list_of_files = []
        while len(list_of_files) == 0:
            list_of_files = glob.glob('../catkin_ws/src/mobot/mobot_gazebo/bagfiles/*.bag.active')
            # print(list_of_files)
            # print(len(list_of_files))
            time.sleep(0.5)

        time.sleep(5)

        date_now = datetime.utcnow().strftime("%Y-%m-%d-%H-%M-%S")
        path_to_output_file = f"../videofiles/ip-camera/{date_now}.mp4"

        time_now = str(time.time())

        vcap = cv2.VideoCapture(address, cv2.CAP_FFMPEG)

        if not vcap.isOpened():
            print("Error opening video file")
            self.finished_thread.emit()

        frame_width = int(vcap.get(3))
        frame_height = int(vcap.get(4))
        frame_size = (frame_width, frame_height)
        fps = 25

        out = cv2.VideoWriter(path_to_output_file, cv2.VideoWriter_fourcc(*'mp4v'),  # ('M', 'J', 'P', 'G'),
                              fps, frame_size)

        while not self.finished_recording and vcap.isOpened():
            ret, frame = vcap.read()
            if ret:
                out.write(frame)
                cv2.imshow("Frame", frame)
                cv2.waitKey(fps)
            else:
                vcap = cv2.VideoCapture(address)
                print('Stream disconnected')
                break
        vcap.release()
        out.release()
        cv2.destroyAllWindows()

        setNewTime(path_to_output_file, time_now)
        # print("Time Start of recording Ip-camera : " + time_now)
        # time_now = str(time.time())
        # print("Time End of recording Ip-camera : " + time_now)

        self.finished_thread.emit()

    def finish(self):
        self.finished_recording = True


class WorkerCameraRobot(QObject):
    finished = pyqtSignal()
    progress = QtCore.pyqtSignal(str, object)

    def run(self):
        name = "autobot05"
        proc = subprocess.Popen(["bash", "../scripts/rosrecord.sh", name, "&"])
        # print("after record")
        # print("PID: ", proc.pid)
        self.finished.emit()


class MainWindow(QMainWindow):
    def __init__(self, parent=None):
        super(MainWindow, self).__init__(parent)
        self.setupUi()
        user = "nikita"
        password = "nik"
        server = "nikita-GL62M-7REX"
        # os.system("bash ../scripts/roslaunch.sh & bash ../scripts/rosrun.sh &")  # .
        #           format(user=user, password=password, server=server))  # "ssh {user}@{server} 'bash -s' < script.sh"

        self.recordingButtonState = False

        self.recordingButton.clicked.connect(self.recording)
        self.synchronizationButton.clicked.connect(self.synchronize)

    def setupUi(self):
        self.setWindowTitle("Synchronization application")
        self.setMinimumSize(QtCore.QSize(1280, 760))
        self.setMaximumSize(QtCore.QSize(1280, 760))
        self.centralWidget = QtWidgets.QWidget(self)
        self.mainLayout = QVBoxLayout(self.centralWidget)
        self.centralWidget.setObjectName("centralWidget")
        self.recordingButton = QtWidgets.QPushButton(self.centralWidget)
        font = QtGui.QFont()
        font.setPointSize(9)
        self.recordingButton.setFont(font)
        self.recordingButton.setShortcut("")
        self.recordingButton.setObjectName("recordingButton")
        self.recordingButton.setText("Начать запись")
        self.recordingButton.setFixedHeight(35)

        recordingLayout = QHBoxLayout()
        recordingLayout.setContentsMargins(540, 0, 540, 0)
        recordingLayout.addWidget(self.recordingButton)

        self.synchronizationButton = QtWidgets.QPushButton(self.centralWidget)
        self.synchronizationButton.setGeometry(QtCore.QRect(510, 590, 260, 50))
        font = QtGui.QFont()
        font.setPointSize(9)
        self.synchronizationButton.setFont(font)
        self.synchronizationButton.setObjectName("synchronization")
        self.synchronizationButton.setText("Синхронизовать и воспроизвести")
        self.synchronizationButton.setFixedHeight(35)

        synchronizationLayout = QHBoxLayout()
        synchronizationLayout.setContentsMargins(500, 0, 500, 0)
        synchronizationLayout.addWidget(self.synchronizationButton)

        self.video1 = VideoPlayer()
        self.video2 = VideoPlayer()

        videoLayout = QHBoxLayout()
        videoLayout.setContentsMargins(0, 0, 0, 0)
        videoLayout.addWidget(self.video1)
        videoLayout.addWidget(self.video2)

        self.mainLayout.addLayout(recordingLayout)
        self.mainLayout.addLayout(videoLayout)
        self.mainLayout.addLayout(synchronizationLayout)

        self.setCentralWidget(self.centralWidget)
        self.menubar = QtWidgets.QMenuBar(self)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 800, 26))
        self.menubar.setObjectName("menubar")
        self.setMenuBar(self.menubar)
        self.statusbar = QtWidgets.QStatusBar(self)
        self.statusbar.setObjectName("statusbar")
        self.setStatusBar(self.statusbar)

    def recording(self):
        if not self.recordingButtonState:
            self.recordingButtonState = True
            self.recordingRobotcamera()
            self.recordingButton.setEnabled(False)
            QTimer.singleShot(1000, lambda: self.recordingButton.setEnabled(True))
            self.recordingButton.setText("Остановить запись")
            self.recordingIpcamera()

        else:
            self.recordingButtonState = False
            self.endRecordingIpcamera()
            self.endRecordingRobotcamera()

            self.recordingButton.setText("Начать запись")

    def recordingRobotcamera(self):
        self.threadRobotCamera = QThread()
        self.workerRobotCamera = WorkerCameraRobot()
        self.workerRobotCamera.moveToThread(self.threadRobotCamera)
        self.threadRobotCamera.started.connect(self.workerRobotCamera.run)
        self.workerRobotCamera.finished.connect(self.threadRobotCamera.quit)
        self.workerRobotCamera.finished.connect(self.workerRobotCamera.deleteLater)
        self.threadRobotCamera.finished.connect(self.threadRobotCamera.deleteLater)
        self.threadRobotCamera.start()

    def endRecordingRobotcamera(self):
        os.system("kill -2 `pgrep record`")
        time.sleep(10)
        print(time.time())
        list_of_files = glob.glob('../catkin_ws/src/mobot/mobot_gazebo/bagfiles/*.bag')
        latest_file = max(list_of_files, key=os.path.getctime)

        info_dict = yaml.load(Bag(latest_file, 'r')._get_yaml_info())
        duration = info_dict['duration']
        start_time = str(info_dict['start'])
        print("start_time: ", start_time)
        print("duration: ", duration)

        latest_file_mp4 = Path(latest_file).with_suffix(".mp4")
        print("latest_file_mp4: ", latest_file_mp4)
        os.system(f"bash ../scripts/convert.sh {latest_file}")
        time.sleep(1)
        setNewTime(latest_file_mp4, start_time)

        os.system(f"mv {latest_file_mp4} ../videofiles/camera_robot")

    def recordingIpcamera(self):
        self.threadIpCamera = QThread()
        self.workerIpCamera = WorkerIpCamera(self)
        self.workerIpCamera.moveToThread(self.threadIpCamera)
        self.threadIpCamera.started.connect(self.workerIpCamera.run)
        self.workerIpCamera.finished_thread.connect(self.threadIpCamera.quit)
        self.workerIpCamera.finished_thread.connect(self.workerIpCamera.deleteLater)
        self.threadIpCamera.finished.connect(self.threadIpCamera.deleteLater)
        self.threadIpCamera.start()

    def endRecordingIpcamera(self):
        self.workerIpCamera.finish()

    def synchronize(self):
        path1 = self.video1.currentFileName
        path2 = self.video2.currentFileName

        if path1 is None and path2 is None:
            showWarning("Для синхронизации не были выбраны видеофайлы!")
            print("Для синхронизации не были выбраны видеофайлы!")
        elif path1 is None:
            showWarning("Для синхронизации не был выбран первый видеофайл!")
            print("Для синхронизации не был выбран первый видеофайл!")
        elif path2 is None:
            showWarning("Для синхронизации не был выбран второй видеофайл!")
            print("Для синхронизации не был выбран второй видеофайл!")
        if path1 is None or path2 is None:
            return

        start_time1 = getStartTimeVideo(path1)
        duration1 = getDurationVideo(path1)
        start_time2 = getStartTimeVideo(path2)
        duration2 = getDurationVideo(path2)

        if start_time1 == -1 and start_time2 == -1:
            showWarning("Оба видеофайла не поддерживают синхронизацию!")
            print("Оба видеофайла не поддерживают синхронизацию!")
        elif start_time1 == -1:
            showWarning("Первый видеофайл не поддерживает синхронизацию!")
            print("Первый видеофайл не поддерживает синхронизацию!")
        elif start_time2 == -1:
            showWarning("Второй видеофайл не поддерживает синхронизацию!")
            print("Второй видеофайл не поддерживает синхронизацию!")
        if start_time1 == -1 or start_time2 == -1:
            return

        print("Start time videofile1: ", start_time1)
        print("Duration videofile1: ", duration1)

        print("Start time videofile2: ", start_time2)
        print("Duration videofile2: ", duration2)

        if start_time1 + duration1 < start_time2 or start_time2 + duration2 < start_time1:
            showWarning("Выбранные файлы не имеют общего времени воспроизведения. Синхронизация невозможна.")
            print("Files do not have a total recording time")
            return

        end_time1 = start_time1 + duration1
        end_time2 = start_time2 + duration2

        start_cut1 = 0
        end_cut1 = duration1
        start_cut2 = 0
        end_cut2 = duration2

        if start_time1 < start_time2:
            if end_time1 <= end_time2:
                start_cut1 = start_time2 - start_time1
                end_cut1 = duration1
                start_cut2 = 0
                end_cut2 = end_time1 - start_time2
            elif end_time1 > end_time2:
                start_cut1 = start_time2 - start_time1
                end_cut1 = end_time2 - start_time1
                start_cut2 = 0
                end_cut2 = duration2
        elif start_time1 > start_time2:
            if end_time1 <= end_time2:
                start_cut1 = 0
                end_cut1 = duration1
                start_cut2 = start_time1 - start_time2
                end_cut2 = end_time1 - start_time2
            elif end_time1 > end_time2:
                start_cut1 = 0
                end_cut1 = end_time2 - start_time1
                start_cut2 = start_time1 - start_time2
                end_cut2 = duration2
        elif start_time1 == start_time2:
            if duration1 <= duration2:
                end_cut2 = duration1
            elif duration2 < duration1:
                end_cut1 = duration2

        print("start_cut1: ", start_cut1)
        print("end_cut1: ", end_cut1)
        print("start_cut2: ", start_cut2)
        print("end_cut2: ", end_cut2)

        output1 = f"{QDir.currentPath()}/cut1.mp4"
        output2 = f"{QDir.currentPath()}/cut2.mp4"

        crop(start_cut1, end_cut1, path1, output1)
        crop(start_cut2, end_cut2, path2, output2)

        self.video1.setVideo(output1)
        self.video2.setVideo(output2)
        self.video1.play()
        self.video2.play()

        os.system(f"rm {output1}")
        os.system(f"rm {output2}")

    def closeEvent(self, a0: QtGui.QCloseEvent):
        os.system("kill -9 `pgrep gz`")
        os.system("kill -9 `pgrep image_view`")


app = QApplication(sys.argv)
mainwindow = MainWindow()
mainwindow.show()

sys.exit(app.exec_())
