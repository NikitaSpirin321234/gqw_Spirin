import sys
import time
from multiprocessing import Process

import cv2
import os
import glob
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import (QApplication, QHBoxLayout, QVBoxLayout)
from PyQt5.QtWidgets import QMainWindow
from datetime import datetime
from pathlib import Path
import multiprocessing as mp
from Videoplayer import VideoPlayer


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.guiInit()
        user = "nikita"
        password = "nik"
        server = "nikita-GL62M-7REX"
        os.system("bash ../scripts/roslaunch.sh & bash ../scripts/rosrun.sh &".
                  format(user=user, password=password, server=server))  # "ssh {user}@{server} 'bash -s' < script.sh"

        self.recordingButtonState = False

        self.recordingButton.clicked.connect(self.recording)

    def guiInit(self):
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

        recordingButtonLayout = QHBoxLayout()
        recordingButtonLayout.setContentsMargins(540, 0, 540, 0)
        recordingButtonLayout.addWidget(self.recordingButton)

        self.synchronization = QtWidgets.QPushButton(self.centralWidget)
        self.synchronization.setGeometry(QtCore.QRect(510, 590, 260, 50))
        font = QtGui.QFont()
        font.setPointSize(9)
        self.synchronization.setFont(font)
        self.synchronization.setObjectName("synchronization")
        self.synchronization.setText("Синхронизовать и воспроизвести")
        self.synchronization.setFixedHeight(35)

        synchronizationLayout = QHBoxLayout()
        synchronizationLayout.setContentsMargins(500, 0, 500, 0)
        synchronizationLayout.addWidget(self.synchronization)

        self.video1 = VideoPlayer()
        self.video2 = VideoPlayer()

        videoLayout = QHBoxLayout()
        videoLayout.setContentsMargins(0, 0, 0, 0)
        videoLayout.addWidget(self.video1)
        videoLayout.addWidget(self.video2)

        self.mainLayout.addLayout(recordingButtonLayout)
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
            # self.recordingIpcamera()

            # p1 = Process(target=self.recordingIpcamera)
            # p1.start()
            # p2 = Process(target=self.recordingRobotcamera)
            # p2.start()

            self.recordingButton.setEnabled(False)
            QTimer.singleShot(1000, lambda: self.recordingButton.setEnabled(True))
            self.recordingButton.setText("Остановить запись")

        else:
            self.recordingButtonState = False

            os.system("bash ../scripts/stoprecord.sh")
            time.sleep(1)
            list_of_files = glob.glob('../catkin_ws/src/mobot/mobot_gazebo/bagfiles/*.bag')
            latest_file = max(list_of_files, key=os.path.getctime)
            latest_file_mp4 = Path(latest_file).with_suffix(".mp4")
            os.system(f"bash ../scripts/convert.sh {latest_file}")
            os.system(f"mv {latest_file_mp4} ../videofiles/camera_robot")

            self.recordingButton.setText("Начать запись")

    def recordingRobotcamera(self):
        os.system("bash ../scripts/rosrecord.sh &")

    def recordingIpcamera(self):
        user = "admin"
        password = "admin"
        server = "192.168.4.100:8554/live"
        address = f"rtsp://{user}:{password}@{server}"
        os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = "rtsp_transport;udp"
        vcap = cv2.VideoCapture(address, cv2.CAP_FFMPEG)

        if not vcap.isOpened():
            print("Error opening video file")
            return

        frame_width = int(vcap.get(3))
        frame_height = int(vcap.get(4))
        frame_size = (frame_width, frame_height)
        fps = 25
        date_now = datetime.utcnow().strftime("%Y-%m-%d-%H-%M-%S")
        path_to_output_file = f"../videofiles/ip-camera/{date_now}.mp4"

        out = cv2.VideoWriter(path_to_output_file, cv2.VideoWriter_fourcc(*'mp4v'),  # ('M', 'J', 'P', 'G'),
                              fps, frame_size)

        while self.recordingButtonState and vcap.isOpened():
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

    def closeEvent(self, a0: QtGui.QCloseEvent):
        os.system("kill -9 `pgrep gz`")
        os.system("kill -9 `pgrep image_view`")


app = QApplication(sys.argv)
mainwindow = MainWindow()
mainwindow.show()

sys.exit(app.exec_())
