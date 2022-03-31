import sys
import cv2
import os
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtWidgets import (QApplication, QHBoxLayout, QVBoxLayout)
from PyQt5.QtWidgets import QMainWindow
from Videoplayer import VideoPlayer


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
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
        self.recordingButtonState = False

        self.recordingButton.clicked.connect(self.recording)

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
            self.recordingButton.setText("Остановить запись")

            user = "admin"
            password = "admin"
            ip = "192.168.3.100:8554"
            address = "rtsp://{user}:{password}@{ip}".format(user=user, password=password, ip=ip)
            os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = "rtsp_transport;udp"
            vcap = cv2.VideoCapture(address, cv2.CAP_FFMPEG)

            if not vcap.isOpened():
                print("Error opening video file")

            frame_width = int(vcap.get(3))
            frame_height = int(vcap.get(4))
            frame_size = (frame_width, frame_height)
            fps = 25
            num = 1
            path_to_output_file = 'captured{num}.avi'.format(num=num)
            while os.path.isfile(path_to_output_file):
                num += 1
                path_to_output_file = 'captured{num}.avi'.format(num=num)

            out = cv2.VideoWriter(path_to_output_file, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'),
                                  fps, frame_size)

            while self.recordingButtonState and vcap.isOpened():
                ret, frame = vcap.read()
                if ret:
                    out.write(frame)
                    cv2.imshow("Frame", frame)
                    cv2.waitKey(fps)
                else:
                    print('Stream disconnected')
                    break
            vcap.release()
            out.release()
            cv2.destroyAllWindows()

        else:
            self.recordingButtonState = False
            self.recordingButton.setText("Начать запись")


app = QApplication(sys.argv)
mainwindow = MainWindow()
mainwindow.showMaximized()

sys.exit(app.exec_())