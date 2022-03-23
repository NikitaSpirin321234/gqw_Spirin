import sys
import cv2
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
        if self.recordingButtonState == False:
            self.recordingButtonState = True
            self.recordingButton.setText("Остановить запись")

            cap = cv2.VideoCapture(0)
            codec = cv2.VideoWriter_fourcc(*'XVID')
            out = cv2.VideoWriter('captured.avi', codec, 25.0, (640, 480))
            while self.recordingButtonState == True:
                ret, frame = cap.read()
                if cv2.waitKey(1) & 0xFF == ('q') or ret == False:
                    break
                cv2.imshow('frame', frame)
                out.write(frame)
            out.release()
            cap.release()
            cv2.destroyAllWindows()

        else:
            self.recordingButtonState = False
            self.recordingButton.setText("Начать запись")


app = QApplication(sys.argv)
mainwindow = MainWindow()
mainwindow.showMaximized()

sys.exit(app.exec_())