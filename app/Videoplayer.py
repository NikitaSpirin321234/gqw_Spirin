from PyQt5.QtCore import QDir, Qt, QUrl
from PyQt5.QtMultimedia import QMediaContent, QMediaPlayer
from PyQt5.QtMultimediaWidgets import QVideoWidget
from PyQt5.QtWidgets import (QFileDialog, QHBoxLayout, QLabel, QSizePolicy, QSlider, QStyle, QVBoxLayout)
from PyQt5.QtWidgets import QWidget, QPushButton


class VideoPlayer(QWidget):
    def __init__(self):
        super().__init__()
        self.mediaPlayer = QMediaPlayer(None, QMediaPlayer.VideoSurface)
        video_widget = QVideoWidget()

        self.playButton = QPushButton()
        self.playButton.setEnabled(False)
        self.playButton.setIcon(self.style().standardIcon(QStyle.SP_MediaPlay))
        self.playButton.clicked.connect(self.play)

        self.positionSlider = QSlider(Qt.Horizontal)
        self.positionSlider.setRange(0, 0)
        self.positionSlider.sliderMoved.connect(self.setPosition)

        self.error = QLabel()
        self.error.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Maximum)

        open_button = QPushButton("Открыть видеофайл")
        open_button.setFixedHeight(28)
        open_button.clicked.connect(self.openFile)

        # Create layouts to place inside widget
        control_layout = QHBoxLayout()
        control_layout.setContentsMargins(0, 0, 0, 0)
        control_layout.addWidget(self.playButton)
        control_layout.addWidget(self.positionSlider)

        open_button_layout = QHBoxLayout()
        open_button_layout.setContentsMargins(220, 0, 220, 0)
        open_button_layout.addWidget(open_button)

        layout = QVBoxLayout()
        layout.addWidget(video_widget)
        layout.addLayout(control_layout)
        layout.addWidget(self.error)
        layout.addLayout(open_button_layout)

        # # Set widget to contain window contents
        self.setLayout(layout)

        self.mediaPlayer.setVideoOutput(video_widget)
        self.mediaPlayer.stateChanged.connect(self.mediaStateChanged)
        self.mediaPlayer.positionChanged.connect(self.positionChanged)
        self.mediaPlayer.durationChanged.connect(self.durationChanged)
        self.mediaPlayer.error.connect(self.handleError)

    def openFile(self):
        fileName, _ = QFileDialog.getOpenFileName(self, "Открыть видеофайл",
                                                  "{current_path}/../videofiles".format(current_path=QDir.currentPath()))

        if fileName != '':
            self.error.clear()
            self.mediaPlayer.setMedia(
                QMediaContent(QUrl.fromLocalFile(fileName)))
            self.playButton.setEnabled(True)

    def play(self):
        if self.mediaPlayer.state() == QMediaPlayer.PlayingState:
            self.mediaPlayer.pause()
        else:
            self.mediaPlayer.play()

    def mediaStateChanged(self, state):
        if self.mediaPlayer.state() == QMediaPlayer.PlayingState:
            self.playButton.setIcon(
                self.style().standardIcon(QStyle.SP_MediaPause))
        else:
            self.playButton.setIcon(
                self.style().standardIcon(QStyle.SP_MediaPlay))

    def positionChanged(self, position):
        self.positionSlider.setValue(position)

    def durationChanged(self, duration):
        self.positionSlider.setRange(0, duration)

    def setPosition(self, position):
        self.mediaPlayer.setPosition(position)

    def handleError(self):
        self.playButton.setEnabled(False)
        self.error.setText("Ошибка: " + self.mediaPlayer.errorString())
        print(self.mediaPlayer.errorString())
