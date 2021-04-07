import sys
from rviz import bindings as rviz
import rospy
import os
import pyqtgraph as pg
import time
import numpy as np
# import qdarkstyle
import serial
import threading

from pyqtgraph import PlotWidget, plot
from std_msgs.msg import Float32
from PyQt5 import QtCore
from PyQt5 import QtGui
from PyQt5 import QtWidgets

from radial_bar import RadialBar

# js = serial.Serial('/dev/ttyCOSO0', 115200)

current_buf = []
for _ in range(200):
    current_buf.append(0)

time_buf = []
for i in range(200):
    time_buf.append(i)

buf = [current_buf, time_buf]
buf_x1 = []
buf_y1 = []
buf_x2 = []
buf_y2 = []

"""def joystick_listener(viz):
    while 1:
        temp = js.read(2)
        if(temp[0] == 0x55 and temp[1] == 0x55):
            js.read(6)
            while 1:
                while(js.in_waiting >= 8):
                    bytes = js.read(8)
                    if(bytes[0] != 0x55 or bytes[1] != 0x55):
                        break
                    angle1 = fmap(bytes[2],0,255,0,360)
                    angle2 = fmap(bytes[3],0,255,0,360)
                    speed1 = fmap(bytes[4],0,100,0,1)
                    speed2 = fmap(bytes[5],0,100,0,1)
                    #button1 = bytes[6]
                    #button2 = bytes[7]
                    x1 = np.sin(np.radians(angle1))*speed1
                    y1 = np.cos(np.radians(angle1))*speed1
                    x2 = np.sin(np.radians(angle2))*speed2
                    y2 = np.cos(np.radians(angle2))*speed2

                    viz.draw_joystick(x1, x2, y1, y2)
                time.sleep(0.01)"""


def fmap(x, in_min, in_max, out_min, out_max):  # simple linear interpolation function
    return (float)(x - in_min) * (out_max - out_min) / (float)(in_max - in_min) + out_min


def listener():
    rospy.init_node('nightcontrol_listener')
    rospy.Subscriber("/voltage", Float32, callbackV)
    rospy.Subscriber("/current", Float32, callbackC)


def callbackV(data):
    r = rospy.Rate(100)
    voltage = round(data.data, 2)
    myviz.batteryVWidget.value = voltage
    r.sleep()


def callbackC(data):
    r = rospy.Rate(50)
    global buf
    current = round(data.data, 2)
    buf[0].append(current)
    r.sleep()


class MyViz(QtWidgets.QWidget):
    def __init__(self):
        QtWidgets.QWidget.__init__(self)

        # rviz widget
        self.frame = rviz.VisualizationFrame()
        self.frame.setSplashPath("")
        self.frame.initialize()
        reader = rviz.YamlConfigReader()
        config = rviz.Config()
        path = os.path.dirname(os.path.realpath(__file__))
        reader.readFile(config, path + "/nightmare_config.rviz")
        self.frame.load(config)
        self.setWindowTitle(config.mapGetChild("Title").getValue())
        self.frame.setMenuBar(None)
        self.frame.setStatusBar(None)
        self.frame.setHideButtonVisibility(False)
        self.manager = self.frame.getManager()
        self.grid_display = self.manager.getRootDisplayGroup().getDisplayAt(0)

        # voltage gauge widget
        self.batteryVWidget = RadialBar()
        self.batteryVWidget.width = 250
        self.batteryVWidget.height = 250
        self.batteryVWidget.progressColor = QtGui.QColor(0, 255, 193)
        self.batteryVWidget.foregroundColor = QtGui.QColor(15, 15, 15)
        self.batteryVWidget.dialWidth = 15
        self.batteryVWidget.suffixText = "V"
        self.batteryVWidget.textFont = QtGui.QFont("Halvetica", 12)
        self.batteryVWidget.textColor = QtGui.QColor(0, 255, 193)
        self.batteryVWidget.setFixedSize(250, 250)

        # graph textbox
        self.textbox = QtWidgets.QLabel()
        self.textbox.setFixedSize(80, 40)
        self.textbox.text = "0.0A"
        self.textbox.setStyleSheet('QLabel#textbox {color: (0,255,193)}')

        # graph widget
        self.graph = pg.PlotWidget()
        self.graph.setBackground(None)
        self.graph.setYRange(0, 10, 0)
        self.graph.setFixedSize(400, 300)
        self.graph.setMouseEnabled(False, False)
        self.graph.setTitle("battery current", color=(0, 255, 193), size='20')
        self.graph.setContentsMargins(0, 10, 30, 10)

        layout = QtWidgets.QHBoxLayout()

        vlayoutL1 = QtWidgets.QVBoxLayout()
        vlayoutL1.addWidget(self.batteryVWidget, 0, QtCore.Qt.AlignHCenter)
        vlayoutL1.addWidget(self.graph)
        vlayoutL1.setAlignment(QtCore.Qt.AlignTop)

        vlayoutL2 = QtWidgets.QVBoxLayout()
        vlayoutL2.setAlignment(QtCore.Qt.AlignBottom)

        vlayoutL = QtWidgets.QVBoxLayout()
        vlayoutL.addLayout(vlayoutL1)
        vlayoutL.addLayout(vlayoutL2)
        vlayoutL.setAlignment(QtCore.Qt.AlignBottom)

        vlayoutR = QtWidgets.QVBoxLayout()
        vlayoutR.setAlignment(QtCore.Qt.AlignBottom)

        layout.addLayout(vlayoutL)
        layout.addWidget(self.frame)
        layout.addLayout(vlayoutR)

        self.setLayout(layout)

    def onTopButtonClick(self):
        print("lol")

    def grab_data(self, buf):
        pen = pg.mkPen(color=(0, 255, 193), width=4)
        self.graph.plot(buf[1], buf[0][-200:], clear=True, pen=pen)


if __name__ == '__main__':
    app = QtWidgets.QApplication(sys.argv)
    # app.setStyleSheet(qdarkstyle.load_stylesheet())

    myviz = MyViz()
    myviz.resize(1920, 1067)
    myviz.show()

    # thread = threading.Thread(target=joystick_listener, args=[myviz])
    # thread.start()

    timer = QtCore.QTimer()
    timer.timeout.connect(lambda: myviz.grab_data(buf))
    timer.start(10)

    listener()
    sys.exit(app.exec_())
    rospy.spin()
