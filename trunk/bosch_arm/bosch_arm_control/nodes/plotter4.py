#!/usr/bin/python
# -*- coding: utf-8 -*-
import roslib; roslib.load_manifest('bosch_arm_control')
import rospy
from sensor_msgs.msg import JointState
from bma180.msg import bma180meas
from bosch_arm_control.msg import Diagnostic
import sys
from PyQt4.QtCore import *
from PyQt4.QtGui import *
from PyQt4.Qwt5 import *
#import numpy
import socket

import thread
import threading
import time

import loggerUi
#from PyQt4.Qwt5.anynumpy import *

class DynamicPlot(QwtPlot):
    def __init__(self, *args):
        QwtPlot.__init__(self, *args)
        self.time = []
        self.curves = {}
        self.tid = 0
        self.active = False
        self.timestep = 20 #50 Hz
        self.updateCurves = None

    def start(self):
        if self.active == False:
          self.tid = self.startTimer(self.timestep)
          self.active = True

    def stop(self):
        if self.active == True:
            self.killTimer(self.tid)
            self.active = False

    def setUpdateCallback(self, callback):
        #Function to call to update plot data before replotting
        self.updateCurves = callback

    def timerEvent(self, e):
        lock.acquire()
        curve1.setData(time,m1values)
        curve2.setData(time,m2values)
        curve3.setData(time,m3values)
        curve4.setData(time,m4values)
        curve5.setData(time,m1command)
        curve6.setData(time,m2command)
        curve7.setData(time,m3command)
        curve8.setData(time,m4command)
        curve9.setData(time,v1)
        curve10.setData(time,v2)
        curve11.setData(time,v3)
        curve12.setData(time,v4)
        curve13.setData(time,t1)
        curve14.setData(time,t2)
        curve15.setData(time,t3)
        curve16.setData(time,t4)
        lock.release()
        self.replot()

def diag_listener(msg):
    da = msg.data.split(',')
    data = [float(i) for i in da]
    valout = data[0]
    #valout = int(data)
    #print valout
    #print msg.data

    pf = 0.5
    vf = 0.08
    tf = 100/0.184

    lock.acquire()
    m1values.append(data[0]*pf)
    m1values.pop(0)
    m2values.append(data[1]*pf)
    m2values.pop(0)
    m3values.append(data[2]*pf)
    m3values.pop(0)
    m4values.append(data[3]*pf)
    m4values.pop(0)
    m1command.append(data[4]*pf)
    m1command.pop(0)
    m2command.append(data[5]*pf)
    m2command.pop(0)
    m3command.append(data[6]*pf)
    m3command.pop(0)
    m4command.append(data[7]*pf)
    m4command.pop(0)
    v1.append(data[8]*vf)
    v1.pop(0)
    v2.append(data[9]*vf)
    v2.pop(0)
    v3.append(data[10]*vf)
    v3.pop(0)
    v4.append(data[11]*vf)
    v4.pop(0)
    t1.append(data[12]*tf)
    t1.pop(0)
    t2.append(data[13]*tf)
    t2.pop(0)
    t3.append(data[14]*tf)
    t3.pop(0)
    t4.append(data[15]*tf)
    t4.pop(0)
    lock.release()

        #print data
def acc_listener(data):
    af = 10
    lock.acquire()
    if data.iChipSelect==0:
        accX1.append(data.fAcclX*af)
        accX1.pop(0)
        accY1.append(data.fAcclY*af)
        accY1.pop(0)
        accZ1.append(data.fAcclZ*af)
        accZ1.pop(0)
    else:
        accX2.append(data.fAcclX*af)
        accX2.pop(0)
        accY2.append(data.fAcclY*af)
        accY2.pop(0)
        accZ2.append(data.fAcclZ*af)
        accZ2.pop(0)        
    lock.release()    

def listen():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/diagnostics", Diagnostic, diag_listener)
    #rospy.Subscriber("/bma180", bma180meas, acc_listener)

def toggle():
    if ui.m1.isChecked():curve1.setPen(QPen(Qt.red))
    else: curve1.setPen(QPen(Qt.NoPen))
    if ui.m2.isChecked():curve2.setPen(QPen(Qt.green))
    else: curve2.setPen(QPen(Qt.NoPen))
    if ui.m3.isChecked():curve3.setPen(QPen(Qt.blue))
    else: curve3.setPen(QPen(Qt.NoPen))
    if ui.m4.isChecked():curve4.setPen(QPen(Qt.gray))
    else: curve4.setPen(QPen(Qt.NoPen))
    if ui.m1c.isChecked():curve5.setPen(QPen(Qt.black))
    else: curve5.setPen(QPen(Qt.NoPen))
    if ui.m2c.isChecked():curve6.setPen(QPen(Qt.black))
    else: curve6.setPen(QPen(Qt.NoPen))
    if ui.m3c.isChecked():curve7.setPen(QPen(Qt.black))
    else: curve7.setPen(QPen(Qt.NoPen))
    if ui.m4c.isChecked():curve8.setPen(QPen(Qt.black))
    else: curve8.setPen(QPen(Qt.NoPen))
    if ui.v1.isChecked():curve9.setPen(QPen(Qt.darkRed))
    else: curve9.setPen(QPen(Qt.NoPen))
    if ui.v2.isChecked():curve10.setPen(QPen(Qt.darkGreen))
    else: curve10.setPen(QPen(Qt.NoPen))
    if ui.v3.isChecked():curve11.setPen(QPen(Qt.darkBlue))
    else: curve11.setPen(QPen(Qt.NoPen))
    if ui.v4.isChecked():curve12.setPen(QPen(Qt.cyan))
    else: curve12.setPen(QPen(Qt.NoPen))
    if ui.t1.isChecked():curve13.setPen(QPen(Qt.magenta))
    else: curve13.setPen(QPen(Qt.NoPen))
    if ui.t2.isChecked():curve14.setPen(QPen(Qt.yellow))
    else: curve14.setPen(QPen(Qt.NoPen))
    if ui.t3.isChecked():curve15.setPen(QPen(Qt.darkCyan))
    else: curve15.setPen(QPen(Qt.NoPen))
    if ui.t4.isChecked():curve16.setPen(QPen(Qt.darkMagenta))
    else: curve16.setPen(QPen(Qt.NoPen))


def toggleplot():
    if plot.active == True:
        plot.stop()
        ui.start_pause.setText("Start")
    else:
        plot.start()
        ui.start_pause.setText("Pause")

if __name__=="__main__":    
    length = 3000
    lock = threading.Lock()

    m1values = [0 for i in range(length)]
    m2values = [0 for i in range(length)]
    m3values = [0 for i in range(length)]
    m4values = [0 for i in range(length)]

    m1command = [0 for i in range(length)]
    m2command = [0 for i in range(length)]
    m3command = [0 for i in range(length)]
    m4command = [0 for i in range(length)]

    v1 = [0 for i in range(length)]
    v2 = [0 for i in range(length)]
    v3 = [0 for i in range(length)]
    v4 = [0 for i in range(length)]

    t1 = [0 for i in range(length)]
    t2 = [0 for i in range(length)]
    t3 = [0 for i in range(length)]
    t4 = [0 for i in range(length)]

    time = [i for i in range(length)]

    app = QApplication(sys.argv)
    window = QMainWindow()
    ui = loggerUi.Ui_MainWindow()
    ui.setupUi(window)

    ui.qwtPlot = DynamicPlot(ui.centralwidget)
    plot = ui.qwtPlot
    plot.setObjectName("qwtPlot")
    ui.gridLayout.addWidget(plot, 0, 0, 1, 1)

    plot.setCanvasBackground(Qt.white)
    plot.setTitle("Datastream")
    plot.insertLegend(QwtLegend(), QwtPlot.BottomLegend);

    curve1 = QwtPlotCurve("M1 position")
    curve1.attach(plot)
    curve1.setPen(QPen(Qt.NoPen))

    curve2 = QwtPlotCurve("M2 position")
    curve2.attach(plot)
    curve2.setPen(QPen(Qt.NoPen))

    curve3 = QwtPlotCurve("M3 position")
    curve3.attach(plot)
    curve3.setPen(QPen(Qt.NoPen))

    curve4 = QwtPlotCurve("M4 position")
    curve4.attach(plot)
    curve4.setPen(QPen(Qt.NoPen))

    curve5 = QwtPlotCurve("M1 command")
    curve5.attach(plot)
    curve5.setPen(QPen(Qt.NoPen))

    curve6 = QwtPlotCurve("M2 command")
    curve6.attach(plot)
    curve6.setPen(QPen(Qt.NoPen))

    curve7 = QwtPlotCurve("M3 command")
    curve7.attach(plot)
    curve7.setPen(QPen(Qt.NoPen))

    curve8 = QwtPlotCurve("M4 command")
    curve8.attach(plot)
    curve8.setPen(QPen(Qt.NoPen))

    curve9 = QwtPlotCurve("V1")
    curve9.attach(plot)
    curve9.setPen(QPen(Qt.NoPen))

    curve10 = QwtPlotCurve("V2")
    curve10.attach(plot)
    curve10.setPen(QPen(Qt.NoPen))

    curve11 = QwtPlotCurve("V3")
    curve11.attach(plot)
    curve11.setPen(QPen(Qt.NoPen))

    curve12 = QwtPlotCurve("V4")
    curve12.attach(plot)
    curve12.setPen(QPen(Qt.NoPen))

    curve13 = QwtPlotCurve("T1")
    curve13.attach(plot)
    curve13.setPen(QPen(Qt.NoPen))

    curve14 = QwtPlotCurve("T2")
    curve14.attach(plot)
    curve14.setPen(QPen(Qt.NoPen))

    curve15 = QwtPlotCurve("T3")
    curve15.attach(plot)
    curve15.setPen(QPen(Qt.NoPen))

    curve16 = QwtPlotCurve("T4")
    curve16.attach(plot)
    curve16.setPen(QPen(Qt.NoPen))

    mY = QwtPlotMarker()
    mY.setLabelAlignment(Qt.AlignRight | Qt.AlignTop)
    mY.setLineStyle(QwtPlotMarker.HLine)
    mY.setYValue(0.0)
    mY.attach(plot)

    #plot.setAxisTitle(QwtPlot.xBottom, "Time (s)")
    #plot.setAxisTitle(QwtPlot.yLeft, "Force (g)")

    listen()

    plot.setAxisScale(QwtPlot.yLeft, -125, 125)
    plot.start()

    window.connect(ui.start_pause, SIGNAL("released()"), toggleplot)
    window.connect(ui.m1, SIGNAL("released()"), toggle)
    window.connect(ui.m2, SIGNAL("released()"), toggle)
    window.connect(ui.m3, SIGNAL("released()"), toggle)
    window.connect(ui.m4, SIGNAL("released()"), toggle)
    window.connect(ui.m1c, SIGNAL("released()"), toggle)
    window.connect(ui.m2c, SIGNAL("released()"), toggle)
    window.connect(ui.m3c, SIGNAL("released()"), toggle)
    window.connect(ui.m4c, SIGNAL("released()"), toggle)
    window.connect(ui.v1, SIGNAL("released()"), toggle)
    window.connect(ui.v2, SIGNAL("released()"), toggle)
    window.connect(ui.v3, SIGNAL("released()"), toggle)
    window.connect(ui.v4, SIGNAL("released()"), toggle)
    window.connect(ui.t1, SIGNAL("released()"), toggle)
    window.connect(ui.t2, SIGNAL("released()"), toggle)
    window.connect(ui.t3, SIGNAL("released()"), toggle)
    window.connect(ui.t4, SIGNAL("released()"), toggle)

    window.show()

    sys.exit(app.exec_())
    

