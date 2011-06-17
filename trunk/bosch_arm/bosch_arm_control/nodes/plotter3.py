#!/usr/bin/python
# -*- coding: utf-8 -*-
import roslib; roslib.load_manifest('bosch_arm_control')
import rospy
from sensor_msgs.msg import JointState
from bma180.msg import bma180meas
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

    #def setUpdateCallback(self, callback):
        #Function to call to update plot data before replotting
        #self.updateCurves = callback

    def timerEvent(self, e):
        lock.acquire()
        curve1.setData(time1,m1values)
        curve2.setData(time1,m2values)
        curve3.setData(time1,m3values)
        curve4.setData(time1,m4values)
        curve5.setData(time2,accX1)
        curve6.setData(time2,accY1)
        curve7.setData(time2,accZ1)
        curve8.setData(time2,accX2)
        curve9.setData(time2,accY2)
        curve10.setData(time2,accZ2)
        lock.release()
        self.replot()
        
def js_callback(data):
    pf = 1.0
    vf = 0.08
    tf = 100/0.184
    lock.acquire()
    
    m1values.append(data.position[0]*pf)
    #print data.position[0], " "
    m1values.pop(0)
    m2values.append(data.position[1]*pf)
    m2values.pop(0)
    m3values.append(data.position[2]*pf)
    m3values.pop(0)
    m4values.append(data.position[3]*pf)
    m4values.pop(0)
    lock.release()    

def acc_callback(data):
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
    rospy.Subscriber("joint_states", JointState, js_callback)
    rospy.Subscriber("bma180", bma180meas, acc_callback)


def toggle():
    if ui.m1.isChecked():curve1.setPen(QPen(Qt.red))
    else: curve1.setPen(QPen(Qt.NoPen))
    if ui.m2.isChecked():curve2.setPen(QPen(Qt.green))
    else: curve2.setPen(QPen(Qt.NoPen))
    if ui.m3.isChecked():curve3.setPen(QPen(Qt.blue))
    else: curve3.setPen(QPen(Qt.NoPen))
    if ui.m4.isChecked():curve4.setPen(QPen(Qt.gray))
    else: curve4.setPen(QPen(Qt.NoPen))
    if ui.accX1.isChecked():curve5.setPen(QPen(Qt.black))
    else: curve5.setPen(QPen(Qt.NoPen))
    if ui.accY1.isChecked():curve6.setPen(QPen(Qt.black))
    else: curve6.setPen(QPen(Qt.NoPen))
    if ui.accZ1.isChecked():curve7.setPen(QPen(Qt.black))
    else: curve7.setPen(QPen(Qt.NoPen))
    if ui.accX2.isChecked():curve8.setPen(QPen(Qt.black))
    else: curve8.setPen(QPen(Qt.NoPen))
    if ui.accY2.isChecked():curve9.setPen(QPen(Qt.darkRed))
    else: curve9.setPen(QPen(Qt.NoPen))
    if ui.accZ2.isChecked():curve10.setPen(QPen(Qt.darkGreen))
    else: curve10.setPen(QPen(Qt.NoPen))

def toggleplot():
    if plot.active == True:
        plot.stop()
        ui.start_pause.setText("Start")
    else:
        plot.start()
        ui.start_pause.setText("Pause")

if __name__=="__main__": 
    reduction=10
    length = 3000
    length2= length/reduction
    lock = threading.Lock()

    m1values = [0 for i in range(length)]
    m2values = [0 for i in range(length)]
    m3values = [0 for i in range(length)]
    m4values = [0 for i in range(length)]

    
    accX1 = [0 for i in range(length2)]
    accY1 = [0 for i in range(length2)]
    accZ1 = [0 for i in range(length2)]
    
    accX2 = [0 for i in range(length2)]
    accY2 = [0 for i in range(length2)]
    accZ2 = [0 for i in range(length2)]
    
    time2 = [i for i in range(1,length,reduction)]
    time1 = [i for i in range(length)]

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

    curve5 = QwtPlotCurve("acc X1")
    curve5.attach(plot)
    curve5.setPen(QPen(Qt.NoPen))

    curve6 = QwtPlotCurve("acc Y1")
    curve6.attach(plot)
    curve6.setPen(QPen(Qt.NoPen))

    curve7 = QwtPlotCurve("acc Z1")
    curve7.attach(plot)
    curve7.setPen(QPen(Qt.NoPen))


    curve8 = QwtPlotCurve("acc X2")
    curve8.attach(plot)
    curve8.setPen(QPen(Qt.NoPen))

    curve9 = QwtPlotCurve("acc Y2")
    curve9.attach(plot)
    curve9.setPen(QPen(Qt.NoPen))

    curve10 = QwtPlotCurve("acc Z2")
    curve10.attach(plot)
    curve10.setPen(QPen(Qt.NoPen))



    #mY = QwtPlotMarker()
    #mY.setLabelAlignment(Qt.AlignRight | Qt.AlignTop)
    #mY.setLineStyle(QwtPlotMarker.HLine)
    #mY.setYValue(0.0)
    #mY.attach(plot)

    #plot.setAxisTitle(QwtPlot.xBottom, "Time (s)")
    #plot.setAxisTitle(QwtPlot.yLeft, "Force (g)")

    #thread.start_new_thread(listen, ())
    listen()
    
    plot.setAxisScale(QwtPlot.yLeft, -125, 125)
    plot.start()

    window.connect(ui.start_pause, SIGNAL("released()"), toggleplot)
    window.connect(ui.m1, SIGNAL("released()"), toggle)
    window.connect(ui.m2, SIGNAL("released()"), toggle)
    window.connect(ui.m3, SIGNAL("released()"), toggle)
    window.connect(ui.m4, SIGNAL("released()"), toggle)
    window.connect(ui.accX1, SIGNAL("released()"), toggle)
    window.connect(ui.accY1, SIGNAL("released()"), toggle)
    window.connect(ui.accZ1, SIGNAL("released()"), toggle)
    window.connect(ui.accX2, SIGNAL("released()"), toggle)
    window.connect(ui.accY2, SIGNAL("released()"), toggle)
    window.connect(ui.accZ2, SIGNAL("released()"), toggle)

    window.show()

    sys.exit(app.exec_())
    

