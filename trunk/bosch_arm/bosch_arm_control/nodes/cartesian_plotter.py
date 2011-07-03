#!/usr/bin/python
# -*- coding: utf-8 -*-
import roslib; roslib.load_manifest('bosch_arm_control')
import rospy
from bosch_arm_control.msg import TipState
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

import loggerUi2
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
        curve_m1.setData(time,m1)
        curve_m2.setData(time,m2)
        curve_m3.setData(time,m3)
        curve_m4.setData(time,m4)
        curve_vm1.setData(time,vm1)
        curve_vm2.setData(time,vm2)
        curve_vm3.setData(time,vm3)
        curve_vm4.setData(time,vm4)
        #curve_cm1.setData(time,cm1)
        #curve_cm2.setData(time,cm2)
        #curve_cm3.setData(time,cm3)
        #curve_cm4.setData(time,cm4)
        curve_tm1.setData(time,tm1)
        curve_tm2.setData(time,tm2)
        curve_tm3.setData(time,tm3)
        curve_tm4.setData(time,tm4)
        #curve_em1.setData(time,em1)
        #curve_em2.setData(time,em2)
        #curve_em3.setData(time,em3)
        #curve_em4.setData(time,em4)
        
        curve_q1.setData(time,q1)
        curve_q2.setData(time,q2)
        curve_q3.setData(time,q3)
        curve_q4.setData(time,q4)
        #curve_vq1.setData(time,vq1)
        #curve_vq2.setData(time,vq2)
        #curve_vq3.setData(time,vq3)
        #curve_vq4.setData(time,vq4)
        #curve_cq1.setData(time,cq1)
        #curve_cq2.setData(time,cq2)
        #curve_cq3.setData(time,cq3)
        #curve_cq4.setData(time,cq4)
        curve_tq1.setData(time,tq1)
        curve_tq2.setData(time,tq2)
        curve_tq3.setData(time,tq3)
        curve_tq4.setData(time,tq4)
        #curve_eq1.setData(time,eq1)
        #curve_eq2.setData(time,eq2)
        #curve_eq3.setData(time,eq3)
        #curve_eq4.setData(time,eq4)
        
        curve_x1.setData(time,x1)
        curve_x2.setData(time,x2)
        curve_x3.setData(time,x3)
        curve_vx1.setData(time,vx1)
        curve_vx2.setData(time,vx2)
        curve_vx3.setData(time,vx3)
        curve_cx1.setData(time,cx1)
        curve_cx2.setData(time,cx2)
        curve_cx3.setData(time,cx3)
        curve_tx1.setData(time,tx1)
        curve_tx2.setData(time,tx2)
        curve_tx3.setData(time,tx3)
        curve_ex1.setData(time,ex1)
        curve_ex2.setData(time,ex2)
        curve_ex3.setData(time,ex3)
        
        curve_accX1.setData(time2,accX1)
        curve_accY1.setData(time2,accY1)
        curve_accZ1.setData(time2,accZ1)
        curve_accX2.setData(time2,accX2)
        curve_accY2.setData(time2,accY2)
        curve_accZ2.setData(time2,accZ2)
        lock.release()
        self.replot()

def diag_listener(msg):
    da = msg.data.split(',')
    data = [float(i) for i in da]
    valout = data[0]
    #valout = int(data)
    #print valout
    #print msg.data

    

    lock.acquire()
    m1.append(data[23]*m_scale)
    m1.pop(0)
    m2.append(data[24]*m_scale)
    m2.pop(0)
    m3.append(data[25]*m_scale)
    m3.pop(0)
    m4.append(data[26]*m_scale)
    m4.pop(0)
    #cm1.append(data[]*cm_scale)
    #cm1.pop(0)
    #cm2.append(data[]*cm_scale)
    #cm2.pop(0)
    #cm3.append(data[]*cm_scale)
    #cm3.pop(0)
    #cm4.append(data[]*cm_scale)
    #cm4.pop(0)
    vm1.append(data[27]*vm_scale)
    vm1.pop(0)
    vm2.append(data[28]*vm_scale)
    vm2.pop(0)
    vm3.append(data[29]*vm_scale)
    vm3.pop(0)
    vm4.append(data[30]*vm_scale)
    vm4.pop(0)
    tm1.append(data[31]*tm_scale)
    tm1.pop(0)
    tm2.append(data[32]*tm_scale)
    tm2.pop(0)
    tm3.append(data[33]*tm_scale)
    tm3.pop(0)
    tm4.append(data[34]*tm_scale)
    tm4.pop(0)
    #em1.append(data[]*em_scale)
    #em1.pop(0)
    #em2.append(data[]*em_scale)
    #em2.pop(0)
    #em3.append(data[]*em_scale)
    #em3.pop(0)
    #em4.append(data[]*em_scale)
    #em4.pop(0)

    q1.append(data[15]*q_scale)
    q1.pop(0)
    q2.append(data[16]*q_scale)
    q2.pop(0)
    q3.append(data[17]*q_scale)
    q3.pop(0)
    q4.append(data[18]*q_scale)
    q4.pop(0)
    #cq1.append(data[]*cq_scale)
    #cq1.pop(0)
    #cq2.append(data[]*cq_scale)
    #cq2.pop(0)
    #cq3.append(data[]*cq_scale)
    #cq3.pop(0)
    #cq4.append(data[]*cq_scale)
    #cq4.pop(0)
    #vq1.append(data[]*vq_scale)
    #vq1.pop(0)
    #vq2.append(data[]*vq_scale)
    #vq2.pop(0)
    #vq3.append(data[]*vq_scale)
    #vq3.pop(0)
    #vq4.append(data[]*vq_scale)
    #vq4.pop(0)
    tq1.append(data[19]*tq_scale)
    tq1.pop(0)
    tq2.append(data[20]*tq_scale)
    tq2.pop(0)
    tq3.append(data[21]*tq_scale)
    tq3.pop(0)
    tq4.append(data[22]*tq_scale)
    tq4.pop(0)
    #eq1.append(data[]*eq_scale)
    #eq1.pop(0)
    #eq2.append(data[]*eq_scale)
    #eq2.pop(0)
    #eq3.append(data[]*eq_scale)
    #eq3.pop(0)
    #eq4.append(data[]*eq_scale)
    #eq4.pop(0)
    
    x1.append(data[0]*x_scale)
    x1.pop(0)
    x2.append(data[1]*x_scale)
    x2.pop(0)
    x3.append(data[2]*x_scale)
    x3.pop(0)
    cx1.append(data[3]*cx_scale)
    cx1.pop(0)
    cx2.append(data[4]*cx_scale)
    cx2.pop(0)
    cx3.append(data[5]*cx_scale)
    cx3.pop(0)
    vx1.append(data[6]*vx_scale)
    vx1.pop(0)
    vx2.append(data[7]*vx_scale)
    vx2.pop(0)
    vx3.append(data[8]*vx_scale)
    vx3.pop(0)
    tx1.append(data[9]*tx_scale)
    tx1.pop(0)
    tx2.append(data[10]*tx_scale)
    tx2.pop(0)
    tx3.append(data[11]*tx_scale)
    tx3.pop(0)
    ex1.append(data[12]*ex_scale)
    ex1.pop(0)
    ex2.append(data[13]*ex_scale)
    ex2.pop(0)
    ex3.append(data[14]*ex_scale)
    ex3.pop(0)


    lock.release()

        #print data
def acc_listener(data):
    af = 100
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
    rospy.Subscriber("/bma180", bma180meas, acc_listener)

def toggle():
    if ui.m1.isChecked():curve_m1.setPen(QPen(Qt.red))
    else: curve_m1.setPen(QPen(Qt.NoPen))
    if ui.m2.isChecked():curve_m2.setPen(QPen(Qt.green))
    else: curve_m2.setPen(QPen(Qt.NoPen))
    if ui.m3.isChecked():curve_m3.setPen(QPen(Qt.blue))
    else: curve_m3.setPen(QPen(Qt.NoPen))
    if ui.m4.isChecked():curve_m4.setPen(QPen(Qt.gray))
    else: curve_m4.setPen(QPen(Qt.NoPen))
    if ui.cm1.isChecked():curve_cm1.setPen(QPen(Qt.darkRed))
    else: curve_cm1.setPen(QPen(Qt.NoPen))
    if ui.cm2.isChecked():curve_cm2.setPen(QPen(Qt.darkGreen))
    else: curve_cm2.setPen(QPen(Qt.NoPen))
    if ui.cm3.isChecked():curve_cm3.setPen(QPen(Qt.darkBlue))
    else: curve_cm3.setPen(QPen(Qt.NoPen))
    if ui.cm4.isChecked():curve_cm4.setPen(QPen(Qt.black))
    else: curve_cm4.setPen(QPen(Qt.NoPen))
    if ui.vm1.isChecked():curve_vm1.setPen(QPen(Qt.red))
    else: curve_vm1.setPen(QPen(Qt.NoPen))
    if ui.vm2.isChecked():curve_vm2.setPen(QPen(Qt.green))
    else: curve_vm2.setPen(QPen(Qt.NoPen))
    if ui.vm3.isChecked():curve_vm3.setPen(QPen(Qt.blue))
    else: curve_vm3.setPen(QPen(Qt.NoPen))
    if ui.vm4.isChecked():curve_vm4.setPen(QPen(Qt.gray))
    else: curve_vm4.setPen(QPen(Qt.NoPen))
    if ui.tm1.isChecked():curve_tm1.setPen(QPen(Qt.yellow))
    else: curve_tm1.setPen(QPen(Qt.NoPen))
    if ui.tm2.isChecked():curve_tm2.setPen(QPen(Qt.cyan))
    else: curve_tm2.setPen(QPen(Qt.NoPen))
    if ui.tm3.isChecked():curve_tm3.setPen(QPen(Qt.magenta))
    else: curve_tm3.setPen(QPen(Qt.NoPen))
    if ui.tm4.isChecked():curve_tm4.setPen(QPen(Qt.gray))
    else: curve_tm4.setPen(QPen(Qt.NoPen))
    if ui.em1.isChecked():curve_em1.setPen(QPen(Qt.darkYellow))
    else: curve_em1.setPen(QPen(Qt.NoPen))
    if ui.em2.isChecked():curve_em2.setPen(QPen(Qt.darkCyan))
    else: curve_em2.setPen(QPen(Qt.NoPen))
    if ui.em3.isChecked():curve_em3.setPen(QPen(Qt.darkMagenta))
    else: curve_em3.setPen(QPen(Qt.NoPen))
    if ui.em4.isChecked():curve_em4.setPen(QPen(Qt.black))
    else: curve_em4.setPen(QPen(Qt.NoPen))
    
    if ui.q1.isChecked():curve_q1.setPen(QPen(Qt.red))
    else: curve_q1.setPen(QPen(Qt.NoPen))
    if ui.q2.isChecked():curve_q2.setPen(QPen(Qt.green))
    else: curve_q2.setPen(QPen(Qt.NoPen))
    if ui.q3.isChecked():curve_q3.setPen(QPen(Qt.blue))
    else: curve_q3.setPen(QPen(Qt.NoPen))
    if ui.q4.isChecked():curve_q4.setPen(QPen(Qt.gray))
    else: curve_q4.setPen(QPen(Qt.NoPen))
    if ui.cq1.isChecked():curve_cq1.setPen(QPen(Qt.darkRed))
    else: curve_cq1.setPen(QPen(Qt.NoPen))
    if ui.cq2.isChecked():curve_cq2.setPen(QPen(Qt.darkGreen))
    else: curve_cq2.setPen(QPen(Qt.NoPen))
    if ui.cq3.isChecked():curve_cq3.setPen(QPen(Qt.darkBlue))
    else: curve_cq3.setPen(QPen(Qt.NoPen))
    if ui.cq4.isChecked():curve_cq4.setPen(QPen(Qt.black))
    else: curve_cq4.setPen(QPen(Qt.NoPen))
    if ui.vq1.isChecked():curve_vq1.setPen(QPen(Qt.red))
    else: curve_vq1.setPen(QPen(Qt.NoPen))
    if ui.vq2.isChecked():curve_vq2.setPen(QPen(Qt.green))
    else: curve_vq2.setPen(QPen(Qt.NoPen))
    if ui.vq3.isChecked():curve_vq3.setPen(QPen(Qt.blue))
    else: curve_vq3.setPen(QPen(Qt.NoPen))
    if ui.vq4.isChecked():curve_vq4.setPen(QPen(Qt.gray))
    else: curve_vq4.setPen(QPen(Qt.NoPen))
    if ui.tq1.isChecked():curve_tq1.setPen(QPen(Qt.yellow))
    else: curve_tq1.setPen(QPen(Qt.NoPen))
    if ui.tq2.isChecked():curve_tq2.setPen(QPen(Qt.cyan))
    else: curve_tq2.setPen(QPen(Qt.NoPen))
    if ui.tq3.isChecked():curve_tq3.setPen(QPen(Qt.magenta))
    else: curve_tq3.setPen(QPen(Qt.NoPen))
    if ui.tq4.isChecked():curve_tq4.setPen(QPen(Qt.gray))
    else: curve_tq4.setPen(QPen(Qt.NoPen))
    if ui.eq1.isChecked():curve_eq1.setPen(QPen(Qt.darkYellow))
    else: curve_eq1.setPen(QPen(Qt.NoPen))
    if ui.eq2.isChecked():curve_eq2.setPen(QPen(Qt.darkCyan))
    else: curve_eq2.setPen(QPen(Qt.NoPen))
    if ui.eq3.isChecked():curve_eq3.setPen(QPen(Qt.darkMagenta))
    else: curve_eq3.setPen(QPen(Qt.NoPen))
    if ui.eq4.isChecked():curve_eq4.setPen(QPen(Qt.black))
    else: curve_eq4.setPen(QPen(Qt.NoPen))
    
    if ui.x1.isChecked():curve_x1.setPen(QPen(Qt.red))
    else: curve_x1.setPen(QPen(Qt.NoPen))
    if ui.x2.isChecked():curve_x2.setPen(QPen(Qt.green))
    else: curve_x2.setPen(QPen(Qt.NoPen))
    if ui.x3.isChecked():curve_x3.setPen(QPen(Qt.blue))
    else: curve_x3.setPen(QPen(Qt.NoPen))
    if ui.cx1.isChecked():curve_cx1.setPen(QPen(Qt.darkRed))
    else: curve_cx1.setPen(QPen(Qt.NoPen))
    if ui.cx2.isChecked():curve_cx2.setPen(QPen(Qt.darkGreen))
    else: curve_cx2.setPen(QPen(Qt.NoPen))
    if ui.cx3.isChecked():curve_cx3.setPen(QPen(Qt.darkBlue))
    else: curve_cx3.setPen(QPen(Qt.NoPen))
    if ui.vx1.isChecked():curve_vx1.setPen(QPen(Qt.red))
    else: curve_vx1.setPen(QPen(Qt.NoPen))
    if ui.vx2.isChecked():curve_vx2.setPen(QPen(Qt.green))
    else: curve_vx2.setPen(QPen(Qt.NoPen))
    if ui.vx3.isChecked():curve_vx3.setPen(QPen(Qt.blue))
    else: curve_vx3.setPen(QPen(Qt.NoPen))
    if ui.tx1.isChecked():curve_tx1.setPen(QPen(Qt.yellow))
    else: curve_tx1.setPen(QPen(Qt.NoPen))
    if ui.tx2.isChecked():curve_tx2.setPen(QPen(Qt.cyan))
    else: curve_tx2.setPen(QPen(Qt.NoPen))
    if ui.tx3.isChecked():curve_tx3.setPen(QPen(Qt.magenta))
    else: curve_tx3.setPen(QPen(Qt.NoPen))
    if ui.ex1.isChecked():curve_ex1.setPen(QPen(Qt.darkYellow))
    else: curve_ex1.setPen(QPen(Qt.NoPen))
    if ui.ex2.isChecked():curve_ex2.setPen(QPen(Qt.darkCyan))
    else: curve_ex2.setPen(QPen(Qt.NoPen))
    if ui.ex3.isChecked():curve_ex3.setPen(QPen(Qt.darkMagenta))
    else: curve_ex3.setPen(QPen(Qt.NoPen))

    
    if ui.accX1.isChecked():curve_accX1.setPen(QPen(Qt.red))
    else: curve_accX1.setPen(QPen(Qt.NoPen))
    if ui.accY1.isChecked():curve_acc.setPen(QPen(Qt.green))
    else: curve_accY1.setPen(QPen(Qt.NoPen))
    if ui.accZ1.isChecked():curve_acc.setPen(QPen(Qt.blue))
    else: curve_accZ1.setPen(QPen(Qt.NoPen))
    if ui.accX2.isChecked():curve_accX2.setPen(QPen(Qt.darkRed))
    else: curve_accX2.setPen(QPen(Qt.NoPen))
    if ui.accY2.isChecked():curve_accY2.setPen(QPen(Qt.darkGreen))
    else: curve_accY2.setPen(QPen(Qt.NoPen))
    if ui.accZ2.isChecked():curve_accZ2.setPen(QPen(Qt.darkBlue))
    else: curve_accZ2.setPen(QPen(Qt.NoPen))

def toggleplot():
    if plot.active == True:
        plot.stop()
        ui.start_pause.setText("Start")
    else:
        plot.start()
        ui.start_pause.setText("Pause")

if __name__=="__main__":    
    length = 2000
    reduction=10
    lock = threading.Lock()
    
    m_scale = 50
    cm_scale= 50
    vm_scale= 5
    tm_scale= 500
    em_scale= 100
    q_scale = 100
    cq_scale= 100
    vq_scale= 100
    tq_scale= 100
    eq_scale= 100
    x_scale = 100
    cx_scale= 100
    vx_scale= 100
    tx_scale= 100
    ex_scale= 1000

    m1 = [0 for i in range(length)]
    m2 = [0 for i in range(length)]
    m3 = [0 for i in range(length)]
    m4 = [0 for i in range(length)]
    vm1 = [0 for i in range(length)]
    vm2 = [0 for i in range(length)]
    vm3 = [0 for i in range(length)]
    vm4 = [0 for i in range(length)]
    tm1 = [0 for i in range(length)]
    tm2 = [0 for i in range(length)]
    tm3 = [0 for i in range(length)]
    tm4 = [0 for i in range(length)]
    cm1 = [0 for i in range(length)]
    cm2 = [0 for i in range(length)]
    cm3 = [0 for i in range(length)]
    cm4 = [0 for i in range(length)]
    em1 = [0 for i in range(length)]
    em2 = [0 for i in range(length)]
    em3 = [0 for i in range(length)]
    em4 = [0 for i in range(length)]
    
    q1 = [0 for i in range(length)]
    q2 = [0 for i in range(length)]
    q3 = [0 for i in range(length)]
    q4 = [0 for i in range(length)]
    vq1 = [0 for i in range(length)]
    vq2 = [0 for i in range(length)]
    vq3 = [0 for i in range(length)]
    vq4 = [0 for i in range(length)]
    tq1 = [0 for i in range(length)]
    tq2 = [0 for i in range(length)]
    tq3 = [0 for i in range(length)]
    tq4 = [0 for i in range(length)]
    cq1 = [0 for i in range(length)]
    cq2 = [0 for i in range(length)]
    cq3 = [0 for i in range(length)]
    cq4 = [0 for i in range(length)]
    eq1 = [0 for i in range(length)]
    eq2 = [0 for i in range(length)]
    eq3 = [0 for i in range(length)]
    eq4 = [0 for i in range(length)]
    
    x1 = [0 for i in range(length)]
    x2 = [0 for i in range(length)]
    x3 = [0 for i in range(length)]
    vx1 = [0 for i in range(length)]
    vx2 = [0 for i in range(length)]
    vx3 = [0 for i in range(length)]
    tx1 = [0 for i in range(length)]
    tx2 = [0 for i in range(length)]
    tx3 = [0 for i in range(length)]
    cx1 = [0 for i in range(length)]
    cx2 = [0 for i in range(length)]
    cx3 = [0 for i in range(length)]
    ex1 = [0 for i in range(length)]
    ex2 = [0 for i in range(length)]
    ex3 = [0 for i in range(length)]

    time = [i for i in range(length)]
    
    length2= length/reduction
    accX1 = [0 for i in range(length2)]
    accY1 = [0 for i in range(length2)]
    accZ1 = [0 for i in range(length2)]
    
    accX2 = [0 for i in range(length2)]
    accY2 = [0 for i in range(length2)]
    accZ2 = [0 for i in range(length2)]
    time2 = [i for i in range(1,length,reduction)]

    app = QApplication(sys.argv)
    window = QMainWindow()
    ui = loggerUi2.Ui_MainWindow()
    ui.setupUi(window)

    ui.qwtPlot = DynamicPlot(ui.centralwidget)
    plot = ui.qwtPlot
    plot.setObjectName("qwtPlot")
    ui.gridLayout.addWidget(plot, 0, 0, 1, 1)

    plot.setCanvasBackground(Qt.white)
    plot.setTitle("Datastream")
    plot.insertLegend(QwtLegend(), QwtPlot.BottomLegend);
    
    

    curve_m1 = QwtPlotCurve("m1"+"*"+str(m_scale))
    curve_m1.attach(plot)
    curve_m1.setPen(QPen(Qt.NoPen))
    curve_m2 = QwtPlotCurve("m2")
    curve_m2.attach(plot)
    curve_m2.setPen(QPen(Qt.NoPen))
    curve_m3 = QwtPlotCurve("m3")
    curve_m3.attach(plot)
    curve_m3.setPen(QPen(Qt.NoPen))
    curve_m4 = QwtPlotCurve("m4")
    curve_m4.attach(plot)
    curve_m4.setPen(QPen(Qt.NoPen))
    curve_vm1 = QwtPlotCurve("vm1"+"*"+str(vm_scale))
    curve_vm1.attach(plot)
    curve_vm1.setPen(QPen(Qt.NoPen))
    curve_vm2 = QwtPlotCurve("vm2")
    curve_vm2.attach(plot)
    curve_vm2.setPen(QPen(Qt.NoPen))
    curve_vm3 = QwtPlotCurve("vm3")
    curve_vm3.attach(plot)
    curve_vm3.setPen(QPen(Qt.NoPen))
    curve_vm4 = QwtPlotCurve("vm4")
    curve_vm4.attach(plot)
    curve_vm4.setPen(QPen(Qt.NoPen))
    curve_tm1 = QwtPlotCurve("tm1"+"*"+str(tm_scale))
    curve_tm1.attach(plot)
    curve_tm1.setPen(QPen(Qt.NoPen))
    curve_tm2 = QwtPlotCurve("tm2")
    curve_tm2.attach(plot)
    curve_tm2.setPen(QPen(Qt.NoPen))
    curve_tm3 = QwtPlotCurve("tm3")
    curve_tm3.attach(plot)
    curve_tm3.setPen(QPen(Qt.NoPen))
    curve_tm4 = QwtPlotCurve("tm4")
    curve_tm4.attach(plot)
    curve_tm4.setPen(QPen(Qt.NoPen))
    curve_cm1 = QwtPlotCurve("cm1"+"*"+str(cm_scale))
    curve_cm1.attach(plot)
    curve_cm1.setPen(QPen(Qt.NoPen))
    curve_cm2 = QwtPlotCurve("cm2")
    curve_cm2.attach(plot)
    curve_cm2.setPen(QPen(Qt.NoPen))
    curve_cm3 = QwtPlotCurve("cm3")
    curve_cm3.attach(plot)
    curve_cm3.setPen(QPen(Qt.NoPen))
    curve_cm4 = QwtPlotCurve("cm4")
    curve_cm4.attach(plot)
    curve_cm4.setPen(QPen(Qt.NoPen))
    curve_em1 = QwtPlotCurve("em1"+"*"+str(em_scale))
    curve_em1.attach(plot)
    curve_em1.setPen(QPen(Qt.NoPen))
    curve_em2= QwtPlotCurve("em2")
    curve_em2.attach(plot)
    curve_em2.setPen(QPen(Qt.NoPen))
    curve_em3 = QwtPlotCurve("em3")
    curve_em3.attach(plot)
    curve_em3.setPen(QPen(Qt.NoPen))
    curve_em4 = QwtPlotCurve("em4")
    curve_em4.attach(plot)
    curve_em4.setPen(QPen(Qt.NoPen))
    
    curve_q1 = QwtPlotCurve("q1"+"*"+str(q_scale))
    curve_q1.attach(plot)
    curve_q1.setPen(QPen(Qt.NoPen))
    curve_q2 = QwtPlotCurve("q2")
    curve_q2.attach(plot)
    curve_q2.setPen(QPen(Qt.NoPen))
    curve_q3 = QwtPlotCurve("q3")
    curve_q3.attach(plot)
    curve_q3.setPen(QPen(Qt.NoPen))
    curve_q4 = QwtPlotCurve("q4")
    curve_q4.attach(plot)
    curve_q4.setPen(QPen(Qt.NoPen))
    curve_vq1 = QwtPlotCurve("vq1"+"*"+str(vq_scale))
    curve_vq1.attach(plot)
    curve_vq1.setPen(QPen(Qt.NoPen))
    curve_vq2 = QwtPlotCurve("vq2")
    curve_vq2.attach(plot)
    curve_vq2.setPen(QPen(Qt.NoPen))
    curve_vq3 = QwtPlotCurve("vq3")
    curve_vq3.attach(plot)
    curve_vq3.setPen(QPen(Qt.NoPen))
    curve_vq4 = QwtPlotCurve("vq4")
    curve_vq4.attach(plot)
    curve_vq4.setPen(QPen(Qt.NoPen))
    curve_tq1 = QwtPlotCurve("tq1"+"*"+str(tq_scale))
    curve_tq1.attach(plot)
    curve_tq1.setPen(QPen(Qt.NoPen))
    curve_tq2 = QwtPlotCurve("tq2")
    curve_tq2.attach(plot)
    curve_tq2.setPen(QPen(Qt.NoPen))
    curve_tq3 = QwtPlotCurve("tq3")
    curve_tq3.attach(plot)
    curve_tq3.setPen(QPen(Qt.NoPen))
    curve_tq4 = QwtPlotCurve("tq4")
    curve_tq4.attach(plot)
    curve_tq4.setPen(QPen(Qt.NoPen))
    curve_cq1 = QwtPlotCurve("cq1"+"*"+str(cq_scale))
    curve_cq1.attach(plot)
    curve_cq1.setPen(QPen(Qt.NoPen))
    curve_cq2 = QwtPlotCurve("cq2")
    curve_cq2.attach(plot)
    curve_cq2.setPen(QPen(Qt.NoPen))
    curve_cq3 = QwtPlotCurve("cq3")
    curve_cq3.attach(plot)
    curve_cq3.setPen(QPen(Qt.NoPen))
    curve_cq4 = QwtPlotCurve("cq4")
    curve_cq4.attach(plot)
    curve_cq4.setPen(QPen(Qt.NoPen))
    curve_eq1 = QwtPlotCurve("eq1"+"*"+str(eq_scale))
    curve_eq1.attach(plot)
    curve_eq1.setPen(QPen(Qt.NoPen))
    curve_eq2= QwtPlotCurve("eq2")
    curve_eq2.attach(plot)
    curve_eq2.setPen(QPen(Qt.NoPen))
    curve_eq3 = QwtPlotCurve("eq3")
    curve_eq3.attach(plot)
    curve_eq3.setPen(QPen(Qt.NoPen))
    curve_eq4 = QwtPlotCurve("eq4")
    curve_eq4.attach(plot)
    curve_eq4.setPen(QPen(Qt.NoPen))


    curve_x1 = QwtPlotCurve("x1"+"*"+str(x_scale))
    curve_x1.attach(plot)
    curve_x1.setPen(QPen(Qt.NoPen))
    curve_x2 = QwtPlotCurve("x2")
    curve_x2.attach(plot)
    curve_x2.setPen(QPen(Qt.NoPen))
    curve_x3 = QwtPlotCurve("x3")
    curve_x3.attach(plot)
    curve_x3.setPen(QPen(Qt.NoPen))
    curve_vx1 = QwtPlotCurve("vx1"+"*"+str(vx_scale))
    curve_vx1.attach(plot)
    curve_vx1.setPen(QPen(Qt.NoPen))
    curve_vx2 = QwtPlotCurve("vx2")
    curve_vx2.attach(plot)
    curve_vx2.setPen(QPen(Qt.NoPen))
    curve_vx3 = QwtPlotCurve("vx3")
    curve_vx3.attach(plot)
    curve_vx3.setPen(QPen(Qt.NoPen))
    curve_tx1 = QwtPlotCurve("tx1"+"*"+str(tx_scale))
    curve_tx1.attach(plot)
    curve_tx1.setPen(QPen(Qt.NoPen))
    curve_tx2 = QwtPlotCurve("tx2")
    curve_tx2.attach(plot)
    curve_tx2.setPen(QPen(Qt.NoPen))
    curve_tx3 = QwtPlotCurve("tx3")
    curve_tx3.attach(plot)
    curve_tx3.setPen(QPen(Qt.NoPen))
    curve_cx1 = QwtPlotCurve("cx1"+"*"+str(cx_scale))
    curve_cx1.attach(plot)
    curve_cx1.setPen(QPen(Qt.NoPen))
    curve_cx2 = QwtPlotCurve("cx2")
    curve_cx2.attach(plot)
    curve_cx2.setPen(QPen(Qt.NoPen))
    curve_cx3 = QwtPlotCurve("cx3")
    curve_cx3.attach(plot)
    curve_cx3.setPen(QPen(Qt.NoPen))
    curve_ex1 = QwtPlotCurve("ex1"+"*"+str(ex_scale))
    curve_ex1.attach(plot)
    curve_ex1.setPen(QPen(Qt.NoPen))
    curve_ex2= QwtPlotCurve("ex2")
    curve_ex2.attach(plot)
    curve_ex2.setPen(QPen(Qt.NoPen))
    curve_ex3 = QwtPlotCurve("ex3")
    curve_ex3.attach(plot)
    curve_ex3.setPen(QPen(Qt.NoPen))

    curve_accX1 = QwtPlotCurve("accX1")
    curve_accX1.attach(plot)
    curve_accX1.setPen(QPen(Qt.NoPen))    
    curve_accY1 = QwtPlotCurve("accY1")
    curve_accY1.attach(plot)
    curve_accY1.setPen(QPen(Qt.NoPen))    
    curve_accZ1 = QwtPlotCurve("accZ1")
    curve_accZ1.attach(plot)
    curve_accZ1.setPen(QPen(Qt.NoPen))    
    curve_accX2 = QwtPlotCurve("accX2")
    curve_accX2.attach(plot)
    curve_accX2.setPen(QPen(Qt.NoPen))
    curve_accY2 = QwtPlotCurve("accY2")
    curve_accY2.attach(plot)
    curve_accY2.setPen(QPen(Qt.NoPen))    
    curve_accZ2 = QwtPlotCurve("accZ2")
    curve_accZ2.attach(plot)
    curve_accZ2.setPen(QPen(Qt.NoPen))
    

    #mY = QwtPlotMarker()
    #mY.setLabelAlignment(Qt.AlignRight | Qt.AlignTop)
    #mY.setLineStyle(QwtPlotMarker.HLine)
    #mY.setYValue(0.0)
    #mY.attach(plot)

    #plot.setAxisTitle(QwtPlot.xBottom, "Time (s)")
    #plot.setAxisTitle(QwtPlot.yLeft, "Force (g)")

    listen()

    plot.setAxisScale(QwtPlot.yLeft, -150, 150)
    plot.start()

    window.connect(ui.start_pause, SIGNAL("released()"), toggleplot)
    window.connect(ui.m1, SIGNAL("released()"), toggle)
    window.connect(ui.m2, SIGNAL("released()"), toggle)
    window.connect(ui.m3, SIGNAL("released()"), toggle)
    window.connect(ui.m4, SIGNAL("released()"), toggle)
    window.connect(ui.vm1, SIGNAL("released()"), toggle)
    window.connect(ui.vm2, SIGNAL("released()"), toggle)
    window.connect(ui.vm3, SIGNAL("released()"), toggle)
    window.connect(ui.vm4, SIGNAL("released()"), toggle)
    window.connect(ui.tm1, SIGNAL("released()"), toggle)
    window.connect(ui.tm2, SIGNAL("released()"), toggle)
    window.connect(ui.tm3, SIGNAL("released()"), toggle)
    window.connect(ui.tm4, SIGNAL("released()"), toggle)
    window.connect(ui.cm1, SIGNAL("released()"), toggle)
    window.connect(ui.cm2, SIGNAL("released()"), toggle)
    window.connect(ui.cm3, SIGNAL("released()"), toggle)
    window.connect(ui.cm4, SIGNAL("released()"), toggle)
    window.connect(ui.em1, SIGNAL("released()"), toggle)
    window.connect(ui.em2, SIGNAL("released()"), toggle)
    window.connect(ui.em3, SIGNAL("released()"), toggle)
    window.connect(ui.em4, SIGNAL("released()"), toggle)
    
    window.connect(ui.q1, SIGNAL("released()"), toggle)
    window.connect(ui.q2, SIGNAL("released()"), toggle)
    window.connect(ui.q3, SIGNAL("released()"), toggle)
    window.connect(ui.q4, SIGNAL("released()"), toggle)
    window.connect(ui.vq1, SIGNAL("released()"), toggle)
    window.connect(ui.vq2, SIGNAL("released()"), toggle)
    window.connect(ui.vq3, SIGNAL("released()"), toggle)
    window.connect(ui.vq4, SIGNAL("released()"), toggle)
    window.connect(ui.tq1, SIGNAL("released()"), toggle)
    window.connect(ui.tq2, SIGNAL("released()"), toggle)
    window.connect(ui.tq3, SIGNAL("released()"), toggle)
    window.connect(ui.tq4, SIGNAL("released()"), toggle)
    window.connect(ui.cq1, SIGNAL("released()"), toggle)
    window.connect(ui.cq2, SIGNAL("released()"), toggle)
    window.connect(ui.cq3, SIGNAL("released()"), toggle)
    window.connect(ui.cq4, SIGNAL("released()"), toggle)
    window.connect(ui.eq1, SIGNAL("released()"), toggle)
    window.connect(ui.eq2, SIGNAL("released()"), toggle)
    window.connect(ui.eq3, SIGNAL("released()"), toggle)
    window.connect(ui.eq4, SIGNAL("released()"), toggle)
    
    window.connect(ui.x1, SIGNAL("released()"), toggle)
    window.connect(ui.x2, SIGNAL("released()"), toggle)
    window.connect(ui.x3, SIGNAL("released()"), toggle)
    window.connect(ui.vx1, SIGNAL("released()"), toggle)
    window.connect(ui.vx2, SIGNAL("released()"), toggle)
    window.connect(ui.vx3, SIGNAL("released()"), toggle)
    window.connect(ui.tx1, SIGNAL("released()"), toggle)
    window.connect(ui.tx2, SIGNAL("released()"), toggle)
    window.connect(ui.tx3, SIGNAL("released()"), toggle)
    window.connect(ui.cx1, SIGNAL("released()"), toggle)
    window.connect(ui.cx2, SIGNAL("released()"), toggle)
    window.connect(ui.cx3, SIGNAL("released()"), toggle)
    window.connect(ui.ex1, SIGNAL("released()"), toggle)
    window.connect(ui.ex2, SIGNAL("released()"), toggle)
    window.connect(ui.ex3, SIGNAL("released()"), toggle)
    
    window.connect(ui.accX1, SIGNAL("released()"), toggle)
    window.connect(ui.accY1, SIGNAL("released()"), toggle)
    window.connect(ui.accZ1, SIGNAL("released()"), toggle)
    window.connect(ui.accX2, SIGNAL("released()"), toggle)
    window.connect(ui.accX2, SIGNAL("released()"), toggle)
    window.connect(ui.accX2, SIGNAL("released()"), toggle)
    window.show()

    sys.exit(app.exec_())
    

