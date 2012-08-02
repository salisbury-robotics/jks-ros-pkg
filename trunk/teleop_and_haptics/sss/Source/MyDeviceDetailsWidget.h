#ifndef MYDEVICEDETAILSWIDGET_H
#define MYDEVICEDETAILSWIDGET_H

#include <QWidget>
#include <QLabel>
//#include <devices/CGenericHapticDevice.h>
#include "MyHapticDevicesModel.h"

class MyDeviceDetailsWidget : public QWidget
{
Q_OBJECT
Q_PROPERTY(cHapticDeviceInfo device READ device WRITE setDevice USER true)

    cHapticDeviceInfo m_device;

    QLabel  *m_model;
    QLabel  *m_manufacturer;
    QLabel  *m_maxForce;
    QLabel  *m_maxTorque;
    QLabel  *m_maxStiffness;
    QLabel  *m_workspace;

public:
    explicit MyDeviceDetailsWidget(QWidget *parent = 0);

    void setDevice(const cHapticDeviceInfo &device);
    cHapticDeviceInfo device() const                    { return m_device; }

signals:

public slots:

};

#endif // MYDEVICEDETAILSWIDGET_H
