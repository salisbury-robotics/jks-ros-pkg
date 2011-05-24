#ifndef MYHAPTICDEVICESWIDGET_H
#define MYHAPTICDEVICESWIDGET_H

#include "MyHapticsGLWidget.h"
#include "MyHapticDevicesModel.h"
#include "MyDeviceDetailsWidget.h"

#include <QWidget>
#include <QListView>

class MyHapticDevicesWidget : public QWidget
{
Q_OBJECT

    MyHapticDevicesModel    *m_devicesModel;
    MyDeviceDetailsWidget   *m_deviceDetails;
    MyHapticsGLWidget       *m_deviceView;
    QListView               *m_deviceList;

public:
    explicit MyHapticDevicesWidget(MyHapticDevicesModel *model, QWidget *parent = 0);

signals:

public slots:

};

#endif // MYHAPTICDEVICESWIDGET_H
