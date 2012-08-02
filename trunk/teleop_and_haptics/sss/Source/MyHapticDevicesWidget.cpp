#include "MyHapticDevicesWidget.h"
#include <QtGui>

// --------------------------------------------------------------------------

MyHapticDevicesWidget::MyHapticDevicesWidget(MyHapticDevicesModel *model, QWidget *parent) :
    QWidget(parent)
{
    m_devicesModel = model;

    QBoxLayout *layout = new QHBoxLayout(this);

    // left side has a list view of all device, and a device details widget
    QBoxLayout *leftSide = new QVBoxLayout;

    m_deviceList = new QListView();
    m_deviceList->setModel(m_devicesModel);
    m_deviceList->setAlternatingRowColors(true);
    m_deviceList->setRowHidden(0, true);
    leftSide->addWidget(m_deviceList);

    QGroupBox *specsBox = new QGroupBox("Specifications");
    QLayout *specsLayout = new QVBoxLayout(specsBox);
    m_deviceDetails = new MyDeviceDetailsWidget();
    specsLayout->addWidget(m_deviceDetails);
    leftSide->addWidget(specsBox);

    layout->addLayout(leftSide);

    // The default QGLFormat has the following properties:
    //  - Double buffer: Enabled.
    //  - Depth buffer: Enabled.
    //  - RGBA: Enabled (i.e., color index disabled).
    //  - Alpha channel: Disabled.
    //  - Accumulator buffer: Disabled.
    //  - Stencil buffer: Disabled.
    //  - Stereo: Disabled.
    //  - Direct rendering: Enabled.
    //  - Overlay: Disabled.
    //  - Plane: 0 (i.e., normal plane).
    //  - Multisample buffers: Disabled.
    // These can be changed here in the format variable if necessary.
    QGLFormat format;
    format.setAlpha(true);
    m_deviceView = new MyHapticsGLWidget(format);
    m_deviceView->setModel(m_devicesModel);
    layout->addWidget(m_deviceView, 2);


    leftSide->addWidget(m_deviceView->settingsWidget());
    leftSide->addStretch(2);


    // set up data mappings to update widgets
    QDataWidgetMapper *mapper = new QDataWidgetMapper(this);
    mapper->setModel(m_devicesModel);
    mapper->addMapping(m_deviceDetails, 1);
    mapper->toFirst();
    mapper->toNext();

    // connect current item in the list view to update the mapper
    QItemSelectionModel *sm = m_deviceList->selectionModel();
    connect(sm, SIGNAL(currentChanged(QModelIndex,QModelIndex)),
            mapper, SLOT(setCurrentModelIndex(QModelIndex)));
}

// --------------------------------------------------------------------------

