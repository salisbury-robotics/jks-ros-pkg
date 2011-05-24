#include "MyDeviceDetailsWidget.h"
#include <QtGui>

MyDeviceDetailsWidget::MyDeviceDetailsWidget(QWidget *parent) :
    QWidget(parent)
{
    QFormLayout *layout = new QFormLayout(this);

    layout->addRow("Model:",        m_model = new QLabel);
    layout->addRow("Manufacturer:", m_manufacturer = new QLabel);
    layout->addRow("Max Force:",    m_maxForce = new QLabel);
    layout->addRow("Max Torque:",   m_maxTorque = new QLabel);
    layout->addRow("Max Stiffness:",m_maxStiffness = new QLabel);
    layout->addRow("Workspace:",    m_workspace = new QLabel);
}

void MyDeviceDetailsWidget::setDevice(const cHapticDeviceInfo &device)
{
    m_device = device;

    m_model->setText(QString::fromStdString(m_device.m_modelName));
    m_manufacturer->setText(QString::fromStdString(m_device.m_manufacturerName));
    m_maxForce->setText(QString("%1 N").arg(m_device.m_maxForce));
    m_maxTorque->setText(QString("%1 Nm").arg(m_device.m_maxTorque));
    m_maxStiffness->setText(QString("%1 N/m").arg(m_device.m_maxForceStiffness));
    m_workspace->setText(QString("%1 m").arg(m_device.m_workspaceRadius));

    update();
}
