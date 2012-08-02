#include "MyHapticDevicesModel.h"
#include <chai3d.h>

MyHapticDevicesModel::MyHapticDevicesModel(QObject *parent) :
    QAbstractTableModel(parent)
{    
    qRegisterMetaType<cHapticDeviceInfo>();

    // create a haptic device handler that tells us what devices are connected
    // to the host computer
    m_handler = new cHapticDeviceHandler();

    // initialize all available haptic devices here
    for (int i = 0; i < m_handler->getNumDevices(); ++i) {
        cGenericHapticDevice *device;
        if (m_handler->getDevice(device, i) == 0) {
            device->open();
//            device->initialize(true);
        }
    }
}

MyHapticDevicesModel::~MyHapticDevicesModel()
{
    if (m_handler) delete m_handler;
}


int MyHapticDevicesModel::rowCount(const QModelIndex &parent) const
{
    // number of devices, plus 1 for null device
    return m_handler->getNumDevices() + 1;
}

int MyHapticDevicesModel::columnCount(const QModelIndex &parent) const
{
    return k_columns;
}

QVariant MyHapticDevicesModel::headerData(int section, Qt::Orientation orientation,
                                          int role) const
{
    if (role == Qt::DisplayRole && orientation == Qt::Horizontal)
    {
        switch (section) {
        case 0: return QString("Name");
        case 1: return QString("Specifications");
        case 2: return QString("Device");
        default: return QString("?");
        }
    }
    else return QVariant();
}

QVariant MyHapticDevicesModel::data(const QModelIndex &index, int role) const
{
    if (!index.isValid())
        return QVariant();

    if (index.row() >= rowCount())
        return QVariant();

    if (index.column() >= k_columns)
        return QVariant();

    if (role == Qt::DisplayRole || role == Qt::EditRole)
    {
        // get the haptic device pointer and specifications structure
        cGenericHapticDevice *device = 0;
        cHapticDeviceInfo info;

        // row zero is the null device
        if (index.row() == 0) {
            info.m_modelName = "(none)";
        }
        else {
            m_handler->getDevice(device, index.row()-1);
            info = device->getSpecifications();
        }

        switch (index.column())
        {
        case 0:     return QString::fromStdString(info.m_modelName);
        case 1:     return QVariant::fromValue(info);
        case 2:     return QVariant::fromValue(device);
        default:    return QVariant();
        }
    }
    else return QVariant();
}
