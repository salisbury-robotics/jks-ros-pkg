#ifndef MYHAPTICDEVICESMODEL_H
#define MYHAPTICDEVICESMODEL_H

#include <QAbstractTableModel>
#include <devices/CHapticDeviceHandler.h>

Q_DECLARE_METATYPE(cHapticDeviceInfo)
Q_DECLARE_METATYPE(cGenericHapticDevice *)

class MyHapticDevicesModel : public QAbstractTableModel
{
Q_OBJECT

    // column 0 is device name, column 1 is pointer to cHapticDeviceInfo,
    // column 3 is a pointer to the actual cGenericHapticDevice
    static const int k_columns = 3;

    // a chai3d haptic device handler that represents the underlying data
    cHapticDeviceHandler *m_handler;

public:
    explicit MyHapticDevicesModel(QObject *parent = 0);
    virtual ~MyHapticDevicesModel();

    // note: data contained within this class is static -- the model is
    //       populated on construction, and won't change for the program's life

    virtual int rowCount(const QModelIndex &parent = QModelIndex()) const;
    virtual int columnCount(const QModelIndex &parent = QModelIndex()) const;

    QVariant headerData(int section, Qt::Orientation orientation,
                        int role = Qt::DisplayRole) const;

    // note: first row is also "(none)" / NULL to allow deselecting
    virtual QVariant data(const QModelIndex &index, int role = Qt::DisplayRole) const;

signals:

public slots:

};

#endif // MYHAPTICDEVICESMODEL_H
