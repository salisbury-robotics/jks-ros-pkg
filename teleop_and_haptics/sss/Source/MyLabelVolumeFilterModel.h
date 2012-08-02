#ifndef MYLABELVOLUMEFILTERMODEL_H
#define MYLABELVOLUMEFILTERMODEL_H

#include <QSortFilterProxyModel>

class MyLabelVolumeFilterModel : public QSortFilterProxyModel
{
Q_OBJECT

protected:
    int     m_referenceIndex;

    virtual bool filterAcceptsRow(int source_row, const QModelIndex &source_parent) const;

public:
    explicit MyLabelVolumeFilterModel(QObject *parent = 0);

    virtual Qt::ItemFlags flags(const QModelIndex &index) const;
    virtual QVariant data(const QModelIndex &index, int role) const;
    virtual bool setData(const QModelIndex &index, const QVariant &value, int role);

    // combines all selected label volumes into the row specified by target
    bool combineSelected(int target);

    // generates an alpha mask from the label data on the target volume
    void postProcessOnly(int target);

signals:
    void notifyReferenceIsValid(bool valid);
    void notifyLabelsCreated(int index);

public slots:
    void setReferenceVolume(int index);
    void createLabelsVolume();
};

#endif // MYLABELVOLUMEFILTERMODEL_H
