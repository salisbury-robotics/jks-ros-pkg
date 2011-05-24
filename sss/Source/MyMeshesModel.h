#ifndef MYMESHESMODEL_H
#define MYMESHESMODEL_H

#include <QAbstractTableModel>
#include <QDir>
#include <QXmlStreamWriter>
#include <QXmlStreamReader>
#include "Graphics/MeshRepository.h"

Q_DECLARE_METATYPE(Mesh *)
Q_DECLARE_METATYPE(Material)

class MyMeshesModel : public QAbstractTableModel
{
Q_OBJECT

    // column 0 is name, column 1 is mesh pointer, column 2 is material
    static const int k_columns = 3;

    // pointer to mesh repository instance
    MeshRepository *m_repository;

public:
    explicit MyMeshesModel(QObject *parent = 0);

    virtual int rowCount(const QModelIndex &parent = QModelIndex()) const;
    virtual int columnCount(const QModelIndex &parent = QModelIndex()) const;

    virtual Qt::ItemFlags flags(const QModelIndex &index) const;

    QVariant headerData(int section, Qt::Orientation orientation,
                        int role = Qt::DisplayRole) const;

    virtual QVariant data(const QModelIndex &index, int role) const;
    virtual bool setData(const QModelIndex &index, const QVariant &value, int role);

    // manipulation of rows
    virtual bool removeRows(int row, int count, const QModelIndex &parent = QModelIndex());

    // attempts to add a mesh to the repository from the specific path
    // returns true if successful
    bool addMeshFromPath(const QString &path);

    // adds the blender monkey as a test mesh
    bool addBlenderMonkey();

    // for saving and loading state
    void saveStateToXml(QXmlStreamWriter &writer, const QDir &base);
    bool restoreStateFromXml(QXmlStreamReader &reader, const QDir &base,
                             const QDir &alternate = QDir());

signals:

public slots:

};

#endif // MYMESHESMODEL_H
