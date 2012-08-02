#ifndef MYVOLUMESMODEL_H
#define MYVOLUMESMODEL_H

#include <QAbstractTableModel>
#include <QPixmap>
#include <QHash>
#include <QDir>
#include <QXmlStreamWriter>
#include <QXmlStreamReader>
#include "Data/VolumeRepository.h"

Q_DECLARE_METATYPE(Volume *)
Q_DECLARE_METATYPE(Histogram *)
Q_DECLARE_METATYPE(Labelling *)

class MyVolumesModel : public QAbstractTableModel
{
Q_OBJECT

    // column 0 is name, column 1 is pointer to Volume, column 2 is histogram,
    // column 3 is pointer to mask volume, column 4 is pointer to labelling
    static const int k_columns = 5;

    // pointer to the volume repository instance that tracks actual data
    VolumeRepository *m_repository;

    // cache of small icons used as decorators for the volumes
    mutable QHash<Volume *, QPixmap> m_volumeIcons;


    QPixmap createVolumeIcon(const Volume *volume) const;

    template <class T>
    unsigned char * extractVolumeSliceRGB(const Volume *volume, int slice) const;

    // helper to load a labelling structure from XML
    Labelling readLabellingXML(QXmlStreamReader &reader);

public:
    explicit MyVolumesModel(QObject *parent = 0);

    virtual int rowCount(const QModelIndex &parent = QModelIndex()) const;
    virtual int columnCount(const QModelIndex &parent = QModelIndex()) const;

    virtual Qt::ItemFlags flags(const QModelIndex &index) const;

    QVariant headerData(int section, Qt::Orientation orientation,
                        int role = Qt::DisplayRole) const;

    virtual QVariant data(const QModelIndex &index, int role) const;
    virtual bool setData(const QModelIndex &index, const QVariant &value, int role);

    // manipulation of rows
    virtual bool removeRows(int row, int count, const QModelIndex &parent = QModelIndex());

    // attempts to add a volume to the repository from the specific path
    // returns true if successful
    bool addVolumeFromPath(const QString &path);

    // adds a custom volume to the repository (data is copied)
    int addCustomVolume(const Volume &header, const void *data);

    // adds a the repository's default test volume to the repository
    void addTestVolume();

    // adds a custom volume, the "Stanford Bunny", from resource to the repository
    void addStanfordBunny();

    // for saving and loading state, saved paths are relative to base
    void saveStateToXml(QXmlStreamWriter &writer, const QDir &base);
    bool restoreStateFromXml(QXmlStreamReader &reader, const QDir &base,
                             const QDir &alternate = QDir());

    // lightweight methods for reading and writing a Volume to XML
    // note that the volume should already be sized and allocated when read --
    // the readVolumeXml() function will only update the data and path
    // (currently used for saving and loading volume dissections or masks)
    static bool writeVolumeXml(Volume *volume, const QString &path);
    static bool readVolumeXml(Volume *volume, const QString &path);
    static void initVolumeXml();

signals:

public slots:

};

#endif // MYVOLUMESMODEL_H
