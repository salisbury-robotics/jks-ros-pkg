#include "MyLabelVolumeFilterModel.h"
#include "MyVolumesModel.h"
#include "MyProgressReporter.h"
#include "Data/LabelsVolumeGenerator.h"
#include <QMessageBox>
#include <QInputDialog>
#include <QApplication>

// --------------------------------------------------------------------------

MyLabelVolumeFilterModel::MyLabelVolumeFilterModel(QObject *parent) :
    QSortFilterProxyModel(parent)
{
    m_referenceIndex = -1;
}

// --------------------------------------------------------------------------

bool MyLabelVolumeFilterModel::filterAcceptsRow(int source_row, const QModelIndex &source_parent) const
{
    QAbstractItemModel *source = sourceModel();

    // get a handle to the reference volume
    if (m_referenceIndex < 0) return false;
    QModelIndex referenceIndex = source->index(m_referenceIndex, 1);
    Volume *reference = source->data(referenceIndex, Qt::DisplayRole).value<Volume *>();

    // get a handle to the volume at source_row
    QModelIndex index = source->index(source_row, 1, source_parent);
    Volume *v = source->data(index, Qt::DisplayRole).value<Volume *>();

    // accept the row if the dimensions are the same
    return v->dimensions == reference->dimensions;
}

// --------------------------------------------------------------------------

Qt::ItemFlags MyLabelVolumeFilterModel::flags(const QModelIndex &index) const
{
    Qt::ItemFlags f = QSortFilterProxyModel::flags(index);
    if (index.isValid() && index.column() == 0)
        f |= Qt::ItemIsUserCheckable;
    return f;
}

QVariant MyLabelVolumeFilterModel::data(const QModelIndex &index, int role) const
{
    if (index.isValid() && role == Qt::CheckStateRole)
    {
        QModelIndex si = mapToSource(index);
        si = sourceModel()->index(si.row(), 4);
        Labelling *label = sourceModel()->data(si, Qt::DisplayRole).value<Labelling *>();
        return label->combine ? Qt::Checked : Qt::Unchecked;
    }
    return QSortFilterProxyModel::data(index, role);
}

bool MyLabelVolumeFilterModel::setData(const QModelIndex &index, const QVariant &value, int role)
{
    if (index.isValid() && role == Qt::CheckStateRole)
    {
        QModelIndex si = mapToSource(index);
        si = sourceModel()->index(si.row(), 4);
        Labelling *label = sourceModel()->data(si, Qt::DisplayRole).value<Labelling *>();
        label->combine = value.toInt() == Qt::Checked;
        label->dirty = true;
        emit dataChanged(index, index);
        return true;
    }
    return QSortFilterProxyModel::setData(index, value, role);
}

// --------------------------------------------------------------------------

void MyLabelVolumeFilterModel::setReferenceVolume(int index)
{
    m_referenceIndex = index;
    invalidate();
    emit notifyReferenceIsValid(index > 0);
}

// --------------------------------------------------------------------------

void MyLabelVolumeFilterModel::createLabelsVolume()
{
    if (m_referenceIndex < 0) return;

    // query for a volume name
    QString name = QInputDialog::getText(QApplication::activeWindow(), "Label Volume Name",
                                         "Enter a name for the new label volume");
    if (name.isEmpty()) return;

    // clone the header from the reference volume
    QModelIndex index = sourceModel()->index(m_referenceIndex, 1);
    Volume *v = sourceModel()->data(index, Qt::DisplayRole).value<Volume *>();
    Volume header = *v;

    // reset certain fields within the header
    header.name = name.toStdString();
    header.path = "";
    header.format = Volume::pfUInt8;

    // temporarily allocate a blank array for the volume
    int n = header.dimensions[0] * header.dimensions[1] * header.dimensions[2];
    unsigned char *data = new unsigned char [n];
    memset(data, 0, n);

    // add the volume to the source model
    MyVolumesModel *model = qobject_cast<MyVolumesModel *>(sourceModel());
    if (model) {
        int position = model->addCustomVolume(header, data);
        QModelIndex si = model->index(position, 4);
        QModelIndex di = mapFromSource(si);
        emit notifyLabelsCreated(di.row());
    }

    // clean up temporary data
    delete [] data;
}

// --------------------------------------------------------------------------

bool MyLabelVolumeFilterModel::combineSelected(int target)
{
    // check that the target volume is of UInt8 data type
    Volume *v = data(index(target, 1), Qt::EditRole).value<Volume *>();
    if (v->format != Volume::pfUInt8) {
        QMessageBox::information(QApplication::activeWindow(), "Combine Label Volumes",
                                 "Target label volume must be 8-bit unsigned data type.");
        return false;
    }

    // count number of checked label rows to combine, and ensure > 0
    QList<int> selected;
    for (int i = 0; i < rowCount(); ++i) {
        Labelling *lab = data(index(i, 4), Qt::DisplayRole).value<Labelling *>();
        if (lab->combine) selected.append(i);
    }
    if (selected.isEmpty()) {
        QMessageBox::information(QApplication::activeWindow(), "Combine Label Volumes",
                                 "No volumes selected to combine.");
        return false;
    }

    // create a labels volume generator object and construct the combined volume
    Volume *mask = data(index(target, 3), Qt::DisplayRole).value<Volume *>();
    Labelling *lab = data(index(target, 4), Qt::DisplayRole).value<Labelling *>();
    LabelsVolumeGenerator generator(v, mask, lab);
    generator.setProgressReporter(MyProgressReporter::instance());
    for (int i = 0; i < selected.size(); ++i) {
        Volume *cv    = data(index(selected[i], 1), Qt::DisplayRole).value<Volume *>();
        Labelling *cl = data(index(selected[i], 4), Qt::DisplayRole).value<Labelling *>();
        generator.addLabelVolume(cv, cl);
    }
    generator.commitToTarget();

    // notify that the target row has changed
    emit dataChanged(index(target, 1), index(target, 4));

    return true;
}

// --------------------------------------------------------------------------

void MyLabelVolumeFilterModel::postProcessOnly(int target)
{

    // notify that the target row has changed
    emit dataChanged(index(target, 1), index(target, 4));
}

// --------------------------------------------------------------------------
