#include "MyVolumesModel.h"
#include "MyProgressReporter.h"
#include "MyMainWindow.h"
#include <QtGui>
#include <cml/cml.h>

using namespace std;

// --------------------------------------------------------------------------
// here's a callback function that allows the user to select from several
// DICOM series contained within a single directory

string MyDicomSelectionPrompt(const vector<DicomInfo> &series)
{
    // construct the dialog to allow the user to choose the series
    QDialog *dialog = new QDialog;
    dialog->setWindowTitle("Select DICOM Series");

    QBoxLayout *layout = new QVBoxLayout(dialog);

    // add a table view with the series information in it
    QTableWidget *table = new QTableWidget(series.size(), 4);
    table->verticalHeader()->hide();
    table->setSelectionBehavior(QAbstractItemView::SelectRows);
    table->setSelectionMode(QAbstractItemView::SingleSelection);

    QStringList headers; headers << "Series" << "Study" << "Modality" << "Images";
    table->setHorizontalHeaderLabels(headers);

    for (int r = 0; r < series.size(); ++r)
    {
        const DicomInfo &info = series[r];
        table->setItem(r, 0, new QTableWidgetItem(info.seriesDescription.c_str()));
        table->setItem(r, 1, new QTableWidgetItem(info.studyDescription.c_str()));
        table->setItem(r, 2, new QTableWidgetItem(info.modality.c_str()));
        table->setItem(r, 3, new QTableWidgetItem(QString("%1").arg(info.dimensions[2])));
    }
    table->resizeColumnsToContents();

    // need this hack to manually compute the size of the table widget
    int width = 4, height = 4 + table->horizontalHeader()->height();
    for (int r = 0; r < table->rowCount(); ++r)
        height += table->rowHeight(r);
    for (int c = 0; c < table->columnCount(); ++c)
        width += table->columnWidth(c);
    table->setFixedSize(width, height);

    layout->addWidget(table);

    // add buttons on the bottom
    QBoxLayout *buttons = new QHBoxLayout;
    buttons->addStretch(2);
    QPushButton *buttonCancel = new QPushButton(QObject::tr("&Cancel"));
    QObject::connect(buttonCancel, SIGNAL(clicked()), dialog, SLOT(reject()));
    buttons->addWidget(buttonCancel);
    QPushButton *buttonOK = new QPushButton(QObject::tr("&OK"));
    QObject::connect(buttonOK, SIGNAL(clicked()), dialog, SLOT(accept()));
    buttons->addWidget(buttonOK);
    layout->addLayout(buttons);

    layout->setSizeConstraint(QLayout::SetFixedSize);
    dialog->adjustSize();

    // attempt to center the dialog
    MyMainWindow *mainWindow = MyMainWindow::instance();
    mainWindow->centerWidget(dialog);

    // run the dialog
    string selected;
    if (dialog->exec() == QDialog::Accepted) {
        int index = table->currentRow();
        selected = series[index].uid;
    }

    // clean up
    delete dialog;
    return selected;
}

// --------------------------------------------------------------------------

MyVolumesModel::MyVolumesModel(QObject *parent) :
    QAbstractTableModel(parent)
{
    m_repository = VolumeRepository::instance();
    m_repository->setDicomSeriesSelector(MyDicomSelectionPrompt);
    m_repository->setProgressReporter(MyProgressReporter::instance());

    // add a test voluem and the Stanford Bunny to the initiatially loaded data
    addTestVolume();
    addStanfordBunny();
}

// --------------------------------------------------------------------------

int MyVolumesModel::rowCount(const QModelIndex &parent) const
{
    return m_repository->getNumberVolumes();
}

int MyVolumesModel::columnCount(const QModelIndex &parent) const
{
    return k_columns;
}

Qt::ItemFlags MyVolumesModel::flags(const QModelIndex &index) const
{
    if (!index.isValid())
        return Qt::ItemIsEnabled;

    Qt::ItemFlags f = QAbstractItemModel::flags(index);
    if (index.column() == 0)
        f |= Qt::ItemIsEditable;

    return f;
}

QVariant MyVolumesModel::headerData(int section, Qt::Orientation orientation, int role) const
{
    if (role == Qt::DisplayRole && orientation == Qt::Horizontal)
    {
        switch (section) {
        case 0: return QString("Name");
        case 1: return QString("Volume");
        case 2: return QString("Histogram");
        case 3: return QString("Mask");
        case 4: return QString("Labelling");
        default: return QString("?");
        }
    }
    else return QVariant();
}

QVariant MyVolumesModel::data(const QModelIndex &index, int role) const
{
    if (!index.isValid())
        return QVariant();

    if (index.row() >= m_repository->getNumberVolumes())
        return QVariant();

    if (index.column() >= k_columns)
        return QVariant();

    if (role == Qt::DisplayRole || role == Qt::EditRole)
    {
        Volume *v = m_repository->getVolume(index.row());
        switch (index.column())
        {
        case 0:     return v->name.c_str();
        case 1:     return QVariant::fromValue(v);
        case 2:     return QVariant::fromValue(&v->histogram);
        case 3:     return QVariant::fromValue(m_repository->getMask(index.row()));
        case 4:     return QVariant::fromValue(m_repository->getLabelling(index.row()));
        default:    return QVariant();
        }
    }
    else if (role == Qt::DecorationRole)
    {
        Volume *v = m_repository->getVolume(index.row());
        if (index.column() == 0) {
            if (!m_volumeIcons.contains(v))
                m_volumeIcons[v] = createVolumeIcon(v);
            return m_volumeIcons[v];
        }
        else return QVariant();
    }
    else return QVariant();
}

bool MyVolumesModel::setData(const QModelIndex &index, const QVariant &value, int role)
{
    if (index.isValid() && role == Qt::EditRole)
    {
        Volume *v = m_repository->getVolume(index.row());
        if (index.column() == 0) v->name = value.toString().toStdString();
        emit dataChanged(index, index);
        return true;
    }
    return false;
}

bool MyVolumesModel::removeRows(int row, int count, const QModelIndex &parent)
{
    beginRemoveRows(parent, row, row+count-1);

    // first clear the icon(s)
    for (int r = row; r < row+count; ++r) {
        Volume *v = m_repository->getVolume(r);
        m_volumeIcons.remove(v);
    }

    // then remove the volume from the repository
    bool result = m_repository->removeVolume(row, count);

    endRemoveRows();
    return result;
}

// --------------------------------------------------------------------------

bool MyVolumesModel::addVolumeFromPath(const QString &path)
{
    int position = m_repository->getNumberVolumes();
    beginInsertRows(QModelIndex(), position, position);

    bool result = m_repository->loadFromDirectory(path.toStdString(),
                                                  VolumeRepository::FormatDICOM);

    endInsertRows();
    return result;
}

// --------------------------------------------------------------------------

int MyVolumesModel::addCustomVolume(const Volume &header, const void *data)
{
    int position = m_repository->getNumberVolumes();
    beginInsertRows(QModelIndex(), position, position);

    // create a new volume within the repository
    m_repository->addCustomVolume(header, data);

    endInsertRows();
    return position;
}

// --------------------------------------------------------------------------

void MyVolumesModel::addTestVolume()
{
    int position = m_repository->getNumberVolumes();
    beginInsertRows(QModelIndex(), position, position);

    // tell the repository to add its test volume
    m_repository->addTestVolume();

    // then set its path to a special string that would be recognized on load
    m_repository->getVolume(position)->path = "$test";

    endInsertRows();
}

// --------------------------------------------------------------------------

void MyVolumesModel::addStanfordBunny()
{
    int position = m_repository->getNumberVolumes();
    beginInsertRows(QModelIndex(), position, position);

    // add the Stanford Bunny volume to the repository
    Volume header;
    header.name = "Stanford Bunny";
    header.path = "$bunny";             // special string recognized on load
    header.format = Volume::pfInt16;
    header.dimensions = cml::vector3i(128, 128, 120);
    header.spacing = cml::vector3f(1.35f, 1.35f, 1.5f);

    QResource bunny(":/StanfordBunny.raw");
    if (bunny.isCompressed()) {
        QByteArray bytes = qUncompress(bunny.data(), bunny.size());
        m_repository->addCustomVolume(header, bytes.data());
    }
    else m_repository->addCustomVolume(header, bunny.data());

    endInsertRows();
}

// --------------------------------------------------------------------------

void MyVolumesModel::saveStateToXml(QXmlStreamWriter &writer, const QDir &base)
{
    // try to get a hold of the main window for parenting model dialogs
    QWidget *window = qobject_cast<QWidget *>(QObject::parent());
    if (window == 0) window = QApplication::activeWindow();

    // start the volumes XML element
    writer.writeStartElement("volumes");

    // start at index 1 to ignore the null volume
    for (int i = 1; i < rowCount(); ++i)
    {
        QVariant variant = data(createIndex(i, 1), Qt::DisplayRole);
        if (variant.canConvert<Volume*>())
        {
            Volume *v = variant.value<Volume*>();
/*
            // check if this is a new volume, and ask to save if it is
            if (v->path.empty())
            {
                // ask if the user wants to save the dissection with the scene
                QMessageBox::StandardButton b = QMessageBox::question(window, "Save Scene",
                    QString("Save new volume %1?").arg(QString::fromStdString(v->name)),
                    QMessageBox::Yes | QMessageBox::No, QMessageBox::Yes);

                // if response was yes, then query for file name
                if (b == QMessageBox::Yes)
                {
                    QString file = QFileDialog::getSaveFileName(window, "Save Volume As...",
                                    base.absoluteFilePath("volume.ssv"), "Simulation Volume (*.ssv");

                    // TODO: finish this!!!
                }
                // otherwise just skip this volume in the save file
                else continue;
            }
*/
            writer.writeStartElement("volume");

            writer.writeTextElement("name", QString::fromStdString(v->name));
            writer.writeTextElement("path", base.relativeFilePath(v->path.c_str()));

            // check to see if the volume mask has been modified, and query
            // user for instructions on saving if it is
            Volume *m = data(index(i, 3), Qt::DisplayRole).value<Volume*>();
            if (m->modified)
            {
                // ask if the user wants to save the dissection with the scene
                QMessageBox::StandardButton b = QMessageBox::question(window, "Save Scene",
                    QString("Save dissection/mask for %1?").arg(QString::fromStdString(v->name)),
                    QMessageBox::Yes | QMessageBox::No, QMessageBox::Yes);

                // if save requested, then save the dissect and record its path
                if (b == QMessageBox::Yes)
                {
                    QString file = QString::fromStdString(m->path);

                    if (file.isEmpty()) {
                        file = QFileDialog::getSaveFileName(window, "Save Dissection As...",
                                base.absoluteFilePath("dissection.ssv"), "Simulation Volume (*.ssv)");
                    }

                    if (!file.isEmpty()) {
                        writeVolumeXml(m, file);
                        writer.writeTextElement("mask", base.relativeFilePath(file));
                    }
                }
            }

            // check to see if the labelling has been modified, and query user for
            // instructions to save
            Labelling *lab = data(index(i, 4), Qt::DisplayRole).value<Labelling *>();
            if (lab->dirty)
            {
                writer.writeStartElement("labelling");

                writer.writeTextElement("combine",   QString("%1").arg(lab->combine));
                writer.writeTextElement("processed", QString("%1").arg(lab->processed));
                writer.writeTextElement("threshold", QString("%1").arg(lab->threshold));
                writer.writeTextElement("tvalue",    QString("%1").arg(lab->thresholdValue));

                writer.writeStartElement("lut");
                writer.writeAttribute("entries", QString("%1").arg(lab->entries()));
                for (int i = 0; i < lab->entries(); ++i) {
                    ostringstream oss;
                    oss << lab->colourLUT[i];
                    writer.writeTextElement("colour", QString::fromStdString(oss.str()));
                }
                writer.writeEndElement(); // lut

                writer.writeEndElement(); // labelling
            }

            writer.writeEndElement(); // volume
        }
    }

    writer.writeEndElement(); // volumes
}

// --------------------------------------------------------------------------

Labelling MyVolumesModel::readLabellingXML(QXmlStreamReader &reader)
{
    Labelling labels;

    while (reader.readNextStartElement())
    {
        if (reader.name() == "combine")
            labels.combine = reader.readElementText().toInt();
        else if (reader.name() == "processed")
            labels.processed = reader.readElementText().toInt();
        else if (reader.name() == "threshold")
            labels.threshold = reader.readElementText().toInt();
        else if (reader.name() == "tvalue")
            labels.thresholdValue = reader.readElementText().toInt();
        else if (reader.name() == "lut") {
            while (reader.readNextStartElement()) {
                if (reader.name() != "colour") continue;
                string s = reader.readElementText().toStdString();
                istringstream iss(s);
                cml::vector4f c;
                iss >> c[0] >> c[1] >> c[2] >> c[3];
                labels.colourLUT.push_back(c);
            }
        }
        else reader.skipCurrentElement();
    }

    // mark as dirty so that the caller knows a labelling was read
    labels.dirty = true;
    return labels;
}

bool MyVolumesModel::restoreStateFromXml(QXmlStreamReader &reader, const QDir &base,
                                         const QDir &alternate)
{
    if (reader.readNextStartElement())
    {
        if (reader.name() != "volumes") return false;

        // send out a reset signal so that anything attached can disconnect
        // (maybe a little drastic, but works)
        beginResetModel();

        // clear out existing volumes first (0-index null volume excepted)
        removeRows(1, rowCount()-1);

        // read volume elements and insert them into the repository
        while (reader.readNextStartElement())
        {
            if (reader.name() != "volume") {
                reader.skipCurrentElement();
                continue;
            }

            // read the elements that describe this volume
            QString name, path, mask;
            Labelling labels;
            while (reader.readNextStartElement())
            {
                if (reader.name() == "name")
                    name = reader.readElementText();
                else if (reader.name() == "path")
                    path = reader.readElementText();
                else if (reader.name() == "mask")
                    mask = reader.readElementText();
                else if (reader.name() == "labelling")
                    labels = readLabellingXML(reader);
                else
                    reader.skipCurrentElement();
            }

            // then load the volume into our repository
            if (!path.isEmpty())
            {
                if (path == "$test")        addTestVolume();
                else if (path == "$bunny")  addStanfordBunny();
                else {
                    QFileInfo info(base.absoluteFilePath(path));
                    if (!info.exists()) {
                        QString first = info.absolutePath();
                        // try the alternate path
                        info.setFile(alternate.absoluteFilePath(path));
                        if (!info.exists()) {
                            QMessageBox::warning(MyMainWindow::instance(), "Load Volume",
                                                 "Could not find volume at path " + first +
                                                 " or " + info.absolutePath());
                            continue;
                        }
                    }
                    if (!addVolumeFromPath(info.absoluteFilePath())) {
                        QMessageBox::warning(MyMainWindow::instance(), "Load Volume",
                                             "Could not load volume at path " +
                                             info.absolutePath());
                        continue;
                    }
                }

                // set the volume name
                int vindex = rowCount() - 1;
                setData(createIndex(vindex, 0), name, Qt::EditRole);
                
                // load the volume mask if one exists
                if (!mask.isEmpty()) {
                    QFileInfo info(base.absoluteFilePath(mask));
                    if (!info.exists()) {
                        QString first = info.absolutePath();
                        // try the alternate path
                        info.setFile(alternate.absoluteFilePath(mask));
                        if (!info.exists()) {
                            QMessageBox::warning(MyMainWindow::instance(), "Load Volume",
                                                 "Could not find mask at path " + first +
                                                 " or " + info.absolutePath());
                            continue;
                        }
                    }

                    // get a pointer to the mask volume, then read into it
                    Volume *m = data(index(vindex, 3), Qt::EditRole).value<Volume*>();
                    if (m) readVolumeXml(m, info.absoluteFilePath());
                }

                // set the labelling if one was loaded
                if (labels.dirty) {
                    Labelling *lab = data(index(vindex, 4), Qt::EditRole).value<Labelling *>();
                    *lab = labels;
                }
            }
        }

        endResetModel();
    }

    return true;
}

// --------------------------------------------------------------------------
// lightweight methods for reading and writing a Volume to XML

static QString g_formatToString[Volume::pfSentinel];
static QMap<QString, Volume::PixelFormat> g_stringToFormat;

void MyVolumesModel::initVolumeXml()
{
    // do a one-time initialization of pixel format to string conversion tables
    if (g_stringToFormat.isEmpty())
    {
        g_formatToString[Volume::pfUInt8] = "UInt8";
        g_stringToFormat["UInt8"] = Volume::pfUInt8;

        g_formatToString[Volume::pfInt16] = "Int16";
        g_stringToFormat["Int16"] = Volume::pfInt16;

        g_formatToString[Volume::pfUInt16] = "UInt16";
        g_stringToFormat["UInt16"] = Volume::pfUInt16;
    }
}

bool MyVolumesModel::writeVolumeXml(Volume *volume, const QString &path)
{
    initVolumeXml();

    QFile ssvFile(path);
    if (ssvFile.open(QIODevice::WriteOnly))
    {
        QFileInfo info(ssvFile);
        QDir::setCurrent(info.path());

        // determine the name of the accompanying data file
        QString dataFileName = info.baseName() + ".dat";

        // write the header in an XML file
        QXmlStreamWriter writer(&ssvFile);

        // preamble
        writer.setAutoFormatting(true);
        writer.writeStartDocument();
        writer.writeStartElement("ssv");
        writer.writeAttribute("version", "1.0");

        // volume header information
        writer.writeTextElement("name", QString::fromStdString(volume->name));

        ostringstream oss;
        oss << volume->dimensions;
        writer.writeTextElement("dimensions", QString::fromStdString(oss.str()));

        oss.str("");
        oss << volume->spacing;
        writer.writeTextElement("spacing", QString::fromStdString(oss.str()));

        writer.writeTextElement("format", g_formatToString[volume->format]);
        writer.writeTextElement("data", dataFileName);

        // postamble
        writer.writeEndElement();
        writer.writeEndDocument();

        ssvFile.close();

        // write the actual volume data in a separate file
        QFile datFile(info.dir().absoluteFilePath(dataFileName));
        if (datFile.open(QIODevice::WriteOnly))
        {
            cml::vector3i d = volume->dimensions;
            datFile.write(reinterpret_cast<const char *>(volume->data),
                          d[0]*d[1]*d[2] * volume->bytesPerVoxel());
            datFile.close();
        }

        // set the path on the volume for next time
        volume->path = info.canonicalFilePath().toStdString();

        return true;
    }

    return false;
}

bool MyVolumesModel::readVolumeXml(Volume *volume, const QString &path)
{
    initVolumeXml();

    QFile ssvFile(path);
    if (ssvFile.open(QIODevice::ReadOnly))
    {
        // update application's current working path
        QFileInfo info(ssvFile);
        QDir::setCurrent(info.path());
        QString dataFileName;

        // first read the XML header
        QXmlStreamReader reader(&ssvFile);
        if (reader.readNextStartElement())
        {
            if (reader.name() == "ssv")
            {
                Volume header;

                // parse the elements within the volume into temp header
                while (reader.readNextStartElement())
                {
                    QString text = reader.readElementText();

                    if (reader.name() == "name")
                        header.name = text.toStdString();
                    else if (reader.name() == "dimensions") {
                        cml::vector3i &d = header.dimensions;
                        istringstream iss(text.toStdString());
                        iss >> d[0] >> d[1] >> d[2];
                    }
                    else if (reader.name() == "spacing") {
                        cml::vector3f &s = header.spacing;
                        istringstream iss(text.toStdString());
                        iss >> s[0] >> s[1] >> s[2];
                    }
                    else if (reader.name() == "format")
                        header.format = g_stringToFormat[text];
                    else if (reader.name() == "data")
                        dataFileName = text;
                }

                // TODO: we do a funny thing here -- if the volume is allocated,
                //       we check if its dimensions match before loaded, and
                //       otherwise we allocate a new volume into the given pointer
                if (volume->data)
                {
                    // check that the saved volume matches the allocated one
                    if (header.dimensions != volume->dimensions) {
                        QMessageBox::critical(MyMainWindow::instance(), "Volume Read Error",
                            "Saved volume dimensions do not match allocated volume.");
                        return false;
                    }
                }
                else
                {
                    header.path = path.toStdString();

                    // allocate the data storage
                    int n = header.dimensions[0] * header.dimensions[1] * header.dimensions[2];
                    switch (header.format) {
                    case Volume::pfUInt8:   header.data = new unsigned char [n];    break;
                    case Volume::pfInt16:   header.data = new signed short [n];     break;
                    case Volume::pfUInt16:  header.data = new unsigned short [n];   break;
                    }

                    // copy over the header
                    *volume = header;
                }
            }
        }

        ssvFile.close();

        // read the actual volume data from the separate file
        QFile datFile(info.dir().absoluteFilePath(dataFileName));
        if (datFile.open(QIODevice::ReadOnly))
        {
            cml::vector3i d = volume->dimensions;
            datFile.read(reinterpret_cast<char *>(volume->data),
                         d[0]*d[1]*d[2] * volume->bytesPerVoxel());
            datFile.close();
        }
        else
        {
            QMessageBox::critical(MyMainWindow::instance(), "Volume Read Error",
                                  QString("Could not open data file %1").arg(dataFileName));
            return false;
        }

        // set the path on the volume to this file
        volume->path = info.canonicalFilePath().toStdString();

        // set modified flag on this mask (assuming it's not a saved blank)
        volume->modified = true;

        return true;
    }

    return false;
}

// --------------------------------------------------------------------------

QPixmap MyVolumesModel::createVolumeIcon(const Volume *volume) const
{
    QPixmap icon;

    if (volume->data)
    {
        // use the middle slice of the volume as the icon
        int slice = volume->dimensions[2] / 2;

        // create a pixmap for the middle slice of the image
        unsigned char *rgb = 0;
        switch (volume->format) {
        case Volume::pfUInt8:
            rgb = extractVolumeSliceRGB<unsigned char>(volume, slice); break;
        case Volume::pfInt16:
            rgb = extractVolumeSliceRGB<signed short>(volume, slice); break;
        case Volume::pfUInt16:
            rgb = extractVolumeSliceRGB<unsigned short>(volume, slice); break;
        default:
            break;
        }

        QImage image(rgb, volume->dimensions[0], volume->dimensions[1],
                     QImage::Format_RGB32);
        icon = QPixmap::fromImage(image.scaled(48, 48, Qt::KeepAspectRatio, Qt::SmoothTransformation));

        // clean up
        delete [] rgb;
    }

    return icon;
}

template <class T>
unsigned char * MyVolumesModel::extractVolumeSliceRGB(const Volume *volume, int slice) const
{
    int w = volume->dimensions[0];
    int h = volume->dimensions[1];

    const T *data = reinterpret_cast<const T *>(volume->data);
    data += w*h*slice;

    int ww = volume->histogram.maxValue - volume->histogram.minValue + 1;

    unsigned char *buffer = new unsigned char [w*h*4];
    unsigned char *b = buffer;
    for (int i = 0; i < w*h; ++i, ++data) {
        int value = (*data - volume->histogram.minValue) * 255 / ww;
        *b++ = value;
        *b++ = value;
        *b++ = value;
        *b++ = value;
    }

    return buffer;
}

// --------------------------------------------------------------------------
