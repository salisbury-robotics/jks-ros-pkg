#include "MyMeshesModel.h"
#include "MyMainWindow.h"
#include <QtGui>

using namespace std;

// --------------------------------------------------------------------------

MyMeshesModel::MyMeshesModel(QObject *parent) :
    QAbstractTableModel(parent)
{
    m_repository = MeshRepository::instance();

    // add a test model (monkey) to the repository
    addBlenderMonkey();
}

// --------------------------------------------------------------------------

int MyMeshesModel::rowCount(const QModelIndex &parent) const
{
    return m_repository->getNumberMeshes();
}

int MyMeshesModel::columnCount(const QModelIndex &parent) const
{
    return k_columns;
}

Qt::ItemFlags MyMeshesModel::flags(const QModelIndex &index) const
{
    if (!index.isValid())
        return Qt::ItemIsEnabled;

    Qt::ItemFlags f = QAbstractItemModel::flags(index);
    if (index.column() == 0)
        f |= Qt::ItemIsEditable | Qt::ItemIsUserCheckable;
    else if (index.column() == 2)
        f |= Qt::ItemIsEditable;

    return f;
}

QVariant MyMeshesModel::headerData(int section, Qt::Orientation orientation, int role) const
{
    if (role == Qt::DisplayRole && orientation == Qt::Horizontal)
    {
        switch (section) {
        case 0: return QString("Name");
        case 1: return QString("Mesh");
        case 2: return QString("Material");
        default: return QString("?");
        }
    }
    else return QVariant();
}

QVariant MyMeshesModel::data(const QModelIndex &index, int role) const
{
    if (!index.isValid())
        return QVariant();

    if (index.row() >= m_repository->getNumberMeshes())
        return QVariant();

    if (index.column() >= k_columns)
        return QVariant();

    if (role == Qt::DisplayRole || role == Qt::EditRole)
    {
        Mesh *m = m_repository->getMesh(index.row());
        switch (index.column())
        {
        case 0:     return m->name().c_str();
        case 1:     return QVariant::fromValue(m);
        case 2:     return QVariant::fromValue(m->material());
        default:    return QVariant();
        }
    }
    else if (role == Qt::CheckStateRole)
    {
        Mesh *m = m_repository->getMesh(index.row());
        switch (index.column())
        {
        case 0:     return m->visible() ? Qt::Checked : Qt::Unchecked;
        default:    return QVariant();
        }
    }
    else return QVariant();
}

bool MyMeshesModel::setData(const QModelIndex &index, const QVariant &value, int role)
{
    if (index.isValid() && (role == Qt::EditRole || role == Qt::CheckStateRole))
    {
        Mesh *m = m_repository->getMesh(index.row());

        if (index.column() == 0) {
            if (role == Qt::EditRole)
                m->setName(value.toString().toStdString());
            else if (role == Qt::CheckStateRole)
                m->setVisible(value.toInt() == Qt::Checked);
        }
        else if (index.column() == 2 && value.canConvert<Material>())
            m->setMaterial(value.value<Material>());

        emit dataChanged(index, index);
        return true;
    }
    return false;
}

bool MyMeshesModel::removeRows(int row, int count, const QModelIndex &parent)
{
    beginRemoveRows(parent, row, row+count-1);
    bool result = m_repository->removeMesh(row, count);
    endRemoveRows();
    return result;
}

// --------------------------------------------------------------------------

bool MyMeshesModel::addMeshFromPath(const QString &path)
{
    int position = m_repository->getNumberMeshes();
    beginInsertRows(QModelIndex(), position, position);

    bool result = m_repository->loadFromFile(path.toStdString(),
                                             MeshRepository::FormatUnknown);

    endInsertRows();
    return result;
}

// --------------------------------------------------------------------------

bool MyMeshesModel::addBlenderMonkey()
{
    QResource monkey(":/Resources/Monkey.obj");
    QByteArray bytes = monkey.isCompressed() ?
                       qUncompress(monkey.data(), monkey.size()) :
                       QByteArray(reinterpret_cast<const char *>(monkey.data()), monkey.size());
    QTemporaryFile temp("Monkey.XXXXXX");
    if (temp.open())
    {
        int position = m_repository->getNumberMeshes();
        beginInsertRows(QModelIndex(), position, position);

        QString filename = temp.fileName();
        temp.write(bytes);
        temp.close();
        m_repository->loadFromFile(filename.toStdString(), MeshRepository::FormatOBJ);
        m_repository->getMesh(position)->setPath("$monkey");

        endInsertRows();

        return true;
    }
    else
        return false;
}

// --------------------------------------------------------------------------

void MyMeshesModel::saveStateToXml(QXmlStreamWriter &writer, const QDir &base)
{
    writer.writeStartElement("meshes");

    for (int i = 0; i < rowCount(); ++i)
    {
        QVariant variant = data(createIndex(i, 1), Qt::DisplayRole);
        if (variant.canConvert<Mesh*>())
        {
            Mesh *m = variant.value<Mesh*>();
            writer.writeStartElement("mesh");
            writer.writeAttribute("visible", QString("%1").arg(m->visible()));

            writer.writeTextElement("name", m->name().c_str());
            writer.writeTextElement("path", base.relativeFilePath(m->path().c_str()));

            Material mat = m->material();
            writer.writeStartElement("material");

            colour colours[] = { mat.ambient, mat.diffuse, mat.specular, mat.emission };
            char elements[][10] = { "ambient", "diffuse", "specular", "emission" };
            for (int i = 0; i < 4; ++i) {
                ostringstream oss;
                oss << colours[i];
                writer.writeTextElement(elements[i], QString::fromStdString(oss.str()));
            }
            writer.writeTextElement("shininess", QString("%1").arg(mat.shininess));
            writer.writeEndElement(); // material

            writer.writeEndElement(); // mesh
        }
    }

    writer.writeEndElement(); // models
}

// --------------------------------------------------------------------------

bool MyMeshesModel::restoreStateFromXml(QXmlStreamReader &reader, const QDir &base,
                                        const QDir &alternate)
{
    if (reader.readNextStartElement())
    {
        if (reader.name() != "meshes") return false;

        // clear out existing meshes first
        removeRows(0, rowCount());

        // read mesh elements and insert them into the repository
        while (reader.readNextStartElement())
        {
            if (reader.name() != "mesh") {
                reader.skipCurrentElement();
                continue;
            }

            // read the elements that describe this mesh
            bool visible = reader.attributes().value("visible").toString().toInt();
            QString name, path;
            Material mat;
            while (reader.readNextStartElement())
            {
                if (reader.name() == "name")
                    name = reader.readElementText();
                else if (reader.name() == "path")
                    path = reader.readElementText();
                else if (reader.name() == "material") {
                    while (reader.readNextStartElement()) {
                        istringstream iss(reader.readElementText().toStdString());
                        if (reader.name() == "ambient")         iss >> mat.ambient;
                        else if (reader.name() == "diffuse")    iss >> mat.diffuse;
                        else if (reader.name() == "specular")   iss >> mat.specular;
                        else if (reader.name() == "emission")   iss >> mat.emission;
                        else if (reader.name() == "shininess")  iss >> mat.shininess;
                        else reader.skipCurrentElement();
                    }
                }
                else
                    reader.skipCurrentElement();
            }

            // then load the mesh into the repository
            if (!path.isEmpty())
            {
                if (path == "$monkey") addBlenderMonkey();
                else {
                    QFileInfo info(base.absoluteFilePath(path));
                    if (!info.exists()) {
                        QString first = info.canonicalFilePath();
                        // try the alternate path
                        info.setFile(alternate.absoluteFilePath(path));
                        if (!info.exists()) {
                            QMessageBox::warning(MyMainWindow::instance(), "Load Mesh",
                                                 "Could not find mesh at path " + first +
                                                 " or " + info.canonicalFilePath());
                            continue;
                        }
                    }
                    if (!addMeshFromPath(info.canonicalFilePath())) {
                        QMessageBox::warning(MyMainWindow::instance(), "Load Mesh",
                                             "Could not load mesh at path " +
                                             info.canonicalFilePath());
                        continue;
                    }
                }

                // set the volume name, visible state, and material
                setData(createIndex(rowCount()-1, 0), name, Qt::EditRole);
                setData(createIndex(rowCount()-1, 0),
                        visible ? Qt::Checked : Qt::Unchecked, Qt::CheckStateRole);
                setData(createIndex(rowCount()-1, 2),
                        QVariant::fromValue(mat), Qt::EditRole);
            }
        }
    }

    return true;
}

// --------------------------------------------------------------------------
