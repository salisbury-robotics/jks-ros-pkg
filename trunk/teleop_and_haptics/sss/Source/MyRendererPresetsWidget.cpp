#include "MyRendererPresetsWidget.h"
#include "s_MyRendererTFEditor.h"
#include <QtGui>

const QString MyRendererPresetsWidget::k_bone       = QString("CT Bone Surface");
const QString MyRendererPresetsWidget::k_boneDVR    = QString("CT Bone DVR");
const QString MyRendererPresetsWidget::k_composite  = QString("CT Composite");
const QString MyRendererPresetsWidget::k_new        = QString("(new)");

// --------------------------------------------------------------------------

MyRendererPresetsWidget::MyRendererPresetsWidget(MyRendererTFEditor *editor,
                                                 QWidget *parent)
    : QGroupBox(parent)
{
    m_editor = editor;
    this->setTitle("Presets");

    // load the two application built-in presets first
    loadData(k_bone,      ":/Presets/CTBoneSurface.tf");
    loadData(k_boneDVR,   ":/Presets/CTBoneDVR.tf");
    loadData(k_composite, ":/Presets/CTComposite.tf");

    // load transfer function data from application settings
    readSettings();

    QGridLayout *layout = new QGridLayout(this);

    m_selections = new QComboBox();
    m_selections->addItems(m_data.keys());
    m_selections->addItem(k_new);
    connect(m_selections, SIGNAL(currentIndexChanged(QString)),
            this, SLOT(setSelection(QString)));
    layout->addWidget(m_selections, 0, 0, 1, 2);

    m_removeButton = new QPushButton(tr("Remove"));
    connect(m_removeButton, SIGNAL(clicked()), this, SLOT(removeCurrent()));
    layout->addWidget(m_removeButton, 1, 0);

    m_saveButton = new QPushButton(tr("Save"));
    connect(m_saveButton, SIGNAL(clicked()), this, SLOT(saveCurrent()));
    layout->addWidget(m_saveButton, 1, 1);

    // whenever the transfer function changes, mark as dirty for saving
    connect(m_editor->m_tfe, SIGNAL(isosurfaceChanged(float,QColor)),
            this, SLOT(setCurrentDirty()));
    connect(m_editor->m_tfe, SIGNAL(transferFunctionChanged(TransferFunction)),
            this, SLOT(setCurrentDirty()));

    // initialize everything connected by setting the selection to current
    setSelection(m_selections->currentText());
}

// --------------------------------------------------------------------------

void MyRendererPresetsWidget::populateMenu(QMenu *menu)
{
    QAction *actionImport = menu->addAction(tr("&Import Preset..."));
    connect(actionImport, SIGNAL(triggered()), this, SLOT(importFromFile()));

    QAction *actionExport = menu->addAction(tr("&Export Preset..."));
    connect(actionExport, SIGNAL(triggered()), this, SLOT(exportToFile()));
}

// --------------------------------------------------------------------------

bool MyRendererPresetsWidget::isRemovable(const QString &name)
{
    // anything that is not a reserved name is removable
    if (name == k_bone)         return false;
    if (name == k_boneDVR)      return false;
    if (name == k_composite)    return false;
    if (name == k_new)          return false;
    return true;
}

void MyRendererPresetsWidget::updateButtonStates(bool clean)
{
    QString current = m_selections->currentText();

    // if we're updating from a freshly loaded transfer function, disable save
    if ((clean || !isRemovable(current)) && current != k_new)
        m_saveButton->setEnabled(false);
    else {
        m_saveButton->setEnabled(true);
        if (current == k_new)   m_saveButton->setText("Save As...");
        else                    m_saveButton->setText("Save");
    }

    // do not allow removal of the (new) option or fixed presets
    if (isRemovable(current))
        m_removeButton->setEnabled(true);
    else
        m_removeButton->setEnabled(false);
}

// --------------------------------------------------------------------------

void MyRendererPresetsWidget::readSettings()
{
    QSettings settings;
    settings.beginGroup("presets");

    int size = settings.beginReadArray("data");
    for (int i = 0; i < size; ++i) {
        settings.setArrayIndex(i);
        QString name = settings.value("name").toString();
        m_data[name] = settings.value("value").toByteArray();
    }
    settings.endArray();

    settings.endGroup();
}

void MyRendererPresetsWidget::writeSettings()
{
    QSettings settings;
    settings.beginGroup("presets");

    // don't write the two built-in transfer functions
    settings.beginWriteArray("data", m_data.size() - k_builtInPresets);
    QMapIterator<QString, QByteArray> it(m_data);
    for (int i = 0; it.hasNext(); ) {
        it.next();
        if (isRemovable(it.key())) {
            settings.setArrayIndex(i++);
            settings.setValue("name", it.key());
            settings.setValue("value", it.value());
        }
    }
    settings.endArray();

    settings.endGroup();
}

// --------------------------------------------------------------------------

void MyRendererPresetsWidget::setSelection(const QString &name)
{
    // if we didn't just select (new), look up and load the preset
    if (name != k_new)
    {
        QBuffer buffer(&m_data[name]);
        buffer.open(QBuffer::ReadOnly);
        QDataStream stream(&buffer);
        m_editor->readPreset(stream);
        buffer.close();
    }

    // update button states with "clean" status
    updateButtonStates(true);
}

// --------------------------------------------------------------------------

void MyRendererPresetsWidget::removeCurrent()
{
    QString name = m_selections->currentText();

    // first verify that the selected item is removable
    if (isRemovable(name))
    {
        // delete it from the data collection
        m_data.remove(name);

        // then remove it from the selections list
        m_selections->removeItem(m_selections->currentIndex());

        // and save out the changes
        writeSettings();
    }
}

// --------------------------------------------------------------------------

void MyRendererPresetsWidget::saveCurrent()
{
    QString name = m_selections->currentText();

    // if we're saving a new transfer function, query for a name
    if (name == k_new)
    {
        bool ok;
        name = QInputDialog::getText(this, "Save Preset", "Transfer Function Name:",
                                     QLineEdit::Normal, "My Transfer Function", &ok);
        if (!ok || name.isEmpty()) return;
    }

    // bail if we're trying to save over a reserved transfer function name
    if (!isRemovable(name)) return;

    // get the transfer function data from the editor
    QBuffer buffer;
    buffer.open(QBuffer::WriteOnly);
    QDataStream stream(&buffer);
    m_editor->writePreset(stream);
    buffer.close();

    // if it's already in our collection, just update
    if (m_data.contains(name))
    {
        m_data[name] = buffer.data();

        // this bit catches saving a new function over an old one (same name)
        if (m_selections->currentText() != name) {
            int index = m_selections->findText(name);
            m_selections->setCurrentIndex(index);
        }
    }
    // otherwise add it to the collection and select it
    else
    {
        m_data[name] = buffer.data();

        // add it to the selections combo box and select it (leave new at end)
        int index = m_selections->count() - 1;
        m_selections->insertItem(index, name);
        m_selections->setCurrentIndex(index);
    }

    // save out the changes
    writeSettings();

    // update button states with "clean" status
    updateButtonStates(true);
}

// --------------------------------------------------------------------------

void MyRendererPresetsWidget::loadData(const QString &name, const QString &file)
{
    QFile infile(file);
    if (infile.open(QFile::ReadOnly)) {
        m_data[name] = infile.readAll();
        infile.close();
    }
}

void MyRendererPresetsWidget::importFromFile(QString path)
{
    // open a dialog to get a filename if needed
    if (path.isEmpty()) {
        path = QFileDialog::getOpenFileName(this, "Import Preset", QString(),
                                            "Transfer Functions (*.tf)");
    }

    if (!path.isEmpty())
    {
        QFile infile(path);
        if (infile.open(QFile::ReadOnly))
        {
            if (infile.size() != 0)
            {
                // select the (new) item so as to not clobber existing preset
                int index = m_selections->count() - 1;
                m_selections->setCurrentIndex(index);

                // read from the file into the tranfer function editor
                QDataStream stream(&infile);
                m_editor->readPreset(stream);
                infile.close();
            }
        }
    }
}

void MyRendererPresetsWidget::exportToFile(QString path)
{
    // open a dialog to get a filename if needed
    if (path.isEmpty()) {
        path = QFileDialog::getSaveFileName(this, "Export Preset", "preset.tf",
                                            "Transfer Functions (*.tf)");
    }

    if (!path.isEmpty())
    {
        QFile outfile(path);
        if (outfile.open(QFile::WriteOnly))
        {
            QDataStream stream(&outfile);
            m_editor->writePreset(stream);
            outfile.close();
        }
    }
}

// --------------------------------------------------------------------------
