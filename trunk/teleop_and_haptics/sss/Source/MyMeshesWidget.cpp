#include "MyMeshesWidget.h"
#include <QtGui>
#include <cml/cml.h>

using namespace cml;

// --------------------------------------------------------------------------

MyMeshesWidget::MyMeshesWidget(MyMeshesModel *model, MyVolumesModel *volumes,
                               QWidget *parent) :
    QWidget(parent)
{
    m_meshesModel = model;

    QBoxLayout *layout = new QHBoxLayout(this);

    // left side consists of the table view
    QLayout *leftSide = new QVBoxLayout;

    m_meshList = new QListView();
    m_meshList->setModel(m_meshesModel);
    m_meshList->setAlternatingRowColors(true);
//    m_meshList->setMaximumWidth(220);
    leftSide->addWidget(m_meshList);

    // bunch of buttons to remove elements from the list
    QLayout *buttons = new QHBoxLayout();
    m_removeButton = new QPushButton("Remove");
    connect(m_removeButton, SIGNAL(clicked()), this, SLOT(removeCurrentMesh()));
    buttons->addWidget(m_removeButton);
    leftSide->addItem(buttons);

    m_meshDetails = new MyMeshDetailsWidget();
    leftSide->addWidget(m_meshDetails);

    m_materialEditor = new MyMaterialEditor();
    leftSide->addWidget(m_materialEditor);

    m_volumesModel = volumes;
    if (m_volumesModel)
    {
        leftSide->addWidget(new QLabel("Current Volume (for positioning):"));
        m_volumeSelector = new QComboBox;
        m_volumeSelector->setModel(m_volumesModel);
        leftSide->addWidget(m_volumeSelector);

        connect(m_volumeSelector, SIGNAL(currentIndexChanged(int)),
                this, SLOT(setCurrentVolume(int)));
    }

    layout->addLayout(leftSide);

    // The default QGLFormat has the following properties:
    //  - Double buffer: Enabled.
    //  - Depth buffer: Enabled.
    //  - RGBA: Enabled (i.e., color index disabled).
    //  - Alpha channel: Disabled.
    //  - Accumulator buffer: Disabled.
    //  - Stencil buffer: Disabled.
    //  - Stereo: Disabled.
    //  - Direct rendering: Enabled.
    //  - Overlay: Disabled.
    //  - Plane: 0 (i.e., normal plane).
    //  - Multisample buffers: Disabled.
    // These can be changed here in the format variable if necessary.
    QGLFormat format;
    m_meshView = new MyMeshGLWidget(format);
    layout->addWidget(m_meshView, 1);


    // set up data mappings
    QDataWidgetMapper *mapper = new QDataWidgetMapper(this);
    mapper->setModel(m_meshesModel);
    mapper->addMapping(m_meshDetails, 1);
    mapper->addMapping(m_materialEditor, 2);

    // connect the current item in the list view to update the mapper
    QItemSelectionModel *sm = m_meshList->selectionModel();
    connect(sm, SIGNAL(currentChanged(QModelIndex,QModelIndex)),
            mapper, SLOT(setCurrentModelIndex(QModelIndex)));

    // make sure the models view refreshes itself when stuff changes
    connect(m_meshesModel, SIGNAL(dataChanged(QModelIndex,QModelIndex)),
            m_meshView, SLOT(updateModels()));
    connect(m_meshesModel, SIGNAL(rowsInserted(QModelIndex,int,int)),
            m_meshView, SLOT(updateModels()));
    connect(m_meshesModel, SIGNAL(rowsRemoved(QModelIndex,int,int)),
            m_meshView, SLOT(updateModels()));


    // TODO: this might be a bit of a hack... but it seems to work
    connect(m_materialEditor, SIGNAL(materialChanged(Material)),
            mapper, SLOT(submit()));

}

// --------------------------------------------------------------------------

void MyMeshesWidget::openFiles(QStringList paths)
{
    // open a dialog to get files if not given a path
    if (paths.isEmpty())
        paths = QFileDialog::getOpenFileNames(this, "Open Mesh Objects", "",
                                              "Wavefront Mesh (*.obj)");

    for (QStringList::iterator it = paths.begin(); it != paths.end(); ++it)
    {
        m_meshesModel->addMeshFromPath(*it);
        emit logUpdated(MeshRepository::instance());
    }

    // update the applications current working path
    if (!paths.empty()) {
        QFileInfo info(paths.front());
        QDir::setCurrent(info.path());
    }
}

// --------------------------------------------------------------------------

void MyMeshesWidget::setCurrentVolume(int index)
{
    // bounds check
    if (index < 0) {
        m_meshView->setGlobalTransform(cml::identity_4x4());
    }
    else {
        VolumeRepository *repository = VolumeRepository::instance();
        Volume *v = repository->getVolume(index);

        // set the global transform to the inverse of the volume transform
        // (ie. bring the model geometry back to the 0-1 cube space)
        m_meshView->setGlobalTransform(inverse(v->transform));
    }
}

// --------------------------------------------------------------------------

void MyMeshesWidget::removeCurrentMesh()
{
    m_meshesModel->removeRow(m_meshList->currentIndex().row());
    m_meshView->updateModels();
}

// --------------------------------------------------------------------------
