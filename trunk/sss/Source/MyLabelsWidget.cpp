#include "MyLabelsWidget.h"
#include <QtGui>
#include <QGLWidget>

// --------------------------------------------------------------------------

MyLabelsWidget::MyLabelsWidget(MyVolumesModel *model, QWidget *parent) :
    QWidget(parent)
{
    m_volumesModel = model;
    m_labelsModel = new MyLabelVolumeFilterModel;
    m_labelsModel->setSourceModel(m_volumesModel);

    QBoxLayout *layout = new QHBoxLayout(this);

    // left side of controls
    QBoxLayout *leftSide = new QVBoxLayout;
    QFormLayout *selectors = new QFormLayout;

    m_primaryVolume = new QComboBox;
    m_primaryVolume->setModel(model);
    selectors->addRow("Primary", m_primaryVolume);

    m_labelsVolume = new QComboBox;
    m_labelsVolume->setModel(m_labelsModel);
    selectors->addRow("Labels", m_labelsVolume);

    m_newLabelVolumeButton = new QPushButton("New Labels Volume");
    m_newLabelVolumeButton->setEnabled(false);
    connect(m_newLabelVolumeButton, SIGNAL(clicked()),
            m_labelsModel, SLOT(createLabelsVolume()));
    selectors->addWidget(m_newLabelVolumeButton);
    leftSide->addLayout(selectors);

    m_volumeList = new QListView;
    m_volumeList->setAlternatingRowColors(true);
    m_volumeList->setModel(m_labelsModel);
    leftSide->addWidget(m_volumeList, 2);

    // buttons for generating a label volume
    m_combineButton = new QPushButton("Combine Selected");
    connect(m_combineButton, SIGNAL(clicked()), this, SLOT(combineSelected()));
    leftSide->addWidget(m_combineButton);

    m_postProcessButton = new QPushButton("Post-Process Only");
    connect(m_postProcessButton, SIGNAL(clicked()), this, SLOT(postProcessLabels()));
    leftSide->addWidget(m_postProcessButton);

    layout->addLayout(leftSide, 0);

    m_labelsView = new MyLabelsViewWidget(new MyLabelsGLWidget);
    layout->addWidget(m_labelsView, 1);

    // use a data widget mapper to connect the list view with the label editor
    m_labellingEditor = new MyLabellingEditor;
    leftSide->addWidget(m_labellingEditor, 0);
    QDataWidgetMapper *mapper = new QDataWidgetMapper(this);
    mapper->setModel(m_labelsModel);
    mapper->addMapping(m_labellingEditor, 4);

    // connect the list view selection to the labelling editor
    QItemSelectionModel *sm = m_volumeList->selectionModel();
    connect(sm, SIGNAL(currentChanged(QModelIndex,QModelIndex)),
            mapper, SLOT(setCurrentModelIndex(QModelIndex)));

    // use another data widget mapper to connect the primary selector to the view
    QDataWidgetMapper *primaryMapper = new QDataWidgetMapper(this);
    primaryMapper->setModel(m_volumesModel);
    primaryMapper->addMapping(m_labelsView, 1);
    connect(m_primaryVolume, SIGNAL(currentIndexChanged(int)),
            primaryMapper, SLOT(setCurrentIndex(int)));

    // and finally one last data widget mapper to connect labels selector to view
    QDataWidgetMapper *labelVolumeMapper = new QDataWidgetMapper(this);
    labelVolumeMapper->setModel(m_labelsModel);
    labelVolumeMapper->addMapping(m_labelsView, 1, "labelVolume");
    connect(m_labelsVolume, SIGNAL(currentIndexChanged(int)),
            labelVolumeMapper, SLOT(setCurrentIndex(int)));
    QDataWidgetMapper *labelPropertiesMapper = new QDataWidgetMapper(this);
    labelPropertiesMapper->setModel(m_labelsModel);
    labelPropertiesMapper->addMapping(m_labelsView, 4, "labelProperties");
    connect(m_labelsVolume, SIGNAL(currentIndexChanged(int)),
            labelPropertiesMapper, SLOT(setCurrentIndex(int)));

    // establish a few more connections....
    connect(m_primaryVolume, SIGNAL(currentIndexChanged(int)),
            m_labelsModel, SLOT(setReferenceVolume(int)));
    connect(m_labelsModel, SIGNAL(notifyReferenceIsValid(bool)),
            m_newLabelVolumeButton, SLOT(setEnabled(bool)));
    connect(m_labelsModel, SIGNAL(notifyLabelsCreated(int)),
            m_labelsVolume, SLOT(setCurrentIndex(int)));
    connect(m_labellingEditor, SIGNAL(labellingChanged(Labelling*)),
            this, SLOT(updateLabelling(Labelling*)));
}

// --------------------------------------------------------------------------

void MyLabelsWidget::updateLabelling(Labelling *lab)
{
    // check if the label view widget's current labelling is the same as the
    // one that just changed, and if it is, update it
    if (m_labelsView->labelProperties() == lab)
        m_labelsView->setLabelProperties(lab);
}

void MyLabelsWidget::combineSelected()
{
    m_labelsModel->combineSelected(m_labelsVolume->currentIndex());
}

void MyLabelsWidget::postProcessLabels()
{

}

// --------------------------------------------------------------------------
