#include "MyVolumesWidget.h"
#include <QtGui>

MyVolumesWidget::MyVolumesWidget(MyVolumesModel *model, QWidget *parent) :
    QWidget(parent)
{
    m_volumesModel = model;
    int first = 1; // index of the first "real" volume (0 is a dummy)

    QBoxLayout *layout = new QHBoxLayout(this);

    // construct the left side with volume list and details
    QLayout *leftSide = new QVBoxLayout;

    m_listView = new QListView();
    m_listView->setAlternatingRowColors(true);
    m_listView->setModel(m_volumesModel);
    m_listView->setRowHidden(0, true);
    m_listView->setCurrentIndex(m_volumesModel->index(first, 0));
    leftSide->addWidget(m_listView);

    // some ugly buttons to remove and clear the list
    QLayout *buttons = new QHBoxLayout();
    m_removeButton = new QPushButton("Remove");
    connect(m_removeButton, SIGNAL(clicked()), this, SLOT(removeCurrentVolume()));
    buttons->addWidget(m_removeButton);
    leftSide->addItem(buttons);

    m_detailsWidget = new MyVolumeDetailsWidget();
    leftSide->addWidget(m_detailsWidget);

    layout->addLayout(leftSide, 0);

    // construct the center panel with the image and histogram
    QLayout *center = new QVBoxLayout;

    m_imageWidget = new MyVolumeImageWidget;
    center->addWidget(m_imageWidget);

    m_histogramWidget = new MyVolumeHistogramWidget;
    center->addWidget(m_histogramWidget);

    layout->addLayout(center, 2);


    // set up data mappings
    QDataWidgetMapper *mapper = new QDataWidgetMapper(this);
    mapper->setModel(m_volumesModel);
    mapper->addMapping(m_detailsWidget, 1);
    mapper->addMapping(m_histogramWidget, 2);
    mapper->setCurrentIndex(first);

    // a mapper can only map section 1 to one widget, so we need another...
    QDataWidgetMapper *mapper2 = new QDataWidgetMapper(this);
    mapper2->setModel(m_volumesModel);
    mapper2->addMapping(m_imageWidget, 1);
    mapper2->setCurrentIndex(first);

    // connect the list view to the data mapper
    QItemSelectionModel *sm = m_listView->selectionModel();

    connect(sm, SIGNAL(currentChanged(QModelIndex,QModelIndex)),
            mapper, SLOT(setCurrentModelIndex(QModelIndex)));

    connect(sm, SIGNAL(currentChanged(QModelIndex,QModelIndex)),
            mapper2, SLOT(setCurrentModelIndex(QModelIndex)));
}


void MyVolumesWidget::openDirectory(QString path)
{
    // open a dialog to get a directory if we're not passed one
    if (path.isEmpty())
        path = QFileDialog::getExistingDirectory(this, "Open Directory");

    if (!path.isEmpty())
    {
        // do a quick sanity check by counting directories
        int tally = 0, limit = 20;
        QStringList q("");
        while (!q.isEmpty() && tally < limit) {
            QDir d(path + "/" + q.takeFirst());
            QStringList entries = d.entryList(QDir::Dirs | QDir::NoDotAndDotDot);
            tally += entries.count();
            q.append(entries);
        }
        if (tally >= limit) {
            QMessageBox::StandardButton response =
                QMessageBox::question(this, "Open Directory", path +
                                      " contains a large number of subdirectories."
                                      " Locating volumes may take a long time. Proceed?",
                                      QMessageBox::Yes | QMessageBox::No, QMessageBox::No);
            if (response == QMessageBox::No) return;
        }

        m_volumesModel->addVolumeFromPath(path);
        emit logUpdated(VolumeRepository::instance());
        QDir::setCurrent(path);
    }
}

void MyVolumesWidget::openFile(QString path)
{
    QMessageBox::information(this, "Simulation Message",
                             "This feature has not yet been implemented");
}

void MyVolumesWidget::saveFile(QString path)
{
    QMessageBox::information(this, "Simulation Message",
                             "This feature has not yet been implemented");
}

void MyVolumesWidget::removeCurrentVolume()
{
    m_volumesModel->removeRow(m_listView->currentIndex().row());
}

void MyVolumesWidget::clearVolumes()
{

}
