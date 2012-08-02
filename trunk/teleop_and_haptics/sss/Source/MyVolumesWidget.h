#ifndef MYVOLUMESWIDGET_H
#define MYVOLUMESWIDGET_H

#include "MyVolumesModel.h"
#include "MyVolumeDetailsWidget.h"
#include "MyVolumeImageWidget.h"
#include "MyVolumeHistogramWidget.h"

#include <QWidget>
#include <QListView>
#include <QPushButton>

class MyVolumesWidget : public QWidget
{
Q_OBJECT

    MyVolumesModel          *m_volumesModel;
    MyVolumeDetailsWidget   *m_detailsWidget;
    MyVolumeImageWidget     *m_imageWidget;
    MyVolumeHistogramWidget *m_histogramWidget;
    QListView               *m_listView;

    QPushButton             *m_removeButton;
    QPushButton             *m_clearButton;

public:
    explicit MyVolumesWidget(MyVolumesModel *model, QWidget *parent = 0);

    void hideDummyVolume()  { m_listView->setRowHidden(0, true); }

signals:
    void logUpdated(Logger *logger);

public slots:
    void openDirectory(QString path = QString());
    void openFile(QString path = QString());
    void saveFile(QString path = QString());

    void removeCurrentVolume();
    void clearVolumes();
};

#endif // MYVOLUMESWIDGET_H
