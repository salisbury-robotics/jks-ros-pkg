#ifndef MYLABELSWIDGET_H
#define MYLABELSWIDGET_H

#include "MyVolumesModel.h"
#include "MyLabelVolumeFilterModel.h"
#include "MyLabelsViewWidget.h"
#include "MyLabellingEditor.h"

#include <QWidget>
#include <QComboBox>
#include <QPushButton>
#include <QListView>

class MyLabelsWidget : public QWidget
{
Q_OBJECT

    MyVolumesModel              *m_volumesModel;
    MyLabelVolumeFilterModel    *m_labelsModel;
    MyLabelsViewWidget          *m_labelsView;
    MyLabellingEditor           *m_labellingEditor;
    QListView                   *m_volumeList;

    QComboBox                   *m_primaryVolume;
    QComboBox                   *m_labelsVolume;
    QPushButton                 *m_newLabelVolumeButton;
    QPushButton                 *m_combineButton;
    QPushButton                 *m_postProcessButton;

public:
    explicit MyLabelsWidget(MyVolumesModel *model, QWidget *parent = 0);

signals:

public slots:
    void updateLabelling(Labelling *lab);
    void combineSelected();
    void postProcessLabels();
};

#endif // MYLABELSWIDGET_H
