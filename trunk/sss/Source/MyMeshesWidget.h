#ifndef MYMESHESWIDGET_H
#define MYMESHESWIDGET_H

#include "MyMeshesModel.h"
#include "MyVolumesModel.h"
#include "MyMeshDetailsWidget.h"
#include "MyMeshGLWidget.h"
#include "MyMaterialEditor.h"

#include <QWidget>
#include <QListView>
#include <QComboBox>
#include <QPushButton>

class MyMeshesWidget : public QWidget
{
Q_OBJECT

    MyMeshesModel       *m_meshesModel;
    MyVolumesModel      *m_volumesModel;
    MyMeshDetailsWidget *m_meshDetails;
    MyMeshGLWidget      *m_meshView;
    MyMaterialEditor    *m_materialEditor;

    QListView           *m_meshList;
    QComboBox           *m_volumeSelector;

    QPushButton         *m_removeButton;
    QPushButton         *m_clearButton;

public:
    explicit MyMeshesWidget(MyMeshesModel *model, MyVolumesModel *volumes = 0,
                            QWidget *parent = 0);

signals:
    void logUpdated(Logger *logger);

public slots:
    void openFiles(QStringList paths = QStringList());
    void setCurrentVolume(int index);

    void removeCurrentMesh();
};

#endif // MYMESHESWIDGET_H
