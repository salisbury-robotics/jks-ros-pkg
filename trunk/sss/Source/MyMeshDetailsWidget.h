#ifndef MYMESHDETAILSWIDGET_H
#define MYMESHDETAILSWIDGET_H

#include <QWidget>
#include <QLabel>
#include "Graphics/Mesh.h"

class MyMeshDetailsWidget : public QWidget
{
Q_OBJECT
Q_PROPERTY(Mesh *mesh READ mesh WRITE setMesh USER true)

    Mesh    *m_mesh;
    QLabel  *m_triangles;
    QLabel  *m_lower;
    QLabel  *m_upper;

public:
    explicit MyMeshDetailsWidget(QWidget *parent = 0);

    void setMesh(Mesh *m);
    Mesh *mesh() const      { return m_mesh; }

signals:

public slots:

};

#endif // MYMESHDETAILSWIDGET_H
