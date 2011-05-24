#include "MyMeshDetailsWidget.h"

#include <QtGui>
#include <sstream>
#include <cml/cml.h>

using namespace std;

MyMeshDetailsWidget::MyMeshDetailsWidget(QWidget *parent) :
    QWidget(parent), m_mesh(0)
{
    m_triangles = new QLabel;
    m_lower = new QLabel;
    m_upper = new QLabel;

    QFormLayout *layout = new QFormLayout(this);
    layout->addRow("Faces:", m_triangles);
    layout->addRow("Lower:", m_lower);
    layout->addRow("Upper:", m_upper);
}

void MyMeshDetailsWidget::setMesh(Mesh *m)
{
    m_mesh = m;

    m_triangles->setText(QString("%1").arg(m->numTriangles()));

    ostringstream oss;
//    oss.setf(ios::fixed);
    oss.precision(3);

    oss << m->boundingBox(Mesh::bbLower);
    m_lower->setText(oss.str().c_str());

    oss.str("");
    oss << m->boundingBox(Mesh::bbUpper);
    m_upper->setText(oss.str().c_str());

    update();
}
