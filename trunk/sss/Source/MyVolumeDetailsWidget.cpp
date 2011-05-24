#include "MyVolumeDetailsWidget.h"

#include <QtGui>
#include <sstream>
#include <cml/cml.h>

using namespace std;

MyVolumeDetailsWidget::MyVolumeDetailsWidget(QWidget *parent) :
    QWidget(parent)
{
    m_format = new QLabel;
    m_dimensions = new QLabel;
    m_spacing = new QLabel;

    QFormLayout *layout = new QFormLayout(this);
    layout->addRow("Format:", m_format);
    layout->addRow("Dimensions:", m_dimensions);
    layout->addRow("Spacing:", m_spacing);

    // map pixel formation enumeration to display names
    m_formatNames[Volume::pfUInt8]  = "8-bit unsigned";
    m_formatNames[Volume::pfInt16]  = "16-bit signed";
    m_formatNames[Volume::pfUInt16] = "16-bit unsigned";
}

void MyVolumeDetailsWidget::setVolume(Volume *v)
{
    m_volume = v;

    m_format->setText(m_formatNames[v->format]);

    ostringstream oss;
    oss << v->dimensions;
    m_dimensions->setText(oss.str().c_str());

    oss.str("");
    oss.setf(ios::fixed);
    oss.precision(2);
    oss << v->spacing << " mm";
    m_spacing->setText(oss.str().c_str());

    update();
}
