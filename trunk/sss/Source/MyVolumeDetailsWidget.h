#ifndef MYVOLUMEDETAILSWIDGET_H
#define MYVOLUMEDETAILSWIDGET_H

#include <QWidget>
#include <QLabel>
#include "Common/Volume.h"

class MyVolumeDetailsWidget : public QWidget
{
Q_OBJECT
Q_PROPERTY(Volume *volume READ volume WRITE setVolume USER true)

    Volume *m_volume;
    QLabel *m_format;
    QLabel *m_dimensions;
    QLabel *m_spacing;

    QString m_formatNames[Volume::pfSentinel];

public:
    explicit MyVolumeDetailsWidget(QWidget *parent = 0);

    void setVolume(Volume *v);
    Volume *volume()            { return m_volume; }


signals:

public slots:

};

#endif // MYVOLUMEDETAILSWIDGET_H
