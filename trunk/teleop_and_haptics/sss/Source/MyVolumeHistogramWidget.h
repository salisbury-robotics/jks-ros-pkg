#ifndef MYVOLUMEHISTOGRAMWIDGET_H
#define MYVOLUMEHISTOGRAMWIDGET_H

#include <QWidget>
#include "Common/Volume.h"

class MyVolumeHistogramWidget : public QWidget
{
Q_OBJECT
Q_PROPERTY(Histogram *histogram READ histogram WRITE setHistogram USER true)

    Histogram *m_histogram;

public:
    explicit MyVolumeHistogramWidget(QWidget *parent = 0);

    void setHistogram(Histogram *h);
    Histogram *histogram() { return m_histogram; }

protected:
    virtual void paintEvent(QPaintEvent *event);

signals:

public slots:

};

#endif // MYVOLUMEHISTOGRAMWIDGET_H
