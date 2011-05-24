#include "MyVolumeHistogramWidget.h"
#include <QtGui>
#include <vector>

using namespace std;

MyVolumeHistogramWidget::MyVolumeHistogramWidget(QWidget *parent) :
    QWidget(parent)
{
    setMinimumWidth(512);
    setMinimumHeight(64);
    setMaximumHeight(128);
}

void MyVolumeHistogramWidget::setHistogram(Histogram *h)
{
    m_histogram = h;
    update();
}

void MyVolumeHistogramWidget::paintEvent(QPaintEvent *event)
{
    if (m_histogram && m_histogram->maxCount > 0)
    {
        float w = width();
        float h = height();

        float dx = w / m_histogram->valueRange();
        float dy = h / m_histogram->maxNonZero;

        // compute the points on the histogram
        vector<float> px, py;
        if (dx > 1.f) {
            float x = 0.f;
            for (int i = m_histogram->minValue; i <= m_histogram->maxValue; ++i, x += dx) {
                px.push_back(x);
                py.push_back(sqrt(float(1+m_histogram->counts[i-m_histogram->minRange])));
            }
        }
        else {
            float dc = m_histogram->valueRange() / w;
            dy /= dc;
            float c = m_histogram->minValue + dc;
            for (int i = 0, j = m_histogram->minValue; i < w; ++i, c += dc) {
                int sum = 0;
                for (; j < c; ++j)
                    sum += m_histogram->counts[j-m_histogram->minRange];
                px.push_back(i);
                py.push_back(sqrt(float(1+sum)));
            }
        }

        // figure out the appropriate scaling of the histogram
        vector<float>::iterator mx = max_element(py.begin(), py.end());
        float high = 1.f;
        if (mx != py.begin()) high = max(high, *max_element(py.begin(), mx));
        if (++mx != py.end()) high = max(high, *max_element(mx, py.end()));
        float scale = h / high;

        // paint the histogram onto the widget
        QPainterPath path;
        path.moveTo(0, h);

        for (int i = 0; i < px.size(); ++i)
            path.lineTo(px[i], h-1.f - py[i] * scale);

        path.lineTo(w, h);

        QPainter painter(this);
        painter.setPen(QPen(QColor(0, 72, 108), 1));
        painter.setBrush(QColor(32, 128, 192));

        painter.drawPath(path);

        painter.drawText(4, 0, 100, 40, Qt::AlignLeft, QString("%1").arg(m_histogram->minValue));
        painter.drawText(4, 0, w-8, 40, Qt::AlignRight, QString("%1").arg(m_histogram->maxValue));
    }
}
