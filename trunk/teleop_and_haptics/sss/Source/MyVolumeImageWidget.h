#ifndef MYVOLUMEIMAGEWIDGET_H
#define MYVOLUMEIMAGEWIDGET_H

#include <QWidget>
#include <QSlider>
#include <QLabel>

#include "Common/Volume.h"
#include "MyImageGLWidget.h"

class MyVolumeImageWidget : public QWidget
{
Q_OBJECT
Q_PROPERTY(Volume *volume   READ volume
                            WRITE setVolume
                            NOTIFY volumeChanged
                            USER true )

protected:

    Volume *m_volume;

    struct State {
        int currentSlice;
        int windowCenter;
        int windowWidth;
    } m_state;

    MyImageGLWidget *m_view;
    QSlider         *m_sliceSlider;
    QLabel          *m_sliceLabel;

    void extractSlice(const Volume *v, int slice, int target = 0);

public:
    explicit MyVolumeImageWidget(MyImageGLWidget *viewWidget = 0, QWidget *parent = 0);

    virtual void setVolume(Volume *v);
    Volume *volume()            { return m_volume; }

signals:
    void volumeChanged(Volume *v);

public slots:
    virtual void setCurrentSlice(int slice);
};

#endif // MYVOLUMEIMAGEWIDGET_H
