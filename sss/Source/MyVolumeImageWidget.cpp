#include "MyVolumeImageWidget.h"

#include <QtGui>

// --------------------------------------------------------------------------

MyVolumeImageWidget::MyVolumeImageWidget(MyImageGLWidget *viewWidget, QWidget *parent)
    : QWidget(parent)
{
    QBoxLayout *layout = new QVBoxLayout(this);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->setSpacing(2);

    // The default QGLFormat has the following properties:
    //  - Double buffer: Enabled.
    //  - Depth buffer: Enabled.
    //  - RGBA: Enabled (i.e., color index disabled).
    //  - Alpha channel: Disabled.
    //  - Accumulator buffer: Disabled.
    //  - Stencil buffer: Disabled.
    //  - Stereo: Disabled.
    //  - Direct rendering: Enabled.
    //  - Overlay: Disabled.
    //  - Plane: 0 (i.e., normal plane).
    //  - Multisample buffers: Disabled.
    // These can be changed here in the format variable if necessary.
    QGLFormat format;
    format.setAlpha(true);
    if (viewWidget == 0)    viewWidget = new MyImageGLWidget(format);
    else                    viewWidget->setFormat(format);
    m_view = viewWidget;
    layout->addWidget(m_view, 1);

    // create the control panel
    QBoxLayout *panel = new QHBoxLayout;
    panel->setSpacing(6);
    layout->addLayout(panel, 0);

    m_sliceSlider = new QSlider(Qt::Horizontal);
    connect(m_sliceSlider, SIGNAL(valueChanged(int)), this, SLOT(setCurrentSlice(int)));
    panel->addWidget(m_sliceSlider);

    m_sliceLabel = new QLabel();
    panel->addWidget(m_sliceLabel);

    QSpinBox *wcSpinBox = new QSpinBox;
    wcSpinBox->setRange(-0x8000, 0x7fff);
    connect(wcSpinBox, SIGNAL(valueChanged(int)), m_view, SLOT(setWindowCenter(int)));
    connect(m_view, SIGNAL(windowCenterChanged(int)), wcSpinBox, SLOT(setValue(int)));
    panel->addWidget(new QLabel("WC:"));
    panel->addWidget(wcSpinBox);

    QSpinBox *wwSpinBox = new QSpinBox;
    wwSpinBox->setRange(1, 0x10000);
    connect(wwSpinBox, SIGNAL(valueChanged(int)), m_view, SLOT(setWindowWidth(int)));
    connect(m_view, SIGNAL(windowWidthChanged(int)), wwSpinBox, SLOT(setValue(int)));
    panel->addWidget(new QLabel("WW:"));
    panel->addWidget(wwSpinBox);

}

// --------------------------------------------------------------------------

void MyVolumeImageWidget::setVolume(Volume *v)
{
    m_volume = v;

    if (m_volume)
    {
        m_sliceSlider->setRange(0, m_volume->dimensions[2]-1);

        int slice = m_volume->dimensions[2] / 2;
        setCurrentSlice(slice);

        int lo = m_volume->histogram.minValue;
        int hi = m_volume->histogram.maxValue;
        m_view->setWindowCenter((lo+hi)/2);
        m_view->setWindowWidth(hi-lo);
    }

}

// --------------------------------------------------------------------------

void MyVolumeImageWidget::extractSlice(const Volume *v, int slice, int target)
{
    int width   = v->dimensions[0];
    int height  = v->dimensions[1];

    // update the image OpenGL view
    if (v->format == Volume::pfInt16) {
        const signed short *p = reinterpret_cast<const signed short *>(v->data);
        p += width*height*slice;
        m_view->updateImage(width, height, GL_LUMINANCE, GL_SHORT, p, target);
    }
    else if (v->format == Volume::pfUInt8) {
        const unsigned char *p = reinterpret_cast<const unsigned char *>(v->data);
        p += width*height*slice;
        m_view->updateImage(width, height, GL_LUMINANCE, GL_UNSIGNED_BYTE, p, target);
    }
    else if (v->format == Volume::pfUInt16) {
        const unsigned short *p = reinterpret_cast<const unsigned short *>(v->data);
        p += width*height*slice;
        m_view->updateImage(width, height, GL_LUMINANCE, GL_UNSIGNED_SHORT, p, target);
    }
}

void MyVolumeImageWidget::setCurrentSlice(int slice)
{
    m_state.currentSlice = slice;

    // extract the slide from the volume and send it to the view
    extractSlice(m_volume, slice);

    // update the control panel's user interface
    m_sliceSlider->setValue(slice);
    m_sliceLabel->setText(QString("%1/%2").arg(slice+1).arg(m_volume->dimensions[2]));
}

// --------------------------------------------------------------------------
