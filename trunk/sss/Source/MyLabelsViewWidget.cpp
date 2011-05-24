#include "MyLabelsViewWidget.h"
#include "MyLabelsGLWidget.h"

MyLabelsViewWidget::MyLabelsViewWidget(MyImageGLWidget *viewWidget, QWidget *parent) :
    MyVolumeImageWidget(viewWidget, parent)
{
}

void MyLabelsViewWidget::setLabelVolume(Volume *v)
{
    m_labelVolume = v;

    // ensure that the volume has the same dimensions as the primary volume?

    // refresh image view
    setCurrentSlice(m_state.currentSlice);
}

void MyLabelsViewWidget::setLabelProperties(Labelling *lab)
{
    m_labelProperties = lab;

    // update the lookup table on the OpenGL widget
    if (m_labelProperties) {
        MyLabelsGLWidget *view = qobject_cast<MyLabelsGLWidget *>(m_view);
        if (view) view->updateLookupTable(m_labelProperties->colourLUT);
    }    
}

void MyLabelsViewWidget::setVolume(Volume *v)
{
    // invalidate the labels volume -- it probably belongs to some other volume
    m_labelVolume = 0;
    m_labelProperties = 0;
    m_view->invalidateImage(1);

    MyVolumeImageWidget::setVolume(v);
}

void MyLabelsViewWidget::setCurrentSlice(int slice)
{
    MyVolumeImageWidget::setCurrentSlice(slice);

    // extract the 2nd (label overlay) image from the label volume, or
    // invalidate the label volume if the label volume is set to null
    if (m_labelVolume)  extractSlice(m_labelVolume, slice, 1);
    else                m_view->invalidateImage(1);
}
