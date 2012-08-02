#ifndef MYLABELSVIEWWIDGET_H
#define MYLABELSVIEWWIDGET_H

#include "MyVolumeImageWidget.h"
#include "MyVolumesModel.h"
#include "MyLabelsGLWidget.h"

class MyLabelsViewWidget : public MyVolumeImageWidget
{
Q_OBJECT
Q_PROPERTY(Volume *labelVolume          READ    labelVolume
                                        WRITE   setLabelVolume)
Q_PROPERTY(Labelling *labelProperties   READ    labelProperties
                                        WRITE   setLabelProperties)
protected:
    Volume      *m_labelVolume;
    Labelling   *m_labelProperties;

public:
    explicit MyLabelsViewWidget(MyImageGLWidget *viewWidget = 0, QWidget *parent = 0);

    Volume *labelVolume()               { return m_labelVolume; }
    Labelling *labelProperties()        { return m_labelProperties; }

    // override the base setVolume, as we need to nullify the label volumes
    virtual void setVolume(Volume *v);

signals:

public slots:
    virtual void setCurrentSlice(int slice);

    virtual void setLabelVolume(Volume *v);
    virtual void setLabelProperties(Labelling *lab);
};

#endif // MYLABELSVIEWWIDGET_H
