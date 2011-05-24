#ifndef MYHAPTICSGLWIDGET_H
#define MYHAPTICSGLWIDGET_H

#include "Graphics/OpenGL.h"
#include "Graphics/ProxyGeometry.h"
#include "Graphics/VolumeRenderer.h"
#include "Graphics/Mesh.h"
#include "Common/VolumeSampler.h"
#include "Data/VolumeData.h"
#include "Haptics/HapticScene.h"
#include "Haptics/HapticIsosurface.h"
#include "MyHapticDevicesModel.h"

#include <QGLWidget>
#include <QGLShaderProgram>
#include <QGLFramebufferObject>
#include <QList>

class MyHapticsGLWidget : public QGLWidget
{
Q_OBJECT

    MyHapticDevicesModel   *m_devicesModel;
    HapticScene            *m_scene;
    HapticIsosurface       *m_isosurface;
    int                     m_sceneIndex;
    QList<HapticDisplay *>  m_displays;

    int                     m_timerID;

    // keep an instance of our own private test volume data for test-rendering
    VolumeData             *m_data;
    VolumeSampler          *m_sampler;

    // for visual rendering of the test volume
    ProxyGeometry          *m_proxy;
    VolumeRenderer         *m_renderer;
    QGLShaderProgram       *m_distanceShader;
    QGLFramebufferObject   *m_distanceFBO;

    // a widget to hold the visual/haptic rendering settings
    QWidget                *m_settingsWidget;

    // list of polygonal models we need for this renderer
    QStringList             m_modelFiles;
    MeshGLM                *m_drill, *m_burr;

protected:
    // Sets up the OpenGL rendering context, defines display lists, etc.
    // Called once before the first time resizeGL() or paintGL() is called.
    virtual void initializeGL();

    // Renders the OpenGL scene. Called whenever the widget needs to be updated.
    virtual void paintGL();

    // Sets up the OpenGL viewport, projection, etc. Gets called whenever the
    // widget has been resized
    virtual void resizeGL(int width, int height);

    // responses to show/hide events
    virtual void showEvent(QShowEvent *event);
    virtual void hideEvent(QHideEvent *event);

    // a timer drives the continus frame update on this widget
    virtual void timerEvent(QTimerEvent *)      { updateGL(); }

    // queries the haptic surface to see if the mask has changed
    void checkForMaskUpdate();

    // helper functions to draw various items in the scene
    void drawDisplays();
    void drawDevice();
    void drawProxy();
    void drawForces(cml::vector3d F, cml::vector3d tau);

    void renderDistanceBuffer();

    // create a widget for controlling rendering parameters
    void createSettingsWidget();

    // reads a pointshell from resource and puts it into the haptic isosurface
    void loadPointShell(QString location);

    // writes model files stored in resources to a temp directory so they can
    // be read by the mesh loaders
    QString expandModelResources();
    void    removeModelResources(const QString &path);

public:
    explicit MyHapticsGLWidget(const QGLFormat & format, QWidget *parent = 0,
                               const QGLWidget *shareWidget = 0, Qt::WindowFlags f = 0);
    ~MyHapticsGLWidget();
    
    void setModel(MyHapticDevicesModel *model)  { m_devicesModel = model; }

    // returns a pointer to a widget that controls renderer settings
    QWidget *settingsWidget()   { return m_settingsWidget; }

signals:

public slots:
    void setIsoValue(int v);        // v is in range [1,100]
//    void setStiffness(double k);
};

#endif // MYHAPTICSGLWIDGET_H
