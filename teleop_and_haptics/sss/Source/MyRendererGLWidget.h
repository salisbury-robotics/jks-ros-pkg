#ifndef MYRENDERERGLWIDGET_H
#define MYRENDERERGLWIDGET_H

#include "Graphics/OpenGL.h"
#include "Graphics/Camera.h"
#include "Graphics/ProxyGeometry.h"
#include "Graphics/VolumeRenderer.h"

#include "Haptics/HapticScene.h"
#include "Haptics/HapticIsosurface.h"
#include "Haptics/HapticInstrument.h"
#include "MyHapticsThread.h"

#include <QGLWidget>
#include <QGLShaderProgram>
#include <QGLFramebufferObject>

#include <QLabel>
#include <QTime>
#include <QTimerEvent>
#include <QLinkedList>

class MyRendererGLWidget : public QGLWidget
{
Q_OBJECT

    OrbitingCamera          *m_camera;
    ProxyGeometry           *m_proxy;
    VolumeRenderer          *m_renderer;

    cml::matrix44f_c         m_meshTransform;

    QGLShaderProgram        *m_distanceShader;
    QGLFramebufferObject    *m_distanceFBO;

    QGLShaderProgram        *m_anaglyphShader;
    GLuint                   m_stereoTexture[2];
    bool                     m_renderStereo;
    float                    m_interocularDistance;
    float                    m_interocularMultiplier;

    // a widget to hold the rendering settings
    QWidget                 *m_settingsWidget;
    int                      m_renderQuality;   // inverse of ray step
    int                      m_deltaIndex;      // gradient uses +/- delta
    static const int         k_deltaOptions[4];
    bool                     m_drawDevice;

    // a haptic scene and node to hold the volume isosurface
    HapticScene             *m_hapticScene;
    HapticIsosurface        *m_hapticSurface;
    HapticDisplay           *m_hapticDisplay;
    MyHapticsThread         *m_hapticsThread;
    int                      m_sceneIndex;

    // for keeping track of what surgical instrument we're using
    enum SurgicalInstrument { siSphere = 0, siDrill, siCane, siWrench, siPeg, siSentinel };
    QStringList              m_availableInstruments;
    SurgicalInstrument       m_selectedInstrument;
    cml::matrix44d_c         m_instrumentTransform;
    cml::matrix44d_c         m_deviceTransform;
    double                   m_toolDiameter;
    bool                     m_autoScaleWorkspace;
    double                   m_motionScale;

    // for handling of the .obj models of the surgical instruments
    QStringList              m_modelFiles;
    HapticInstrument        *m_instruments[siSentinel];

    // remembering the mouse position
    QPoint                   m_mouseLast;

    // state variables for estimating frame rate
    QTime                    m_frameTimer;
    int                      m_frameCount;
    float                    m_fpsEstimate;
    QLabel                   m_fpsLabel;

    // saved frames for recording a video
    bool                     m_recording;
    static const int         m_recordedFrameLimit = 1800;
    QLinkedList<QImage>      m_recordedFrames;

    // rendering can be run event-driven (default) or timer-driven
public:
    enum TimingMode { tmEventDriven, tmTimerDriven, tmSentinel };
private:
    TimingMode               m_timing;
    int                      m_timerID;
    int                      m_timerInterval;

protected:
    // Sets up the OpenGL rendering context, defines display lists, etc.
    // Called once before the first time resizeGL() or paintGL() is called.
    virtual void initializeGL();

    // Renders the OpenGL scene. Called whenever the widget needs to be updated.
    virtual void paintGL();

    // Sets up the OpenGL viewport, projection, etc. Gets called whenever the
    // widget has been resized
    virtual void resizeGL(int width, int height);

    // mouse input handling functions
    virtual void mousePressEvent(QMouseEvent *event);
    virtual void mouseMoveEvent(QMouseEvent *event);
    virtual void mouseReleaseEvent(QMouseEvent *event);
    virtual void wheelEvent(QWheelEvent *event);

    // responses to show/hide events
    virtual void showEvent(QShowEvent *event);
    virtual void hideEvent(QHideEvent *event);

    // timer response
    virtual void timerEvent(QTimerEvent *event)
        { if (event->timerId() == m_timerID) updateGL(); }

    // positions the current haptic display in front of the camera
    void positionHapticDisplay();

    // updates the point shell on the virtual instrument
    void updateInstrumentPointShell(double radius);

    // queries the haptic surface to see if the mask has changed
    void checkForMaskUpdate();

    // some basic drawing functions
    void drawSaturation(float fsat, float tsat);
    void drawGrid(float size);
    void drawSphere(float radius);
    void drawInstrument(float radius, bool active = false);
    void drawCorners(bool front = true, bool back = true);

    void renderMeshes(bool geometryOnly = false);
    void renderDistanceBuffer();
    void renderAnaglyph(float parallax);
    void renderTexture(GLuint texture);

    // creates a widget for controlling rendering parameters
    void createSettingsWidget();
    void readSettings();

    // writes model files stored in resources to a temp directory so they can
    // be read by the mesh loaders
    QString expandModelResources();
    void    removeModelResources(const QString &path);

public:
    explicit MyRendererGLWidget(const QGLFormat & format, QWidget *parent = 0,
                                const QGLWidget *shareWidget = 0, Qt::WindowFlags f = 0);
    ~MyRendererGLWidget();

    // sets the timing mode for rendering, and optionally the target frame rate
    void setTimingMode(TimingMode mode, int targetFPS = 30);

    // returns a list of available surgical instrument models
    const QStringList &availableInstruments() const { return m_availableInstruments; }

    // returns a pointer to a widget showing the estimated frame rate
    QWidget *fpsWidget()        { return &m_fpsLabel; }

    // returns a pointer to a widget that controls renderer settings
    QWidget *settingsWidget()   { return m_settingsWidget; }

    // saves current rendering parameters to application QSettings
    void writeSettings();

    // for recording frames
    bool toggleRecording()      { return m_recording = !m_recording; }
    void saveRecordedFrames(const QString &directory);

signals:
    // emitted when haptic interaction is turned on or off
    void hapticsEnabled(bool);

public slots:
    void setPrimaryVolume(int index);
    void setSecondaryVolume(int index);
    void setLabelsVolume(int index);
    void setHapticDevice(cGenericHapticDevice *device);
    void setInstrument(int index);

    // camera control
    void resetCameraOrigin();

    // receiving rendering information from the transfer function editor
    void setTransferFunction(const TransferFunction &tf,
                             VolumeRenderer::VolumeRole role = VolumeRenderer::vrPrimary);
    void setIsosurface(float value, const QColor &color);

    // haptic surface stiffness
    void setStiffness(int k)    { if (m_hapticDisplay) m_hapticDisplay->setStiffness(k); }
    void setTorsion(int k)      { if (m_hapticDisplay) m_hapticDisplay->setTorsionalStiffness(0.001 * double(k)); }
    void setClutch(bool b)      { if (m_hapticDisplay) m_hapticDisplay->setClutch(b); }
    void setToolDiameter(double mmDiameter);    // note diameter in physical units
    void setAutoScaleWorkspace(bool autoScale) {
        m_autoScaleWorkspace = autoScale;
        positionHapticDisplay();
    }
    void setMotionScale(double scale) {
        m_motionScale = scale;
        positionHapticDisplay();
    }

    // receiving information back from the settings widget
    void setRenderQuality(int q);
    void setDeltaInverse(int index);

    void setTimerDriven(bool t) { setTimingMode(t ? tmTimerDriven : tmEventDriven); }

    void setRenderStereo(bool b)    { m_renderStereo = b;
                                      if (m_timing == tmEventDriven) updateGL(); }
    void setIODMultiplier(double m) { m_interocularMultiplier = m;
                                      if (m_timing == tmEventDriven) updateGL(); }
    void setDrawDevice(bool b)      { m_drawDevice = b; }

    // methods handling the dissection mask
    void resetMask();
    void revertMask();
    bool saveMask(QString path = QString());
    bool saveMaskAs();
    bool loadMask(QString file = QString());

};

#endif // MYRENDERERGLWIDGET_H
