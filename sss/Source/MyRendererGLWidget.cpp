#include "MyRendererGLWidget.h"
#include "MyVolumesModel.h"
#include "MyProgressReporter.h"
#include "Data/VolumeRepository.h"
#include "Graphics/Material.h"
#include "Graphics/TrackballCamera.h"
#include "Graphics/MeshRepository.h"
#include "Haptics/ModifiedMorrisSurface.h"
#include "Haptics/PointShellIsosurface.h"
#include <QtGui>
#include <chai3d.h>
#include <cml/cml.h>
#include <algorithm>

using namespace cml;

// --------------------------------------------------------------------------

MyRendererGLWidget::MyRendererGLWidget(const QGLFormat &format, QWidget *parent,
                                       const QGLWidget *shareWidget, Qt::WindowFlags f)
          : QGLWidget(format, parent, shareWidget, f)
{
    setMinimumSize(800, 600);

    // initialize a trackball camera centered at (.5, .5, .5), as the volume
    // object will be a [0,1] cube
    TrackballCamera *track = new TrackballCamera(vector3f(.5f, .5f, .5f), 2.f);
    track->orient(y_axis_3D(), z_axis_3D());
    track->resize(width(), -height());

    m_camera = track;

    m_proxy = new ProxyGeometry();
    m_renderer = new VolumeRenderer();
    m_distanceFBO = 0;
    m_frameCount = 0;
    m_fpsEstimate = 0.f;
    m_recording = false;

    m_drawDevice = false;
    m_renderStereo = false;
    m_interocularMultiplier = 1.0;

    // create a scene for haptic rendering
    m_hapticScene = new HapticScene();
    m_hapticSurface = 0;
    m_hapticDisplay = 0;
    m_hapticsThread = MyHapticsThread::instance();
    m_sceneIndex = m_hapticsThread->addScene(m_hapticScene);

    // initialize list of available surgical instruments
    m_modelFiles << "drill.obj" << "drill.mtl" << "drill_te.ppm"
                 << "burr1.obj" << "burr2.obj"
                 << "pointshell_h.obj" << "pointshell_b.obj";
    m_modelFiles << "cane.obj" << "cane.mtl" << "teak2.ppm" << "cane_ps.obj";
    m_modelFiles << "wrench.obj" << "wrench.mtl";
    m_modelFiles << "peg.obj" << "peg.mtl" << "peg_ps.obj" << "peg_ps2.obj";

    m_availableInstruments << "Sphere" << "Drill" << "Cane" << "Wrench" << "Peg";
    m_selectedInstrument = siSphere;
    m_toolDiameter = 4.0;
    m_autoScaleWorkspace = false;
    m_motionScale = 1.0;

    // initialize timing mode variables to default event-driven rendering
    m_timing = tmEventDriven;
    m_timerID = -1;

    // initialize to an identity mesh transform
    m_meshTransform = identity_4x4();

    // restored application saved settings and create initial settings widget
    readSettings();
    createSettingsWidget();
}

MyRendererGLWidget::~MyRendererGLWidget()
{
    if (m_camera)       delete m_camera;
    if (m_proxy)        delete m_proxy;
    if (m_renderer)     delete m_renderer;
}

const int MyRendererGLWidget::k_deltaOptions[4] = { 64, 128, 256, 512 };

// --------------------------------------------------------------------------

void MyRendererGLWidget::setPrimaryVolume(int index)
{
    if (m_renderer)
    {
        makeCurrent();

        VolumeRepository *repository = VolumeRepository::instance();
        Volume *volume = 0, *mask = 0;

        // do bounds check on the index before grabbing the volume and mask
        if (in_range(index, 0, repository->getNumberVolumes()-1))
        {
            volume = repository->getVolume(index);
            mask = repository->getMask(index);
        }

        // set volume and mask on the renderer
        m_renderer->setVolume(volume, VolumeRenderer::vrPrimary);
        m_renderer->setVolume(mask, VolumeRenderer::vrMask);

        // create a haptic isosurface and add it to our haptic scene
        bool active = m_hapticsThread->pause();
        m_hapticScene->clearNodes();
        if (volume == 0) { m_hapticSurface = 0; }
        else {
//            m_hapticSurface = new HapticIsosurface(volume, mask, m_renderer->isosurfaceValue());
//            ModifiedMorrisSurface *surface =
//                    new ModifiedMorrisSurface(volume, mask, m_renderer->isosurfaceValue());
//            if (m_hapticDisplay) surface->setToolRadius(m_hapticDisplay->toolRadius());
            PointShellIsosurface *surface =
                    new PointShellIsosurface(volume, mask, m_renderer->isosurfaceValue());
            m_hapticSurface = surface;
            if (m_hapticDisplay) updateInstrumentPointShell(m_hapticDisplay->toolRadius());
            m_hapticSurface->setGradientDelta(1.0 / k_deltaOptions[m_deltaIndex]);
            m_hapticScene->addNode(m_hapticSurface);
        }
        if (active) m_hapticsThread->resume();

        // pull the positioning transform from the primary volume
        // TODO: add option to toggle this on/off
        if (volume) m_meshTransform = inverse(volume->transform);
        else        m_meshTransform = identity_4x4();

        // we need to reset the tool diameter for this volume's physical dimensions
        setToolDiameter(m_toolDiameter);

        // center the camera at the center of the volume
        // (updateGL is called from within resetCameraOrigin)
        resetCameraOrigin();
    }
}

void MyRendererGLWidget::setSecondaryVolume(int index)
{
    if (m_renderer)
    {
        makeCurrent();

        VolumeRepository *repository = VolumeRepository::instance();
        index = std::max(index, 0);  // use the dummy 0 volume for -1 index
        m_renderer->setVolume(repository->getVolume(index),
                              VolumeRenderer::vrSecondary);
        updateGL();
    }
}

void MyRendererGLWidget::setLabelsVolume(int index)
{
    if (m_renderer)
    {
        makeCurrent();

        VolumeRepository *repository = VolumeRepository::instance();
        index = std::max(index, 0);  // use the dummy 0 volume for -1 index
        m_renderer->setLabels(repository->getVolume(index),
                              repository->getMask(index),
                              repository->getLabelling(index));
        updateGL();

        // set the labelling on the haptic volume too
        ModifiedMorrisSurface *surface = dynamic_cast<ModifiedMorrisSurface *>(m_hapticSurface);
        if (surface)
        {
            // pause the simulation first
            bool active = m_hapticsThread->pause();
            surface->setLabelVolume(repository->getMask(index));
            if (active) m_hapticsThread->resume();
        }
    }
}

// --------------------------------------------------------------------------

void MyRendererGLWidget::setHapticDevice(cGenericHapticDevice *device)
{
    m_hapticsThread->pause();
    m_hapticScene->clearDisplays();

    // if we're setting the display to the null device, disable haptics
    if (device == 0)
    {
        m_hapticDisplay = 0;
        emit hapticsEnabled(false);
    }
    else
    {
        m_hapticDisplay = m_hapticScene->addHapticDisplay(device);
        positionHapticDisplay();
        emit hapticsEnabled(true);
        if (isVisible()) m_hapticsThread->resume();
    }
}

void MyRendererGLWidget::setToolDiameter(double mmDiameter)
{
    // convert the physical diameter into a world unit radius
    double r = (0.5 * mmDiameter) / m_renderer->physicalScale();
    m_toolDiameter = mmDiameter;

    if (m_hapticDisplay) m_hapticDisplay->setToolRadius(r);

    if (m_hapticSurface)
    {
        bool active = m_hapticsThread->pause();

        m_hapticSurface->setToolRadius(r);

        // if it's a pointshell isosurface, then update the tool point shell too
        updateInstrumentPointShell(r);

        if (active) m_hapticsThread->resume();
    }
}

void MyRendererGLWidget::setInstrument(int index)
{
    m_selectedInstrument = SurgicalInstrument(index);
    if (m_hapticDisplay) updateInstrumentPointShell(m_hapticDisplay->toolRadius());
}

// --------------------------------------------------------------------------

void MyRendererGLWidget::setTransferFunction(const TransferFunction &tf,
                                             VolumeRenderer::VolumeRole role)
{
    if (m_renderer) {
        makeCurrent();
        m_renderer->setTransferFunction(tf, role);
        if (m_timing == tmEventDriven) updateGL();
    }
}

void MyRendererGLWidget::setIsosurface(float value, const QColor &color)
{
    // retrieve components from the QColor structure
    qreal r, g, b, a;
    color.getRgbF(&r, &g, &b, &a);    

    m_renderer->setIsosurfaceValue(value);
    m_renderer->setIsosurfaceColor(vector4f(r, g, b, a));

    // set the isosurface value on the haptic surface as well
    if (m_hapticSurface) {
        bool active = m_hapticsThread->pause();
        m_hapticSurface->setIsosurfaceValue(value);
        if (active) m_hapticsThread->resume();
    }

    if (m_timing == tmEventDriven) updateGL();
}

// --------------------------------------------------------------------------

void MyRendererGLWidget::resetMask()
{
    bool active = m_hapticsThread->pause();
    m_renderer->resetMask();
    if (active) m_hapticsThread->resume();

    if (m_timing == tmEventDriven) updateGL();
}

void MyRendererGLWidget::revertMask()
{
    Volume *mask = m_renderer->getMask();

    // if the mask has never been saved, this is the same as a reset
    if (mask->path.empty()) resetMask();

    // otherwise execute a load mask on the saved file
    else loadMask(QString::fromStdString(mask->path));
}

bool MyRendererGLWidget::saveMask(QString path)
{
    Volume *mask = m_renderer->getMask();
    if (mask == 0) return false;

    // if no file name was given, check the mask volume for a file name
    if (path.isEmpty())
    {
        path = QString::fromStdString(mask->path);

        // if the path is still empty, then treat it as a save as operation
        if (path.isEmpty()) return saveMaskAs();
    }

    // get the volumes model to save the mask to the given path
    return MyVolumesModel::writeVolumeXml(mask, path);
}

bool MyRendererGLWidget::saveMaskAs()
{
    // check if the current volume has a mask
    if (m_renderer->getMask() == 0) return false;

    // get a file name for the dissection mask first
    QString file = QFileDialog::getSaveFileName(
            this, "Save Dissection As...", QDir::currentPath() + "/dissection.ssv",
            "Simulation Volume (*.ssv)");

    if (!file.isEmpty()) return saveMask(file);
    return false;
}

bool MyRendererGLWidget::loadMask(QString file)
{

    // check if the current volume has a mask
    Volume *mask = m_renderer->getMask();
    if (mask == 0) return false;

    // get a file name for the dissection mask to load
    if (file.isEmpty()) {
        file = QFileDialog::getOpenFileName(
            this, "Load Dissection...", QDir::currentPath(), "Simulation Volume (*.ssv)");
    }

    if (!file.isEmpty())
    {
        bool active = m_hapticsThread->pause();
        MyVolumesModel::readVolumeXml(mask, file);
        makeCurrent();
        m_renderer->updateMask(0, mask->dimensions[2]-1);
        if (active) m_hapticsThread->resume();
        if (m_timing == tmEventDriven) updateGL();
    }
    return false;
}

// --------------------------------------------------------------------------

void MyRendererGLWidget::createSettingsWidget()
{
    m_settingsWidget = new QGroupBox("Renderer");
    QFormLayout *layout = new QFormLayout(m_settingsWidget);

    // toggle for timer vs event driven rendering
    QCheckBox *timingBox = new QCheckBox();
    connect(timingBox, SIGNAL(toggled(bool)), this, SLOT(setTimerDriven(bool)));
    connect(this, SIGNAL(hapticsEnabled(bool)), timingBox, SLOT(setChecked(bool)));
    connect(this, SIGNAL(hapticsEnabled(bool)), timingBox, SLOT(setDisabled(bool)));
    timingBox->setChecked(m_timing == tmTimerDriven);
    layout->addRow(new QLabel("Timer Driven"), timingBox);

    // gradient central difference delta control
    QComboBox *deltaBox = new QComboBox();
    for (int i = 0; i < 4; ++i)
        deltaBox->addItem(QString("1/%1").arg(k_deltaOptions[i]));
    connect(deltaBox, SIGNAL(currentIndexChanged(int)), this, SLOT(setDeltaInverse(int)));
    deltaBox->setCurrentIndex(m_deltaIndex);
    layout->addRow(new QLabel(QString("Gradient ") + QChar(0x3b4)), deltaBox);

    // render quality (ray casting step size) control
    QLabel *qualityLabel = new QLabel();
    QSlider *qualitySlider = new QSlider(Qt::Horizontal);
    qualitySlider->setRange(50, 500);
    connect(qualitySlider, SIGNAL(valueChanged(int)), qualityLabel, SLOT(setNum(int)));
    connect(qualitySlider, SIGNAL(valueChanged(int)), this, SLOT(setRenderQuality(int)));
    qualitySlider->setValue(m_renderQuality);
    layout->addRow(new QLabel("Quality"), qualityLabel);
    layout->addRow(qualitySlider);

    // toggle for drawing the device itself
    QCheckBox *drawDeviceBox = new QCheckBox();
    drawDeviceBox->setChecked(m_drawDevice);
    connect(drawDeviceBox, SIGNAL(toggled(bool)), this, SLOT(setDrawDevice(bool)));
    layout->addRow(new QLabel("Draw Device"), drawDeviceBox);

    // toggle for stereo rendering
    QCheckBox *stereoBox = new QCheckBox();
    stereoBox->setChecked(m_renderStereo);
    connect(stereoBox, SIGNAL(toggled(bool)), this, SLOT(setRenderStereo(bool)));
    layout->addRow(new QLabel("Anaglyph Stereo"), stereoBox);

    QDoubleSpinBox *iodSpinner = new QDoubleSpinBox();
    iodSpinner->setMinimum(0.2);
    iodSpinner->setMaximum(5.0);
    iodSpinner->setSingleStep(0.2);
    iodSpinner->setValue(m_interocularMultiplier);
    connect(iodSpinner, SIGNAL(valueChanged(double)), this, SLOT(setIODMultiplier(double)));
    layout->addRow(new QLabel("IOD Multiplier"), iodSpinner);
}

void MyRendererGLWidget::readSettings()
{
    QSettings settings;
    settings.beginGroup("renderer");

    m_timing        = TimingMode(settings.value("timingMode", tmEventDriven).toInt());
    m_deltaIndex    = settings.value("deltaIndex", 1).toInt();
    m_renderQuality = settings.value("renderQuality", 100).toInt();

    settings.endGroup();
}

void MyRendererGLWidget::writeSettings()
{
    QSettings settings;
    settings.beginGroup("renderer");

    settings.setValue("timingMode", m_timing);
    settings.setValue("deltaIndex", m_deltaIndex);
    settings.setValue("renderQuality", m_renderQuality);

    settings.endGroup();
}

void MyRendererGLWidget::setRenderQuality(int q)
{
    m_renderQuality = q;
    m_renderer->setRayStep(1.f / q);
    if (m_timing == tmEventDriven) updateGL();
}

void MyRendererGLWidget::setDeltaInverse(int index)
{
    m_deltaIndex = index;
    m_renderer->setGradientDelta(1.f / k_deltaOptions[index]);
    if (m_hapticSurface) m_hapticSurface->setGradientDelta(1.0 / k_deltaOptions[index]);
    if (m_timing == tmEventDriven) updateGL();
}

// --------------------------------------------------------------------------

void MyRendererGLWidget::setTimingMode(TimingMode mode, int targetFPS)
{
    // first kill existing timer
    if (isVisible() && m_timing == tmTimerDriven)
        killTimer(m_timerID);

    // then set new timing mode
    if (mode == tmEventDriven) {
        m_timing = mode;
        m_timerID = -1;
    }
    else if (mode == tmTimerDriven) {
        m_timing = mode;
        m_timerInterval = 1000 / targetFPS;
        if (isVisible())
            m_timerID = startTimer(m_timerInterval);
    }
}

// --------------------------------------------------------------------------

void MyRendererGLWidget::resetCameraOrigin()
{
    if (OrbitingCamera *camera = dynamic_cast<OrbitingCamera *>(m_camera)) {
        vector3f origin = m_renderer->volumeCenter();
        camera->setOrigin(origin);
        camera->computeViewMatrix();
        positionHapticDisplay();
        updateGL();
    }
}

// --------------------------------------------------------------------------

void MyRendererGLWidget::initializeGL()
{
    // create a distance shader for starting/terminating rays
    m_distanceShader = new QGLShaderProgram(this);
    if (!m_distanceShader->addShaderFromSourceFile(QGLShader::Vertex, ":/Shaders/Distance.vert"))
        QMessageBox::critical(this, "OpenGL Shader Compile Error", m_distanceShader->log());
    if (!m_distanceShader->addShaderFromSourceFile(QGLShader::Fragment, ":/Shaders/Distance.frag"))
        QMessageBox::critical(this, "OpenGL Shader Compile Error", m_distanceShader->log());
    if (!m_distanceShader->link())
        QMessageBox::critical(this, "OpenGL Shader Link Error", m_distanceShader->log());

    // create an anaglyph shader for rendering anaglyphic stereo 3D
    m_anaglyphShader = new QGLShaderProgram(this);
    if (!m_anaglyphShader->addShaderFromSourceFile(QGLShader::Fragment, ":/Shaders/Anaglyph.frag"))
        QMessageBox::critical(this, "OpenGL Shader Compile Error", m_anaglyphShader->log());
    if (!m_anaglyphShader->link())
        QMessageBox::critical(this, "OpenGL Shader Link Error", m_anaglyphShader->log());

    // generate textures to store rendered stereo images
    glGenTextures(2, m_stereoTexture);

    m_renderer->initialize();

    // load meshes from their .obj files stored in the executable resources
    QString location = expandModelResources();
    string directory = location.toStdString();

    // drill instrument
    HapticInstrument *hi = new HapticInstrument;
    hi->addMesh(directory + "/drill.obj");
    hi->addMesh(directory + "/burr1.obj", HapticInstrument::mtVisual,
                HapticInstrument::msVariable, HapticInstrument::isPassive);
    hi->addMesh(directory + "/burr2.obj", HapticInstrument::mtVisual,
                HapticInstrument::msVariable, HapticInstrument::isActive);
    hi->addMesh(directory + "/pointshell_h.obj", HapticInstrument::mtPointShell);
    hi->addMesh(directory + "/pointshell_b.obj", HapticInstrument::mtPointShell,
                HapticInstrument::msVariable);
    m_instruments[siDrill] = hi;

    // cane instrument
    hi = new HapticInstrument;
    hi->addMesh(directory + "/cane.obj", HapticInstrument::mtVisual,
                HapticInstrument::msVariable);
    hi->addMesh(directory + "/cane_ps.obj", HapticInstrument::mtPointShell,
                HapticInstrument::msVariable);
    m_instruments[siCane] = hi;

    // wrench instrument
    hi = new HapticInstrument;
    hi->addMesh(directory + "/wrench.obj", HapticInstrument::mtVisual,
                HapticInstrument::msVariable);
    hi->addMesh(directory + "/wrench.obj", HapticInstrument::mtPointShell,
                HapticInstrument::msVariable);
    m_instruments[siWrench] = hi;

    // peg instrument
    hi = new HapticInstrument;
    hi->addMesh(directory + "/peg.obj");
    hi->addMesh(directory + "/peg_ps2.obj", HapticInstrument::mtPointShell);
    m_instruments[siPeg] = hi;

    removeModelResources(location);

    // start the frame rate timer
//    m_frameTimer.start();
}

// --------------------------------------------------------------------------

void MyRendererGLWidget::paintGL()
{
    // first check for and update the mask volume if needed
    checkForMaskUpdate();

    // cache the surgical instrument pose to avoid changes between rendering passes
    if (m_hapticDisplay) {
        matrix_affine_transform(m_instrumentTransform,
                                m_hapticDisplay->proxyOrientation(),
                                m_hapticDisplay->proxyPosition());
        matrix_affine_transform(m_deviceTransform,
                                m_hapticDisplay->toolOrientation(),
                                m_hapticDisplay->toolPosition());
    }

    // render two images if we're doing stereo
    int images = m_renderStereo ? 2 : 1;
    float radius = m_camera->radius();
    m_interocularDistance = m_interocularMultiplier * 0.02 * (radius+1.f) / radius;

    GLint viewport[4];
    glGetIntegerv(GL_VIEWPORT, viewport);

    for (int i = 0; i < images; ++i)
    {
        // clear the frame buffer and render a new scene
        glClearColor(.2f, .2f, .2f, 0.f);
        //glClearColor(1.f, 1.f, 1.f, 0.f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        glEnable(GL_DEPTH_TEST);

        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();

        // position the left/right camera if rendering stereo
        if (m_renderStereo)
            glTranslatef((-.5f+i)*m_interocularDistance, 0.f, 0.f);

        // pose the camera
        glMultMatrixf(m_camera->viewMatrix().data());

        // render polygonal meshes first
        renderMeshes();

        // render corner lines behind the volume
        drawCorners(false, true);

        // then render the volume
        renderDistanceBuffer();
        m_renderer->render(m_proxy, m_distanceFBO->texture());

        // and render the corner lines in front on top
        drawCorners(true, false);

        // finally, read the contents of the framebuffer into the left/right texture
        if (m_renderStereo) {
            glBindTexture(GL_TEXTURE_2D, m_stereoTexture[i]);
            glCopyTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, viewport[0], viewport[1],
                             viewport[2], viewport[3], 0);
        }
    }

    // call the renderStereo method to composite the left and right frames
    if (m_renderStereo) {
        float a = float(viewport[2]) / float(viewport[3]);
        float parallax = m_interocularDistance / (2 * (radius-.2f) * a * 0.577);
        renderAnaglyph(parallax);
    }

    if (m_hapticDisplay) {
        drawSaturation(m_hapticDisplay->forceSaturation(),
                       m_hapticDisplay->torqueSaturation());
    }

    // if we're recording, then save the frame
    if (m_recording && m_recordedFrames.size() < m_recordedFrameLimit)
        m_recordedFrames.append(grabFrameBuffer());

    // estimate the frame rate (every second)
    ++m_frameCount;
    if (m_frameTimer.elapsed() >= 1000) {
        m_fpsEstimate = 1000.f * m_frameCount / m_frameTimer.elapsed();
        m_fpsLabel.setText(QString(" %1 FPS").arg(m_fpsEstimate, 0, 'f', 1));
        m_fpsLabel.update();
        m_frameCount = 0;
        m_frameTimer.start();
    }

    // check for OpenGL errors
    GLenum error = glGetError();
    if (error != GL_NO_ERROR) {
        const GLubyte *str = gluErrorString(error);
    }
}

// --------------------------------------------------------------------------

void MyRendererGLWidget::resizeGL(int width, int height)
{
    glViewport(0, 0, GLsizei(width), GLsizei(height));    

    // set up perspective projection
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    double aspect = double(width) / double(height);
    gluPerspective(60.0, aspect, 0.01, 25.0);

    // set up an FBO for storing ray start/end distances
    if (m_distanceFBO) delete m_distanceFBO;
    m_distanceFBO = new QGLFramebufferObject(width, height, QGLFramebufferObject::Depth,
                                             GL_TEXTURE_2D, GL_RGBA16);

    // if we're using a trackball camera, we need to tell it how big we are
    if (TrackballCamera *camera = dynamic_cast<TrackballCamera *>(m_camera))
        camera->resize(width, -height);

    // indicate the size of the viewport on the FPS label
    m_fpsLabel.setText(QString(" %1x%2").arg(width).arg(height));
    m_fpsLabel.update();
}

// --------------------------------------------------------------------------

// note: we invert the Y coordinate so that it points up in screen coordinates

void MyRendererGLWidget::mousePressEvent(QMouseEvent *event)
{
    if (event->button() == Qt::LeftButton)
        m_camera->mouseDown(event->x(), -event->y());
    else if (event->button() == Qt::RightButton)
        m_mouseLast = event->pos();

    event->accept();
}

void MyRendererGLWidget::mouseMoveEvent(QMouseEvent *event)
{
    // on MacOS, the Meta key modifier means emulating right mouse button...
    if (event->buttons() & Qt::RightButton || event->modifiers() & Qt::MetaModifier) {
        QPoint delta = event->pos() - m_mouseLast;
        m_camera->move(float(delta.x()) / width(), float(-delta.y()) / height(), 0.f);
        m_mouseLast = event->pos();
    }
    else if (event->buttons() & Qt::LeftButton)
        m_camera->mouseMove(event->x(), -event->y());

    positionHapticDisplay();
    if (m_timing == tmEventDriven) updateGL();

    event->accept();
}

void MyRendererGLWidget::mouseReleaseEvent(QMouseEvent *event)
{
    if (event->button() == Qt::LeftButton)
        m_camera->mouseUp(event->x(), -event->y());

    event->accept();
}

void MyRendererGLWidget::wheelEvent(QWheelEvent *event)
{
    m_camera->mouseScroll(event->delta() / 8);
    positionHapticDisplay();
    if (m_timing == tmEventDriven) updateGL();

    event->accept();
}

// --------------------------------------------------------------------------

void MyRendererGLWidget::showEvent(QShowEvent *event)
{
    // add the renderer's FPS estimate indicator to the main window status bar
    if (QMainWindow *mainWindow = qobject_cast<QMainWindow *>(window())) {
        mainWindow->statusBar()->addWidget(fpsWidget());
        fpsWidget()->show();
        mainWindow->statusBar()->addWidget(m_hapticsThread->hpsWidget());
        m_hapticsThread->hpsWidget()->show();
    }

    if (m_hapticSurface) m_hapticSurface->reset();

    // restart the haptics thread if we have an active haptic display
    m_hapticsThread->selectScene(m_sceneIndex);
    if (m_hapticDisplay) m_hapticsThread->resume();

    // we have to (re)start the rendering timer if mode is timer driven
    if (m_timing == tmTimerDriven)
        m_timerID = startTimer(m_timerInterval);
}

void MyRendererGLWidget::hideEvent(QHideEvent *event)
{
    // pause the haptics thread first
    m_hapticsThread->pause();
    m_hapticScene->clearForces();

    // remove the renderer's FPS estimate indicator to the main window status bar
    if (QMainWindow *mainWindow = qobject_cast<QMainWindow *>(window())) {
        mainWindow->statusBar()->removeWidget(fpsWidget());
        mainWindow->statusBar()->removeWidget(m_hapticsThread->hpsWidget());
    }

    // stop the rendering timer if mode is timer driven
    if (m_timing == tmTimerDriven)
        killTimer(m_timerID);
}

// --------------------------------------------------------------------------

void MyRendererGLWidget::positionHapticDisplay()
{
    if (m_hapticDisplay == 0) return;
    bool active = m_hapticsThread->pause();

    // make the offset to the device workspace center from the camera
    // proportional to the camera's "focal distance"
    float d = 2.f;
    if (OrbitingCamera *camera = dynamic_cast<OrbitingCamera *>(m_camera))
        d = camera->radius();

    // move the center of the workspace out 3/4 of the way to the camera origin
    matrix44f_c vm = m_camera->viewMatrix();
    vector3f t = matrix_get_translation(vm);
    t[2] += 0.75 * d;
    matrix_set_translation(vm, t);

    // set the haptic display's transform and workspace to half the orbit radius
    m_hapticDisplay->setTransform(vm);

    // set the workspace radius appropriately, either according to the zoom
    // level or to a 1:1 physical space mapping
    if (m_autoScaleWorkspace) {
        m_hapticDisplay->setWorkspaceRadius(0.5 * d);
    }
    else {
        double r = m_hapticDisplay->deviceWorkspace() /
                   (m_renderer->physicalScale() * 0.001); // convert mm to m
        m_hapticDisplay->setWorkspaceRadius(r / m_motionScale);
    }

    // reset the haptic isosurface so that moving the camera won't result in
    // a jarring force sent to the user
    if (m_hapticSurface) m_hapticSurface->reset();

    // restart the haptics thread if it was active
    if (active) m_hapticsThread->resume();
}

// --------------------------------------------------------------------------

void MyRendererGLWidget::checkForMaskUpdate()
{
    if (m_hapticSurface && m_hapticSurface->maskAltered())
    {
        vector3i lower, upper;
        m_hapticSurface->maskFetchAndResetRegion(lower, upper);
        m_renderer->updateMask(lower[2], upper[2]);
    }
}

// --------------------------------------------------------------------------

void MyRendererGLWidget::renderMeshes(bool geometryOnly)
{
    glPushMatrix();
    glMultMatrixf(m_meshTransform.data());

    // disable lighting if we're only rendering the geometry for distances
    if (geometryOnly) {
        glDisable(GL_NORMALIZE);
        glDisable(GL_LIGHTING);
    }
    else
    {
        glEnable(GL_NORMALIZE);
        glEnable(GL_LIGHTING);
        glEnable(GL_LIGHT0);

        // make the light source coincident with the camera
        matrix44f_c vm = m_camera->viewMatrix();
        vector3f d = transform_vector(inverse(vm), vector3f(0,0,1));
        Light l(vector4f(d[0], d[1], d[2], 0.f));
        l.apply();
    }

    MeshRepository *repo = MeshRepository::instance();
    int n = repo->getNumberMeshes();
    for (int i = 0; i < n; ++i)
    {
        Mesh *m = repo->getMesh(i);
        if (m->visible()) m->render();
    }

    glPopMatrix();

    // render haptic device avatar(s) here
    if (m_hapticDisplay)
    {
        cml::matrix44d_c transform[2] =
            { m_instrumentTransform, m_deviceTransform };
        int iterations = m_drawDevice ? 2 : 1;
        glPushAttrib(GL_POLYGON_BIT);

        for (int i = 0; i < iterations; ++i)
        {
            if (i) glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
            glPushMatrix();
            glMultMatrixd(transform[i].data());
            switch (m_selectedInstrument) {
            case siSphere:
                drawSphere(m_hapticDisplay->toolRadius());
                break;
            default:
                drawInstrument(m_hapticDisplay->toolRadius(),
                               m_hapticDisplay->buttonState(0));
                break;
            }
            glPopMatrix();
        }

        glPopAttrib();
    }
}

// --------------------------------------------------------------------------

void MyRendererGLWidget::renderDistanceBuffer()
{
    glPushAttrib(GL_ENABLE_BIT | GL_POLYGON_BIT |
                 GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    m_distanceFBO->bind();

    glClearColor(0.f, 1.f, 0.f, 0.f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glEnable(GL_CULL_FACE);

    // bind the distance computation shader (max dist = 1/scale = 10 units)
    m_distanceShader->bind();
    m_distanceShader->setUniformValue("g_scale", 0.04f);

    // write the proxy geometry front faces to the red (start) channel
    glColorMask(GL_TRUE, GL_FALSE, GL_TRUE, GL_TRUE);
    glDepthMask(GL_FALSE);
    glCullFace(GL_BACK);
    m_proxy->render();

    // write other geometry into the green (terminate) channel
    glColorMask(GL_FALSE, GL_TRUE, GL_TRUE, GL_TRUE);
    glDepthMask(GL_TRUE);
    glCullFace(GL_FRONT);
    m_proxy->render();

    // now render the non-volumetric geometry
    glCullFace(GL_BACK);
    renderMeshes(true);

    // restore all settings
    m_distanceShader->release();
    m_distanceFBO->release();
    glPopAttrib();
}

// --------------------------------------------------------------------------

void MyRendererGLWidget::renderAnaglyph(float parallax)
{
    // clear the frame buffer and render a new scene
    glClearColor(.2f, .2f, .2f, 0.f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // save matrices and set to identity
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();

    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();

    glPushAttrib(GL_TEXTURE_BIT);
    GLfloat border[] = { .2f, .2f, .2f, 1.f };

    // bind the stereo images as textures
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, m_stereoTexture[0]);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);
    glTexParameterfv(GL_TEXTURE_2D, GL_TEXTURE_BORDER_COLOR, border);

    glActiveTexture(GL_TEXTURE1);
    glBindTexture(GL_TEXTURE_2D, m_stereoTexture[1]);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);
    glTexParameterfv(GL_TEXTURE_2D, GL_TEXTURE_BORDER_COLOR, border);

    // get the current viewport size
    GLfloat viewport[4];
    glGetFloatv(GL_VIEWPORT, viewport);

    // bind the shader and set the uniforms
    m_anaglyphShader->bind();
    m_anaglyphShader->setUniformValue("g_stereoLeft", 0);
    m_anaglyphShader->setUniformValue("g_stereoLeft", 1);
    m_anaglyphShader->setUniformValue("g_parallax", parallax);
    m_anaglyphShader->setUniformValue("g_viewport", viewport[0],
                                                    viewport[1],
                                                    viewport[2],
                                                    viewport[3]);

    // draw a rectangle
    glRectf(-1.f, -1.f, 1.f, 1.f);

    // unbind the shader
    m_anaglyphShader->release();

    glPopAttrib();

    // restore matrices
    glPopMatrix();
    glMatrixMode(GL_MODELVIEW);
    glPopMatrix();
}

// --------------------------------------------------------------------------

void MyRendererGLWidget::renderTexture(GLuint texture)
{
    // save matrices and set to identity
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();

    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();

    // render the texture
    glPushAttrib(GL_ENABLE_BIT);
    glDisable(GL_LIGHTING);
    drawTexture(QRectF(0,0,1,1), texture);
    glPopAttrib();

    // restore matrices
    glPopMatrix();
    glMatrixMode(GL_MODELVIEW);
    glPopMatrix();
}

// --------------------------------------------------------------------------

void MyRendererGLWidget::drawSaturation(float fsat, float tsat)
{
    // figure out the dimensions of the viewport
    GLfloat viewport[4];
    glGetFloatv(GL_VIEWPORT, viewport);
    double h = viewport[3];
    double d = 8.f / viewport[2];

    // save matrices and set to identity
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();

    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    glOrtho(-2.0 - d, 2.0 + d, 0.0, h, -1.0, 1.0);

    // render the two quads
    glPushAttrib(GL_ENABLE_BIT);
    glDisable(GL_LIGHTING);
    glDisable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);

    fsat = std::min(fsat, 1.f);
    tsat = std::min(tsat, 1.f);
    const vector4f fcolour(1.f, .5f, 0.f, .7f);
    const vector4f tcolour(.5f, 1.f, 0.f, .7f);
    const vector4f acolour(0.2f, 0.2f, 1.f, 0.f);
    vector4f fc = fcolour + fsat * acolour;
    vector4f tc = tcolour + tsat * acolour;

    glBegin(GL_QUADS);
        glColor4fv(fcolour.data());
        glVertex2f(1.f, 10.f);      glVertex2f(1.f, 7.f);
        glColor4fv(fc.data());
        glVertex2f(1.f+fsat, 7.f);  glVertex2f(1.f+fsat, 10.f);
        glColor4fv(tcolour.data());
        glVertex2f(1.f, 5.f);      glVertex2f(1.f, 2.f);
        glColor4fv(tc.data());
        glVertex2f(1.f+tsat, 2.f);  glVertex2f(1.f+tsat, 5.f);
    glEnd();

    glPopAttrib();

    // restore matrices
    glPopMatrix();
    glMatrixMode(GL_MODELVIEW);
    glPopMatrix();
}

// --------------------------------------------------------------------------

void MyRendererGLWidget::drawGrid(float size)
{
    glPushAttrib(GL_ENABLE_BIT);
    glDisable(GL_LIGHTING);
    glEnable(GL_LINE_SMOOTH);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    glColor4f(.5f, .75f, .75f, .5f);
    float gspan = 0.5f * size;
    glBegin(GL_LINES);
    for (float f = -gspan; f < 1.05f * gspan; f += 0.5f * gspan) {
        glVertex3f(f, 0.f, -gspan); glVertex3f(f, 0.f, gspan);
        glVertex3f(-gspan, 0.f, f); glVertex3f(gspan, 0.f, f);
    }
    glEnd();
    glPopAttrib();
}

// --------------------------------------------------------------------------

void MyRendererGLWidget::drawSphere(float radius)
{
    static cShapeSphere *sphere = 0;
    if (sphere == 0)
        sphere = new cShapeSphere(radius);
    else
        sphere->setRadius(radius);
    sphere->renderSceneGraph();
}

// --------------------------------------------------------------------------

void MyRendererGLWidget::drawInstrument(float radius, bool active)
{
    // render the model of the surgical instrument
    HapticInstrument *hi = m_instruments[m_selectedInstrument];
    if (hi)
    {
        float fs = 100.f / m_renderer->physicalScale();
        float vs = radius / 0.02;
        hi->render(fs, vs, active);
    }
/*
    glPushAttrib(GL_ENABLE_BIT);

    glPushMatrix();
    float s = 100.f / m_renderer->physicalScale();
    glScalef(s, s, s);
    m_meshes[imDrill]->render();
    glPopMatrix();

    glPushMatrix();
    s = radius / 0.02;
    glScalef(s, s, s);
    if (active) m_meshes[imBurr2]->render();
    else        m_meshes[imBurr1]->render();
    glPopMatrix();

    glPopAttrib();
    */
}

void MyRendererGLWidget::updateInstrumentPointShell(double radius)
{
    PointShellIsosurface *surface = dynamic_cast<PointShellIsosurface *>(m_hapticSurface);
    if (surface == 0) return;
    surface->setToolRadius(radius);

    // use the mesh vertices as a point shell on the haptic isosurface
    HapticInstrument *hi = m_instruments[m_selectedInstrument];
    if (hi)
    {
        double fs = 100.f / m_renderer->physicalScale();
        double vs = radius / 0.02;
        hi->updatePointShell(surface, fs, vs);
    }

/*
    double s = 100.0 / m_renderer->physicalScale();
    MeshGLM *mesh = m_meshes[psDrill];
    surface->setPointShell(mesh->vertexPointer(), mesh->numVertices(), s);

    s = radius / 0.02;
    mesh = m_meshes[psBurr];
    surface->setPointShell(mesh->vertexPointer(), mesh->numVertices(), s, false);
*/
}

// --------------------------------------------------------------------------
// front and back arguments control which lines get drawn, as if the lines
// were painted on the outer (front) faces of a cube

void MyRendererGLWidget::drawCorners(bool front, bool back)
{
    glPushAttrib(GL_ENABLE_BIT);
    glDisable(GL_LIGHTING);
    glEnable(GL_LINE_SMOOTH);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    const vector3f a(0.f, 0.f, 0.f), b(1.f, 1.f, 1.f);

    // retrieve the current modelview matrix
    matrix44f_c modelviewMatrix;
    glGetFloatv(GL_MODELVIEW_MATRIX, modelviewMatrix.data());

    // apply the inverse transform to the original to obtain eye position
    vector3f p = transform_point(modelviewMatrix.inverse(), vector3f(0, 0, 0));

    // these are our basis vectors
    vector3f x, y, z;
    x.cardinal(0);
    y.cardinal(1);
    z.cardinal(2);
    bool face;

    // draw hairlines from each of the lower and upper corners
    glBegin(GL_LINES);

        vector3f v = p - a;

        face = (dot(v, y) < 0.f) || (dot(v, z) < 0.f);
        if (face == front || !face == back) {
            glColor4f(.3f, .3f, .3f, 1.f);  glVertex3fv(a.data());
            glColor4f(.5f, .0f, .0f, 0.f);  glVertex3f(b[0], a[1], a[2]);
        }
        face = (dot(v, z) < 0.f) || (dot(v, x) < 0.f);
        if (face == front || !face == back) {
            glColor4f(.3f, .3f, .3f, 1.f);  glVertex3fv(a.data());
            glColor4f(.0f, .5f, .0f, 0.f);  glVertex3f(a[0], b[1], a[2]);
        }
        face = (dot(v, x) < 0.f) || (dot(v, y) < 0.f);
        if (face == front || !face == back) {
            glColor4f(.3f, .3f, .3f, 1.f);  glVertex3fv(a.data());
            glColor4f(.0f, .0f, .5f, 0.f);  glVertex3f(a[0], a[1], b[2]);
        }

        v = p - b;

        face = (dot(v, y) > 0.f) || (dot(v, z) > 0.f);
        if (face == front || !face == back) {
            glColor4f(.7f, .7f, .7f, 1.f);  glVertex3fv(b.data());
            glColor4f(.5f, .0f, .0f, 0.f);  glVertex3f(a[0], b[1], b[2]);
        }
        face = (dot(v, z) > 0.f) || (dot(v, x) > 0.f);
        if (face == front || !face == back) {
            glColor4f(.7f, .7f, .7f, 1.f);  glVertex3fv(b.data());
            glColor4f(.0f, .5f, .0f, 0.f);  glVertex3f(b[0], a[1], b[2]);
        }
        face = (dot(v, x) > 0.f) || (dot(v, y) > 0.f);
        if (face == front || !face == back) {
            glColor4f(.7f, .7f, .7f, 1.f);  glVertex3fv(b.data());
            glColor4f(.0f, .0f, .5f, 0.f);  glVertex3f(b[0], b[1], a[2]);
        }
    glEnd();

    // restore state
    glPopAttrib();
}

// --------------------------------------------------------------------------

QString MyRendererGLWidget::expandModelResources()
{
    for (QStringList::iterator it = m_modelFiles.begin(); it != m_modelFiles.end(); ++it)
    {
        QResource resource(":/Instruments/" + *it);
        QByteArray bytes = resource.isCompressed() ?
                           qUncompress(resource.data(), resource.size()) :
                           QByteArray(reinterpret_cast<const char *>(resource.data()), resource.size());
        QFile file(QDir::temp().absoluteFilePath(*it));
        if (file.open(QIODevice::WriteOnly))
        {
            file.write(bytes);
            file.close();
        }
    }
    return QDir::temp().absolutePath();
}

void MyRendererGLWidget::removeModelResources(const QString &path)
{
    QDir location(path);
    for (QStringList::iterator it = m_modelFiles.begin(); it != m_modelFiles.end(); ++it)
    {
        QFile::remove(location.absoluteFilePath(*it));
    }
}

// --------------------------------------------------------------------------

void MyRendererGLWidget::saveRecordedFrames(const QString &directory)
{
    int frame = 0;
    if (!directory.isEmpty())
    {
        QDir d(directory);
        MyProgressReporter *progress = MyProgressReporter::instance();
        progress->start("Video Recording", "Saving recorded frames...");

        for (QLinkedList<QImage>::iterator it = m_recordedFrames.begin();
                                           it != m_recordedFrames.end(); ++it)
        {
            QString name = QString("frame%1.png").arg(frame++, 4, 10, QChar('0'));
            it->save(d.filePath(name));
            progress->report(100*frame / m_recordedFrames.size());
        }

        progress->finish();
    }
    m_recordedFrames.clear();
}

// --------------------------------------------------------------------------
