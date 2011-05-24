#include "MyHapticsGLWidget.h"
#include "MyHapticsThread.h"
#include "Graphics/Material.h"
#include "Haptics/ModifiedMorrisSurface.h"
#include "Haptics/PointShellIsosurface.h"
#include <QtGui>
#include <sstream>
#include <chai3d.h>
#include <cml/cml.h>

using namespace std;
using namespace cml;

// --------------------------------------------------------------------------

MyHapticsGLWidget::MyHapticsGLWidget(const QGLFormat &format, QWidget *parent,
                                     const QGLWidget *shareWidget, Qt::WindowFlags f)
    : QGLWidget(format, parent, shareWidget, f)
{
    m_devicesModel = 0;
    m_scene = 0;
    m_isosurface = 0;
    m_data = new VolumeTestData();

    m_proxy = new ProxyGeometry();
    m_renderer = new VolumeRenderer();
    m_distanceFBO = 0;

    // initialize list of available surgical instruments
    m_modelFiles << "drill.obj" << "drill.mtl" << "drill_te.ppm"
                 << "burr1.obj" << "pointshell.obj";
    m_drill = m_burr = 0;

    createSettingsWidget();
}

MyHapticsGLWidget::~MyHapticsGLWidget()
{
    if (m_proxy)    delete m_proxy;
    if (m_renderer) delete m_renderer;
    if (m_data)     delete m_data;
}

// --------------------------------------------------------------------------

void MyHapticsGLWidget::createSettingsWidget()
{
    m_settingsWidget = new QGroupBox("Parameters");
    QFormLayout *layout = new QFormLayout(m_settingsWidget);

    // gradient central difference delta control
//    QComboBox *deltaBox = new QComboBox();
//    for (int i = 0; i < 4; ++i)
//        deltaBox->addItem(QString("1/%1").arg(k_deltaOptions[i]));
//    connect(deltaBox, SIGNAL(currentIndexChanged(int)), this, SLOT(setDeltaInverse(int)));
//    deltaBox->setCurrentIndex(m_deltaIndex);
//    layout->addRow(new QLabel(QString("Gradient ") + QChar(0x3b4)), deltaBox);

    // isosurface value control
    QLabel *isoValueLabel = new QLabel();
    QSlider *isoValueSlider = new QSlider(Qt::Horizontal);
    isoValueSlider->setRange(1, 100);
    connect(isoValueSlider, SIGNAL(valueChanged(int)), isoValueLabel, SLOT(setNum(int)));
    connect(isoValueSlider, SIGNAL(valueChanged(int)), this, SLOT(setIsoValue(int)));
    isoValueSlider->setValue(50);
    layout->addRow(new QLabel("Isosurface"), isoValueLabel);
    layout->addRow(isoValueSlider);

}

// --------------------------------------------------------------------------

void MyHapticsGLWidget::loadPointShell(QString location)
{
    PointShellIsosurface *surface = dynamic_cast<PointShellIsosurface *>(m_isosurface);
    if (surface == 0) return;

    string directory = location.toStdString();
    MeshGLM mesh("pointshell", directory + "/pointshell.obj");

    // use the mesh vertices as a point shell on the haptic isosurface
    surface->setPointShell(mesh.vertexPointer(), mesh.numVertices());
}

// --------------------------------------------------------------------------

void MyHapticsGLWidget::setIsoValue(int v)
{
    double value = double(v) / 100.0;

    if (m_renderer) m_renderer->setIsosurfaceValue(value);
    if (m_isosurface) {
        bool wasPaused = MyHapticsThread::instance()->pause();
        m_isosurface->setIsosurfaceValue(value);
        if (wasPaused) MyHapticsThread::instance()->resume();
    }
}

// --------------------------------------------------------------------------

void MyHapticsGLWidget::initializeGL()
{
    // initialize the scene with our own private test volume instance
    Volume *volume = m_data->getVolume();
    Volume *mask = m_data->getMask();
    m_sampler = new VolumeSampler(volume, mask);

    // create a distance shader for starting/terminating rays
    m_distanceShader = new QGLShaderProgram(this);
    if (!m_distanceShader->addShaderFromSourceFile(QGLShader::Vertex, ":/Shaders/Distance.vert"))
        QMessageBox::critical(this, "OpenGL Shader Compile Error", m_distanceShader->log());
    if (!m_distanceShader->addShaderFromSourceFile(QGLShader::Fragment, ":/Shaders/Distance.frag"))
        QMessageBox::critical(this, "OpenGL Shader Compile Error", m_distanceShader->log());
    if (!m_distanceShader->link())
        QMessageBox::critical(this, "OpenGL Shader Link Error", m_distanceShader->log());

    // set up the visual renderer
    m_renderer->initialize();
    m_renderer->setVolume(volume, VolumeRenderer::vrPrimary);
    m_renderer->setVolume(mask, VolumeRenderer::vrMask);
    m_renderer->setIsosurfaceValue(0.5);
    m_renderer->setIsosurfaceColor(colour(1.0, 1.0, 0.5, 0.5));


    // create a haptics scene with all the haptic displays in it
    m_scene = new HapticScene();
    
    if (m_devicesModel)
    {
        int n = m_devicesModel->rowCount();
        for (int i = 0; i < n; ++i) {
            cGenericHapticDevice *device = 
                m_devicesModel->data(m_devicesModel->index(i, 2))
                    .value<cGenericHapticDevice *>();
            if (device)
            {
                HapticDisplay *display = m_scene->addHapticDisplay(device);
                m_displays.append(display);

                // center the device at (.5, .5, .5) where the volume will be
                matrix44f_c center, orient = identity_4x4();
                matrix_translation(center, -.5f, -.75f, -.5f);
                matrix_set_basis_vectors(orient, x_axis_3D(), -z_axis_3D(), y_axis_3D());
                display->setTransform(orient * center);

                // TODO: increase stiffness when speed problem is solved
                double k = display->stiffness();
//                display->setStiffness(0.25 * k);
            }
        }
    }

    // save obj models to disk so we can load them back
    QString location = expandModelResources();
    string directory = location.toStdString();
    m_drill = new MeshGLM("drill", directory + "/drill.obj",
                          MeshGLM::k_useTexture | MeshGLM::k_useMaterial);
    m_burr = new MeshGLM("burr", directory + "/burr1.obj",
                         MeshGLM::k_useMaterial);

    // add a haptic isosurface to the scene
//    m_isosurface = new HapticIsosurface(volume, mask, 0.5);
    m_isosurface = new PointShellIsosurface(volume, mask, 0.5);
    loadPointShell(location);
    m_scene->addNode(m_isosurface);

    // clean up the temporary model files from the disk
    removeModelResources(location);
    
    // add the scene to the haptics thread and select it for rendering
    MyHapticsThread *hthread = MyHapticsThread::instance();
    m_sceneIndex = hthread->addScene(m_scene);
    hthread->selectScene(m_sceneIndex);

}

// --------------------------------------------------------------------------

void MyHapticsGLWidget::paintGL()
{
    // first check for and update the mask volume if needed
    checkForMaskUpdate();

    glClearColor(.2f, .2f, .2f, 0.f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    // place the camera in the same orientation used to view the volumes
    gluLookAt(0.5,-1.0, 1.0,
              0.5, 0.5, 0.5,
              0.0, 0.0, 1.0);

    // add a light (for the visual representation of the haptic display)
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    Light l(cml::vector4f(1.f, -2.f, 1.f, 0.f));
    l.apply();
    
    // draw the visual representations of the haptic displays
    drawDisplays();

    // render the volume
    renderDistanceBuffer();
    m_renderer->render(m_proxy, m_distanceFBO->texture());

    // now label the drawn displays with their physical coordinates
    glDisable(GL_LIGHTING);
    glDisable(GL_DEPTH_TEST);
    glColor3f(1.f, 0.f, 1.f);

    // old debug junk
//    glBegin(GL_LINES);
//    cml::vector3d p = m_displays[0]->proxyPosition();
//    glVertex3dv(p.data());
//    p += m_isosurface->m_debug;
//    glVertex3dv(p.data());
//    glEnd();

    for (int i = 0; i < m_displays.size(); ++i)
    {
        HapticDisplay *display = m_displays.at(i);
        cml::vector3d p = display->devicePosition();
        ostringstream oss;
        oss.setf(ios::fixed);
        oss.precision(2);
        oss << p * 1000.0 << " (mm)";
        p = display->toolPosition();
        renderText(p[0], p[1], p[2], QString::fromStdString(oss.str()));
    }

}

// --------------------------------------------------------------------------

void MyHapticsGLWidget::resizeGL(int width, int height)
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

}

// --------------------------------------------------------------------------

void MyHapticsGLWidget::checkForMaskUpdate()
{    
    ModifiedMorrisSurface *surface = dynamic_cast<ModifiedMorrisSurface *>(m_isosurface);
    if (surface && surface->maskAltered())
    {
        vector3i lower, upper;
        surface->maskFetchAndResetRegion(lower, upper);
        m_renderer->updateMask(lower[2], upper[2]);
    }
}

// --------------------------------------------------------------------------

void MyHapticsGLWidget::renderDistanceBuffer()
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
    drawDisplays();

    // restore all settings
    m_distanceShader->release();
    m_distanceFBO->release();
    glPopAttrib();
}

// --------------------------------------------------------------------------

void MyHapticsGLWidget::drawDisplays()
{
    for (int i = 0; i < m_displays.size(); ++i)
    {
        // draw the device
        HapticDisplay *display = m_displays.at(i);
        matrix44d_c transform;
        matrix_affine_transform(transform, display->toolOrientation(),
                                           display->toolPosition());
        glPushMatrix();
        glMultMatrixd(transform.data());
        drawDevice();
        glPopMatrix();

        // draw the proxy
        matrix_affine_transform(transform, display->proxyOrientation(),
                                           display->proxyPosition());
        glPushMatrix();
        glMultMatrixd(transform.data());
        drawProxy();
        glPopMatrix();

        // draw the force lines
        glPushMatrix();
        vector3d p = display->proxyPosition();
        glTranslated(p[0], p[1], p[2]);
        drawForces(0.25 * display->lastForce(), display->lastTorque());
        glPopMatrix();
    }
}

void MyHapticsGLWidget::drawDevice()
{
    static cShapeSphere *sphere = 0;
    if (sphere == 0) {
        sphere = new cShapeSphere(0.02);
        sphere->m_material.m_diffuse = CHAI_COLOR_WHITE;
        sphere->setFrameSize(0.1, 0.2);
        sphere->setShowFrame(true);
    }
    glEnable(GL_NORMALIZE);
    sphere->renderSceneGraph();
}

void MyHapticsGLWidget::drawProxy()
{
    // if we are using a point shell isosurface, draw some debug points
    PointShellIsosurface *surface = dynamic_cast<PointShellIsosurface *>(m_isosurface);
    if (surface)
    {
        // render the graphical representation of the surgical instrument
        glPushAttrib(GL_ENABLE_BIT);
        glDisable(GL_CULL_FACE);
        glEnable(GL_DEPTH_TEST);
        m_drill->render();
        m_burr->render();

        // render the pointshell points
        glDisable(GL_LIGHTING);

        std::vector<cml::vector3d> points;
        surface->getContactPoints(points);
        glPointSize(3.f);
        glColor3f(0.f, 1.f, 1.f);
        glBegin(GL_POINTS);
        for (std::vector<cml::vector3d>::iterator it = points.begin(); it != points.end(); ++it)
            glVertex3dv(it->data());
        glEnd();

        surface->getDebugPoints(points);
        glPointSize(1.f);
        glColor3f(.5f, .5f, .5f);
        glBegin(GL_POINTS);
        for (std::vector<cml::vector3d>::iterator it = points.begin(); it != points.end(); ++it)
            glVertex3dv(it->data());
        glEnd();

        glPopAttrib();
    }
    // if we're not using a point shell, just draw a sphere for the proxy
    else
    {
        static cShapeSphere *sphere = 0;
        if (sphere == 0)
            sphere = new cShapeSphere(0.015);
        sphere->renderSceneGraph();
    }
}

void MyHapticsGLWidget::drawForces(cml::vector3d F, cml::vector3d tau)
{
    glPushAttrib(GL_ENABLE_BIT);
    glDisable(GL_LIGHTING);

    // Z-axis is up!!!
    glBegin(GL_LINES);

    glColor3f(0.5f, 0.5f, 0.f);
    glVertex3d(0.0, 0.0, 0.0);
    glVertex3d(F[0], -F[2], F[1]);

    glColor3f(0.f, 0.5f, 0.5f);
    glVertex3d(0.0, 0.0, 0.0);
    glVertex3d(tau[0], -tau[2], tau[1]);

/*
    glColor3f(0.5f, 0.f, 0.f);
    glVertex3d(-.5, 0.0, 0.0);
    glVertex3d(0.5, 0.0, 0.0);
    glColor3f(0.f, 0.5f, 0.f);
    glVertex3d(0.0, -.5, 0.0);
    glVertex3d(0.0, 0.5, 0.0);
    glColor3f(0.f, 0.f, 0.5f);
    glVertex3d(0.0, 0.0, -.5);
    glVertex3d(0.0, 0.0, 0.5);
*/
    glEnd();

    glPopAttrib();
}

// --------------------------------------------------------------------------

QString MyHapticsGLWidget::expandModelResources()
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

void MyHapticsGLWidget::removeModelResources(const QString &path)
{
    QDir location(path);
    for (QStringList::iterator it = m_modelFiles.begin(); it != m_modelFiles.end(); ++it)
    {
        QFile::remove(location.absoluteFilePath(*it));
    }
}

// --------------------------------------------------------------------------

void MyHapticsGLWidget::showEvent(QShowEvent *event)
{
    MyHapticsThread *hthread = MyHapticsThread::instance();

    // add the renderer's FPS estimate indicator to the main window status bar
    if (QMainWindow *mainWindow = qobject_cast<QMainWindow *>(window())) {
        mainWindow->statusBar()->addWidget(hthread->hpsWidget());
        hthread->hpsWidget()->show();
    }

    if (m_isosurface) m_isosurface->reset();

    // switch back to this widget's haptic scene and resume
    hthread->selectScene(m_sceneIndex);
    hthread->resume();

    // we have to (re)start the rendering timer if mode is timer driven
    m_timerID = startTimer(30);
}

void MyHapticsGLWidget::hideEvent(QHideEvent *event)
{
    MyHapticsThread *hthread = MyHapticsThread::instance();
    hthread->pause();
    if (m_scene) m_scene->clearForces();

    // remove the renderer's FPS estimate indicator to the main window status bar
    if (QMainWindow *mainWindow = qobject_cast<QMainWindow *>(window()))
        mainWindow->statusBar()->removeWidget(hthread->hpsWidget());

    // stop the rendering timer if mode is timer driven
    killTimer(m_timerID);
}

// --------------------------------------------------------------------------
