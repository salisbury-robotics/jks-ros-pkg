#include "MyMeshGLWidget.h"
#include "Graphics/Material.h"
#include "Graphics/MeshRepository.h"
#include <QtGui>
#include <cml/cml.h>

using namespace cml;

// --------------------------------------------------------------------------

MyMeshGLWidget::MyMeshGLWidget(const QGLFormat &format, QWidget *parent,
                               const QGLWidget *shareWidget, Qt::WindowFlags f)
          : QGLWidget(format, parent, shareWidget, f)
{
    setMinimumSize(512, 512);

    // initialize a spherical camera centered at (.5, .5, .5), as the volume
    // object will be a [0,1] cube
    SphericalCamera *sphere = new SphericalCamera(vector3f(.5f, .5f, .5f), 2.f, 2);
    sphere->setRange(0.05f, 5.f);
    sphere->setAzimuth(-90.f);
    sphere->computeViewMatrix();
    m_camera = sphere;

    m_bboxLower = vector3f( 1.f, 1.f, 1.f);
    m_bboxUpper = vector3f(-1.f,-1.f,-1.f);

    m_globalTransform = identity_4x4();

    m_initialized = false;
}

MyMeshGLWidget::~MyMeshGLWidget()
{
    if (m_camera)   delete m_camera;
}

// --------------------------------------------------------------------------

void MyMeshGLWidget::initializeGL()
{
    if (m_initialized) return;
    updateModels();
    m_initialized = true;
}

// --------------------------------------------------------------------------

void MyMeshGLWidget::paintGL()
{
    glClearColor(.2f, .2f, .2f, 0.f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    // pose the camera
    glMultMatrixf(m_camera->viewMatrix().data());

    // draw a scene
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glEnable(GL_NORMALIZE);

    Light l(vector4f(1,-2,3,0));
    l.apply();

    Material m;
    m.apply();

    // apply the global transform and render the models
    glPushMatrix();
    glMultMatrixf(m_globalTransform.data());

    MeshRepository *repo = MeshRepository::instance();
    int n = repo->getNumberMeshes();
    for (int i = 0; i < n; ++i)
    {
        Mesh *m = repo->getMesh(i);
        if (m->visible()) m->render();
    }

    glPopMatrix();

    drawBBox();

    glTranslatef(.5f, .5f, .0f);
    glRotatef(90.f, 1.f, 0.f, 0.f);
    drawGrid(2.f);    
}

// --------------------------------------------------------------------------

void MyMeshGLWidget::resizeGL(int width, int height)
{
    glViewport(0, 0, GLsizei(width), GLsizei(height));

    // set up perspective projection
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    double aspect = double(width) / double(height);
    gluPerspective(60.0, aspect, 0.01, 1000.0);
}

// --------------------------------------------------------------------------

// note: we invert the Y coordinate so that it points up in screen coordinates

void MyMeshGLWidget::mousePressEvent(QMouseEvent *event)
{
    m_camera->mouseDown(event->x(), -event->y());

    event->accept();
}

void MyMeshGLWidget::mouseMoveEvent(QMouseEvent *event)
{
    m_camera->mouseMove(event->x(), -event->y());
    updateGL();

    event->accept();
}

void MyMeshGLWidget::mouseReleaseEvent(QMouseEvent *event)
{
    m_camera->mouseUp(event->x(), -event->y());

    event->accept();
}

void MyMeshGLWidget::wheelEvent(QWheelEvent *event)
{
    m_camera->mouseScroll(event->delta() / 8);
    updateGL();

    event->accept();
}

// --------------------------------------------------------------------------

void MyMeshGLWidget::updateModels()
{
    float f = 1e6;
    vector3f lower( f,  f,  f);
    vector3f upper(-f, -f, -f);

    // compute a bounding box for all the meshes
    MeshRepository *repo = MeshRepository::instance();
    int n = repo->getNumberMeshes();
    for (int i = 0; i < n; ++i)
    {
        Mesh *m = repo->getMesh(i);
        if (m->visible()) {
            lower.minimize(m->boundingBox(Mesh::bbLower));
            upper.maximize(m->boundingBox(Mesh::bbUpper));
        }
    }

    // adjust the bounding box with the global transform
    lower = transform_point(m_globalTransform, lower);
    upper = transform_point(m_globalTransform, upper);

    // re-center the spherical camera based on the bounding box (if changed)
    if (m_bboxLower != lower || m_bboxUpper != upper)
    {
        vector3f center;
        float radius;

        if (upper < lower) {
            center = vector3f(.5f, .5f, .5f);
            radius = 2.f;
        }
        else {
            center = 0.5 * (lower + upper);
            radius = length(upper - lower);
        }

        m_camera->setRange(radius * .2f, radius * 2.f);
        m_camera->setRadius(radius);
        m_camera->setOrigin(center);
        m_camera->computeViewMatrix();

        m_bboxLower = lower;
        m_bboxUpper = upper;
    }

    if (m_initialized) updateGL();
}

// --------------------------------------------------------------------------

void MyMeshGLWidget::drawGrid(float size)
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

void MyMeshGLWidget::drawBBox()
{
    // don't draw if there's no model/bounding box
    if (m_bboxUpper < m_bboxLower) return;

    glPushAttrib(GL_ENABLE_BIT);
    glDisable(GL_LIGHTING);
    glEnable(GL_LINE_SMOOTH);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    const vector3f &a = m_bboxLower, &b = m_bboxUpper;    

    // draw hairlines from each of the lower and upper corners
    glBegin(GL_LINES);

        glColor4f(.3f, .3f, .3f, 1.f);  glVertex3fv(a.data());
        glColor4f(.5f, .0f, .0f, 0.f);  glVertex3f(b[0], a[1], a[2]);

        glColor4f(.3f, .3f, .3f, 1.f);  glVertex3fv(a.data());
        glColor4f(.0f, .5f, .0f, 0.f);  glVertex3f(a[0], b[1], a[2]);

        glColor4f(.3f, .3f, .3f, 1.f);  glVertex3fv(a.data());
        glColor4f(.0f, .0f, .5f, 0.f);  glVertex3f(a[0], a[1], b[2]);

        glColor4f(.7f, .7f, .7f, 1.f);  glVertex3fv(b.data());
        glColor4f(.5f, .0f, .0f, 0.f);  glVertex3f(a[0], b[1], b[2]);

        glColor4f(.7f, .7f, .7f, 1.f);  glVertex3fv(b.data());
        glColor4f(.0f, .5f, .0f, 0.f);  glVertex3f(b[0], a[1], b[2]);

        glColor4f(.7f, .7f, .7f, 1.f);  glVertex3fv(b.data());
        glColor4f(.0f, .0f, .5f, 0.f);  glVertex3f(b[0], b[1], a[2]);

    glEnd();

    // compute a precision for the text coordinate labels
    int precision = 1;
    if (abs(a[index_of_max_abs(a[0], a[1], a[2])]) >= 100.f ||
        abs(b[index_of_max_abs(b[0], b[1], b[2])]) >= 100.f)
        precision = 0;

    // draw text coordinate labels at the corners
    glColor4f(.6f, .6f, .6f, .5f);
    renderText(a[0], a[1], a[2], QString("(%1, %2, %3)")
               .arg(a[0], 0, 'f', precision)
               .arg(a[1], 0, 'f', precision)
               .arg(a[2], 0, 'f', precision));

    glColor4f(1.f, 1.f, 1.f, .5f);
    renderText(b[0], b[1], b[2], QString("(%1, %2, %3)")
               .arg(b[0], 0, 'f', precision)
               .arg(b[1], 0, 'f', precision)
               .arg(b[2], 0, 'f', precision));

    // restore state
    glPopAttrib();
}

// --------------------------------------------------------------------------
