#include "MySliceGLWidget.h"
#include <QtGui>
#include <algorithm>
#include <cmath>

using namespace std;

// --------------------------------------------------------------------------

MySliceGLWidget::MySliceGLWidget(const QGLFormat &format, QWidget *parent,
                                 const QGLWidget *shareWidget, Qt::WindowFlags f)
    : QGLWidget(format, parent, shareWidget, f)
{
    m_initialized   = false;

    m_minRange      = 0;
    m_maxRange      = 0xff;

    m_zoom          = 1.f;
    m_centerX       = 0.f;
    m_centerY       = 0.f;
}

// --------------------------------------------------------------------------

GLuint MySliceGLWidget::allocateTexture()
{
    GLuint texture;

    // create a 2D texture for storing the image
    glGenTextures(1, &texture);
    glBindTexture(GL_TEXTURE_2D, texture);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);

    return texture;
}

// --------------------------------------------------------------------------

void MySliceGLWidget::initializeGL()
{
    // allocate a single texture first
    m_imageTextures.push_back(allocateTexture());
    m_imageValid.push_back(false);

    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

    unsigned char data[256];
    for (int i = 0; i < 256; ++i) data[i] = i;
    glTexImage2D(GL_TEXTURE_2D, 0, GL_LUMINANCE16, 8, 32, 0,
                 GL_LUMINANCE, GL_UNSIGNED_BYTE, data);

    m_initialized = true;
}

// --------------------------------------------------------------------------

void MySliceGLWidget::paintGL()
{
    glClearColor(.2f, .2f, .2f, 0.f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // enable blending so that images can be overlayed
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    // iterate through and paint our list of images in order
    glEnable(GL_TEXTURE_2D);
    for (int i = 0; i < m_imageTextures.size(); ++i)
    {
        // don't draw invalid images
        if (m_imageValid[i] == false) continue;

        glBindTexture(GL_TEXTURE_2D, m_imageTextures[i]);

        // bind the optional shader before draw
        bindShader(i);

        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();

        glScalef(m_zoom, m_zoom, m_zoom);
        glTranslatef(m_centerX, m_centerY, 0.f);

        glColor3f(1.f, 1.f, 1.f);
        glBegin(GL_QUADS);
            glTexCoord2f(0.f, 1.f); glVertex2f(-1.f, -1.f);
            glTexCoord2f(1.f, 1.f); glVertex2f( 1.f, -1.f);
            glTexCoord2f(1.f, 0.f); glVertex2f( 1.f,  1.f);
            glTexCoord2f(0.f, 0.f); glVertex2f(-1.f,  1.f);
        glEnd();

        // unbind the optional shader after draw
        releaseShader(i);
    }
}

// --------------------------------------------------------------------------

void MySliceGLWidget::resizeGL(int width, int height)
{
    glViewport(0, 0, GLsizei(width), GLsizei(height));

    double aspectw = float(width) / float(height);
    double aspecth = float(height) / float(width);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    if (aspectw > aspecth)
        glOrtho(-aspectw, aspectw, -1.0, 1.0, -1.0, 1.0);
    else
        glOrtho(-1.0, 1.0, -aspecth, aspecth, -1.0, 1.0);

    glMatrixMode(GL_TEXTURE);
    glLoadIdentity();

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
}

// --------------------------------------------------------------------------

void MySliceGLWidget::mousePressEvent(QMouseEvent *event)
{
    m_mouseX = event->x();
    m_mouseY = event->y();
    event->accept();
}

void MySliceGLWidget::mouseMoveEvent(QMouseEvent *event)
{
    int dx = event->x() - m_mouseX;
    int dy = event->y() - m_mouseY;


    // update translation
    float size = min(width(), height());
    m_centerX += dx * 2.f / size / m_zoom;
    m_centerY -= dy * 2.f / size / m_zoom;
    updateGL();

    // update stored mouse position
    m_mouseX = event->x();
    m_mouseY = event->y();
    event->accept();
}

void MySliceGLWidget::wheelEvent(QWheelEvent *event)
{
    float dz = float(event->delta()) / 2880.f;
    m_zoom = m_zoom * exp(dz);
    m_zoom = min(max(0.25f, m_zoom), 25.f);

    updateGL();
}

// --------------------------------------------------------------------------

void MySliceGLWidget::updateImage(int width, int height, GLenum format,
                                  GLenum type, const void *data, int index)
{
    if (!m_initialized || data == 0) return;

    // remember the input data type's range to compute window in [0,1] range
    if (index == 0)
    {
        switch (type) {
        case GL_BYTE:           m_minRange = -0x80;     m_maxRange = 0x7f;      break;
        case GL_UNSIGNED_BYTE:  m_minRange = 0;         m_maxRange = 0xff;      break;
        case GL_SHORT:          m_minRange = -0x8000;   m_maxRange = 0x7fff;    break;
        case GL_UNSIGNED_SHORT: m_minRange = 0;         m_maxRange = 0xffff;    break;
        }
    }

    // update the texture and re-render the image
    makeCurrent();

    // if the index is beyond what we have, allocate more images
    while (index >= m_imageTextures.size()) {
        m_imageTextures.push_back(allocateTexture());
        m_imageValid.push_back(false);
    }

    // store the updated image data
    glPushAttrib(GL_PIXEL_MODE_BIT);

    if (type == GL_BYTE || type == GL_SHORT) {
        glPixelTransferf(GL_RED_SCALE, 0.5f);
        glPixelTransferf(GL_RED_BIAS, 0.5f);
    }

    glBindTexture(GL_TEXTURE_2D, m_imageTextures[index]);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_LUMINANCE16, width, height, 0,
                 format, type, data);

    glPopAttrib();

    m_imageValid[index] = true;

    updateGL();
}

// --------------------------------------------------------------------------

void MySliceGLWidget::invalidateImage(int index)
{
    // if the index is beyond what we have, allocate more images
    while (index >= m_imageValid.size())
        m_imageValid.push_back(false);

    m_imageValid[index] = false;
}

// --------------------------------------------------------------------------
