#include "MyImageGLWidget.h"
#include <QtGui>
#include <algorithm>
#include <cmath>

using namespace std;

// --------------------------------------------------------------------------

MyImageGLWidget::MyImageGLWidget(const QGLFormat &format, QWidget *parent,
                                 const QGLWidget *shareWidget, Qt::WindowFlags f)
    : MySliceGLWidget(format, parent, shareWidget, f)
{
    setMinimumSize(512, 512);

    m_windowCenterF = 0.5;
    m_windowWidthF  = 1.0;
}

// --------------------------------------------------------------------------

void MyImageGLWidget::initializeGL()
{
    // initialize the base class
    MySliceGLWidget::initializeGL();

    // create the windowing shader
    m_windowingShader = new QGLShaderProgram(this);
    if (!m_windowingShader->addShaderFromSourceFile(QGLShader::Fragment, ":/Shaders/Windowing.frag"))
        QMessageBox::critical(this, "OpenGL Shader Compile Error", m_windowingShader->log());
    if (!m_windowingShader->link())
        QMessageBox::critical(this, "OpenGL Shader Link Error", m_windowingShader->log());
}

// --------------------------------------------------------------------------

void MyImageGLWidget::bindShader()
{
    m_windowingShader->bind();
    m_windowingShader->setUniformValue("image", 0);
    m_windowingShader->setUniformValue("windowWidth", m_windowWidthF);
    m_windowingShader->setUniformValue("windowCenter", m_windowCenterF);
}

void MyImageGLWidget::releaseShader()
{
    m_windowingShader->release();
}

// --------------------------------------------------------------------------

void MyImageGLWidget::mouseMoveEvent(QMouseEvent *event)
{
    int dx = event->x() - m_mouseX;
    int dy = event->y() - m_mouseY;

    // right button (or modifier on mac) controls translation
    if (event->buttons() & Qt::RightButton || event->modifiers() & Qt::MetaModifier)
    {
        float size = min(width(), height());
        m_centerX += dx * 2.f / size / m_zoom;
        m_centerY -= dy * 2.f / size / m_zoom;
        updateGL();
    }
    else if (event->buttons() & Qt::LeftButton)
    {
        // compute a change in window center as percentage of current width
        int dc = dy * windowWidth() / 0x200;
        if (dc == 0 && dy != 0) dc = (dy > 0) ? 1 : -1;
        setWindowCenter(windowCenter() + dc);

        // compute a change in window width as percentage of current width
        int dw = dx * windowWidth() / 0x100;
        if (dw == 0 && dx != 0) dw = (dx > 0) ? 1 : -1;
        setWindowWidth(windowWidth() + dw);
    }

    // update stored mouse position
    m_mouseX = event->x();
    m_mouseY = event->y();
    event->accept();
}

// --------------------------------------------------------------------------

void MyImageGLWidget::setWindowCenter(int wcenter)
{
    bool changed = (wcenter != m_windowCenter);
    m_windowCenter = wcenter;
    m_windowCenterF = float(wcenter - m_minRange) / float(m_maxRange - m_minRange);
    updateGL();

    if (changed) emit windowCenterChanged(m_windowCenter);
}

void MyImageGLWidget::setWindowWidth(int wwidth)
{
    wwidth = max(1, wwidth);
    bool changed = (wwidth != m_windowWidth);
    m_windowWidth = wwidth;
    m_windowWidthF = float(wwidth) / float(m_maxRange - m_minRange);
    updateGL();

    if (changed) emit windowWidthChanged(m_windowWidth);
}

// --------------------------------------------------------------------------

