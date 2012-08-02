#ifndef MYMESHGLWIDGET_H
#define MYMESHGLWIDGET_H

#include "Graphics/OpenGL.h"
#include "Graphics/SphericalCamera.h"
#include <QGLWidget>

class MyMeshGLWidget : public QGLWidget
{
Q_OBJECT

    SphericalCamera *m_camera;
    cml::vector3f    m_bboxLower;
    cml::vector3f    m_bboxUpper;
    bool             m_initialized;

    cml::matrix44f_c m_globalTransform;

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

    // some basic drawing functions
    void drawGrid(float size);
    void drawBBox();

public:
    explicit MyMeshGLWidget(const QGLFormat & format, QWidget *parent = 0,
                            const QGLWidget *shareWidget = 0, Qt::WindowFlags f = 0);
    ~MyMeshGLWidget();

    void setGlobalTransform(const cml::matrix44f_c &m)
        { m_globalTransform = m; updateModels(); }

signals:

public slots:
    void updateModels();
};

#endif // MYMESHGLWIDGET_H
