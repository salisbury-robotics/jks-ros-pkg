#ifndef MYSLICEGLWIDGET_H
#define MYSLICEGLWIDGET_H

#include "Graphics/OpenGL.h"
#include <QGLWidget>
#include <QGLShaderProgram>
#include <vector>

class MySliceGLWidget : public QGLWidget
{
Q_OBJECT

protected:

    // stored images and the state
    std::vector<GLuint> m_imageTextures;
    std::vector<bool>   m_imageValid;
    int                 m_minRange, m_maxRange;
    int                 m_mouseX, m_mouseY;
    bool                m_initialized;

    // positioning of the image
    float               m_zoom;
    float               m_centerX, m_centerY;

    // helper to allocate and initialize an OpenGL texture
    GLuint allocateTexture();

    // virtual functions to provide the option of inserting a shader during render
    virtual void bindShader()               { }
    virtual void releaseShader()            { }

    virtual void bindShader(int index)      { if (index == 0) bindShader(); }
    virtual void releaseShader(int index)   { if (index == 0) releaseShader(); }

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
    virtual void wheelEvent(QWheelEvent *event);

public:
    explicit MySliceGLWidget(const QGLFormat & format, QWidget *parent = 0,
                             const QGLWidget *shareWidget = 0, Qt::WindowFlags f = 0);

    virtual void updateImage(int width, int height, GLenum format, GLenum type,
                             const void *data, int index = 0);

    virtual void invalidateImage(int index = 0);

signals:

public slots:

};

#endif // MYSLICEGLWIDGET_H
