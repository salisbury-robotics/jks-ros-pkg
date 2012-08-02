#ifndef MYIMAGEGLWIDGET_H
#define MYIMAGEGLWIDGET_H

#include "Graphics/OpenGL.h"
#include "MySliceGLWidget.h"

class MyImageGLWidget : public MySliceGLWidget
{
Q_OBJECT
Q_PROPERTY(int windowCenter READ windowCenter WRITE setWindowCenter)
Q_PROPERTY(int windowWidth  READ windowWidth  WRITE setWindowWidth )

    QGLShaderProgram   *m_windowingShader;
    int                 m_windowCenter, m_windowWidth;
    float               m_windowCenterF, m_windowWidthF;

protected:
    // Sets up the OpenGL rendering context, defines display lists, etc.
    // Called once before the first time resizeGL() or paintGL() is called.
    virtual void initializeGL();

    // override default move event handler to allow changing window/level
    virtual void mouseMoveEvent(QMouseEvent *event);

    // overrides to insert the window/level shader during paint
    virtual void bindShader();
    virtual void releaseShader();

public:
    explicit MyImageGLWidget(const QGLFormat & format, QWidget *parent = 0,
                             const QGLWidget *shareWidget = 0, Qt::WindowFlags f = 0);

    int windowCenter()  { return m_windowCenter; }
    int windowWidth()   { return m_windowWidth; }

signals:
    void windowCenterChanged(int wcenter);
    void windowWidthChanged(int wwidth);

public slots:
    void setWindowCenter(int wcenter);
    void setWindowWidth(int wwidth);
};

#endif // MYIMAGEGLWIDGET_H
