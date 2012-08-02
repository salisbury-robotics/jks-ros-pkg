#ifndef MYLABELSGLWIDGET_H
#define MYLABELSGLWIDGET_H

#include "MyImageGLWidget.h"
#include <cml/mathlib/typedef.h>

class MyLabelsGLWidget : public MyImageGLWidget
{
Q_OBJECT

    QGLShaderProgram    *m_lookupShader;
    GLuint               m_lookupTexture;

protected:
    // override initialize function to load shader and allocate texture
    virtual void initializeGL();

    virtual void bindShader(int index);
    virtual void releaseShader(int index);

public:
    explicit MyLabelsGLWidget(const QGLFormat & format = QGLFormat(), QWidget *parent = 0,
                              const QGLWidget *shareWidget = 0, Qt::WindowFlags f = 0);

    void updateLookupTable(const std::vector<cml::vector4f> &colours);

signals:

public slots:

};

#endif // MYLABELSGLWIDGET_H
