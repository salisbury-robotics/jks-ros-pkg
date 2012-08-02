#include "MyLabelsGLWidget.h"
#include <QMessageBox>

MyLabelsGLWidget::MyLabelsGLWidget(const QGLFormat &format, QWidget *parent,
                                   const QGLWidget *shareWidget, Qt::WindowFlags f)
    : MyImageGLWidget(format, parent, shareWidget, f)
{
}

void MyLabelsGLWidget::initializeGL()
{
    // initialize the base class
    MyImageGLWidget::initializeGL();

    // create the colour lookup shader
    m_lookupShader = new QGLShaderProgram(this);
    if (!m_lookupShader->addShaderFromSourceFile(QGLShader::Fragment, ":/Shaders/Labelling.frag"))
        QMessageBox::critical(this, "OpenGL Shader Compile Error", m_lookupShader->log());
    if (!m_lookupShader->link())
        QMessageBox::critical(this, "OpenGL Shader Link Error", m_lookupShader->log());


    // allocate a texture for the colour lookup table
    glGenTextures(1, &m_lookupTexture);
    glBindTexture(GL_TEXTURE_1D, m_lookupTexture);
    glTexParameteri(GL_TEXTURE_1D, GL_TEXTURE_WRAP_S, GL_CLAMP);
    glTexParameteri(GL_TEXTURE_1D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_1D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
}

void MyLabelsGLWidget::bindShader(int index)
{
    if (index == 1)
    {
        // bind the lookup texture to unit 1
        glPushAttrib(GL_TEXTURE_BIT);
        glActiveTexture(GL_TEXTURE1);
        glEnable(GL_TEXTURE_1D);
        glBindTexture(GL_TEXTURE_1D, m_lookupTexture);
        glActiveTexture(GL_TEXTURE0);

        m_lookupShader->bind();
        m_lookupShader->setUniformValue("image", 0);
        m_lookupShader->setUniformValue("lookup", 1);
    }
    else
        MySliceGLWidget::bindShader(index);
}

void MyLabelsGLWidget::releaseShader(int index)
{
    if (index == 1)
    {
        m_lookupShader->release();
        glPopAttrib();
    }
    else
        MySliceGLWidget::releaseShader(index);
}

void MyLabelsGLWidget::updateLookupTable(const std::vector<cml::vector4f> &colours)
{
    if (!colours.empty())
    {
        makeCurrent();
        glBindTexture(GL_TEXTURE_1D, m_lookupTexture);

        // first "clear" the texture contents
        unsigned char data[1024] = { 0 };
        glTexImage1D(GL_TEXTURE_1D, 0, GL_RGBA, 256, 0, GL_RGBA, GL_UNSIGNED_BYTE, data);

        // then upload the new look up table
        glTexSubImage1D(GL_TEXTURE_1D, 0, 1, colours.size(), GL_RGBA, GL_FLOAT, &colours[0]);

        // refresh the view
        updateGL();
    }
}
