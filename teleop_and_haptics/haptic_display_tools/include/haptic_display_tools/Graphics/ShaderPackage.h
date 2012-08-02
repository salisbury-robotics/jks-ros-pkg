#ifndef SHADERPACKAGE_H
#define SHADERPACKAGE_H

#include <QGLShaderProgram>

// --------------------------------------------------------------------------
// A shader wrapper class which basically inherits all functionality from
// QGLShaderProgram.  Mainly exists to allow the implementation to be changed.

class ShaderProgram : public QGLShaderProgram
{
public:
};

// --------------------------------------------------------------------------

class ShaderPackage
{
    ShaderProgram m_program;

public:
    ShaderPackage();
};

// --------------------------------------------------------------------------
#endif // SHADERPACKAGE_H
