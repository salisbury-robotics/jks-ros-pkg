#ifndef OPENGL_H
#define OPENGL_H

// a theoretically platform-independent include for the OpenGL headers

#if defined(_WIN32)
#include <windows.h>
#include <GL/GLee.h>
#elif defined(__APPLE__)
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#else // (UNIX/LINUX) ?
#define GL_GLEXT_PROTOTYPES
#include <GL/gl.h>
#include <GL/glu.h>
#endif

#endif // OPENGL_H
