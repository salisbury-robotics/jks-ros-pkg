#ifndef VOLUMERENDERER_H
#define VOLUMERENDERER_H

#include "Common/Volume.h"
#include "Common/Labelling.h"
#include "ProxyGeometry.h"
#include "TransferFunction.h"
#include <cml/mathlib/typedef.h>
#include <QGLShaderProgram>

typedef QGLShaderProgram ShaderProgram;

class VolumeRenderer
{
public:
    enum VolumeRole { vrPrimary, vrSecondary, vrLabels, vrMask, vrSentinel };

protected:
    bool                m_initialized;
    ShaderProgram      *m_rayCastShader;

    // store different shaders that can be used... (TODO: unify these?)
    enum ShaderMode { smRayCast, smRayCastLabels, smSentinel };
    ShaderProgram      *m_shaders[smSentinel];

    // volume properties
    static const int    k_volumes = 4;
    Volume             *m_volumes[k_volumes];
    GLuint              m_volumeTextures[k_volumes];

    // the scale factor between world space 0-1 and volume texture space
    cml::vector3f       m_textureScale;

    // the scale factor between world space 0-1 and physical space (in mm)
    float               m_physicalScale;

    // coordinates of the center of the volume (nominally .5, .5, .5)
    cml::vector3f       m_volumeCenter;

    // transfer function properties
    static const int    k_tfTextureSize = 4096;
    TransferFunction    m_transferFunctions[k_volumes];
    GLuint              m_transferTextures[k_volumes];
    unsigned char       m_transferBytes[k_tfTextureSize*4];

    // isosurface rendering properties
    float               m_isosurfaceValue;
    cml::vector4f       m_isosurfaceColor;

    // renderer parameters
    float               m_rayStep;
    float               m_gradientDelta;

    // helper functions
    cml::vector3f       currentViewPosition();
    void                uploadVolumeTexture(VolumeRole vr, int z0 = -1, int z1 = -1);
    void                uploadTransferFunction(VolumeRole vr);

public:
    VolumeRenderer();
    virtual ~VolumeRenderer();

    virtual void initialize();
    virtual void render(ProxyGeometry *proxy, GLuint distanceTexture);

    // re-uploads part of the mask volume between z0 and z1 to reflect changes
    virtual void updateMask(int z0, int z1) { uploadVolumeTexture(vrMask, z0, z1); }
    virtual void resetMask();
    Volume      *getMask()                  { return m_volumes[vrMask]; }

    virtual void setVolume(Volume *v, VolumeRole role = vrPrimary)
    {
        m_volumes[role] = v;
        if (m_initialized) uploadVolumeTexture(role);
    }

    // need a special case for the labels volume, the way it's done now
    // TODO: can we refactor to fix this?
    virtual void setLabels(Volume *v, Volume *m, Labelling *lab);

    virtual void setTransferFunction(const TransferFunction &tf,
                                     VolumeRole role = vrPrimary)
    {
        m_transferFunctions[role] = tf;
        if (m_initialized) uploadTransferFunction(role);
    }

    // accessors for isosurface parameters
    float           isosurfaceValue()   { return m_isosurfaceValue; }
    cml::vector4f   isosurfaceColor()   { return m_isosurfaceColor; }

    // mutators for isosurface parameters
    void setIsosurfaceValue(float value)
        { m_isosurfaceValue = value; }
    void setIsosurfaceColor(const cml::vector4f &color)
        { m_isosurfaceColor = color; }

    // mutators for renderer parameters
    void setRayStep(float step)         { m_rayStep = step; }
    void setGradientDelta(float delta)  { m_gradientDelta = delta; }

    // accessor for scale factor from world to physical size (in mm)
    float physicalScale() const         { return m_physicalScale; }

    // accessor for the center point of the current volume
    cml::vector3f volumeCenter() const  { return m_volumeCenter; }
};

#endif // VOLUMERENDERER_H
