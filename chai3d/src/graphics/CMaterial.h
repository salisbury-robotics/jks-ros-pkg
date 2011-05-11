//===========================================================================
/*
    This file is part of the CHAI 3D visualization and haptics libraries.
    Copyright (C) 2003-#YEAR# by CHAI 3D. All rights reserved.

    This library is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License("GPL") version 2
    as published by the Free Software Foundation.

    For using the CHAI 3D libraries with software that can not be combined
    with the GNU GPL, and for taking advantage of the additional benefits
    of our support services, please contact CHAI 3D about acquiring a
    Professional Edition License.

    \author:    <http://www.chai3d.org>
    \author:    Francois Conti
    \version    #CHAI_VERSION#
*/
//===========================================================================

//---------------------------------------------------------------------------
#ifndef CMaterialH
#define CMaterialH
//---------------------------------------------------------------------------
#include "graphics/CColor.h"
//---------------------------------------------------------------------------

//===========================================================================
/*!
      \file       CMaterial.h
      \class      cMaterial
      \brief      cMaterial describes the graphic and haptic properties of a
                  solid.
                  
                  Graphic properties include the OpenGL favorites:
                  
                  ambient color, diffuse color, specular color, emissive color, and shininess

                  Haptic properties currently include stiffness, dynamic friction, and static friction
*/
//===========================================================================
struct cMaterial
{
  public:
    // CONSTRUCTOR & DESTRUCTOR:
    //! Constructor of cMaterial.
    cMaterial();

    //! Destructor of cMaterial.
    ~cMaterial() {};

    //! Render the material in OpenGL
    virtual void render();


    // METHODS - GRAPHIC PROPERTIES:

    //! Set shininess (the exponent used for specular lighting)
    void setShininess(GLuint a_shininess);

    //! Get shininess
    GLuint getShininess() { return (m_shininess); }

    //! set transparency level (sets the alpha value for all color properties)
    void setTransparencyLevel(float a_levelTransparency);
    
    //! tells you whether this material includes partial transparency
    inline bool isTransparent() const
    {
        return (m_ambient[4] < 1.0 ||
                m_diffuse[4] < 1.0 ||
                m_specular[4] < 1.0 ||
                m_emission[4]);
    }

    //! Ambient color
    cColorf m_ambient;

    //! Diffuse color
    cColorf m_diffuse;

    //! Specular color
    cColorf m_specular;

    //! Emissive color
    cColorf m_emission;

    //! For debugging: prints the colors contained in this material
    void print() const;



    // METHODS - HAPTIC PROPERTIES:

    //! set stiffness level [N/m]
    void setStiffness(double a_stiffness);

    //! get stiffness level [N/m]
    inline double getStiffness() const { return (m_stiffness); }

    //! set static friction level
    void setStaticFriction(double a_friction);

    //! get static friction level
    inline double getStaticFriction() const { return (m_static_friction); }

    //! set dynamic friction level
    void setDynamicFriction(double a_friction);

    //! get dynamic friction level
    inline double getDynamicFriction() const { return (m_dynamic_friction); }

    //! set level of viscosity
    void setViscosity(double a_viscosity);

    //! get level of viscosity
    inline double getViscosity() { return (m_viscosity); }

    //! set vibration frequency [Hz]
    void setVibrationFrequency(double a_vibrationFrequency);

    //! get vibration frequency [Hz]
    inline double getVibrationFrequency() {return (m_vibrationFrequency); }

    //! set vibration amplitude [max N]
    void  setVibrationAmplitude(double a_vibrationAmplitude);

    //! get vibration amplitude [max N]
    inline double getVibrationAmplitude() {return (m_vibrationAmplitude); }

    //! set the maximum force applied by the magnet [N]
    void setMagnetMaxForce(double a_magnetMaxForce);

    //! get the maximum force applied by the magnet [N]
    inline double getMagnetMaxForce() { return (m_magnetMaxForce); }

    //! set the maximum distance from the object where the force can be perceived [m]
    void setMagnetMaxDistance(double a_magnetMaxDistance);

    //! get the maximum distance from the object where the force can be perceived [m]
    inline double getMagnetMaxDistance() { return (m_magnetMaxDistance); }

    //! set the maximum force threshold for the stick and slip model [N]
    void setStickSlipForceMax(double a_stickSlipForceMax);

    //! get the maximum force threshold for the stick and slip model [N]
    inline double getStickSlipForceMax() { return (m_stickSlipForceMax); }

    //! set the stiffness for the stick and slip model [N/m]
    void setStickSlipStiffness(double a_stickSlipStiffness);

    //! get the stiffness for the stick and slip model [N/m]
    inline double getStickSlipStiffness() { return (m_stickSlipStiffness); }

  protected:
    //! OpenGL shininess
    GLuint m_shininess;

    //! level of viscosity
    double m_viscosity;

    //! Stiffness [netwons per meter]
    double m_stiffness;

    //! Static friction constant
    double m_static_friction;

    //! Dynamic friction constant
    double m_dynamic_friction;

    //! Frequency of vibrations
    double m_vibrationFrequency;

    //! Amplitude of vibrations
    double m_vibrationAmplitude;

    //! Maximum force applied by the magnet
    double m_magnetMaxForce;

    //! Maximum distance from the object where the force can be perceived
    double m_magnetMaxDistance;

    //! Force threshold for stick and slip effect
    double m_stickSlipForceMax;

    //! model stiffnes for stick slip
    double m_stickSlipStiffness;
};

//---------------------------------------------------------------------------
#endif
//---------------------------------------------------------------------------

