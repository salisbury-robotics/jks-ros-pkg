//===========================================================================
/*
    Software License Agreement (BSD License)
    Copyright (c) 2003-2012, CHAI3D.
    (www.chai3d.org)

    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:

    * Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.

    * Redistributions in binary form must reproduce the above
    copyright notice, this list of conditions and the following
    disclaimer in the documentation and/or other materials provided
    with the distribution.

    * Neither the name of CHAI3D nor the names of its contributors may
    be used to endorse or promote products derived from this software
    without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
    FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
    COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
    BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
    LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
    ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
    POSSIBILITY OF SUCH DAMAGE. 

    \author    <http://www.chai3d.org>
    \author    Francois Conti
    \version   $MAJOR.$MINOR.$RELEASE $Rev: 368 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#ifndef CSpotLightH
#define CSpotLightH
//---------------------------------------------------------------------------
#include "lighting/CPositionalLight.h"
#include "lighting/CDirectionalLight.h"
//---------------------------------------------------------------------------

//===========================================================================
/*!
    \file       CSpotLight.h

    \brief 
    <b> Lighting </b> \n 
    Spot Light.
*/
//===========================================================================

//===========================================================================
/*!
    \class      cSpotLight
    \ingroup    lighting

    \brief      
    cSpotLight describes a spot light. 
*/
//===========================================================================

class cSpotLight : public cPositionalLight, public cDirectionalLight
{
  public:
    
    //-----------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //-----------------------------------------------------------------------

    //! Constructor of cSpotLight.
    cSpotLight(cWorld* a_world);
	
    //! Destructor of cSpotLight.
    virtual ~cSpotLight();


  	//-----------------------------------------------------------------------
    // MEMBERS: (shadows)
	//-----------------------------------------------------------------------

    //! Shadow map.
	cShadowMap* m_shadowMap;

    //! Update shadow map.
    void updateShadowMap();

    //! Define the near and near clipping planes of the shadow.
    void setShadowMapProperties(const double& a_nearClippingPlane, 
                                const double& a_farClippingPlane);

    //! Set the status of the shadow map.
    void setShadowMapEnabled(const bool a_enabled);

    //! Get the status of the shadow map.
    bool getShadowMapEnabled() const;


	//-----------------------------------------------------------------------
    // METHODS: (lighting properties)
	//-----------------------------------------------------------------------

    //! Set concentration level of the light.
    void setSpotExponent(const GLfloat& a_value) { m_spotExponent = cClamp(a_value, 0.0f, 100.0f); }

    //! Read concentration level of the light.
    GLfloat getSpotExponent() const { return (m_spotExponent); }
    
    //! Set the cutoff angle in degrees (only affects spotlights) (positional lights with angular cutoffs).
    void setCutOffAngleDeg(const GLfloat& a_angleDeg);

    //! Read Cut off angle.
    GLfloat getCutOffAngleDEG() const { return (m_cutOffAngleDEG); }


	//-----------------------------------------------------------------------
    // METHODS: (display model)
	//-----------------------------------------------------------------------

    //! Set display settings of light source. To be used for debugging purposes to display location and direction of light source.
    void setDisplaySettings(const double& a_sourceRadius = 0.02, 
                            const double& a_coneLength = 1.0, 
                            const bool a_displayEnabled = true);


  	//-----------------------------------------------------------------------
    // MEMBERS:
	//-----------------------------------------------------------------------

    //! Color used for rendering the light cone of the display model.
    cColorf m_displayConeColor;

  protected:
  
  	//-----------------------------------------------------------------------
    // MEMBERS: (lighting properties)
	//-----------------------------------------------------------------------
    
	//! Concentration of the light.
    GLfloat m_spotExponent;
    
	//! Cut off angle (for spot lights only). Only values in the range 0 to 90 degrees
    GLfloat m_cutOffAngleDEG;


    //-----------------------------------------------------------------------
    // MEMBERS: (shadow map)
    //-----------------------------------------------------------------------
    
    //! Shadow map near clipping plane.
    double m_shadowNearClippingPlane;

    //! Shadow map far clipping plane.
    double m_shadowFarClippingPlane;


  	//-----------------------------------------------------------------------
    // MEMBERS: (display model)
	//-----------------------------------------------------------------------

    //! Length of light cone for display model.
    double m_displayConeLength;


	//-----------------------------------------------------------------------
	// METHODS:
	//-----------------------------------------------------------------------

    //! Render the lighting properties of this light source in OpenGL.
    virtual void renderLightSource(cRenderOptions& a_options);

    //! Render a graphical representation (display model) of the light source. (used for debuging purposes typically).
    virtual void render(cRenderOptions& a_options);
};

//---------------------------------------------------------------------------
#endif
//---------------------------------------------------------------------------

