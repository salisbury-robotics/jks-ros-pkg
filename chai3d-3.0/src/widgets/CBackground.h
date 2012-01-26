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
    \version   $MAJOR.$MINOR.$RELEASE $Rev: 714 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#ifndef CBackground
#define CBackground
//---------------------------------------------------------------------------
#include "world/CMesh.h"
#include "graphics/CImage.h"
//---------------------------------------------------------------------------

//===========================================================================
/*!
    \file       CBackground.h

    \brief 
    <b> Graphics </b> \n 
    World background.
*/
//===========================================================================

//===========================================================================
/*!
    \class      cBackground
    \ingroup    graphics

    \brief      
    Implementation of a world colored background.
*/
//===========================================================================
class cBackground : public cMesh
{
  public:
    
    //-----------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //-----------------------------------------------------------------------

    //! Constructor of cBackground.
    cBackground(cWorld* a_world);

    //! Destructor of cBackground.
    virtual ~cBackground() {};

    //-----------------------------------------------------------------------
    // METHODS:
    //-----------------------------------------------------------------------

    //! Create a copy of current object.
    cBackground* copy(bool a_duplicateMaterialData = false,
					  bool a_duplicateTextureData = false, 
					  bool a_duplicateMeshData = false,
					  bool a_buildCollisionDetector = false);

    //! Update the dimension of the background in pixel coordinates.
    void update(const unsigned int a_bottomLeftX,
                const unsigned int a_bottomLeftY,
                const unsigned int a_topRightX,
                const unsigned int a_topRightY);

    //! Set uniform color.
    void setUniformColor(cColorf a_color);

    //! Set a vertical gradient color background.
    void setVerticalLinearGradient(cColorf a_topColor, 
                                   cColorf a_bottomColor);

    //! Set a horizontal gradient color background.
    void setHorizontalLinearGradient(cColorf a_leftColor, 
                                     cColorf a_rightColor);

    //! Set a color property at each corner.
    void setCornerColors(cColorf a_topLeftColor,
                         cColorf a_topRightColor, 
                         cColorf a_bottomLeftColor,    
                         cColorf a_bottomRightColor);

    //! Load background image
    bool loadFromFile(string a_filename);
    bool loadFromImage(cImage *a_image);


  protected:

    //-----------------------------------------------------------------------
    // MEMBERS:
    //-----------------------------------------------------------------------

    //! Top left vertex.
    unsigned int m_vertexTL;

    //! Top right vertex.
    unsigned int m_vertexTR;

    //! Bottom left vertex.
    unsigned int m_vertexBL;

    //! Bottom right vertex.
    unsigned int m_vertexBR;


    //-----------------------------------------------------------------------
    // METHODS:
    //-----------------------------------------------------------------------

    //! Render background in OpenGL.
    virtual void render(cRenderOptions& a_options);
};

//---------------------------------------------------------------------------
#endif
//---------------------------------------------------------------------------
