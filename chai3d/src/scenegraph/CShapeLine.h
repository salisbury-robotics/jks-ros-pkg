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
#ifndef CShapeLineH
#define CShapeLineH
//---------------------------------------------------------------------------
#include "scenegraph/CGenericObject.h"
#include "graphics/CMaterial.h"
//---------------------------------------------------------------------------

//===========================================================================
/*!
      \file       cShapeLine.h
      \class      cShapeLine
      \brief      cShapeLine describes a simple line potential field
*/
//===========================================================================
class cShapeLine : public cGenericObject
{
  public:
    // CONSTRUCTOR & DESTRUCTOR:
    //! Constructor of cShapeLine.
    cShapeLine(const cVector3d& a_pointA, const cVector3d& a_pointB);

    //! Destructor of cShapeLine.
    virtual ~cShapeLine() {};

    // METHODS:
    //! Render object in OpenGL.
    virtual void render(const int a_renderMode=0);

    //! Update bounding box of current object
    virtual void updateBoundaryBox();

    //! Object scaling
    virtual void scaleObject(const cVector3d& a_scaleFactors);

    //! Update the geometrical relationship between the tool and the current object
    virtual void computeLocalInteraction(const cVector3d& a_toolPos,
                                         const cVector3d& a_toolVel,
                                         const unsigned int a_IDN);

    //! Point A of line
    cVector3d m_pointA;

    //! Point A of line
    cVector3d m_pointB;
};

//---------------------------------------------------------------------------
#endif
//---------------------------------------------------------------------------
