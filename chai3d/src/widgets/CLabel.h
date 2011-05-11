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
#ifndef CLabelH
#define CLabelH
//---------------------------------------------------------------------------
#include "extras/CGlobals.h"
#include "scenegraph/CGenericObject.h"
#include "widgets/CFont.h"
#include "graphics/CColor.h"
#include "math/CString.h"
//---------------------------------------------------------------------------

//===========================================================================
/*!
      \class      cLabel
      \brief      This class provides functionalities to display one line
                  of text.
*/
//===========================================================================
class cLabel : public cGenericObject
{
  public:

    // CONSTRUCTOR & DESTRUCTOR:

    //! Constructor of cLabel
    cLabel();

    //! Destructor of cFont
    virtual ~cLabel();

    //! Font
    cFont* m_font;

    //! Font color
    cColorf m_fontColor;

    //! String
    string m_string;

    //! Render object in OpenGL.
    virtual void render(const int a_renderMode=0);
};

//---------------------------------------------------------------------------
#endif
//---------------------------------------------------------------------------
