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
    \author    Dan Morris
    \version   $MAJOR.$MINOR.$RELEASE $Rev: 707 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#include "graphics/CMacrosGL.h"
//---------------------------------------------------------------------------

//===========================================================================
/*!
    Align the current -z axis with a reference frame; à la gluLookAt.

    \fn     void cLookAt(const cVector3d& a_eye, const cVector3d& a_at, 
                         const cVector3d& a_up)      
    \param  a_eye  Position of eye.
    \param  a_at  Lookat position.
    \param  a_up  Up direction.
*/
//===========================================================================
void cLookAt(const cVector3d& a_eye, const cVector3d& a_at, const cVector3d& a_up)
{
    // Define our look vector (z axis)
    cVector3d look = a_at - a_eye;
    look.normalize();

    // Define our new x axis
    cVector3d xaxis;
    xaxis = cCross(look,a_up);
    xaxis.normalize();

    // Define our new y axis as the cross of the x and z axes
    cVector3d upv = cCross(xaxis,look);

    // Turn around the z axis
    look.mul(-1.0);
  
    // Put it all into a GL-friendly matrix
    double dm[16];
    dm[0]  = xaxis(0) ;
    dm[1]  = xaxis(1) ;
    dm[2]  = xaxis(2) ;
    dm[3] = 0.f;
    dm[4]  = upv(0) ;
    dm[5]  = upv(1) ;
    dm[6]  = upv(2) ;
    dm[7] = 0.f;
    dm[8]  = look(0) ;
    dm[9]  = look(1) ;
    dm[10] = look(2) ;
    dm[11] = 0.f;
    dm[12] = a_eye(0) ;
    dm[13] = a_eye(1) ;
    dm[14] = a_eye(2) ;
    dm[15] = 1.f;

    // Push it onto the matrix stack
    glMultMatrixd(dm);
}
