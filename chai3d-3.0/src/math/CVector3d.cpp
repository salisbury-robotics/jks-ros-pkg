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
    \author	Dan Morris
    \version   $MAJOR.$MINOR.$RELEASE $Rev: 714 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#include "math/CVector3d.h"
#include "math/CMaths.h"
//---------------------------------------------------------------------------

//-----------------------------------------------------------------------
/*!
    Returns the squared distance from this segment to a_point and the
    position along the segment (from 0.0 to 1.0) of the closest point.
    
    \fn       cSegment3d::distanceSquaredToPoint(const cVector3d& a_point,
                                                 double& a_t, cVector3d*
                                                 a_closestPoint)
    \param    a_point Point to test.
    \param    a_t return value for the position along the segment.
    \param    a_closestPoint The closest point on this segment to the supplied point.
    \return   The distance from a_point to this segment.
*/
//-----------------------------------------------------------------------
double cSegment3d::distanceSquaredToPoint(const cVector3d& a_point,
                                          double& a_t,
                                          cVector3d* a_closestPoint)
{
    double mag = cDistance(m_start,m_end);

    // Project this point onto the line
    a_t = (a_point - m_start) * (m_end - m_start) / (mag * mag);

    // Clip to segment endpoints
    if (a_t < 0.0)
        a_t = 0.0;
    else if (a_t > 1.0)
        a_t = 1.0;

    // Find the intersection point
    cVector3d intersection = m_start + a_t * (m_end - m_start);
    if (a_closestPoint)
    {
        *a_closestPoint = intersection;
    }
    
    // Compute distance
    return cDistanceSq(a_point,intersection);
}
