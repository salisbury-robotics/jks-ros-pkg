//===========================================================================
/*
    This file is part of the GEM  dynamics engine.
    Copyright (C) 2006-2008 by Francois Conti, Stanford University.
    All rights reserved.

    \author:    <http://www.chai3d.org>
    \author:    Francois Conti
*/
//===========================================================================

//---------------------------------------------------------------------------
#ifndef CGEMH
#define CGEMH
//---------------------------------------------------------------------------

//===========================================================================
/*!
      \file     GEM.h
      \brief    Includes all the related header files for GEM.
*/
//===========================================================================

extern double default_node_radius;
extern double default_node_kDampingPos;
extern double default_node_kDampingRot;
extern double default_node_mass;
extern bool default_node_showFrame;
extern cColorf default_node_color;
extern bool default_node_useGravity;
extern cVector3d default_node_gravity;
extern double default_link_kSpringElongation;
extern double default_link_kSpringFlexion;
extern double default_link_kSpringTorsion;
extern cColorf default_link_color;

#include "CDefNode.h"
#include "CDefMesh.h"
#include "CDefNode.h"
#include "CDefLink.h"
#include "CDefWorld.h"

//---------------------------------------------------------------------------
#endif
//---------------------------------------------------------------------------
