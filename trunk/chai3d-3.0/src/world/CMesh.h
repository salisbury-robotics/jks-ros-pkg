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
    \author    Chris Sewell
    \version   $MAJOR.$MINOR.$RELEASE $Rev: 720 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#ifdef _MSVC
#pragma warning (disable : 4786)
#endif
//---------------------------------------------------------------------------
#ifndef CMeshH
#define CMeshH
//---------------------------------------------------------------------------
#include "world/CGenericObject.h"
#include "materials/CMaterial.h"
#include "materials/CTexture2d.h"
#include "graphics/CColor.h"
#include <vector>
#include <list>
//---------------------------------------------------------------------------
using std::list;
using std::vector;
//---------------------------------------------------------------------------
class cWorld;
class cTriangle;
class cVertex;
//---------------------------------------------------------------------------

//===========================================================================
/*!
    \file       CMesh.h

    \brief 
    <b> Scenegraph </b> \n 
    Virtual Mesh.
*/
//===========================================================================

//===========================================================================
/*!    
    \class      cMesh
    \ingroup    scenegraph

    \brief      
    cMesh represents a collection of vertices, triangles, materials,
    and texture properties that can be rendered graphically and haptically.
*/
//===========================================================================
class cMesh : public cGenericObject
{

  public:

    //-----------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //-----------------------------------------------------------------------

    //! Constructor of cMesh.
    cMesh(cWorld* a_world,
          cMaterial* a_material = NULL);

    //! Destructor of cMesh.
    virtual ~cMesh();


    //-----------------------------------------------------------------------
    // METHODS - GENERAL
    //-----------------------------------------------------------------------

    //! Create a copy of itself.
    virtual cMesh* copy(bool a_duplicateMaterialData = false,
                        bool a_duplicateTextureData = false, 
                        bool a_duplicateMeshData = false,
                        bool a_buildCollisionDetector = true);

    //! Get parent world.
    cWorld* getParentWorld() const { return (m_parentWorld); }

    //! Set parent world.
    void setParentWorld(cWorld* a_world) { m_parentWorld = a_world; }


    //-----------------------------------------------------------------------
    // METHODS - VERTICES
    //-----------------------------------------------------------------------

    //! Create a new vertex and add it to the vertex list.
    unsigned int newVertex(const double a_x, 
                           const double a_y, 
                           const double a_z);

    //! Create a new vertex and add it to the vertex list.
    unsigned int newVertex(const double a_x, 
                           const double a_y, 
                           const double a_z,
                           const double a_normalX, 
                           const double a_normalY, 
                           const double a_normalZ);

    //! Create a new vertex and add it to the vertex list.
    unsigned int newVertex(const double a_x, 
                           const double a_y, 
                           const double a_z,
                           const double a_normalX, 
                           const double a_normalY, 
                           const double a_normalZ,
						   const double a_textureCoordX,
						   const double a_textureCoordY,
						   const double a_textureCoordZ = 0.0);

    //! Create a new vertex and add it to the vertex list.
    unsigned int newVertex(const cVector3d& a_pos);

    //! Create a new vertex and add it to the vertex list.
    unsigned int newVertex(const cVector3d& a_pos, 
                           const cVector3d& a_normal);

    //! Create a new vertex and add it to the vertex list.
    unsigned int newVertex(const cVector3d& a_pos, 
                           const cVector3d& a_normal,
						   const cVector3d& a_textureCoord);

    //! Add an array of vertices to the vertex list given an array of vertex positions.
    void addVertices(const cVector3d* a_vertexPositions, 
                     const unsigned int& a_numVertices);

    //! Remove the vertex at the specified position in my vertex array.
    bool removeVertex(const unsigned int a_index);

    //! Access the vertex at the specified position in my vertex array (and maybe my childrens' arrays).
	inline cVertex* getVertex(unsigned int a_index) { return (&m_vertices->at(a_index)); }

    //! Read the number of stored vertices.
	inline unsigned int getNumVertices() const { return (unsigned int)(m_vertices->size()); }

    
    //-----------------------------------------------------------------------
    // METHODS - TRIANGLES
    //-----------------------------------------------------------------------

    //! Create a new triangle by passing vertex indices.
    unsigned int newTriangle(const unsigned int a_indexVertex0,
                             const unsigned int a_indexVertex1, 
                             const unsigned int a_indexVertex2);

    //! Create a new triangle and three new vertices by passing vertex positions.
    unsigned int newTriangle(const cVector3d& a_vertex0, 
                             const cVector3d& a_vertex1,
                             const cVector3d& a_vertex2);

    //! Create a new triangle and three new vertices by passing vertex positions and normals.
    unsigned int newTriangle(const cVector3d& a_vertex0, 
                             const cVector3d& a_vertex1,
                             const cVector3d& a_vertex2,
                             const cVector3d& a_normal0, 
                             const cVector3d& a_normal1,
                             const cVector3d& a_normal2);

    //! Create a new triangle and three new vertices by passing vertex positions, normals and texture coordinates.
    unsigned int newTriangle(const cVector3d& a_vertex0, 
                             const cVector3d& a_vertex1,
                             const cVector3d& a_vertex2,
                             const cVector3d& a_normal0, 
                             const cVector3d& a_normal1,
                             const cVector3d& a_normal2,
							 const cVector3d& a_textureCoord0, 
                             const cVector3d& a_textureCoord1,
                             const cVector3d& a_textureCoord2);

    //! Remove a triangle from my triangle array.
    bool removeTriangle(const unsigned int a_index);

    //! Access the triangle at the specified position in my triangle array.
    cTriangle* getTriangle(unsigned int a_index);	

    //! Read the number of stored triangles.
	unsigned int getNumTriangles();

    //! Clear all triangles and vertices of mesh.
    void clear();


    //-----------------------------------------------------------------------
    // METHODS - GRAPHIC RENDERING
    //-----------------------------------------------------------------------

    //! Set the alpha value at each vertex and in all of my material colors.
    virtual void setTransparencyLevel(const float a_level,
                                      const bool a_applyToTextures = false,
                                      const bool a_affectChildren = true);

    //! Set color of each vertex, optionally propagating the operation to my children.
    void setVertexColor(const cColorf& a_color);

    //! Enable or disable the use of a display list for rendering, optionally propagating the operation to my children.
    void useDisplayList(const bool a_useDisplayList);

    //! Enable or disable the use vertex arrays for rendering, optionally propagating the operation to my children.
    void useVertexArrays(const bool a_useVertexArrays);

    //! Ask whether I'm currently rendering with a display list.
    bool getDisplayListEnabled() const { return m_useDisplayList; }

    //! Invalidate any existing display lists.
    void invalidateDisplayList();

    //! Enable or disable the rendering of vertex normals, optionally propagating the operation to my children.
	void setShowNormals(const bool a_showNormals) { m_showNormals = a_showNormals; }

    //! Returns whether rendering of normals is enabled.
    bool getShowNormals() const { return (m_showNormals); }

    //! Set graphic properties for normal-rendering, optionally propagating the operation to my children.
    void setNormalsProperties(const double a_length, 
                              const cColorf& a_color);

    //! Are vertex colors currently enabled?
    bool getColorsEnabled() const { return m_useVertexColors; }


    //-----------------------------------------------------------------------
    // METHODS - COLLISION DETECTION:
    //-----------------------------------------------------------------------

    //! Set up a brute force collision detector for this mesh.
    virtual void createBruteForceCollisionDetector();

    //! Set up an AABB collision detector for this mesh.
    virtual void createAABBCollisionDetector(double a_radius);

	//! Update the relationship between the tool and the current object.
	void computeLocalInteraction(const cVector3d& a_toolPos,
                                 const cVector3d& a_toolVel,
                                 const unsigned int a_IDN);


    //-----------------------------------------------------------------------
    // METHODS - MESH MANIPULATION:
    //-----------------------------------------------------------------------

    //! Compute all triangle normals, optionally propagating the operation to my children.
    void computeAllNormals();

	//! Shifts all vertex positions by the specified amount.
    virtual void offsetVertices(const cVector3d& a_offset, 
                                const bool a_updateCollisionDetector = true);

    //! Scale vertices and normals by the specified scale factors and re-normalize.
    virtual void scaleObject(const double& a_scaleFactor);

    //! Compute the center of mass of this mesh, based on vertex positions.
    virtual cVector3d getCenterOfMass();

    //! Reverse all normals on this model.
    virtual void reverseAllNormals();

	//! Render triangles, material and texture properties.
    virtual void renderMesh(cRenderOptions& a_options);


  protected:

    //-----------------------------------------------------------------------
    // METHODS:
    //-----------------------------------------------------------------------

    //! Render the mesh itself.
    virtual void render(cRenderOptions& a_options);

    //! Draw a small line for each vertex normal.
    virtual void renderNormals();

    //! Update the global position of each of my vertices.
    virtual void updateGlobalPositions(const bool a_frameOnly);

    //! Update my boundary box dimensions based on my vertices.
    virtual void updateBoundaryBox();


    //-----------------------------------------------------------------------
    // MEMBERS - DISPLAY PROPERTIES:
    //-----------------------------------------------------------------------

    //! Parent world.
    cWorld *m_parentWorld;

    //! If \b true, then normals are displayed.
    bool m_showNormals;

    //! Color used to render lines representing normals.
    cColorf m_showNormalsColor;

    //! Length of each normal (for graphic rendering of normals).
    double m_showNormalsLength;

    //! Should we use a display list to render this mesh?
    bool m_useDisplayList;

    //! Should we use vertex arrays to render this mesh?
    bool m_useVertexArrays;

    //! The OpenGL display list used to draw this mesh, if display lists are enabled.
    int m_displayList;


    //-----------------------------------------------------------------------
    // MEMBERS - TRIANGLE AND VERTEX DATA:
    //-----------------------------------------------------------------------

  public:

    //! Array of vertices.
    vector<cVertex> *m_vertices;

	//! Array of triangles.
    vector<cTriangle> *m_triangles;


  private:

    //! List of free slots in the vertex array.
    list<unsigned int> *m_freeVertices;

    //! List of free slots in the triangle array.
    list<unsigned int> *m_freeTriangles;
};

//---------------------------------------------------------------------------
#endif
//---------------------------------------------------------------------------

