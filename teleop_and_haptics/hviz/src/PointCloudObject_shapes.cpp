//===========================================================================
/*
    CS277 - Experimental Haptics
    Winter 2011, Stanford University

    This class encapsulates the visual and haptic rendering for an implicit
    surface.  It inherits the regular CHAI3D cMesh class, and taps into
    an external implementation of the Marching Cubes algorithm to create
    a triangle mesh from the implicit surface function.

    Your job is to implement the method that tracks the position of the
    proxy as the tool moves and interacts with the object, as described by
    the implicit surface rendering algorithm in Salisbury & Tarr 1997.

    \author    Sonny Chan
    \date      January 2010
*/
//===========================================================================

#include "PointCloudObject.h"

#include <vector>
#include <algorithm>
#include <ctime>
#include <stdio.h>
//#include <unistd.h

#include <hviz/HapticsConfig.h>

//#include "kdtree.h"

#include <pcl/ModelCoefficients.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>
//#include <pcl/visualization/cloud_viewer.h>



const double THRESH_MULT = 3;

//void PointCloudObject::computeCloudSpecs()
//{
//  double average_distance = 0;
//  if(m_vertices.size()){
//    for(unsigned int i = 0; i < m_vertices.size(); i++)
//    {
//      average_distance += tree->nearest(m_vertices[i]);
//    }
//    average_distance /= m_vertices.size();
//  }
//
//  if(m_configPtr->auto_threshold){
//    m_configPtr->basis_radius= 4*average_distance;
//    m_configPtr->meta_thresh = THRESH_MULT*m_configPtr->basis_radius;
//    printf("Average separation in cloud is %f , setting basis_radius to % 5.3f and threshold to % 5.2f  \n",
//    average_distance, m_configPtr->basis_radius, m_configPtr->meta_thresh);
//  }
//
//}

void PointCloudObject::computeCloudSpecs()
{
  boost::recursive_mutex::scoped_lock lock(mutex_);
  ros::Time begin = ros::Time::now();
  ROS_DEBUG_NAMED("haptics", "Computing cloud specs ... %f", begin.toSec());
  double average_distance = 0;
  int NN = 3;
  if(last_points->size() != 0){
    std::vector< int >   	k_indices(NN+1, 0);
    std::vector< float >  k_sqr_distances(NN+1, 0);
    for(int index = 0; index < last_points->size(); index++)
    {
      //printf("Index: %d, size %d\n", index, last_points->size());
      int N = last_tree->nearestKSearch ( index, NN+1, k_indices, k_sqr_distances);
      if( N == 0 ) continue;
      float distance = 0;
      for(int d = 1; d < N; d++)
      {
        //printf("For seed %d, neighbor %d distance %f\n", index, k_indices[d], sqrt(k_sqr_distances[d]));
        distance += sqrt(k_sqr_distances[d]);
      }
      distance = 1.0/(N-1)*distance;
      last_points->points[index].data[3] = distance*m_configPtr->radius_multiple;
      average_distance += distance;
    }
    average_distance /= last_points->size();
  }

  if(m_configPtr->auto_threshold){
    m_active_radius = m_configPtr->radius_multiple*average_distance;
    m_configPtr->basis_radius= m_active_radius;
    //m_configPtr->meta_thresh = THRESH_MULT*m_configPtr->basis_radius;
    //m_configPtr->meta_thresh = 0.5;
    ROS_INFO("Average separation in cloud is %f , setting basis_radius to % 5.3f and threshold to % 5.2f ",
    average_distance, m_configPtr->basis_radius, m_configPtr->meta_thresh);
  }
  else
  {
    m_active_radius = m_configPtr->basis_radius;
  }

  ros::Time end = ros::Time::now();
  ros::Duration elapsed = end-begin;
  //ROS_INFO("Begin: %f, end: %f, elapsed: %f", begin.toSec(), end.toSec(), elapsed.toSec());
  ROS_DEBUG_NAMED("time", "Computing cloud specs took %.3lf ms", elapsed.toSec()*1000.0);
}

//! Updates with new shape
bool PointCloudObject::updateShape(int shape, bool force_refresh)
{
  //Return if there is nothing to update
  if(m_shape == shape) return false;
  m_shape = shape;
  ROS_INFO("Shape changed to %d", m_shape);

  if(m_shape == hviz::Haptics_POINTS){
    this->createUniformPlane(2, 6, 0.06, 0);
  }
  if(m_shape == hviz::Haptics_PLANE){
    this->createUniformPlane(4, 4, 0.03, m_configPtr->synthetic_noise);
  }
  if(m_shape == hviz::Haptics_SHELL){
    this->createSphericalShell(0.1, 0.1, -CHAI_PI/2.5, CHAI_PI/3, -CHAI_PI, CHAI_PI, m_configPtr->synthetic_noise);
  }
  if(m_shape == hviz::Haptics_BOX){
    this->createOpenTopBox(0.1, 0.01, m_configPtr->synthetic_noise);
  }
  if(m_shape == hviz::Haptics_GRADIENT){
    this->createGradientPlane(0.4, 0.4, 0.01, 1.3, m_configPtr->synthetic_noise);
  }
  if(m_shape == hviz::Haptics_CLOUD){
     //Do nothing?
  }
  return true;
}

union RGBvalue
{
  struct
  {
    unsigned char blue;
    unsigned char green;
    unsigned char red;
    unsigned char alpha;
  };
  float float_value;
} ;

//! Populates m_vertices with a (non-uniform) spherical shell of points.
bool PointCloudObject::createFromCloud(const pcl::PointCloud<PointT> &cloud)
{
  if(m_refresh_cloud)
    return false;

  boost::recursive_mutex::scoped_lock lock(mutex_);

  ros::WallTime begin = ros::WallTime::now();

  //m_isReady = false;
  this->clear();
  this->invalidateDisplayList();

  ROS_DEBUG_NAMED("haptics", "Loading cloud with %d points!",
           (unsigned int)cloud.size());
  // Create a KD-Tree
  last_points.reset( new pcl::PointCloud<PointT> ());
  *last_points = cloud;
  //last_points->header = cloud.header;
  ROS_DEBUG_NAMED("haptics", "Building kdtree...");

  last_tree.reset(new pcl::KdTreeFLANN<PointT> ());
  last_tree->setInputCloud (last_points);
  //tree->addPointsFromInputCloud();

  ROS_DEBUG_NAMED("time", "Initializing kdtree took %.3lf ms", (ros::WallTime::now() - begin).toSec()*1000.0);


  last_normals.reset(new pcl::PointCloud<pcl::Normal> ());
  //if(m_configPtr->estimate_normals)
  if( true )
  {
    begin = ros::WallTime::now();
    ROS_DEBUG_NAMED("haptics", "Estimating normals...");
    pcl::NormalEstimationOMP<PointT, pcl::Normal> mls;

    // Set parameters
    mls.setInputCloud (last_points);
    mls.setSearchMethod (last_tree);
    mls.setKSearch(9);
    // Reconstruct
    ROS_DEBUG_NAMED("haptics", "computing...");
    mls.compute(*last_normals);
    ROS_DEBUG_NAMED("haptics", "...done!");
    ROS_DEBUG_NAMED("time", "OMP Normal estimation took %.3lf ms", (ros::WallTime::now() - begin).toSec()*1000.0);
  }

//  pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
//  viewer.showCloud (m_cloud_points);
//  //  viewer.showCloud (m_cloud_normals);
//  viewer.showCloud();
//  while (!viewer.wasStopped ())
//  {
//  }

  //  printf("Created cloud with %d points!\n", (unsigned int)m_vertices.size());
  computeCloudSpecs();

  //ROS_INFO("All done!\n");

  m_refresh_cloud = true;
  return true;
}


//! This function is in no way thread safe. Only call at the right time!
void PointCloudObject::applyLastCloud()
{
  boost::recursive_mutex::scoped_lock lock(mutex_);

  ROS_DEBUG_NAMED("haptics", "Before push Num stored clouds: %d %d %d",
                    trees.size(), m_cloud_points.size(), m_cloud_normals.size()
                  );
  trees.push_front(last_tree);
  m_cloud_points.push_front(last_points);
  m_cloud_normals.push_front(last_normals);

  ROS_DEBUG_NAMED("haptics", "After push stored clouds: %d %d %d",
                    trees.size(), m_cloud_points.size(), m_cloud_normals.size()
                  );

  while( trees.size() > m_configPtr->num_clouds - 1 )
  {
    trees.pop_back();
    m_cloud_points.pop_back();
    m_cloud_normals.pop_back();
    ROS_DEBUG_NAMED("haptics", "After pop stored clouds: %d %d %d",
                      trees.size(), m_cloud_points.size(), m_cloud_normals.size()
                    );
  }

  m_refresh_cloud = false;
  m_isReady = true;

}

void PointCloudObject::createGradientPlane(float width, float length, float initial_step, float growth_rate, double noise_size)
{
    this->clear();
    this->invalidateDisplayList();

    // Seed random generator for adding noise
    // Note this noise is uniform on a unit cube... NOT spherical gaussian

    pcl::PointCloud<PointT> cloud;
    cloud.header.stamp = ros::Time::now();
    cloud.header.frame_id = "/tool_frame";

    srand(time(0));

    float u_step=initial_step;
    float v_step=initial_step;

    for( float u = -width/2; u <= width/2; u+=u_step, u_step*=growth_rate )
    {
      v_step=initial_step;
      for( float v = -length/2; v < length/2; v+=v_step, v_step*=growth_rate )
      {

            double x = u + (((float)rand() / (float)(RAND_MAX)) - 0.5)*noise_size;
            double y = v + (((float)rand() / (float)(RAND_MAX)) - 0.5)*noise_size;
            double z = 0 + (((float)rand() / (float)(RAND_MAX)) - 0.5)*noise_size;
            PointT point;
            point.x = x;
            point.y = y;
            point.z = z;
            cloud.push_back(point);
      }
    }

    printf("Created plane cloud with %d points!\n", (unsigned int)cloud.size());

    this->createFromCloud(cloud);

}

void PointCloudObject::createUniformPlane(int x_count, int y_count, double linear_step, double noise_size)
{
    this->clear();
    this->invalidateDisplayList();
    //if(tree) delete tree;

    // Seed random generator for adding noise
    // Note this noise is uniform on a unit cube... NOT spherical gaussian

    pcl::PointCloud<PointT> cloud;
    cloud.header.stamp = ros::Time::now();
    cloud.header.frame_id = "/tool_frame";

    srand(time(0));

    for( int i = 0; i < x_count; i++ )
	  {
      for( int j = 0; j < y_count; j++ )
      {

            //if( x_count > 7 && abs(i+0.5-x_count/2.0) < 1 && abs(j+0.5-y_count/2.0) < 1 ) continue;

            double x = (i+0.5 + (-x_count)/2.0)*linear_step + (((float)rand() / (float)(RAND_MAX)) - 0.5)*noise_size;
            double y = (j+0.5 + (-y_count)/2.0)*linear_step  + (((float)rand() / (float)(RAND_MAX)) - 0.5)*noise_size;
            double z = 0 + (((float)rand() / (float)(RAND_MAX)) - 0.5)*noise_size;
            PointT point;
            point.x = x;
            point.y = y;
            point.z = z;
            cloud.push_back(point);
      }
    }

    printf("Created plane cloud with %d points!\n", (unsigned int)cloud.size());

    this->createFromCloud(cloud);
}

//bool PointCloudObject::loadFromFile(const char* filepath, double scale, bool mean_shift)
//{
//  const int MaxLineLength = 256;
//  FILE* cloud_file = fopen(filepath, "r");
//  char Line[MaxLineLength];
//	int CurLineIndex = 1;
//
//	if(!cloud_file)
//	{
//		fprintf(stderr, "Could not open file '%s'.\n", filepath);
//		return false;
//	}
//
//  printf("Loading cloud data from "); printf("%s",filepath); printf("\n");
//
//  // Clear the current cloud
//  m_isReady = false;
//  this->clear();
//  this->invalidateDisplayList();
//  if(tree) delete tree;
//
//  // Loop and get all the points!
//	while (fgets(Line, MaxLineLength, cloud_file))
//	{
//    char* str = Line; //strtok(Line, " \t\n\r");
//
//		cVector3d point;
//    float colorf = 0.f;
//    sscanf(str, "%lf %lf %lf %f\n",
//				&point.x, &point.y, &point.z, &colorf);
//    point*= scale;
//
//    unsigned int index = newVertex(point);
//    m_vertices[index].m_localPos = point;
//    //m_vertices[index].m_color = cColorf(decomp.x, decomp.y, decomp.z, 1.0f);
//    m_vertices[index].m_color = cColorf(0.6, 0.6, 0.8, 1.0f);
//    m_vertices[index].m_normal = cVector3d(1.0, 0, 0);
//    m_vertices[index].m_allocated = true;
//
//		fflush(stdout);
//		CurLineIndex++;
//	}
//	fclose(cloud_file);
//
//  this->updateBoundaryBox();
//
//
//  if(mean_shift){
//    cVector3d cloud_center = getBoundaryCenter();
//    cloud_center.print();
//    for(unsigned int i = 0; i < m_vertices.size(); i++)
//      m_vertices[i].m_localPos -= cloud_center;
//  }
//
//    printf("Loaded cloud with %d points!\n", (unsigned int)m_vertices.size());
//
//    prepareTree();
//
//    m_isReady = true;
//    return true;
//}


//! Populates m_vertices with a (non-uniform) spherical shell of points.
void PointCloudObject::createSphericalShell(double radius, double angle_step, double min_lat, double max_lat, double min_lon, double max_lon, double noise_size)
{
    this->clear();
    this->invalidateDisplayList();
    //if(tree) delete tree;

    // Seed random generator for adding noise
    // Note this noise is uniform on a unit cube... NOT spherical gaussian
    srand(time(0));

    pcl::PointCloud<PointT> cloud;
    cloud.header.stamp = ros::Time::now();
    cloud.header.frame_id = "/tool_frame";

    for( double theta = min_lat; theta < max_lat; theta +=angle_step )
    {
	    for( double phi = min_lon; phi < max_lon; phi+= angle_step  )
        {
            double x = radius*cos(phi)*cos(theta) + (((float)rand() / (float)(RAND_MAX)) - 0.5)*noise_size;
            double y = radius*sin(phi)*cos(theta) + (((float)rand() / (float)(RAND_MAX)) - 0.5)*noise_size;
            double z = radius*sin(theta) + (((float)rand() / (float)(RAND_MAX)) - 0.5)*noise_size;
            PointT point;
            point.x = x;
            point.y = y;
            point.z = z;
            cloud.push_back(point);
        }
    }

    printf("Created spherical cloud with %d points!\n", (unsigned int)cloud.size());

    this->createFromCloud(cloud);
}

//! Populates m_vertices with a (non-uniform) spherical shell of points.
void PointCloudObject::createOpenTopBox(double edge_size, double linear_step, double noise_size)
{
    this->clear();
    this->invalidateDisplayList();
    //if(tree) delete tree;

    // Seed random generator for adding noise
    // Note this noise is uniform on a unit cube... NOT spherical gaussian
    srand(time(0));

    pcl::PointCloud<PointT> cloud;
    cloud.header.stamp = ros::Time::now();
    cloud.header.frame_id = "/tool_frame";

    cVector3d Xn(1,0,0);
    cVector3d Yn(0,1,0);
    cVector3d Zn(0,0,1);

    for( int face = 0; face < 5; face++)
    {
        for( double u = -edge_size/2; u < edge_size/2; u += linear_step )
	    {
	        for( double v = -edge_size/2; v < edge_size/2; v += linear_step )
            {
                double x = (face == 0) * edge_size/2 + (face == 1) * -edge_size/2 + u * (face == 2 || face == 3 || face == 4 || face == 5);
                double y = (face == 2) * edge_size/2 + (face == 3) * -edge_size/2 + u * (face == 0 || face == 1) + v * (face == 4 || face == 5);
                double z = (face == 4) * -edge_size/2 + (face == 5) * edge_size/2 + v * (face == 0 || face == 1 || face == 2 || face == 3);

                x += (((float)rand() / (float)(RAND_MAX)) - 0.5)*noise_size;
                y += (((float)rand() / (float)(RAND_MAX)) - 0.5)*noise_size;
                z += (((float)rand() / (float)(RAND_MAX)) - 0.5)*noise_size;

                PointT point;
                point.x = x;
                point.y = y;
                point.z = z;
                cloud.push_back(point);
            }
        }
    }

    printf("Created spherical cloud with %d points!\n", (unsigned int)cloud.size());
    
    this->createFromCloud(cloud);
}


//===========================================================================
/*!
     Compute the axis-aligned boundary box that encloses all triangles in this mesh

     \fn       void cMesh::updateBoundaryBox()
*/
//===========================================================================
void PointCloudObject::updateBoundaryBox()
{
    if (m_vertices.size() == 0)
    {
        m_boundaryBoxMin.zero();
        m_boundaryBoxMax.zero();
        return;
    }

    double xMin = CHAI_LARGE;
    double yMin = CHAI_LARGE;
    double zMin = CHAI_LARGE;
    double xMax = -CHAI_LARGE;
    double yMax = -CHAI_LARGE;
    double zMax = -CHAI_LARGE;

    // loop over all my triangles
    for(unsigned int i=0; i<m_vertices.size(); i++)
    {
        // get next vertex
        cVertex* vtx = &m_vertices[i];

        if (vtx->m_allocated)
        {
          xMin = cMin(vtx->m_localPos.x, xMin);
          yMin = cMin(vtx->m_localPos.y, yMin);
          zMin = cMin(vtx->m_localPos.z, zMin);
          xMax = cMax(vtx->m_localPos.x, xMax);
          yMax = cMax(vtx->m_localPos.y, yMax);
          zMax = cMax(vtx->m_localPos.z, zMax);
        }
    }

    m_boundaryBoxMin.set(xMin, yMin, zMin);
    m_boundaryBoxMax.set(xMax, yMax, zMax);

}


////===========================================================================
///*!
//     Render the cloud itself.  This function is declared public to allow
//     sharing of data among meshes, which is not possible given most
//     implementations of 'protected'.  But it should only be accessed
//     from within render() or derived versions of render().
//
//     \fn       void PointCloudObject::renderMesh(const int a_renderMode)
//*/
////===========================================================================
//void PointCloudObject::renderCloud(const int a_renderMode)
//{
//    //-----------------------------------------------------------------------
//    // INITIALIZATION
//    //-----------------------------------------------------------------------
//    // check if object contains any points
//    if ((this->m_vertices->size() == 0))
//    {
//        return;
//    }
//
//    // we are not currently creating a display list
//    bool creating_display_list = false;
//
//
//    //-----------------------------------------------------------------------
//    // DISPLAY LIST
//    //-----------------------------------------------------------------------
//    // Should we render with a display list?
//    if (m_useDisplayList)
//    {
//        // If the display list doesn't exist, create it
//        if (m_displayList == -1)
//        {
//             m_displayList = glGenLists(1);
//             if (m_displayList == -1) return;
//
//             // On some machines, GL_COMPILE_AND_EXECUTE totally blows for some reason,
//             // so even though it's more complex on the first rendering pass, we use
//             // GL_COMPILE (and _repeat_ the first rendering pass)
//             glNewList(m_displayList, GL_COMPILE);
//
//             // the list has been created
//             creating_display_list = true;
//
//             // Go ahead and render; we'll create this list now...
//             // we'll make another (recursive) call to renderMesh()
//             // at the end of this function.
//        }
//
//        // Otherwise all we have to do is call the display list
//        else
//        {
//            glCallList(m_displayList);
//
//            // All done...
//            return;
//        }
//    }
//    //-----------------------------------------------------------------------
//    // RENDERING WITH VERTEX ARRAYS OR CLASSIC OPENGL CALLS
//    //-----------------------------------------------------------------------
//
//    glDisableClientState(GL_COLOR_ARRAY);
//    glDisableClientState(GL_TEXTURE_COORD_ARRAY);
//    glDisableClientState(GL_INDEX_ARRAY);
//    glDisableClientState(GL_EDGE_FLAG_ARRAY);
//
//    if (m_useVertexArrays)
//    {
//        glEnableClientState(GL_NORMAL_ARRAY);
//        glEnableClientState(GL_VERTEX_ARRAY);
//    }
//
//    /////////////////////////////////////////////////////////////////////////
//    // RENDER MATERIAL
//    /////////////////////////////////////////////////////////////////////////
//    if (m_useMaterialProperty)
//    {
//        m_material->render();
//    }
//
//
//    /////////////////////////////////////////////////////////////////////////
//    // RENDER VERTEX COLORS
//    /////////////////////////////////////////////////////////////////////////
//    if (m_useVertexColors)
//    {
//        // Clear the effects of material properties...
//        if (!m_useMaterialProperty)
//        {
//            float fnull[4] = {0,0,0,0};
//            glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, (const float *)&fnull);
//            glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION, (const float *)&fnull);
//        }
//
//        // enable vertex colors
//        glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
//        glEnable(GL_COLOR_MATERIAL);
//        if (m_useVertexArrays)
//        {
//            glEnableClientState(GL_COLOR_ARRAY);
//        }
//    }
//    else
//    {
//        glDisable(GL_COLOR_MATERIAL);
//        glDisableClientState(GL_COLOR_ARRAY);
//    }
//
//    /////////////////////////////////////////////////////////////////////////
//    // FOR OBJECTS WITH NO DEFINED COLOR/MATERIAL SETTINGS
//    /////////////////////////////////////////////////////////////////////////
//    // A default color for objects that don't have vertex colors or
//    // material properties (otherwise they're invisible)...
//    if ((!m_useVertexColors) && (!m_useMaterialProperty))
//    {
//        glEnable(GL_COLOR_MATERIAL);
//        glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
//        glColor4f(1,1,1,1);
//    }
//
//    /////////////////////////////////////////////////////////////////////////
//    // TEXTURE
//    /////////////////////////////////////////////////////////////////////////
//    if ((m_texture != NULL) && (m_useTextureMapping))
//    {
//        glEnable(GL_TEXTURE_2D);
//        if (m_useVertexArrays)
//        {
//            glEnableClientState(GL_TEXTURE_COORD_ARRAY);
//        }
//        m_texture->render();
//    }
//
//
//    /////////////////////////////////////////////////////////////////////////
//    // RENDER TRIANGLES WITH VERTEX ARRAYS
//    /////////////////////////////////////////////////////////////////////////
//    if (m_useVertexArrays)
//    {
//        // Where does our vertex array live?
//        vector<cVertex>* vertex_vector = m_vertices;
//        cVertex* vertex_array = (cVertex*) &((*vertex_vector)[0]);
//
//        // specify pointers to rendering arrays
//        glVertexPointer(3, GL_DOUBLE, sizeof(cVertex), &(vertex_array[0].m_localPos));
//        glNormalPointer(GL_DOUBLE, sizeof(cVertex), &(vertex_array[0].m_normal));
//        glColorPointer(3, GL_FLOAT, sizeof(cVertex), vertex_array[0].m_color.pColor());
//        glTexCoordPointer(2, GL_DOUBLE, sizeof(cVertex), &(vertex_array[0].m_texCoord));
//
//        // variables
//        unsigned int i;
//        unsigned int numPoints = m_vertices.size();
//
//        // begin rendering points
//        glPointSize(2.0);
//        glBegin(GL_POINTS);
//
//
//        // render all active triangles
//        for(i=0; i<numPoints; i++)
//        {
//            //bool allocated = m_triangles[i].m_allocated;
//            //if (allocated)
//            {
//                //unsigned int index0 = m_triangles[i].m_indexVertex0;
//                //unsigned int index1 = m_triangles[i].m_indexVertex1;
//                //unsigned int index2 = m_triangles[i].m_indexVertex2;
//                glArrayElement(i);
//                //glArrayElement(index1);
//                //glArrayElement(index2);
//            }
//        }
//
//        // finalize rendering list of triangles
//        glEnd();
//    }
//
//    /////////////////////////////////////////////////////////////////////////
//    // RENDER TRIANGLES USING CLASSIC OPENGL COMMANDS
//    /////////////////////////////////////////////////////////////////////////
//    else
//    {
//        printf("Error!! Should be using vertex lists!");
//
//        // variables
//        unsigned int i;
//        unsigned int numPoints = m_vertices.size();
//
//        // begin rendering triangles
//
//        glBegin(GL_POINTS);
//
//        // render all active triangles
//        if ((!m_useTextureMapping) && (!m_useVertexColors))
//        {
//            for(i=0; i<numPoints; i++)
//            {
//                glVertex3dv(&m_vertices[i].m_localPos.x);
//            }
//        }
//        else if ((!m_useTextureMapping) && (m_useVertexColors))
//        {
//            for(i=0; i<numPoints; i++)
//            {
//                glColor4fv(m_vertices[i].m_color.pColor());
//                cVertex v;
//
//                glVertex3dv(&m_vertices[i].m_localPos.x);
//            }
//        }
//        else if ((m_useTextureMapping) && (m_useVertexColors))
//        {
//            printf("Error! Unsupported case (m_useTextureMapping) && (m_useVertexColors) in PointCloudObject::renderCloud!\n");
//            //for(i=0; i<numItems; i++)
//            //{
//            //    // get pointers to vertices
//            //    cVertex* v0 = m_triangles[i].getVertex(0);
//            //    cVertex* v1 = m_triangles[i].getVertex(1);
//            //    cVertex* v2 = m_triangles[i].getVertex(2);
//
//            //    // render vertex 0
//            //    glNormal3dv(&v0->m_normal.x);
//            //    glColor4fv(v0->m_color.pColor());
//            //    glTexCoord2dv(&v0->m_texCoord.x);
//            //    glVertex3dv(&v0->m_localPos.x);
//
//            //    // render vertex 1
//            //    glNormal3dv(&v1->m_normal.x);
//            //    glColor4fv(v1->m_color.pColor());
//            //    glTexCoord2dv(&v1->m_texCoord.x);
//            //    glVertex3dv(&v1->m_localPos.x);
//
//            //    // render vertex 2
//            //    glNormal3dv(&v2->m_normal.x);
//            //    glColor4fv(v2->m_color.pColor());
//            //    glTexCoord2dv(&v2->m_texCoord.x);
//            //    glVertex3dv(&v2->m_localPos.x);
//            //}
//        }
//
//        // finalize rendering list of triangles
//        glEnd();
//    }
//
//    //-----------------------------------------------------------------------
//    // FINALIZE
//    //-----------------------------------------------------------------------
//
//    // restore OpenGL settings to reasonable defaults
//    glDisable(GL_BLEND);
//    glDepthMask(GL_TRUE);
//    glDisable(GL_COLOR_MATERIAL);
//    glColorMaterial(GL_FRONT_AND_BACK,GL_AMBIENT_AND_DIFFUSE);
//    glDisable(GL_TEXTURE_2D);
//    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
//
//    // Turn off any array variables I might have turned on...
//    glDisableClientState(GL_NORMAL_ARRAY);
//    glDisableClientState(GL_VERTEX_ARRAY);
//    glDisableClientState(GL_COLOR_ARRAY);
//    glDisableClientState(GL_TEXTURE_COORD_ARRAY);
//
//    // If we've gotten this far and we're using a display list for rendering,
//    // we must be capturing it right now...
//    if ((m_useDisplayList) && (m_displayList != -1) && (creating_display_list))
//    {
//        // finalize list
//        glEndList();
//
//        // Recursively make a call to actually render this object if
//        // we didn't use compile_and_execute
//        renderCloud(a_renderMode);
//    }
//}
