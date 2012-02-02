//===========================================================================
/*
    CS277 - Experimental Haptics
    Winter 2010, Stanford University

    This class encapsulates the visual and haptic rendering for an implicit
    surface.  It inherits the regular CHAI3D cMesh class, and taps into
    an external implementation of the Marching Cubes algorithm to create
    a triangle mesh from the implicit surface function.

    Your job is to implement the method that tracks the position of the
    proxy as the tool moves and interacts with the object, as described by
    the implicit surface rendering algorithm in Salisbury & Tarr 1997.

    \author    Adam Leeper & Sonny Chan
    \date      January 2010
*/
//===========================================================================

#ifndef PointCloudObject_H
#define PointCloudObject_H

#include "chai3d.h"
#include "pcl/point_types.h"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
//#include <pcl/octree/octree_pointcloud.h>
//#include <pcl/octree/octree_impl.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <hviz/HapticsConfig.h>
#include <boost/thread/recursive_mutex.hpp>

#include "cplane.h"

//struct kdtree;

//const int NUM_CLOUDS = 5;

typedef pcl::PointXYZRGB PointT;

class PointCloudObject : public cMesh
{
    //! A visible sphere that tracks the position of the proxy on the surface
    cShapeSphere m_projectedSphere;

    //! The function that renders the actual point cloud.
    void renderCloud(const int a_renderMode);

    // Stuff I added
    bool m_isReady;

    bool evaluateCloud( const std::vector< std::vector<int> > &indices,
                        const cVector3d &position, double &func, cVector3d &grad, int basis_type);

    bool seed2surface( const std::vector< std::vector<int> > &indices,
                       cVector3d &p, cVector3d &grad, double epsilon, int max_steps = 50);

    int computeNeighborIndices( PointT position, float radius,
                                std::vector< std::vector <int> >    &k_indices,
                                std::vector< std::vector <float> >  &k_sqr_distances);

    
public:
    PointCloudObject(cWorld *a_world);

    //! Populates m_vertices from a pcl::PointCloud
    bool createFromCloud(const pcl::PointCloud<PointT> &cloud);

    //! Create a polygon mesh from an implicit surface function for visual rendering.
    void createFromFunction(double (*f)(double, double, double),
                            cVector3d a_lowerBound, cVector3d a_upperBound,
                            double a_granularity);


    //! Populates m_vertices with a (non-uniform) spherical shell of points.
    void createUniformPlane(int x_count, int y_count, double linear_step, double noise_size = 0.01);

    //! Populates m_vertices with a (non-uniform) spherical shell of points.
    void createSphericalShell(double radius, double angle_step, double min_lat, double max_lat, double min_lon, double max_lon, double noise_size = 0.01);
    
    //! Populate m_vertices with a Open-top Box
    void createOpenTopBox(double edge_size, double linear_step, double noise_size);

    //! Populates m_vertices with a plane of points of non-uniform density.
    void createGradientPlane(float width, float length, float initial_step, float growth_rate, double noise_size = 0.01);

    //! Contains code for graphically rendering this object in OpenGL.
    virtual void render(const int a_renderMode=CHAI_RENDER_MODE_RENDER_ALL);

    //! update the bounding box for the points in this cloud.
    virtual void updateBoundaryBox();

    //! Update the geometric relationship between the tool and the current object.
    virtual void computeLocalInteraction(const cVector3d& a_toolPos,
                                         const cVector3d& a_toolVel,
                                         const unsigned int a_IDN);

//    bool loadFromFile( const char* filepath, double scale = 1.0, bool mean_shift = false);

    bool updateShape(int shape, bool force_refresh = false);

    //! Helper function to compute average spacing, etc.
    void computeCloudSpecs();

    void applyLastCloud();

    //! An array of pointers to a set of vertices
    vector<cVertex> m_neighbors;


    bool m_useFriction;
    bool m_showTangent;
    bool m_normalsFlip;
    float m_active_radius;
    bool m_refresh_cloud;

    int m_basisType;
    int m_cloudType;
    int m_shape;
    hviz::HapticsConfig* m_configPtr;

    boost::recursive_mutex mutex_;


    //double metaball_threshold;
    //double basis_size;
    double scaleZ;

    cPlane*             tPlane;
//    cVector3d           tPlaneNormal;
    //bool m_inContact;

    //kdtree *tree;
    unsigned int unused_cloud_index;
    unsigned int last_cloud_index;
    //unsigned int unused_cloud_index;
    //boost::shared_ptr< pcl::octree::OctreePointCloud<PointT> > tree;
    std::list< pcl::KdTree<PointT>::Ptr >           trees;
    std::list< pcl::PointCloud<PointT>::Ptr >       m_cloud_points;
    std::list< pcl::PointCloud<pcl::Normal>::Ptr >  m_cloud_normals;

    pcl::KdTree<PointT>::Ptr            last_tree;
    pcl::PointCloud<PointT>::Ptr        last_points;
    pcl::PointCloud<pcl::Normal>::Ptr   last_normals;

    cGeneric3dofPointer *m_tool;

    //! Array of points.
    //vector<Point3> m_points;
};

#endif
