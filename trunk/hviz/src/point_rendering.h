#ifndef _POINT_RENDERING_H_
#define _POINT_RENDERING_H_

#include "chai3d.h"
#include "pcl/point_types.h"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <hviz/HapticsConfig.h>


class PointRendering : public cMesh
{
    //! A visible sphere that tracks the position of the proxy on the surface
    cShapeSphere m_projectedSphere;

    //! The function that renders the actual point cloud.
    //void renderCloud(const int a_renderMode);

    //! Prepares a cloud in an efficient data structure.
    void prepareKdTree();

    // Stuff I added
    bool m_isReady;

    kdtree *tree;
    pcl::KdTreeFLANN<PointT>::Ptr pclTree;


public:
    PointCloudObject(cWorld *a_world);

    //! Populates m_vertices from a pcl::PointCloud
    bool createFromCloud(pcl::PointCloud<PointT> &cloud);

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

    //! Contains code for graphically rendering this object in OpenGL.
    virtual void render(const int a_renderMode=CHAI_RENDER_MODE_RENDER_ALL);

    //! update the bounding box for the points in this cloud.
    virtual void updateBoundaryBox();

    //! Update the geometric relationship between the tool and the current object.
    virtual void computeLocalInteraction(const cVector3d& a_toolPos,
                                         const cVector3d& a_toolVel,
                                         const unsigned int a_IDN);

    bool evaluateCloud( const cVector3d &position, double &func, cVector3d &grad, int basis_type);
    //bool evaluateSurfelCloud( const cVector3d &position, double &func, cVector3d &grad, int basis_type);

    bool seed2surface( cVector3d &p, cVector3d &grad, double epsilon, int max_steps = 50);

    bool loadFromFile( const char* filepath, double scale = 1.0, bool mean_shift = false);

    void getPointCloud();

    bool updateShape(int shape, bool force_refresh = false);

    //! Helper function to compute average spacing, etc.
    void computeCloudSpecs();


    //! An array of pointers to a set of vertices
    vector<cVertex*> m_neighbors;
    vector<cColorf> m_neighbors_colors;

    bool m_useFriction;
    bool m_showTangent;
    bool m_normalsFlip;

    int m_basisType;
    int m_cloudType;
    int m_shape;
    haptic_points::HapticsConfig* m_configPtr;

    double scaleZ;

    cMesh*              tPlane;
    cVector3d           tPlaneNormal;
    bool m_inContact;

    cGeneric3dofPointer *m_tool;

    //! Array of points.
    //vector<Point3> m_points;
};










#endif _POINT_RENDERING_H_
