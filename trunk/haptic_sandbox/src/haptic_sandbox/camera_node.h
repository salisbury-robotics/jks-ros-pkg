#ifndef _CAMERA_NODE_H_
#define _CAMERA_NODE_H_


#include <haptic_sandbox/tf_scenegraph_object.h>
#include <Eigen/Geometry>


namespace something {

typedef tf::Vector3 Vector3;
typedef tf::Quaternion Quaternion;
typedef tf::Transform Transform;

class CameraNode: public tf::SceneGraphNode {


public:
  // Methods only!

  // Constructor
    CameraNode(const std::string &frame_id,
               tf::TransformListener *tfl, tf::TransformBroadcaster *tfb)
      : SceneGraphNode(frame_id, tfl, tfb)
    {
        init();
    }

    void init()
    {

    }



protected:
    // Methods

protected:
    // Members


};

}  // namespace something

#endif
