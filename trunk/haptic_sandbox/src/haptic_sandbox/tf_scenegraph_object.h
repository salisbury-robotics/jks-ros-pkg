
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <geometry_msgs/PoseStamped.h>

namespace tf {

class SceneGraphNode{


public:
  // Methods

    SceneGraphNode(tf::TransformListener *tfl, tf::TransformBroadcaster *tfb)
        : tfl_(tfl), tfb_(tfb)
    {



    }

    ~ScenGraphNode()
    {
        // cleanup...
    }

    bool addChild(tf::SceneGraphNode *node)
    {
        node->setParent(this);
        children_.at(node->getFrameId()) = node;
    }

    bool removeChild(tf::SceneGraphNode *node)
    {


        //        // Recursively add all children to the list
//        std::list<tf::SceneGraphNode>::iterator it = children_.begin();
//        while(it != children_.end())
//        {
//            (*it).updateTransforms(now, transforms);
//        }
    }

    bool removeChild(std::string )
    {


        //        // Recursively add all children to the list
//        std::list<tf::SceneGraphNode>::iterator it = children_.begin();
//        while(it != children_.end())
//        {
//            (*it).updateTransforms(now, transforms);
//        }
    }

    std::string getFrameId()
    {
        return transform_.child_frame_id_;
    }

    std::string getParentFrameId()
    {
        return transform_.frame_id_;
    }


    void updateTransforms(const ros::Time now)
    {
        std::vector<tf::StampedTransform> transforms;
        getTransforms(now, transforms);
        tfb_->sendTransform(transforms);
    }

    void getTransforms(const ros::Time now, std::vector<tf::StampedTransform> &transforms)
    {
        // Add this node to the list
        transform_.stamp_ = now;
        transforms->push_back(transform_);

        // Recursively add all children to the list
        std::map<std::string, tf::SceneGraphNode*>::iterator it = children_.begin();
        while(it != children_.end())
        {
            it->second->updateTransforms(now, transforms);
        }
    }

//    void updateTransforms(const ros::Time now, std::vector<tf::StampedTransform> *transforms = 0)
//    {
//        // -------------------------------------------
//        // Check if this is the root node:

//        bool delete_transforms = false;
//        if(!transforms)
//        {
//            transforms = new std::vector<tf::StampedTransform>();
//            delete_transforms = true;
//        }

//        // -------------------------------------------
//        // All elements do this:
//        transform_.stamp_ = now;
//        transforms->push_back(transform_);

//        std::list<tf::SceneGraphNode>::iterator it = children_.begin();
//        while(it != children_.end())
//        {
//            (*it).updateTransforms(now, transforms);
//        }
//        // -------------------------------------------

//        if(delete_transforms)
//        {
//            tfb_->sendTransform(*transforms);
//            delete transforms;
//        }
//    }


protected:
    // Methods
    setParent(const tf::SceneGraphNode *parent)
    {
        parent_->removeChild(this);
        parent_ = parent;
    }

  // Members
  //std::string parent_frame_id_;
  //std::string frame_id_;
  tf::StampedTransform transform_;

  tf::TransformBroadcaster *tfb_;
  tf::TransformListener *tfl_;

  SceneGraphNode *parent_;
  std::map<std::string, tf::SceneGraphNode*> children_;

};





}  // namespace something
