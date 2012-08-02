#ifndef _TF_SCENEGRAPH_OBJECT_H_
#define _TF_SCENEGRAPH_OBJECT_H_


#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <geometry_msgs/PoseStamped.h>

#include <ros/ros.h>


namespace tf {

class SceneGraphNode{


public:
  // Methods

    SceneGraphNode(const std::string &frame_id, tf::TransformListener *tfl, tf::TransformBroadcaster *tfb)
        : tfl_(tfl), tfb_(tfb), parent_(0)
    {
        transform_.child_frame_id_ = frame_id;
        transform_.setIdentity();
    }

    virtual ~SceneGraphNode()
    {
        // cleanup...
    }

    virtual void setPosition(const tf::Vector3 &position)   { transform_.setOrigin(position); }
    virtual void setQuaternion(const tf::Quaternion &quaternion)   { transform_.setRotation(quaternion); }
    virtual void setTransform(const tf::Transform &transform)
    {
        transform_.setOrigin(transform.getOrigin());
        transform_.setRotation(transform.getRotation());
    }

    virtual tf::Vector3     getPosition() const       { return transform_.getOrigin();   }
    virtual tf::Quaternion  getQuaternion() const     { return transform_.getRotation(); }
    virtual tf::Transform   getTransform() const      { return transform_; }


    tf::SceneGraphNode* accessChild(const std::string &key)
    {
        tf::SceneGraphNode* node = 0;
        if(getFrameId() == key) return this;

        // Recursively check children
        std::map<std::string, tf::SceneGraphNode*>::iterator it = children_.begin();
        for( ; it != children_.end(); it++)
        {
            node = it->second->accessChild(key);
            if(node) return node;
        }
        return 0;
    }

    void addChild(tf::SceneGraphNode *node)
    {
        node->setParent(this);
        children_[node->getFrameId()] = node;
    }

    bool removeChild(tf::SceneGraphNode *node)
    {
        std::map<std::string, tf::SceneGraphNode*>::iterator it = children_.begin();
        for( ; it != children_.end(); it++)
        {
            if(it->second == node)
            {
                children_.erase(it);
                return true;
            }
        }
        return false;
    }

    bool removeChild(const std::string &key)
    {
        return (bool)children_.erase(key); // returns 1 if it erased a child, 0 otherwise
    }

    void printChildren(const bool &recursive = false)
    {
        std::vector<std::string> names;
        names.reserve(children_.size());

        // Get children
        std::map<std::string, tf::SceneGraphNode*>::iterator it = children_.begin();
        for( ; it != children_.end(); it++)
        {
            names.push_back(it->first);
        }


        printf("Frame %s has %zd children: ", getFrameId().c_str(), names.size());
        std::string children_string = "";
        for(size_t i = 0; i < names.size(); i++)
        {
            children_string += names[i] + " ";
        }
        printf("%s\n", children_string.c_str());

        if(recursive)
        {
            // Recurse down
            std::map<std::string, tf::SceneGraphNode*>::iterator it = children_.begin();
            for( ; it != children_.end(); it++)
            {
                it->second->printChildren(recursive);
            }
        }
    }

    std::string getFrameId()
    {
        return transform_.child_frame_id_;
    }

    std::string getParentFrameId()
    {
        return transform_.frame_id_;
    }


    void publishTransformTree(const ros::Time now)
    {
        std::vector<tf::StampedTransform> transforms;
        addTransformsToVector(now, transforms);
        tfb_->sendTransform(transforms);
    }

    void addTransformsToVector(const ros::Time now, std::vector<tf::StampedTransform> &transforms)
    {
        // Add this node to the list
        transform_.stamp_ = now;
        transforms.push_back(transform_);

        // Recursively add all children to the list
        std::map<std::string, tf::SceneGraphNode*>::iterator it = children_.begin();
        for( ; it != children_.end(); it++)
        {
            it->second->addTransformsToVector(now, transforms);
        }
    }

    virtual tf::StampedTransform getTransform()
    {
        return transform_;
    }

    virtual void drawSelf()
    {


    }

    virtual void publishMarkers( const bool &recursive)
    {
        //
        drawSelf();

        if(recursive)
        {
            std::map<std::string, tf::SceneGraphNode*>::iterator it = children_.begin();
            for( ; it != children_.end(); it++)
            {
                it->second->publishMarkers(recursive);
            }
        }
      // Default implementation does nothing because I suppose we could have different geometry representations.
    }

protected:
    // Methods
    void setParent(tf::SceneGraphNode* const parent)
    {
        if(parent_)
            parent_->removeChild(getFrameId());
        parent_ = parent;
        setParentFrameId(parent->getFrameId());
    }

    void setParentFrameId(const std::string &parent_id)
    {
        transform_.frame_id_ = parent_id;
    }

  // Members
  tf::StampedTransform transform_;

  tf::TransformListener *tfl_;
  tf::TransformBroadcaster *tfb_;

  tf::SceneGraphNode *parent_;
  std::map<std::string, tf::SceneGraphNode*> children_;

};


}  // namespace something

#endif // header
