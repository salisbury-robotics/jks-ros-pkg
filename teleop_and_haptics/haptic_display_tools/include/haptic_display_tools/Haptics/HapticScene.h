#ifndef HAPTICSCENE_H
#define HAPTICSCENE_H

#include "HapticDisplay.h"
#include <scenegraph/CWorld.h>
#include <list>

// A HapticScene is a loose wrapper around a cWorld object.  It holds one or
// more HapticDisplays and a collection of objects that can be rendered.
// The design of this class WILL evolve over time!

class HapticNode
{
public:
    virtual ~HapticNode() { }
    virtual void update(HapticDisplay *display) = 0;
};

class HapticScene
{
protected:
//    cWorld                     *m_chai3dWorld;
    std::list<HapticDisplay *>  m_displays;
    std::list<HapticNode *>     m_nodes;

public:
    HapticScene();
    virtual ~HapticScene();

    // adds a new HapticDisplay to this scene, returning its pointer
    HapticDisplay *addHapticDisplay(cGenericHapticDevice *device);

    // adds a node to this haptic scene (takes ownership)
    void addNode(HapticNode *node) { m_nodes.push_back(node); }

    // methods to clear the scene of displays or nodes
    void clearDisplays();
    void clearNodes();
    void clearForces();

    // should be called once every haptic servo loop to render the scene
    virtual void update();
};

#endif // HAPTICSCENE_H
