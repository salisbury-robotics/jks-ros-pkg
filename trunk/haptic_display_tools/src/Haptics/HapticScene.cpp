#include "HapticScene.h"

using namespace std;

HapticScene::HapticScene()
{
//    m_chai3dWorld = new cWorld();
}

HapticScene::~HapticScene()
{
    clearDisplays();
    clearNodes();

    // destroy the world
    // TODO:  Can't destroy the world, or else it double-deletes!
//    if (m_chai3dWorld) delete m_chai3dWorld;    
}

HapticDisplay *HapticScene::addHapticDisplay(cGenericHapticDevice *device)
{
    // TODO: this way of assigning display identifiers should be improved --
    //       it will not allow for correct removal and re-addition
    int identifier = m_displays.size();
    m_displays.push_back(new HapticDisplay(device, identifier));
    return m_displays.back();
}

void HapticScene::clearDisplays()
{
    // destroy haptic display objects
    for (list<HapticDisplay *>::iterator it = m_displays.begin();
         it != m_displays.end(); it = m_displays.erase(it))
    {
        delete *it;
    }
}

void HapticScene::clearNodes()
{
    // destroy haptic nodes objects
    for (list<HapticNode *>::iterator it = m_nodes.begin();
         it != m_nodes.end(); it = m_nodes.erase(it))
    {
        delete *it;
    }
}

void HapticScene::clearForces()
{
    // destroy haptic display objects
    for (list<HapticDisplay *>::iterator it = m_displays.begin();
         it != m_displays.end(); ++it)
    {
        (*it)->clearForces();
    }
}

void HapticScene::update()
{
//    m_chai3dWorld->computeGlobalPositions(true);

    // update each of the attached haptic displays
    for (list<HapticDisplay *>::iterator it = m_displays.begin();
         it != m_displays.end(); ++it)
    {
        (*it)->update();

        // update each of the haptic nodes in the scene with this device
        for (list<HapticNode *>::iterator jt = m_nodes.begin();
             jt != m_nodes.end(); ++jt)
        {
            (*jt)->update(*it);
        }

        if (m_nodes.empty())
            (*it)->setProxyPosition((*it)->toolPosition());

        // apply any computed forces on the haptic display
        (*it)->applyForces();
    }
}
