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
    \version   $MAJOR.$MINOR.$RELEASE $Rev: 840 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
//---------------------------------------------------------------------------
#include "chai3d.h"
//---------------------------------------------------------------------------
#include "GEL3D.h"
//---------------------------------------------------------------------------
#ifndef MACOSX
#include "GL/glut.h"
#else
#include "GLUT/glut.h"
#endif
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
// DECLARED CONSTANTS
//---------------------------------------------------------------------------

// mouse menu options (right button)
const int OPTION_FULLSCREEN     = 1;
const int OPTION_WINDOWDISPLAY  = 2;
const int OPTION_SHOWSKELETON   = 3;
const int OPTION_HIDESKELETON   = 4;


//---------------------------------------------------------------------------
// DECLARED VARIABLES
//---------------------------------------------------------------------------

// a world that contains all objects of the virtual environment
cWorld* world;

// a camera that renders the world in a window display
cCamera* camera;

// a light source to illuminate the objects in the virtual scene
cSpotLight *light;

// a little "chai3d" bitmap logo at the bottom of the screen
cBitmap* logo;

// width and height of the current window display
int displayW  = 0;
int displayH  = 0;

// a haptic device handler
cHapticDeviceHandler* handler;

// a haptic device
cGenericHapticDevice* hapticDevice;

// virtual tool
cToolCursor* tool;

// force scale factor
double deviceForceScale;

// scale factor between the device workspace and cursor workspace
double workspaceScaleFactor;

// desired workspace radius of the virtual cursor
double cursorWorkspaceRadius;

// status of the main simulation haptics loop
bool simulationRunning = false;

// simulation clock
cPrecisionClock simClock;

// root resource path
string resourceRoot;

// has exited haptics simulation thread
bool simulationFinished = false;


//---------------------------------------------------------------------------
// DECLARED MACROS
//---------------------------------------------------------------------------
// convert to resource path
#define RESOURCE_PATH(p)    (char*)((resourceRoot+string(p)).c_str())


//---------------------------------------------------------------------------
// GEL 3D
//---------------------------------------------------------------------------

// deformable world
cGELWorld* defWorld;

// object mesh
cGELMesh* defObject;

// dynamic nodes
cGELSkeletonNode* nodes[20][20];

// device  model
cShapeSphere* device;
double deviceRadius;

// radius of the dynamic model sphere (GEM)
double radius;

// stiffness properties between the haptic device tool and the model (GEM)
double stiffness;


//---------------------------------------------------------------------------
// DECLARED FUNCTIONS
//---------------------------------------------------------------------------

// callback when the window display is resized
void resizeWindow(int w, int h);

// callback when a keyboard key is pressed
void keySelect(unsigned char key, int x, int y);

// callback when the right mouse button is pressed to select a menu item
void menuSelect(int value);

// function called before exiting the application
void close(void);

// main graphics timer and callback
void graphicsTimer(int data);
void updateGraphics(void);

// main haptics loop
void updateHaptics(void);

// compute forces between tool and environment
cVector3d computeForce(const cVector3d& a_cursor,
                       double a_cursorRadius,
                       const cVector3d& a_spherePos,
                       double a_radius,
                       double a_stiffness);


//===========================================================================
/*
    DEMO:    GEM_membrane.cpp

    This application illustrates the use of the GEM libraries to simulate
    deformable object. In this example we load a simple mesh object and
    build a dynamic skeleton composed of volumetric spheres and 3 dimensional
    springs which model torsion, flexion and elongation properties.
*/
//===========================================================================

int main(int argc, char* argv[])
{
    //-----------------------------------------------------------------------
    // INITIALIZATION
    //-----------------------------------------------------------------------

    printf ("\n");
    printf ("-----------------------------------\n");
    printf ("CHAI3D\n");
    printf ("Demo: 54-GEL-ring\n");
    printf ("Copyright 2003-2012\n");
    printf ("-----------------------------------\n");
    printf ("\n\n");
    printf ("Keyboard Options:\n\n");
    printf ("[1] - Show GEL Skeleton\n");
    printf ("[2] - Hide GEL Skeleton\n");
    printf ("[x] - Exit application\n");
    printf ("\n\n");

    // parse first arg to try and locate resources
    resourceRoot = string(argv[0]).substr(0,string(argv[0]).find_last_of("/\\")+1);


    //-----------------------------------------------------------------------
    // 3D - SCENEGRAPH
    //-----------------------------------------------------------------------

    // create a new world.
    world = new cWorld();

    // set the background color of the environment
    // the color is defined by its (R,G,B) components.
    world->m_backgroundColor.setWhite();

    // create a camera and insert it into the virtual world
    camera = new cCamera(world);
    world->addChild(camera);

    // position and oriente the camera
    camera->set( cVector3d (2.5, 0.0, 0.0),    // camera position (eye)
                 cVector3d (0.0, 0.0,-0.5),    // lookat position (target)
                 cVector3d (0.0, 0.0, 1.0));   // direction of the "up" vector

    // set the near and far clipping planes of the camera
    // anything in front/behind these clipping planes will not be rendered
    camera->setClippingPlanes(0.01, 10.0);

    // Enable shadow casting
    camera->setUseShadowCasting(true);

    // create a light source
    light = new cSpotLight(world);

    // attach light to camera
    world->addChild(light);    

    // enable light source
    light->setEnabled(true);                   

    // position the light source
    light->setLocalPos( 1.5, 0.40, 1.5);             

    // define the direction of the light beam
    light->setDir(-2.0,-0.5,-2.0);             

    // enable this light source to generate shadows
    light->setShadowMapEnabled(true);


    //-----------------------------------------------------------------------
    // HAPTIC DEVICES / TOOLS
    //-----------------------------------------------------------------------

    // create a haptic device handler
    handler = new cHapticDeviceHandler();

    // get access to the first available haptic device
    handler->getDevice(hapticDevice, 0);

    // retrieve information about the current haptic device
    cHapticDeviceInfo info = hapticDevice->getSpecifications();

    // if the haptic devices carries a gripper, enable it to behave like a user switch
    hapticDevice->setEnableGripperUserSwitch(true);

    hapticDevice->open();

    /*
    // create a 3D tool and add it to the world
    tool = new cToolCursor(world);
    world->addChild(tool);

    // connect the haptic device to the tool
    tool->setHapticDevice(hapticDevice);

    // initialize tool by connecting to haptic device
    tool->start();

    // map the physical workspace of the haptic device to a larger virtual workspace.
    tool->setWorkspaceRadius(1.0);

    // define the radius of the tool (sphere)
    double toolRadius = 0.01;

    // define a radius for the tool
    tool->setRadius(toolRadius);

    // hide the device sphere. only show proxy.
    tool->setShowContactPoints(true, false);

    // enable if objects in the scene are going to rotate of translate
    // or possibly collide against the tool. If the environment
    // is entirely static, you can set this parameter to "false"
    tool->enableDynamicObjects(true);

    // read the scale factor between the physical workspace of the haptic
    // device and the virtual workspace defined for the tool
    double workspaceScaleFactor = tool->getWorkspaceScaleFactor();

    // define a maximum stiffness that can be handled by the current
    // haptic device. The value is scaled to take into account the
    // workspace scale factor
    double stiffnessMax = info.m_maxLinearStiffness / workspaceScaleFactor;
    */
    // define a scale factor between the force perceived at the cursor and the
    // forces actually sent to the haptic device
    deviceForceScale = 0.1 * info.m_maxLinearForce;

    // create a large sphere that represents the haptic device
    deviceRadius = 0.1;
    device = new cShapeSphere(deviceRadius);
    world->addChild(device);
    device->m_material->m_ambient.set(0.4, 0.4, 0.4, 0.7);
    device->m_material->m_diffuse.set(0.7, 0.7, 0.7, 0.7);
    device->m_material->m_specular.set(1.0, 1.0, 1.0, 0.7);
    device->m_material->setShininess(100);
    stiffness = 40;


    //-----------------------------------------------------------------------
    // COMPOSE THE VIRTUAL SCENE
    //-----------------------------------------------------------------------

    // create a world which supports deformable object
    defWorld = new cGELWorld();
    world->addChild(defWorld);

    // Play with thse numbers carefully!
    cGELSkeletonNode::default_kDampingPos = 0.1;
    cGELSkeletonNode::default_kDampingRot = 0.1;
    defWorld->m_integrationTime = 0.001;

    // create a deformable mesh
    defObject = new cGELMesh();
    defWorld->m_gelMeshes.push_front(defObject);
    bool fileload;
    fileload = defObject->loadFromFile(RESOURCE_PATH("resources/models/ring/ring2.obj"));
    if (!fileload)
    {
        #if defined(_MSVC)
        fileload = defObject->loadFromFile("../../../bin/resources/models/ring/ring2.obj");
        #endif
    }
    if (!fileload)
    {
        printf("Error - 3D Model failed to load correctly.\n");
        close();
        return (-1);
    }

    defObject->scale(0.01, true);

    // set some material color on the object
    cMaterial mat;
    mat.m_ambient.set(0.2, 0.2, 0.2);
    mat.m_diffuse.set(0.6, 0.6, 0.6);
    mat.m_ambient.set(1.0, 1.0, 1.0);
    mat.setShininess(100);
    defObject->setMaterial(mat, true);

    // let's create a some environment mapping
    cTexture2d* texture = new cTexture2d();
    fileload = texture->loadFromFile(RESOURCE_PATH("resources/images/spheremap-6.jpg"));
    if (!fileload)
    {
        #if defined(_MSVC)
        fileload = texture->loadFromFile("../../../bin/resources/images/spheremap-6.jpg");
        #endif
    }
    if (!fileload)
    {
        printf("Error - Texture image failed to load correctly.\n");
        close();
        return (-1);
    }

    texture->setEnvironmentMode(GL_DECAL);
    texture->setSphericalMappingEnabled(true);
    defObject->setTexture(texture, true);
    defObject->setUseTexture(true, true);

    // build dynamic vertices
    defObject->buildVertices();
    defObject->m_useSkeletonModel = true;
   // defObject->setWireMode(true, true);

    // setup default values for nodes
    cGELSkeletonNode::default_radius        = 0.07;
    cGELSkeletonNode::default_kDampingPos   = 1.0;
    cGELSkeletonNode::default_kDampingRot   = 0.05;
    cGELSkeletonNode::default_mass          = 0.01;  // [kg]
    cGELSkeletonNode::default_showFrame     = false;
    cGELSkeletonNode::default_color.set(1.0, 0.6, 0.6);
    cGELSkeletonNode::default_useGravity    = true;
    cGELSkeletonNode::default_gravity.set(0.00, 0.00, -1.00);
    radius = cGELSkeletonNode::default_radius;

    // create an array of nodes
    double radius = 0.45;
    for (int y=0; y<16; y++)
    {
        for (int x=0; x<2; x++)
        {
            cGELSkeletonNode* newNode = new cGELSkeletonNode();
            nodes[x][y] = newNode;
           // nodes[x][y]->m_fixed = true;
            defObject->m_nodes.push_front(newNode);
            newNode->m_pos.set( (0.06 + 0.12*(double)x), 
                                radius * cCosDeg(y*(double)360.0/16.0), 
                                radius * cSinDeg(y*(double)360.0/16.0)
                                );
        }
    }

    // fix 4 nodes 9corners)
    nodes[0][0]->m_fixed = true;
  //  nodes[0][9]->m_fixed = true;
  //  nodes[9][0]->m_fixed = true;
  //  nodes[9][9]->m_fixed = true;

    // setup default values for links
    cGELSkeletonLink::default_kSpringElongation = 400.0; // [N/m]
    cGELSkeletonLink::default_kSpringFlexion    = 1.0;   // [Nm/RAD]
    cGELSkeletonLink::default_kSpringTorsion    = 0.2;   // [Nm/RAD]
    cGELSkeletonLink::default_color.set(0.2, 0.2, 1.0);

    // create links between nodes
    for (int y=0; y<16; y++)
    {
        for (int x=0; x<1; x++)
        {
            cGELSkeletonLink* newLinkX0 = new cGELSkeletonLink(nodes[x+0][y+0], nodes[x+1][y+0]);
            cGELSkeletonLink* newLinkY0 = new cGELSkeletonLink(nodes[x+0][y+0], nodes[x+0][(y+1)%16]);
            cGELSkeletonLink* newLinkY1 = new cGELSkeletonLink(nodes[x+1][y+0], nodes[x+1][(y+1)%16]);
            defObject->m_links.push_front(newLinkX0);
            defObject->m_links.push_front(newLinkY0);
            defObject->m_links.push_front(newLinkY1);
        }
    }
   
    // connect skin (mesh) to skeleton (GEM)
    defObject->connectVerticesToSkeleton(false);

    // show/hide underlying dynamic skeleton model
    defObject->m_showSkeletonModel = false;

    // define the integration time constant of the dynamics model
    defWorld->m_integrationTime = 0.005;

    world->computeGlobalPositions(true);

    //-----------------------------------------------------------------------
    // OPEN GL - WINDOW DISPLAY
    //-----------------------------------------------------------------------

    // initialize GLUT
    glutInit(&argc, argv);

    // retrieve the resolution of the computer display and estimate the position
    // of the GLUT window so that it is located at the center of the screen
    int screenW = glutGet(GLUT_SCREEN_WIDTH);
    int screenH = glutGet(GLUT_SCREEN_HEIGHT);
    int windowW = 0.7 * screenH;
    int windowH = 0.7 * screenH;
    int windowPosX = (screenW - windowH) / 2;
    int windowPosY = (screenH - windowW) / 2;

    // initialize the OpenGL GLUT window
    glutInitWindowPosition(windowPosX, windowPosY);
    glutInitWindowSize(windowW, windowH);
    glutInitDisplayMode(GLUT_RGB | GLUT_DEPTH | GLUT_DOUBLE);
    glutCreateWindow(argv[0]);
    glutDisplayFunc(updateGraphics);
    glutKeyboardFunc(keySelect);
    glutReshapeFunc(resizeWindow);
    glutSetWindowTitle("CHAI3D");

    // create a mouse menu (right button)
    glutCreateMenu(menuSelect);
    glutAddMenuEntry("full screen", OPTION_FULLSCREEN);
    glutAddMenuEntry("window display", OPTION_WINDOWDISPLAY);
    glutAddMenuEntry("show skeleton", OPTION_SHOWSKELETON);
    glutAddMenuEntry("hide skeleton", OPTION_HIDESKELETON);
    glutAttachMenu(GLUT_RIGHT_BUTTON);


    //-----------------------------------------------------------------------
    // START SIMULATION
    //-----------------------------------------------------------------------

    // simulation in now running
    simulationRunning = true;

    // create a thread which starts the main haptics rendering loop
    cThread* hapticsThread = new cThread();
    hapticsThread->start(updateHaptics, CTHREAD_PRIORITY_HAPTICS);

    // start the main graphics rendering loop
    glutTimerFunc(30, graphicsTimer, 0);
    glutMainLoop();

    // close everything
    close();

    // exit
    return (0);
}

//---------------------------------------------------------------------------

void resizeWindow(int w, int h)
{
    // update the size of the viewport
    displayW = w;
    displayH = h;
    glViewport(0, 0, displayW, displayH);
}

//---------------------------------------------------------------------------

void keySelect(unsigned char key, int x, int y)
{
    // escape key
    if ((key == 27) || (key == 'x'))
    {
        // close everything
        close();

        // exit application
        exit(0);
    }

    // option 1:
    if (key == '1')
    {
        // show skeleton
        defObject->m_showSkeletonModel = true;
    }

    // option 2:
    if (key == '2')
    {
        // hide skeleton
        defObject->m_showSkeletonModel = false;
    }
}

//---------------------------------------------------------------------------

void menuSelect(int value)
{
    static int _wx, _wy, _ww, _wh;

    switch (value)
    {
        // enable full screen display
        case OPTION_FULLSCREEN:
            _wx = glutGet(GLUT_WINDOW_X);
            _wy = glutGet(GLUT_WINDOW_Y);
            _ww = glutGet(GLUT_WINDOW_WIDTH);
            _wh = glutGet(GLUT_WINDOW_HEIGHT);
            glutFullScreen();
            break;

        // reshape window to original size
        case OPTION_WINDOWDISPLAY:
            glutPositionWindow(_wx, _wy);
            glutReshapeWindow(_ww, _wh);
            break;

        // show the dynamic skeleton of the object
        case OPTION_SHOWSKELETON:
            defObject->m_showSkeletonModel = true;
            break;

        // hide the dynamic skeleton of the object
        case OPTION_HIDESKELETON:
            defObject->m_showSkeletonModel = false;
            break;
    }
}

//---------------------------------------------------------------------------

void close(void)
{
    // stop the simulation
    simulationRunning = false;

    // wait for graphics and haptics loops to terminate
    while (!simulationFinished) { cSleepMs(100); }

    // close haptic device
    hapticDevice->close();
}

//---------------------------------------------------------------------------

void graphicsTimer(int data)
{
    // inform the GLUT window to call updateGraphics again (next frame)
    if (simulationRunning)
    {
        glutPostRedisplay();
    }

    glutTimerFunc(30, graphicsTimer, 0);
}

//---------------------------------------------------------------------------

void updateGraphics(void)
{
    // update mesh of deformable model
    defWorld->updateSkins();
    defObject->computeAllNormals();

    // render world
    camera->renderView(displayW, displayH);

  //  light->m_shadowMap->copyMapFromGPUtoImage(logo->m_image);

    // Swap buffers
    glutSwapBuffers();

    // check for any OpenGL errors
    GLenum err;
    err = glGetError();
    if (err != GL_NO_ERROR) printf("Error:  %s\n", gluErrorString(err));
}

//---------------------------------------------------------------------------

void updateHaptics(void)
{
    // reset clock
    simClock.reset();
    workspaceScaleFactor = 3.0;

    // main haptic simulation loop
    while(simulationRunning)
    {
        // stop the simulation clock
        simClock.stop();

        // read the time increment in seconds
        double timeInterval = simClock.getCurrentTimeSeconds();
        if (timeInterval > 0.001) { timeInterval = 0.001; }

        // restart the simulation clock
        simClock.reset();
        simClock.start();

        // read position from haptic device
        cVector3d pos;
        hapticDevice->getPosition(pos);
        pos.mul(workspaceScaleFactor);
        device->setLocalPos(pos);

        // init temp variable
        cVector3d force;
        force.zero();

        // compute reaction forces
        for (int y=0; y<16; y++)
        {
            for (int x=0; x<2; x++)
            {
               cVector3d nodePos = nodes[x][y]->m_pos;
               cVector3d f = computeForce(pos, deviceRadius, nodePos, radius, stiffness);
               cVector3d tmpfrc = cNegate(f);
               nodes[x][y]->setExternalForce(tmpfrc);
               force.add(f);
            }
        }
    
        // integrate dynamics
        defWorld->updateDynamics(timeInterval);

        // scale force
        force.mul(deviceForceScale);

        // send forces to haptic device
        hapticDevice->setForce(force);
    }

    // exit haptics thread
    simulationFinished = true;
}

//---------------------------------------------------------------------------

cVector3d computeForce(const cVector3d& a_cursor,
                       double a_cursorRadius,
                       const cVector3d& a_spherePos,
                       double a_radius,
                       double a_stiffness)
{

    // compute the reaction forces between the tool and the ith sphere.
    cVector3d force;
    force.zero();
    cVector3d vSphereCursor = a_cursor - a_spherePos;

    // check if both objects are intersecting
    if (vSphereCursor.length() < 0.0000001)
    {
        return (force);
    }

    if (vSphereCursor.length() > (a_cursorRadius + a_radius))
    {
        return (force);
    }

    // compute penetration distance between tool and surface of sphere
    double penetrationDistance = (a_cursorRadius + a_radius) - vSphereCursor.length();
    cVector3d forceDirection = cNormalize(vSphereCursor);
    force = cMul( penetrationDistance * a_stiffness, forceDirection);

    // return result
    return (force);
}
