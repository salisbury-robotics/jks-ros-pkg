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
#include "CODE.h"
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

// initial size (width/height) in pixels of the display window
const int WINDOW_SIZE_W         = 512;
const int WINDOW_SIZE_H         = 512;

// mouse menu options (right button)
const int OPTION_FULLSCREEN     = 1;
const int OPTION_WINDOWDISPLAY  = 2;


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

// a virtual tool representing the haptic device in the scene
cToolCursor* tool;

// radius of the tool proxy
double proxyRadius;

// a pointer the ODE object grasped by the tool
cODEGenericBody* graspObject;

// grasp position is respect to object
cVector3d graspPos;

// is grasp currently active?
bool graspActive = false;

// a small line used to display a grasp
cShapeLine* graspLine;

// maximum stiffness to be used with virtual objects in scene
double stiffnessMax;

// status of the main simulation haptics loop
bool simulationRunning = false;

// clock
cPrecisionClock startHapticsClock;

// ODE world
cODEWorld* ODEWorld;

// ODE object
cODEGenericBody* ODEGear1;
cODEGenericBody* ODEGear2;
cODEGenericBody* ODEAxis1;
cODEGenericBody* ODEAxis2;

cODEGenericBody* ODEGPlane0;
cODEGenericBody* ODEGPlane1;
cODEGenericBody* ODEGPlane2;
cODEGenericBody* ODEGPlane3;
cODEGenericBody* ODEGPlane4;
cODEGenericBody* ODEGPlane5;

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


//===========================================================================
/*
    DEMO:    ODE_mesh.cpp

    This example illustrates the use of the ODE framework for simulating
    haptic interaction with dynamic bodies. In this demo we introduce
    triangle based objects (cMesh) for which collision detection between
    triangles is computed. Collision detection using meshes can be a
    rather tricky business under ODE, so make sure to read more about the
    ODE API if you are experiencing some instabilities.

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
    printf ("Demo: ODE mesh\n");
    printf ("Copyright 2003-2011\n");
    printf ("-----------------------------------\n");
    printf ("\n\n");
    printf ("Instructions:\n\n");
    printf ("- Use haptic device and user switch to manipulate gear parts \n");
    printf ("\n\n");
    printf ("Keyboard Options:\n\n");
    printf ("[1] - Enable gravity\n");
    printf ("[2] - Disable gravity\n");
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
    world->setBackgroundColor(0.0, 0.0, 0.0);

    // create a camera and insert it into the virtual world
    camera = new cCamera(world);
    world->addChild(camera);

    // position and oriente the camera
    camera->set( cVector3d (1.2, 0.0, 0.8),    // camera position (eye)
                 cVector3d (0.0, 0.0,-0.5),    // lookat position (target)
                 cVector3d (0.0, 0.0, 1.0));   // direction of the "up" vector

    // set the near and far clipping planes of the camera
    // anything in front/behind these clipping planes will not be rendered
    camera->setClippingPlanes(0.01, 10.0);

    // create a light source and attach it to the camera
    light = new cSpotLight(world);
    camera->addChild(light);                   // attach light to camera
    light->setEnabled(true);                   // enable light source
    light->setLocalPos(cVector3d( 2.0, 0.5, 1.0));  // position the light source
    light->setDir(cVector3d(-2.0, 0.5, 1.0));  // define the direction of the light beam
    light->m_ambient.set(0.6, 0.6, 0.6);
    light->m_diffuse.set(0.8, 0.8, 0.8);
    light->m_specular.set(0.8, 0.8, 0.8);


    //-----------------------------------------------------------------------
    // 2D - WIDGETS
    //-----------------------------------------------------------------------

    // create a 2D bitmap logo
    logo = new cBitmap();

    // add logo to the front plane
    camera->m_frontLayer->addChild(logo);

    // load a "chai3d" bitmap image file
    logo->setImage (NEW_CHAI3D_LOGO());

    // position the logo at the bottom left of the screen (pixel coordinates)
    logo->setLocalPos(10, 10, 0);

    // scale the logo along its horizontal and vertical axis
    logo->setZoom(0.25, 0.25);

    // here we replace all black pixels (0,0,0) of the logo bitmap
    // with transparent black pixels (0, 0, 0, 0). This allows us to make
    // the background of the logo look transparent.
    logo->m_image->setTransparentColor(0x00, 0x00, 0x00, 0x00);

    // enable transparency
    logo->setUseTransparency(true);


    //-----------------------------------------------------------------------
    // HAPTIC DEVICES / TOOLS
    //-----------------------------------------------------------------------

    // create a haptic device handler
    handler = new cHapticDeviceHandler();

    // get access to the first available haptic device
    cGenericHapticDevice* hapticDevice;
    handler->getDevice(hapticDevice, 0);

    // retrieve information about the current haptic device
    cHapticDeviceInfo info;
    if (hapticDevice)
    {
        info = hapticDevice->getSpecifications();
    }

    // create a 3D tool and add it to the world
    tool = new cToolCursor(world);
    world->addChild(tool);

    // connect the haptic device to the tool
    tool->setHapticDevice(hapticDevice);

    // initialize tool by connecting to haptic device
    tool->start();

    // map the physical workspace of the haptic device to a larger virtual workspace.
    tool->setWorkspaceRadius(1.4);

    // define a radius for the tool (graphical display)
    tool->setRadius(0.015);

    // hide the device sphere. only show proxy.
    tool->setShowContactPoints(true, false);

    // set the physical readius of the proxy.
    proxyRadius = 0.0;
    tool->setRadiusContact(proxyRadius);
    //tool->m_proxyPointForceModel->m_collisionSettings.m_checkBothSidesOfTriangles = false;

    // enable if objects in the scene are going to rotate of translate
    // or possibly collide against the tool. If the environment
    // is entirely static, you can set this parameter to "false"
    tool->enableDynamicObjects(true);

    // ajust the color of the tool
   // tool->m_materialProxy = tool->m_materialProxyButtonPressed;

    // read the scale factor between the physical workspace of the haptic
    // device and the virtual workspace defined for the tool
    double workspaceScaleFactor = tool->getWorkspaceScaleFactor();

    // define a maximum stiffness that can be handled by the current
    // haptic device. The value is scaled to take into account the
    // workspace scale factor
    stiffnessMax = info.m_maxLinearStiffness / workspaceScaleFactor;

    // create a small white line that will be enabled every time the operator
    // grasps an object. The line indicated the connection between the
    // position of the tool and the grasp position on the object
    graspLine = new cShapeLine(cVector3d(0,0,0), cVector3d(0,0,0));
    world->addChild(graspLine);
    graspLine->m_colorPointB.set(1.0, 1.0, 1.0);
    graspLine->m_colorPointB.set(1.0, 1.0, 1.0);
    graspLine->setShowEnabled(false);


    //-----------------------------------------------------------------------
    // COMPOSE THE VIRTUAL SCENE
    //-----------------------------------------------------------------------

    // create an ODE world to simulate dynamic bodies
    ODEWorld = new cODEWorld(world);

    // add ODE world as a node inside world
    world->addChild(ODEWorld);

    // set some gravity
    ODEWorld->setGravity(cVector3d(0.0, 0.0, -0.5));

    // create a new ODE object that is automatically added to the ODE world
    ODEGear1 = new cODEGenericBody(ODEWorld);
    ODEGear2 = new cODEGenericBody(ODEWorld);
    ODEAxis1 = new cODEGenericBody(ODEWorld);
    ODEAxis2 = new cODEGenericBody(ODEWorld);

    // create a virtual mesh  that will be used for the geometry
    // representation of the dynamic body
    cMultiMesh* gear1 = new cMultiMesh();
    cMultiMesh* gear2 = new cMultiMesh();
    cMultiMesh* axis1 = new cMultiMesh();
    cMultiMesh* axis2 = new cMultiMesh();

    // load object 1 - a gear
    bool fileload;
    fileload = gear1->loadFromFile(RESOURCE_PATH("resources/models/gear/gear.3ds"));
    if (!fileload)
    {
        #if defined(_MSVC)
        fileload = gear1->loadFromFile("../../../bin/resources/models/gear/gear.3ds");
        #endif
    }
    gear1->scale(0.0015);
    gear1->createAABBCollisionDetector(proxyRadius);


    // load object 2 - a gear
    fileload = gear2->loadFromFile(RESOURCE_PATH("resources/models/gear/gear.3ds"));
    if (!fileload)
    {
        #if defined(_MSVC)
        fileload = gear2->loadFromFile("../../../bin/resources/models/gear/gear.3ds");
        #endif
    }
    gear2->scale(0.0015);
    gear2->createAABBCollisionDetector(proxyRadius);

    // load static object 1 - a shaft
    fileload = axis1->loadFromFile(RESOURCE_PATH("resources/models/gear/axis.3ds"));
    if (!fileload)
    {
        #if defined(_MSVC)
        fileload = axis1->loadFromFile("../../../bin/resources/models/gear/axis.3ds");
        #endif
    }
    axis1->scale(0.0015);
    axis1->createAABBCollisionDetector(proxyRadius);

    // load static object 2 - a shaft
    fileload = axis2->loadFromFile(RESOURCE_PATH("resources/models/gear/axis.3ds"));
    if (!fileload)
    {
        #if defined(_MSVC)
        fileload = axis2->loadFromFile("../../../bin/resources/models/gear/axis.3ds");
        #endif
    }
    axis2->scale(0.0015);
    axis2->createAABBCollisionDetector(proxyRadius);

    // define some material properties for both types of objects
    cMaterial mat0, mat1, mat2;
    mat0.m_ambient.set(0.7, 0.1, 0.2);
    mat0.m_diffuse.set(0.8, 0.15, 0.2);
    mat0.m_specular.set(1.0, 0.2, 0.2);
    mat0.setStiffness(0.5 * stiffnessMax);
    mat0.setDynamicFriction(0.4);
    mat0.setStaticFriction(0.9);
    gear1->setMaterial(mat0, true);
    gear2->setMaterial(mat0, true);

    mat1.m_ambient.set(0.2, 0.2, 0.2);
    mat1.m_diffuse.set(0.8, 0.8, 0.8);
    mat1.m_specular.set(1.0, 1.0, 1.0);
    mat1.setStiffness(0.5 * stiffnessMax);
    mat1.setDynamicFriction(0.4);
    mat1.setStaticFriction(0.9);
    axis1->setMaterial(mat1, true);
    axis2->setMaterial(mat1, true);

    // define image models
    ODEGear1->setImageModel(gear1);
    ODEGear2->setImageModel(gear2);
    ODEAxis1->setImageModel(axis1);
    ODEAxis2->setImageModel(axis2);

    // create a dynamic model of the ODE object. Here we decide to use a box just like
    // the object mesh we just defined
    ODEGear1->createDynamicMesh();
    ODEGear2->createDynamicMesh();
    ODEAxis1->createDynamicCapsule(0.050, 0.47, true);  // true = make object static
    ODEAxis2->createDynamicCapsule(0.050, 0.47, true);  // true = make object static

    // define some mass properties for each gear
    ODEGear1->setMass(0.6);
    ODEGear2->setMass(0.6);

    // define temp variables
    cVector3d pos;
    cMatrix3d rot;

    // default distance between both gears when demo starts
    double DISTANCE = 0.260;

    // position and oriente gear 1
    pos = cVector3d(0.0,-DISTANCE,  -0.5);
    rot.identity();
    rot.rotateAboutGlobalAxisRad(cVector3d(1,0,0), cDegToRad(90));
    ODEGear1->setPosition(pos);
    ODEGear1->setRotation(rot);

    // position and oriente gear 1
    pos = cVector3d(0.0, DISTANCE,  -0.5);
    rot.rotateAboutGlobalAxisRad(cVector3d(0,0,1), cDegToRad(14));
    ODEGear2->setPosition(pos);
    ODEGear2->setRotation(rot);

    // position shaft 1
    pos = cVector3d(0.0,-DISTANCE, -0.7);
    ODEAxis1->setPosition(pos);

    // position shaft 2
    pos = cVector3d(0.0, DISTANCE, -0.7);
    ODEAxis2->setPosition(pos);

    // we now create 6 static walls to bound the workspace of our simulation
    ODEGPlane0 = new cODEGenericBody(ODEWorld);
    ODEGPlane1 = new cODEGenericBody(ODEWorld);
    ODEGPlane2 = new cODEGenericBody(ODEWorld);
    ODEGPlane3 = new cODEGenericBody(ODEWorld);
    ODEGPlane4 = new cODEGenericBody(ODEWorld);
    ODEGPlane5 = new cODEGenericBody(ODEWorld);

    double size = 1.0;
    ODEGPlane0->createStaticPlane(cVector3d(0.0, 0.0,  2.0 *size), cVector3d(0.0, 0.0 ,-1.0));
    ODEGPlane1->createStaticPlane(cVector3d(0.0, 0.0, -size), cVector3d(0.0, 0.0 , 1.0));
    ODEGPlane2->createStaticPlane(cVector3d(0.0,  size, 0.0), cVector3d(0.0,-1.0, 0.0));
    ODEGPlane3->createStaticPlane(cVector3d(0.0, -size, 0.0), cVector3d(0.0, 1.0, 0.0));
    ODEGPlane4->createStaticPlane(cVector3d( size, 0.0, 0.0), cVector3d(-1.0,0.0, 0.0));
    ODEGPlane5->createStaticPlane(cVector3d(-0.8 * size, 0.0, 0.0), cVector3d( 1.0,0.0, 0.0));

    //////////////////////////////////////////////////////////////////////////
    // Create some reflexion
    //////////////////////////////////////////////////////////////////////////

    // we create an intermediate node to which we will attach
    // a copy of the object located inside the world
    cGenericObject* reflexion = new cGenericObject();

    // set this object as a ghost node so that no haptic interactions or
    // collision detecting take place within the child nodes added to the
    // reflexion node.
    reflexion->setGhostEnabled(true);

    // add reflexion node to world
    world->addChild(reflexion);

    // turn off culling on each object (objects now get rendered on both sides)
    gear1->setUseCulling(false, true);
    gear2->setUseCulling(false, true);
    axis1->setUseCulling(false, true);
    axis2->setUseCulling(false, true);

    // create a symmetry rotation matrix (z-plane)
    cMatrix3d rotRefexion;
    rotRefexion.set(1.0, 0.0, 0.0,
                    0.0, 1.0, 0.0,
                    0.0, 0.0, -1.0);
    reflexion->setLocalRot(rotRefexion);
    reflexion->setLocalPos(0.0, 0.0, -2.005);

    // add objects to the world
    reflexion->addChild(ODEWorld);
    reflexion->addChild(tool);

    //////////////////////////////////////////////////////////////////////////
    // Create a Ground
    //////////////////////////////////////////////////////////////////////////

    // create mesh to model ground surface
    cMesh* ground = new cMesh();
    world->addChild(ground);

    // create 4 vertices (one at each corner)
    double groundSize = 2.0;
    int vertices0 = ground->newVertex(-groundSize, -groundSize, 0.0);
    int vertices1 = ground->newVertex( groundSize, -groundSize, 0.0);
    int vertices2 = ground->newVertex( groundSize,  groundSize, 0.0);
    int vertices3 = ground->newVertex(-groundSize,  groundSize, 0.0);

    // compose surface with 2 triangles
    ground->newTriangle(vertices0, vertices1, vertices2);
    ground->newTriangle(vertices0, vertices2, vertices3);

    // compute surface normals
    ground->computeAllNormals();

    // position ground at the right level
    ground->setLocalPos(0.0, 0.0, -1.0);

    // define some material properties and apply to mesh
    cMaterial matGround;
    matGround.setStiffness(stiffnessMax);
    matGround.setDynamicFriction(0.7);
    matGround.setStaticFriction(1.0);
    matGround.m_ambient.set(0.0, 0.0, 0.0);
    matGround.m_diffuse.set(0.0, 0.0, 0.0);
    matGround.m_specular.set(0.0, 0.0, 0.0);
    ground->setMaterial(matGround);

    // enable and set transparency level of ground
    ground->setTransparencyLevel(0.75);
    ground->setUseTransparency(true);


    //-----------------------------------------------------------------------
    // OPEN GL - WINDOW DISPLAY
    //-----------------------------------------------------------------------

    // initialize GLUT
    glutInit(&argc, argv);

    // retrieve the resolution of the computer display and estimate the position
    // of the GLUT window so that it is located at the center of the screen
    int screenW = glutGet(GLUT_SCREEN_WIDTH);
    int screenH = glutGet(GLUT_SCREEN_HEIGHT);
    int windowPosX = (screenW - WINDOW_SIZE_W) / 2;
    int windowPosY = (screenH - WINDOW_SIZE_H) / 2;

    // initialize the OpenGL GLUT window
    glutInitWindowPosition(windowPosX, windowPosY);
    glutInitWindowSize(WINDOW_SIZE_W, WINDOW_SIZE_H);
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
        // enable gravity
        ODEWorld->setGravity(cVector3d(0.0, 0.0, -0.5));
    }

    // option 2:
    if (key == '2')
    {
        // disable gravity
        ODEWorld->setGravity(cVector3d(0.0, 0.0, 0.0));
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
    tool->stop();
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
    // render world
    camera->renderView(displayW, displayH);

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
    // simulation clock
    cPrecisionClock simClock;
    simClock.start(true);

    // reset haptics activation clock
    startHapticsClock.reset();
	startHapticsClock.start();
	bool hapticsReady = false;

    // main haptic simulation loop
    while(simulationRunning)
    {
        // wait for some time before enabling haptics
		if (!hapticsReady)
		{
			if (startHapticsClock.getCurrentTimeSeconds() > 3.0)
			{
				hapticsReady = true;
			}
		}

        // compute global reference frames for each object
        world->computeGlobalPositions(true);

        // update position and orientation of tool
        tool->updatePose();

        // compute interaction forces
        tool->computeInteractionForces();

        // check if the tool is touching an object
        cGenericObject* object = tool->m_hapticPoints[0]->m_algorithmFingerProxy->m_collisionEvents[0]->m_object;

        // read user switch status
        bool userSwitch = tool->getUserSwitch(0);

        // if the tool is currently grasping an object we simply update the interaction grasp force
        // between the tool and the object (modeled by a virtual spring)
        if (graspActive && userSwitch)
        {
            // retrieve latest position and orientation of grasped ODE object in world coordinates
            cMatrix3d globalGraspObjectRot = graspObject->getGlobalRot();
            cVector3d globalGraspObjectPos = graspObject->getGlobalPos();

            // compute the position of the grasp point on object in global coordinates
            cVector3d globalGraspPos = globalGraspObjectPos + cMul(globalGraspObjectRot, graspPos);

            // retrieve the position of the tool in global coordinates
            cVector3d globalToolPos  = tool->m_hapticPoint->getGlobalPosProxy();

            // compute the offset between the tool and grasp point on the object
            cVector3d offset = globalToolPos - globalGraspPos;

            // model a spring between both points
            double STIFFNESS = 4;
            cVector3d force = STIFFNESS * offset;

            // apply attraction force (grasp) onto object
            graspObject->addExternalForceAtPoint(force, globalGraspPos);

            // scale magnitude and apply opposite force to haptic device
            tool->m_lastComputedGlobalForce.add(cMul(-1.0, force));

            // update both end points of the line which is used for display purposes only
            graspLine->m_pointA = globalGraspPos;
            graspLine->m_pointB = globalToolPos;
        }

        // the user is not or no longer currently grasping the object
        else
        {
            // was the user grasping the object at the previous simulation loop
            if (graspActive)
            {
                // we disable grasping
                graspActive = false;

                // we hide the virtual line between the tool and the grasp point
                graspLine->setShowEnabled(false);

                // we enable haptics interaction between the tool and the previously grasped object
                if (graspObject != NULL)
                {
                    graspObject->m_imageModel->setHapticEnabled(true, true);
                }
            }

            // the user is touching an object
            if (object != NULL)
            {
                // check if object is attached to an external ODE parent
                cGenericType* externalParent = object->getParent();
                cODEGenericBody* ODEobject = dynamic_cast<cODEGenericBody*>(externalParent);
                if (ODEobject != NULL)
                {
                    // get position of tool
                    cVector3d pos = tool->m_hapticPoint->getGlobalPosProxy();

                    // check if user has enabled the user switch to gras the object
                    if (userSwitch)
                    {
                        // a new object is being grasped
                        graspObject = ODEobject;

                        // retrieve the grasp position on the object in local coordinates
                        graspPos    = tool->m_hapticPoint->getGlobalPosProxy();

                        // grasp in now active!
                        graspActive = true;

                        // enable small line which display the offset between the tool and the grasp point
                        graspLine->setShowEnabled(true);

                        // disable haptic interaction between the tool and the grasped device.
                        // this is performed for stability reasons.
                        graspObject->m_imageModel->setHapticEnabled(false, true);
                    }

                    // retrieve the haptic interaction force being applied to the tool
                    cVector3d force = tool->m_lastComputedGlobalForce;

                    // apply haptic force to ODE object
                    cVector3d tmpfrc = cNegate(force);
                    if (hapticsReady)
                    {
                        ODEobject->addExternalForceAtPoint(tmpfrc, pos);
                    }
                }
            }
        }

        // send forces to device
        tool->applyForces();

        // retrieve simulation time and compute next interval
        double time = simClock.getCurrentTimeSeconds();
        double nextSimInterval = cClamp(time, 0.001, 0.004);

        // reset clock
        simClock.reset();
        simClock.start();

        // update simulation
        ODEWorld->updateDynamics(nextSimInterval);
    }

    // exit haptics thread
    simulationFinished = true;
}

//---------------------------------------------------------------------------
