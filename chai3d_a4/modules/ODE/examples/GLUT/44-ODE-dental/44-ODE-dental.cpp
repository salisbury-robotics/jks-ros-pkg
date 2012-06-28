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

// stiffness of virtual spring
double linGain = 0.2;
double angGain = 0.03;
double linG;
double angG;
double linStiffness = 800;
double angStiffness = 30;

// scale factor between virtual tool and haptic device
double workspaceScaleFactor;

// status of the main simulation haptics loop
bool simulationRunning = false;

// ODE world
cODEWorld* ODEWorld;

// ODE objects
cODEGenericBody* ODETeeth;
cODEGenericBody* ODETool;

cGenericObject* frame;
cShapeLine* line;

// haptic device
cGenericHapticDevice* hapticDevice;

// a virtual tool representing the haptic device in the scene
cToolCursor* tool;

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

// creates a cube mesh
void createCube(cMesh* a_mesh, double a_size);


//===========================================================================
/*
    DEMO:    ODE_dental.cpp

    This example illustrates the use of the ODE framework for simulating
    haptic interaction with dynamic bodies. In this scene we demonstrate a 
    dental applicationwhere a drill is interacting with a set of teeth.
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
    printf ("Demo: 44-ODE-dental\n");
    printf ("Copyright 2003-2011\n");
    printf ("-----------------------------------\n");
    printf ("\n\n");
    printf ("Instructions:\n\n");
    printf ("- Use haptic device and user switch to manipulate cubes \n");
    printf ("\n\n");
    printf ("Keyboard Options:\n\n");
    printf ("[h] - Display help menu\n");;
    printf ("[1] - Enable gravity\n");
    printf ("[2] - Disable gravity\n");
    printf ("\n");
    printf ("[3] - decrease linear haptic gain\n");
    printf ("[4] - increase linear haptic gain\n");
    printf ("[5] - decrease angular haptic gain\n");
    printf ("[6] - increase angular haptic gain\n");
    printf ("\n");
    printf ("[7] - decrease linear stiffness\n");
    printf ("[8] - increase linear stiffness\n");
    printf ("[9] - decrease angular stiffness\n");
    printf ("[0] - increase angular stiffness\n");
    printf ("\n");
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
    camera->set( cVector3d (3.0, 0.0, 0.3),    // camera position (eye)
				         cVector3d (0.0, 0.0, 0.0),    // lookat position (target)
				         cVector3d (0.0, 0.0, 1.0));   // direction of the "up" vector

    // set the near and far clipping planes of the camera
    // anything in front/behind these clipping planes will not be rendered
    camera->setClippingPlanes(0.01, 10.0);
    camera->setUseShadowCasting(false);

    // create a light source and attach it to the camera
    light = new cSpotLight(world);
    camera->addChild(light);                   // attach light to camera
    light->setEnabled(true);                   // enable light source
    light->setLocalPos(cVector3d( 0.0, 0.5, 0.0));  // position the light source
    light->setDir(cVector3d(-3.0,-0.5, 0.0));  // define the direction of the light beam
    light->m_ambient.set(0.6, 0.6, 0.6);
    light->m_diffuse.set(0.8, 0.8, 0.8);
    light->m_specular.set(0.8, 0.8, 0.8);


    frame = new cGenericObject();
    //world->addChild(frame);
    frame->setShowFrame(true);
    line = new cShapeLine(cVector3d(0,0,0), cVector3d(0,0,0));
    //world->addChild(line);

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
    tool->setWorkspaceRadius(1.3);

    // define a radius for the tool (graphical display)
    tool->setRadius(0.02);

    // hide the device sphere. only show proxy.
    tool->setShowContactPoints(true, false);

    // set the physical readius of the proxy.
    double proxyRadius = 0.0;
    tool->setRadiusContact(proxyRadius);
    //tool->m_proxyPointForceModel->m_collisionSettings.m_checkBothSidesOfTriangles = false;

    // enable if objects in the scene are going to rotate of translate
    // or possibly collide against the tool. If the environment
    // is entirely static, you can set this parameter to "false"
    tool->enableDynamicObjects(false);


    //-----------------------------------------------------------------------
    // COMPOSE THE VIRTUAL SCENE
    //-----------------------------------------------------------------------

    // create an ODE world to simulate dynamic bodies
    ODEWorld = new cODEWorld(world);

    // add ODE world as a node inside world
    world->addChild(ODEWorld);

    // set some gravity
    ODEWorld->setGravity(cVector3d(0.0, 0.0, 0.0));

    // create a new ODE object that is automatically added to the ODE world
    ODETeeth = new cODEGenericBody(ODEWorld);

    // create a virtual mesh  that will be used for the geometry
    // representation of the dynamic body
    cMultiMesh* imgTeeth = new cMultiMesh();

    // load model
    bool fileload;
    fileload = imgTeeth->loadFromFile(RESOURCE_PATH("resources/models/dental/teeth-bottom.3ds"));
    if (!fileload)
    {
        #if defined(_MSVC)
        fileload = imgTeeth->loadFromFile("../../../bin/resources/models/dental/teeth-bottom.3ds");
        #endif
    }
    imgTeeth->scale(0.004);
    imgTeeth->createAABBCollisionDetector(0.0);
    imgTeeth->setStiffness(300, true);
   
    // add mesh to ODE object
    ODETeeth->setImageModel(imgTeeth);

    // create a dynamic model of the ODE object. Here we decide to use a box just like
    // the object mesh we just defined
    ODETeeth->createDynamicMesh(true);

    // position and oriente gear 1
    cVector3d pos;
    cMatrix3d rot;
    pos = cVector3d(0.0, 0.0, -0.5);
    rot.identity();
    rot.rotateAboutGlobalAxisRad(cVector3d(0,0,1), cDegToRad(90));
    rot.rotateAboutGlobalAxisRad(cVector3d(0,1,0), cDegToRad(20));
    ODETeeth->setPosition(pos);
    ODETeeth->setRotation(rot);

    // create a virtual tool
    ODETool = new cODEGenericBody(ODEWorld);
    cMultiMesh* imgTool = new cMultiMesh();

    fileload = imgTool->loadFromFile(RESOURCE_PATH("resources/models/dental/drill.obj"));
    if (!fileload)
    {
        #if defined(_MSVC)
        fileload = imgTool->loadFromFile("../../../bin/resources/models/dental/drill.obj");
        #endif
    }
    imgTool->scale(0.01);


    // define some material properties for each cube
    cMaterial matTool;
    matTool.m_ambient.set(0.1, 0.1, 0.1);
    matTool.m_diffuse.set(0.4, 0.4, 0.4);
    matTool.m_specular.set(0.5, 0.5, 0.5);
    imgTool->setMaterial(matTool, true);
    imgTool->setShowNormals(true);
    imgTool->setHapticEnabled(false);
    //imgTool->computeAllNormals(true);
    //imgTool->setShowFrame(true);

    // add mesh to ODE object
    ODETool->setImageModel(imgTool);
    ODETool->createDynamicMesh(false);

    // define some mass properties for each cube
    ODETool->setMass(0.01);
    dBodySetAngularDamping(ODETool->m_ode_body, 0.06);
    dBodySetLinearDamping(ODETool->m_ode_body, 0.06);


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

	// help menu:
	if (key == 'h')
    {
		printf ("\n\n");
		printf ("Keyboard Options:\n\n");
		printf ("[h] - Display help menu\n");;
		printf ("[1] - Enable gravity\n");
		printf ("[2] - Disable gravity\n");
		printf ("\n");
		printf ("[3] - decrease linear haptic gain\n");
		printf ("[4] - increase linear haptic gain\n");
		printf ("[5] - decrease angular haptic gain\n");
		printf ("[6] - increase angular haptic gain\n");
		printf ("\n");
		printf ("[7] - decrease linear stiffness\n");
		printf ("[8] - increase linear stiffness\n");
		printf ("[9] - decrease angular stiffness\n");
		printf ("[0] - increase angular stiffness\n");
		printf ("\n");
		printf ("[x] - Exit application\n");
		printf ("\n\n");
	}

    // option 1:
    if (key == '1')
    {
        // enable gravity
        ODEWorld->setGravity(cVector3d(0.0, 0.0, -9.81));
		printf("gravity ON:\n");
    }

    // option 2:
    if (key == '2')
    {
        // disable gravity
        ODEWorld->setGravity(cVector3d(0.0, 0.0, 0.0));
		printf("gravity OFF:\n");
    }

    // option 3: decrease linear haptic gain
    if (key == '3')
    {
        linGain = linGain - 0.05;
		if (linGain < 0)
			linGain = 0;
		printf("linear haptic gain:  %f\n", linGain);
    }

    // option 4: increase linear haptic gain
    if (key == '4')
    {
        linGain = linGain + 0.05;
		printf("linear haptic gain:  %f\n", linGain);
    }

    // option 5: decrease angular haptic gain
    if (key == '5')
    {
        angGain = angGain - 0.005;
		if (angGain < 0)
			angGain = 0;
		printf("angular haptic gain:  %f\n", angGain);
    }

    // option 6: increase angular haptic gain
    if (key == '6')
    {
		angGain = angGain + 0.005;
		printf("angular haptic gain:  %f\n", angGain);
    }

    // option 7: decrease linear stiffness
    if (key == '7')
    {
        linStiffness = linStiffness - 50;
		if (linStiffness < 0)
			linStiffness = 0;
		printf("linear stiffness:  %f\n", linStiffness);
    }

    // option 8: increase linear stiffness
    if (key == '8')
    {
        linStiffness = linStiffness + 50;
		printf("linear stiffness:  %f\n", linStiffness);
    }

    // option 9: decrease angular stiffness
    if (key == '9')
    {
        angStiffness = angStiffness - 1;
		if (angStiffness < 0)
			angStiffness = 0;
		printf("angular stiffness:  %f\n", angStiffness);
    }

    // option 0: increase angular stiffness
    if (key == '0')
    {
        angStiffness = angStiffness + 1;
		printf("angular stiffness:  %f\n", angStiffness);
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
    // start haptic device
    hapticDevice->open();

    // simulation clock
    cPrecisionClock simClock;
    simClock.start(true);

    cMatrix3d prevRotTool;
    prevRotTool.identity();

    // main haptic simulation loop
    while(simulationRunning)
    {
      // retrieve simulation time and compute next interval
      double time = simClock.getCurrentTimeSeconds();
      double nextSimInterval = 0.0005;//cClamp(time, 0.00001, 0.0002);

      // reset clock
      simClock.reset();
      simClock.start();

      // compute global reference frames for each object
      world->computeGlobalPositions(true);

      // update position and orientation of tool
      tool->updatePose();

      // compute interaction forces
      tool->computeInteractionForces();

      // update position and orientation of tool
      cVector3d posDevice;
      cMatrix3d rotDevice;
      //hapticDevice->getPosition(posDevice);
      //hapticDevice->getRotation(rotDevice);
      posDevice = tool->m_hapticPoint->getGlobalPosProxy();
      rotDevice = tool->getDeviceGlobalRot();

      // read position of tool
      cVector3d posTool = ODETool->getLocalPos();
      cMatrix3d rotTool = ODETool->getLocalRot();

      // compute position and angular error between tool and haptic device
      cVector3d deltaPos = (posDevice - posTool);
      cMatrix3d deltaRot = cMul(cTranspose(rotTool), rotDevice);
      double angle;
      cVector3d axis;
      deltaRot.toAngleAxis(angle, axis);

      // compute force and torque to apply to tool
      cVector3d force, torque;
      force = linStiffness * deltaPos;
      ODETool->addExternalForce(force);

      torque = cMul((angStiffness * angle), axis);
      rotTool.mul(torque);
      ODETool->addExternalTorque(torque);

      // update data for display
      frame->setLocalRot(deltaRot);

      // compute force and torque to apply to haptic device
      force = -linG * force;
      torque = -angG * torque;
      line->m_pointB = torque;

      // add force contribution from ODE model
      tool->m_lastComputedGlobalForce.add(force);

      // send forces to haptic device.
      tool->applyForces();
      //hapticDevice->setForceAndTorqueAndGripperForce(force, torque, 0.0);

      if (linG < linGain)
      {
        linG = linG + 0.1 * time * linGain;
      }
      else
      {
        linG = linGain;
      }

      if (angG < angGain)
      {
        angG = angG + 0.1 * time * angGain;
      }
      else
      {
        angG = angGain;
      }

      // update simulation
      ODEWorld->updateDynamics(nextSimInterval);
    }

    // exit haptics thread
    simulationFinished = true;
}

//---------------------------------------------------------------------------

void createCube(cMesh* a_mesh, double a_size)
{
    const double HALFSIZE = a_size / 2.0;
    int vertices [6][6];

    // face -x
    vertices[0][0] = a_mesh->newVertex(-HALFSIZE,  HALFSIZE, -HALFSIZE);
    vertices[0][1] = a_mesh->newVertex(-HALFSIZE, -HALFSIZE, -HALFSIZE);
    vertices[0][2] = a_mesh->newVertex(-HALFSIZE, -HALFSIZE,  HALFSIZE);
    vertices[0][3] = a_mesh->newVertex(-HALFSIZE,  HALFSIZE,  HALFSIZE);

    // face +x
    vertices[1][0] = a_mesh->newVertex( HALFSIZE, -HALFSIZE, -HALFSIZE);
    vertices[1][1] = a_mesh->newVertex( HALFSIZE,  HALFSIZE, -HALFSIZE);
    vertices[1][2] = a_mesh->newVertex( HALFSIZE,  HALFSIZE,  HALFSIZE);
    vertices[1][3] = a_mesh->newVertex( HALFSIZE, -HALFSIZE,  HALFSIZE);

    // face -y
    vertices[2][0] = a_mesh->newVertex(-HALFSIZE,  -HALFSIZE, -HALFSIZE);
    vertices[2][1] = a_mesh->newVertex( HALFSIZE,  -HALFSIZE, -HALFSIZE);
    vertices[2][2] = a_mesh->newVertex( HALFSIZE,  -HALFSIZE,  HALFSIZE);
    vertices[2][3] = a_mesh->newVertex(-HALFSIZE,  -HALFSIZE,  HALFSIZE);

    // face +y
    vertices[3][0] = a_mesh->newVertex( HALFSIZE,   HALFSIZE, -HALFSIZE);
    vertices[3][1] = a_mesh->newVertex(-HALFSIZE,   HALFSIZE, -HALFSIZE);
    vertices[3][2] = a_mesh->newVertex(-HALFSIZE,   HALFSIZE,  HALFSIZE);
    vertices[3][3] = a_mesh->newVertex( HALFSIZE,   HALFSIZE,  HALFSIZE);

    // face -z
    vertices[4][0] = a_mesh->newVertex(-HALFSIZE,  -HALFSIZE, -HALFSIZE);
    vertices[4][1] = a_mesh->newVertex(-HALFSIZE,   HALFSIZE, -HALFSIZE);
    vertices[4][2] = a_mesh->newVertex( HALFSIZE,   HALFSIZE, -HALFSIZE);
    vertices[4][3] = a_mesh->newVertex( HALFSIZE,  -HALFSIZE, -HALFSIZE);

    // face +z
    vertices[5][0] = a_mesh->newVertex( HALFSIZE,  -HALFSIZE,  HALFSIZE);
    vertices[5][1] = a_mesh->newVertex( HALFSIZE,   HALFSIZE,  HALFSIZE);
    vertices[5][2] = a_mesh->newVertex(-HALFSIZE,   HALFSIZE,  HALFSIZE);
    vertices[5][3] = a_mesh->newVertex(-HALFSIZE,  -HALFSIZE,  HALFSIZE);

    // create triangles
    for (int i=0; i<6; i++)
    {
    a_mesh->newTriangle(vertices[i][0], vertices[i][1], vertices[i][2]);
    a_mesh->newTriangle(vertices[i][0], vertices[i][2], vertices[i][3]);
    }

    // set material properties to light gray
    a_mesh->m_material->m_ambient.set(0.5f, 0.5f, 0.5f, 1.0f);
    a_mesh->m_material->m_diffuse.set(0.7f, 0.7f, 0.7f, 1.0f);
    a_mesh->m_material->m_specular.set(1.0f, 1.0f, 1.0f, 1.0f);
    a_mesh->m_material->m_emission.set(0.0f, 0.0f, 0.0f, 1.0f);

    // compute normals
    a_mesh->computeAllNormals();
}
