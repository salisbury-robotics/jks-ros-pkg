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
    \version   $MAJOR.$MINOR.$RELEASE $Rev: 721 $
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
#ifndef MACOSX
#include "GL/glut.h"
#else
#include "GLUT/glut.h"
#endif
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
// DECLARED CONSTANTS
//---------------------------------------------------------------------------

// initial size (width/height) in pixels of the window display
const int WINDOW_SIZE_W         = 800;
const int WINDOW_SIZE_H         = 800;

// mouse menu options (right button)
const int OPTION_FULLSCREEN     = 1;
const int OPTION_WINDOWDISPLAY  = 2;

// use stereo display
const bool USE_STEREO_DISPLAY   = false;


//---------------------------------------------------------------------------
// DECLARED VARIABLES
//---------------------------------------------------------------------------

// a world that contains all objects of the virtual environment
cWorld* world;

// a camera that renders the world in the window display
cCamera* camera;

// a light source to illuminate the objects in the world
cSpotLight *light;

// width and height of the current window display
int displayWidth  = 0;
int displayHeight = 0;

// a haptic device handler
cHapticDeviceHandler* handler;

// a pointer to the current haptic device
cGenericHapticDevice* hapticDevice;

// a virtual tool representing the haptic device in the scene
cToolCursor* tool;

// a virtual mesh like object
cMesh* object;

// a label to display the rate [Hz] at which the simulation is running
cLabel* labelHapticRate;

// indicates if the haptic simulation currently running
bool simulationRunning = false;

// indicates if the haptic simulation has terminated
bool simulationFinished = false;

// frequency counter to measure the simulation haptic rate
cFrequencyCounter frequencyCounter;


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

// main graphics callback
void updateGraphics(void);

// main haptics loop
void updateHaptics(void);


//===========================================================================
/*
    DEMO:    12-polygons.cpp

    This example illustrates how to build an object composed of triangle
    with individual colors. A finger-proxy algorithm is used to compute
    the interaction force between the tool and the object.
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
    printf ("Demo: 12-polygons\n");
    printf ("Copyright 2003-2012\n");
    printf ("-----------------------------------\n");
    printf ("\n\n");
    printf ("Keyboard Options:\n\n");
    printf ("[1] - Wireframe (ON/OFF)\n");
    printf ("[x] - Exit application\n");
    printf ("\n\n");

    // parse first arg to try and locate resources
    string resourceRoot = string(argv[0]).substr(0,string(argv[0]).find_last_of("/\\")+1);


    //-----------------------------------------------------------------------
    // 3D - SCENEGRAPH
    //-----------------------------------------------------------------------

    // create a new world.
    world = new cWorld();

    // set the background color of the environment
    world->m_backgroundColor.setBlack();

    // create a camera and insert it into the virtual world
    camera = new cCamera(world);
    world->addChild(camera);

    // position and oriente the camera
    camera->set( cVector3d (3.0, 0.0, 0.0),    // camera position (eye)
                 cVector3d (0.0, 0.0, 0.0),    // lookat position (target)
                 cVector3d (0.0, 0.0, 1.0));   // direction of the (up) vector

    // set the near and far clipping planes of the camera
    // anything in front/behind these clipping planes will not be rendered
    camera->setClippingPlanes(0.01, 10.0);

    // create a light source
    light = new cSpotLight(world);

    // attach light to camera
    camera->addChild(light);    

    // enable light source
    light->setEnabled(true);                   

    // position the light source
    light->setPos( 0.0, 0.5, 0.0);             

    // define the direction of the light beam
    light->setDir(-3.0,-0.5, 0.0);             

    // enable shadow casting
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
    double toolRadius = 0.1;

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


    //-----------------------------------------------------------------------
    // COMPOSE THE VIRTUAL SCENE
    //-----------------------------------------------------------------------

    // create a virtual mesh
    object = new cMesh(world);

    // add object to world
    world->addChild(object);

    // set the position of the object at the center of the world
    object->setPos(0.0, 0.0, 0.0);

    // Since we want to see our polygons from both sides, we disable culling.
    object->setUseCulling(false, true);

    // here we create a small cloud of triangles
    double ang = 0;
    double angStep = 5;
    while (ang < 360)
    {
        double radius = 0.2;
        double radiusStep = 0.1;
        double tang = ang;
        while (radius < 1.0)
        {
            // we create three vectors p0, p1, p2 that describe the position of the vertices
            cVector3d p0 = cVector3d(0.0, radius * cCosDeg(tang), radius * cSinDeg(tang));
            cVector3d p1 = cVector3d(0.0, (radius + radiusStep) * cCosDeg(tang+angStep), (radius + radiusStep) * cSinDeg(tang+angStep));
            cVector3d p2 = cVector3d(0.0, (radius + radiusStep) * cCosDeg(tang-angStep), (radius + radiusStep) * cSinDeg(tang-angStep));

            // we generate a triangles by passing the position of the new vertices
            unsigned int index = object->newTriangle(p0, p1, p2);

            // for each triangle we define a color
            cColorf color;
            color.set(ang / 360, 1.0 - ang/360, radius);

            // we get a pointer to the latest created triangle
            cTriangle* triangle = object->getTriangle(index);

            // we get pointers to its respective vertices.
            cVertex* vertex0 = triangle->getVertex0();
            cVertex* vertex1 = triangle->getVertex1();
            cVertex* vertex2 = triangle->getVertex2();

            // for each vertex we attribute the new color. In this example
            // the color is the same for all vertices, but it is of course
            // possible to define a specific color for each vertex
            vertex0->setColor(color);
            vertex1->setColor(color);
            vertex2->setColor(color);

            // we increment the different counters
            radiusStep = 1.1 * radiusStep;
            radius = radius + 1.2 * radiusStep;
            tang = tang + 0.5 * angStep;
        }
        ang = ang + 3 * angStep;
    }

    // we indicate that we ware rendering the triangle by using the specific
    // colors for each of them (see abov)
    object->setUseVertexColors(true);
	
    // compute a boundary box
    object->computeBoundaryBox(true);

    // compute collision detection algorithm
    object->createAABBCollisionDetector(1.01 * toolRadius);

    // define a default stiffness for the object
    object->m_material->setStiffness(0.3 * stiffnessMax);

    // render triangles haptcally on front side only
    object->m_material->setRenderTriangles(true, true);


    //-----------------------------------------------------------------------
    // WIDGETS
    //-----------------------------------------------------------------------

    // create a font
    cFont *font = NEW_CFONTCALIBRI20();
    
    // create a label to display the haptic rate of the simulation
    labelHapticRate = new cLabel(font);
    camera->m_frontLayer.addChild(labelHapticRate);


    //-----------------------------------------------------------------------
    // OPEN GL - WINDOW DISPLAY
    //-----------------------------------------------------------------------

    // simulation in now running!
    simulationRunning = true;

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
    if (USE_STEREO_DISPLAY)
    {
        glutInitDisplayMode(GLUT_RGB | GLUT_DEPTH | GLUT_DOUBLE | GLUT_STEREO);
        camera->setUseStereo(true);
    }
    else
    {
        glutInitDisplayMode(GLUT_RGB | GLUT_DEPTH | GLUT_DOUBLE);
        camera->setUseStereo(false);
    }
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

    // create a thread which starts the main haptics rendering loop
    cThread* hapticsThread = new cThread();
    hapticsThread->set(updateHaptics, CTHREAD_PRIORITY_HAPTICS);

    // start the main graphics rendering loop
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
    displayWidth  = w;
    displayHeight = h;
    glViewport(0, 0, displayWidth, displayHeight);
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
        bool useWireMode = object->getWireMode();
        object->setWireMode(!useWireMode);
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

void updateGraphics(void)
{
    // update haptic rate label
    labelHapticRate->m_string = "haptic rate: "+cStr(frequencyCounter.getFrequency(), 0) + " [Hz]";

    int px = (int)(0.5 * (displayWidth - labelHapticRate->getLength()));
    labelHapticRate->setPos(px, 10, 0);

    // render world
    camera->renderView(displayWidth, displayHeight);

    // swap buffers
    glutSwapBuffers();

    // check for any OpenGL errors
    GLenum err;
    err = glGetError();
    if (err != GL_NO_ERROR) printf("Error:  %s\n", gluErrorString(err));

    // inform the GLUT window to call updateGraphics again (next frame)
    if (simulationRunning)
    {
        glutPostRedisplay();
    }
}

//---------------------------------------------------------------------------

void updateHaptics(void)
{
    // angular velocity of object
    cVector3d angVel(0,0,0);

    // reset clock
    cPrecisionClock clock;
    clock.reset();

    // main haptic simulation loop
    while(simulationRunning)
    {
        /////////////////////////////////////////////////////////////////////
        // SIMULATION TIME    
        /////////////////////////////////////////////////////////////////////

        // stop the simulation clock
        clock.stop();

        // read the time increment in seconds
        double timeInterval = clock.getCurrentTimeSeconds();

        // restart the simulation clock
        clock.reset();
        clock.start();

        // update frequency counter
        frequencyCounter.signal(1);


        /////////////////////////////////////////////////////////////////////
        // HAPTIC FORCE COMPUTATION
        /////////////////////////////////////////////////////////////////////

        // compute global reference frames for each object
        world->computeGlobalPositions(true);

        // update position and orientation of tool
        tool->updatePose();

        // compute interaction forces
        tool->computeInteractionForces();

        // send forces to device
        tool->applyForces();

        // get position of cursor in global coordinates
        cVector3d toolPos = tool->getDeviceGlobalPos();

        // get position of object in global coordinates
        cVector3d objectPos = object->getGlobalPos();

        // compute a vector from the center of mass of the object (point of rotation) to the tool
        cVector3d v = cSub(toolPos, objectPos);

        // compute angular acceleration based on the interaction forces
        // between the tool and the object
        cVector3d angAcc(0,0,0);
        if (v.length() > 0.0)
        {
            // get the last force applied to the cursor in global coordinates
            // we negate the result to obtain the opposite force that is applied on the
            // object
            cVector3d toolForce = cNegate(tool->m_lastComputedGlobalForce);

            // compute the effective force that contributes to rotating the object.
            cVector3d force = toolForce - cProject(toolForce, v);

            // compute the resulting torque
            cVector3d torque = cMul(v.length(), cCross( cNormalize(v), force));

            // update rotational acceleration
            const double INERTIA = 0.4;
            angAcc = (1.0 / INERTIA) * torque;
        }

        // update rotational velocity
        angVel.add(timeInterval * angAcc);

        // set a threshold on the rotational velocity term
        const double MAX_ANG_VEL = 10.0;
        double vel = angVel.length();
        if (vel > MAX_ANG_VEL)
        {
            angAcc.mul(MAX_ANG_VEL / vel);
        }

        // add some damping too
        const double DAMPING = 0.1;
        angVel.mul(1.0 - DAMPING * timeInterval);

        // if user switch is pressed, set velocity to zero
        if (tool->getUserSwitch(0) == 1)
        {
            angVel.zero();
        }

        // compute the next rotation configuration of the object
        if (angVel.length() > C_SMALL)
        {
            object->rotate(cNormalize(angVel), timeInterval * angVel.length());
        }
    }
    
    // exit haptics thread
    simulationFinished = true;
}

//---------------------------------------------------------------------------
