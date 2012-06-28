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
    \version   $MAJOR.$MINOR.$RELEASE $Rev: 843 $
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

// mouse menu options (right button)
const int OPTION_FULLSCREEN     = 1;
const int OPTION_WINDOWDISPLAY  = 2;

// use stereo display
const bool USE_STEREO_DISPLAY   = false;


//---------------------------------------------------------------------------
// DECLARED VARIABLES
//---------------------------------------------------------------------------

// width and height of the current window display
int displayW = 0;
int displayH = 0;

// a world that contains all objects of the virtual environment
cWorld* world;

// a camera that renders the world in the window display
cCamera* camera;

// a light source to illuminate the objects in the world
cDirectionalLight *light;

// virtual drill mesh
cMultiMesh* drill;

// a virtual tooth mesh
cMultiMesh* tooth;

// transparency level
double transparencyLevel = 0.3;

// a haptic device handler
cHapticDeviceHandler* handler;

// a pointer to the current haptic device
cGenericHapticDevice* hapticDevice;

// a virtual tool representing the haptic device in the scene
cToolCursor* tool;

// a label to display the rate [Hz] at which the simulation is running
cLabel* labelHapticRate;

// indicates if the haptic simulation currently running
bool simulationRunning = false;

// indicates if the haptic simulation has terminated
bool simulationFinished = false;

// frequency counter to measure the simulation haptic rate
cFrequencyCounter frequencyCounter;

// root resource path
string resourceRoot;

// temp variable to store positions and orientations
// of tooth and drill
cVector3d lastPosObject;
cMatrix3d lastRotObject;
cVector3d lastPosDevice;
cMatrix3d lastRotDevice;
cVector3d lastDeviceObjectPos;
cMatrix3d lastDeviceObjectRot;


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
    DEMO:    tooth.cpp

    This demonstration shows how to attach a 3D mesh file to a virtual
    tool. A second mesh (tooth) is loaded in the scene. By pressing the user
    switch of the haptic device it is possible to translate or oriente
    the tooth object accordingly.

    In the main haptics loop function  "updateHaptics()" , the position
    of the haptic device is retrieved at each simulation iteration.
    The interaction forces are then computed and sent to the device.
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
    printf ("Demo: 23-tooth\n");
    printf ("Copyright 2003-2012\n");
    printf ("-----------------------------------\n");
    printf ("\n\n");
    printf ("Instructions:\n\n");
    printf ("- Use haptic device and user switch to rotate \n");
    printf ("  rotate and translate tooth. \n");
    printf ("\n\n");
    printf ("Keyboard Options:\n\n");
    printf ("[1] - Wireframe (ON/OFF)\n");
    printf ("[+] - Increase Opacity\n");
    printf ("[-] - Reduce Opacity\n");
    printf ("[x] - Exit application\n");
    printf ("\n\n>\r");

    // parse first arg to try and locate resources
    resourceRoot = string(argv[0]).substr(0,string(argv[0]).find_last_of("/\\")+1);


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
    camera->set( cVector3d (3.0, 0.0, 0.6),    // camera position (eye)
                 cVector3d (0.0, 0.0, 0.0),    // lookat position (target)
                 cVector3d (0.0, 0.0, 1.0));   // direction of the (up) vector

    // set the near and far clipping planes of the camera
    // anything in front/behind these clipping planes will not be rendered
    camera->setClippingPlanes(0.01, 10.0);

    // Enable shadow casting
    camera->setUseShadowCasting(true);

    // create a light source
    light = new cDirectionalLight(world);

    // attach light to camera
    camera->addChild(light);    

    // enable light source
    light->setEnabled(true);                   

    // position the light source
    light->setLocalPos( 0.0, 0.5, 0.0);             

    // define the direction of the light beam
    light->setDir(-3.0,-0.5, 0.0);      


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


    //-----------------------------------------------------------------------
    // COMPOSE WORLD
    //-----------------------------------------------------------------------

    // create a virtual mesh
    tooth = new cMultiMesh();

    // add object to world
    world->addChild(tooth);

    // set the position and orientation of the object at the center of the world
    tooth->setLocalPos(0.0, 0.0, 0.0);
    tooth->rotateAboutGlobalAxisDeg(cVector3d(0.0, 0.0, 1.0),-10);
    tooth->rotateAboutGlobalAxisDeg(cVector3d(0.0, 1.0, 0.0), 10);

    // load an object file
    bool fileload;
    fileload = tooth->loadFromFile(RESOURCE_PATH("resources/models/tooth/tooth.obj"));
    if (!fileload)
    {
        #if defined(_MSVC)
        fileload = tooth->loadFromFile("../../../bin/resources/models/tooth/tooth.obj");
        #endif
    }
    if (!fileload)
    {
        printf("Error - 3D Model failed to load correctly.\n");
        close();
        return (-1);
    }

    // make the outside of the tooth rendered in semi-transparent
    tooth->getMesh(1)->setUseTransparency(false);
    tooth->getMesh(1)->setTransparencyLevel(transparencyLevel);

    // compute a boundary box
    tooth->computeBoundaryBox(true);

    // resize tooth to screen
    tooth->scale(0.003);

    // compute collision detection algorithm
    tooth->createAABBCollisionDetector(toolRadius);

    // define a default stiffness for the object
    tooth->setStiffness(0.8 * stiffnessMax, true);

    // create a new mesh.
    drill = new cMultiMesh();

    // load a drill like mesh and attach it to the tool
    fileload = drill->loadFromFile(RESOURCE_PATH("resources/models/drill/drill.3ds"));
    if (!fileload)
    {
        #if defined(_MSVC)
        fileload = drill->loadFromFile("../../../bin/resources/models/drill/drill.3ds");
        #endif
    }
    if (!fileload)
    {
        printf("Error - 3D Model failed to load correctly.\n");
        close();
        return (-1);
    }

    // resize tool mesh model
    drill->scale(0.004);

    // remove the collision detector. we do not want to compute any
    // force feedback rendering on the object itself.
    drill->deleteCollisionDetector(true);

    // define a material property for the mesh
    cMaterial mat;
    mat.m_ambient.set(0.5, 0.5, 0.5);
    mat.m_diffuse.set(0.8, 0.8, 0.8);
    mat.m_specular.set(1.0, 1.0, 1.0);
    drill->setMaterial(mat, true);
    drill->computeAllNormals();

    // attach drill to tool
    tool->m_image->addChild(drill);

    //-----------------------------------------------------------------------
    // WIDGETS
    //-----------------------------------------------------------------------

    // create a font
    cFont *font = NEW_CFONTCALIBRI20();
    
    // create a label to display the haptic rate of the simulation
    labelHapticRate = new cLabel(font);
    labelHapticRate->m_fontColor.setWhite();
    camera->m_frontLayer->addChild(labelHapticRate);


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
    int windowW = 0.7 * screenH;
    int windowH = 0.7 * screenH;
    int windowPosX = (screenW - windowH) / 2;
    int windowPosY = (screenH - windowW) / 2;

    // initialize the OpenGL GLUT window
    glutInitWindowPosition(windowPosX, windowPosY);
    glutInitWindowSize(windowW, windowH);
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
        // only enable/disable wireframe of one part of the tooth model
        bool useWireMode = tooth->getMesh(1)->getWireMode();
        tooth->getMesh(1)->setWireMode(!useWireMode);
    }
    
    // option -:
    if (key == '-')
    {
        // decrease transparency level
        transparencyLevel = transparencyLevel - 0.1;
        if (transparencyLevel < 0.0) { transparencyLevel = 0.0; }

        // apply changes to tooth
        tooth->getMesh(1)->setTransparencyLevel(transparencyLevel);
        tooth->getMesh(1)->setUseTransparency(true);

        // if object is almost transparent, make it invisible
        if (transparencyLevel < 0.1)
        {
            tooth->getMesh(1)->setShowEnabled(false, true);
            tooth->getMesh(1)->setHapticEnabled(false, true);
        }
    }

    // option +:
    if (key == '+')
    {
        // increase transparency level
        transparencyLevel = transparencyLevel + 0.1;
        if (transparencyLevel > 1.0) { transparencyLevel = 1.0; }

        // apply changes to tooth
        tooth->getMesh(1)->setTransparencyLevel(transparencyLevel);

        // make object visible
        if (transparencyLevel >= 0.1)
        {
            tooth->getMesh(1)->setShowEnabled(true, true);
            tooth->getMesh(1)->setHapticEnabled(true, true);
        }

        // disable transparency is transparency level is set to 1.0
        if (transparencyLevel == 1.0)
        {
            tooth->getMesh(1)->setUseTransparency(false);
        }
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
    // update haptic rate label
    labelHapticRate->setString ("haptic rate: "+cStr(frequencyCounter.getFrequency(), 0) + " [Hz]");

    int px = (int)(0.5 * (displayW - labelHapticRate->getWidth()));
    labelHapticRate->setLocalPos(px, 15);

    // render world
    camera->renderView(displayW, displayH);

    // swap buffers
    glutSwapBuffers();

    // check for any OpenGL errors
    GLenum err;
    err = glGetError();
    if (err != GL_NO_ERROR) printf("Error:  %s\n", gluErrorString(err));
}

//---------------------------------------------------------------------------

void updateHaptics(void)
{
    // retrieve information about the current haptic device
    cHapticDeviceInfo info = hapticDevice->getSpecifications();

    // main haptic simulation loop
    while(simulationRunning)
    {
        /////////////////////////////////////////////////////////////////////////
        // HAPTIC RENDERING
        /////////////////////////////////////////////////////////////////////////

        // update frequency counter
        frequencyCounter.signal(1);

        // compute global reference frames for each object
        world->computeGlobalPositions(true);

        // update position and orientation of tool
        tool->updatePose();

        // compute interaction forces
        tool->computeInteractionForces();

        // send forces to device
        tool->applyForces();  

 
        /////////////////////////////////////////////////////////////////////////
        // MANIPULATION
        /////////////////////////////////////////////////////////////////////////

        // if the haptic device does track orientations, we automatically
        // oriente the drill to remain perpendicular to the tooth
        cVector3d pos = tool->m_hapticPoint->getGlobalPosProxy();
        cMatrix3d rot = tool->getDeviceGlobalRot();

        if (info.m_sensedRotation == false)
        {
            cVector3d pos = tool->m_hapticPoint->getGlobalPosProxy();
            rot.identity();

            cVector3d vx, vy, vz;
            cVector3d zUp (0,0,1);
            cVector3d yUp (0,1,0);
            vx = pos - tooth->getLocalPos();
            if (vx.length() > 0.001)
            {
                vx.normalize();

                if (cAngle(vx,zUp) > 0.001)
                {
                    vy = cCross(zUp, vx);
                    vy.normalize();
                    vz = cCross(vx, vy);
                    vz.normalize();

                }
                else
                {
                    vy = cCross(yUp, vx);
                    vy.normalize();
                    vz = cCross(vx, vy);
                    vz.normalize();
                }

                rot.setCol(vx, vy, vz);
                drill->setLocalRot(rot);
            }
        }


        int button = tool->getUserSwitch(0);
        if (button == 0)
        {
            lastPosDevice = pos;
            lastRotDevice = rot;
            lastPosObject = tooth->getLocalPos();
            lastRotObject = tooth->getLocalRot();
            lastDeviceObjectPos = cTranspose(lastRotDevice) * ((lastPosObject - lastPosDevice) + 0.01*cNormalize(lastPosObject - lastPosDevice));
            lastDeviceObjectRot = cMul(cTranspose(lastRotDevice), lastRotObject);
            tool->setShowEnabled(true, true);
            drill->setShowEnabled(true, true);
        }
        else
        {
            tool->setShowEnabled(false, true);
            drill->setShowEnabled(false, true);
            cMatrix3d rotDevice01 = cMul(cTranspose(lastRotDevice), rot);
            cMatrix3d newRot =  cMul(rot, lastDeviceObjectRot);
            cVector3d newPos = cAdd(pos, cMul(rot, lastDeviceObjectPos));
            tooth->setLocalPos(newPos);
            tooth->setLocalRot(newRot);
            world->computeGlobalPositions(true);
        }
    }

    // exit haptics thread
    simulationFinished = true;
}

//---------------------------------------------------------------------------
