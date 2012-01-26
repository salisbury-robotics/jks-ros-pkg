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
cDirectionalLight *light;

// width and height of the current window display
int displayWidth  = 0;
int displayHeight = 0;

// a haptic device handler
cHapticDeviceHandler* handler;

// a pointer to the current haptic device
cGenericHapticDevice* hapticDevice;

// a virtual tool representing the haptic device in the scene
cToolCursor* tool;

// a few spherical objects
cShapeSphere* object0;
cShapeSphere* object1;
cShapeSphere* object2;
cShapeSphere* object3;

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

// callback when a key from the representing is pressed
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
    DEMO:    effects.cpp

    In this example illustrates the use of haptic effects. The application
    begins by creating four spheres with different graphical (
    material and texture) properties. For each sphere, one or more effects
    are programmed to obtain the desired haptic illusion. Parameters to
    adjust each effect are located in the cMaterial class.

    If you wish to develop your own shapes and force effect, take a look at
    the example located in "src/effects". You can also implement your own
    shapes by following the approach presented in the cShapeSphere,
    cShapeTorus or cShapeLine classes for instance.

    In the main haptics loop function  "updateHaptics()" , the position
    of the haptic device is retrieved at each simulation iteration.
    The interaction forces are then computed and sent to the haptic device.
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
    printf ("Demo: 11-effects\n");
    printf ("Copyright 2003-2013\n");
    printf ("-----------------------------------\n");
    printf ("\n\n");
    printf ("Keyboard Options:\n\n");
    printf ("[x] - Exit application\n");
    printf ("\n\n");

    // parse first arg to try and locate resources
    resourceRoot = string(argv[0]).substr(0,string(argv[0]).find_last_of("/\\")+1);


    //-----------------------------------------------------------------------
    // WORLD - CAMERA - LIGHTING
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

    // enable multi-pass rendering to handle transparent objects
    camera->setUseMultipassTransparency(true);

    // create a light source
    light = new cDirectionalLight(world);

    // add light to world
    world->addChild(light);

    // enable light source
    light->setEnabled(true);                   

    // define the direction of the light beam
    light->setDir(-1.0, 0.0, 0.0);             


    //-----------------------------------------------------------------------
    // HAPTIC DEVICES / TOOLS
    //-----------------------------------------------------------------------

    // create a haptic device handler
    handler = new cHapticDeviceHandler();

    // get access to the first available haptic device
    handler->getDevice(hapticDevice, 0);

    // retrieve information about the current haptic device
    cHapticDeviceInfo hapticDeviceInfo = hapticDevice->getSpecifications();

    // create a 3D tool and add it to the world
    tool = new cToolCursor(world);
    world->addChild(tool);

    // connect the haptic device to the tool
    tool->setHapticDevice(hapticDevice);

    // initialize tool by connecting to haptic device
    tool->start();

    // map the physical workspace of the haptic device to a larger virtual workspace.
    tool->setWorkspaceRadius(1.0);

    // define a radius for the tool
    tool->setRadius(0.03);

    // read the scale factor between the physical workspace of the haptic
    // device and the virtual workspace defined for the tool
    double workspaceScaleFactor = tool->getWorkspaceScaleFactor();


    //-----------------------------------------------------------------------
    // CREATING OBJECTS
    //----------------------------------------------------------------------

    // temp variable
    cGenericEffect* newEffect;

    // properties
    double maxForce     = hapticDeviceInfo.m_maxLinearForce;
    double maxStiffness = hapticDeviceInfo.m_maxLinearStiffness / workspaceScaleFactor;
    double maxDamping   = hapticDeviceInfo.m_maxLinearDamping / workspaceScaleFactor;


    /////////////////////////////////////////////////////////////////////////
    // OBJECT 0: "MAGNET"
    /////////////////////////////////////////////////////////////////////////

    // create a sphere and define its radius
    object0 = new cShapeSphere(0.3);

    // add object to world
    world->addChild(object0);

    // set the position of the object at the center of the world
    object0->setPos(0.0, -0.5, 0.0);

    // set graphic properties
    bool fileload;
    object0->m_texture = new cTexture2d();
    fileload = object0->m_texture->loadFromFile(RESOURCE_PATH("resources/images/chrome.bmp"));
    if (!fileload)
    {
        #if defined(_MSVC)
        fileload = object0->m_texture->loadFromFile("../../../bin/resources/images/chrome.bmp");
        #endif
    }
    if (!fileload)
    {
        printf("Error - Texture image failed to load correctly.\n");
        close();
        return (-1);
    }

    object0->m_texture->setSphericalMappingEnabled(true);
    object0->setUseTexture(true);
    object0->m_material->setWhite();

    // set haptic properties
    object0->m_material->setStiffness(0.4 * maxStiffness);   // 40% of maximum linear stiffness
    object0->m_material->setMagnetMaxForce(0.4 * maxForce);  // 40% of maximum linear force 
    object0->m_material->setMagnetMaxDistance(0.12);
    object0->m_material->setViscosity(0.2 * maxDamping);     // 20% of maximum linear damping

    // create a haptic surface effect
    newEffect = new cEffectSurface(object0);
    object0->addEffect(newEffect);

    // create a haptic magnetic effect
    newEffect = new cEffectMagnet(object0);
    object0->addEffect(newEffect);


    /////////////////////////////////////////////////////////////////////////
    // OBJECT 1: "FLUID"
    ////////////////////////////////////////////////////////////////////////

    // create a sphere and define its radius
    object1 = new cShapeSphere(0.3);

    // add object to world
    world->addChild(object1);

    // set the position of the object at the center of the world
    object1->setPos(0.0, 0.5, 0.0);

    // set graphic properties
    object1->m_texture = new cTexture2d();
		fileload = object1->m_texture->loadFromFile(RESOURCE_PATH("resources/images/water.bmp"));
		if (!fileload)
		{
				#if defined(_MSVC)
				fileload = object1->m_texture->loadFromFile("../../../bin/resources/images/water.bmp");
				#endif
		}
		if (!fileload)
		{
				printf("Error - Texture image failed to load correctly.\n");
				close();
				return (-1);
		}

    object1->m_material->m_ambient.set(0.1, 0.1, 0.6, 0.5);
    object1->m_material->m_diffuse.set(0.3, 0.3, 0.9, 0.5);
    object1->m_material->m_specular.set(1.0, 1.0, 1.0, 0.5);
    object1->setTransparencyLevel(0.5);
    object1->setUseTexture(true);

    // set haptic properties
    object1->m_material->setViscosity(0.9 * maxDamping);    // 90% of maximum linear damping

    // create a haptic viscous effect
    newEffect = new cEffectViscosity(object1);
    object1->addEffect(newEffect);


    /////////////////////////////////////////////////////////////////////////
    // OBJECT 2: "STICK-SLIP"
    /////////////////////////////////////////////////////////////////////////

    // create a sphere and define its radius
    object2 = new cShapeSphere(0.3);

    // add object to world
    world->addChild(object2);

    // set the position of the object at the center of the world
    object2->setPos(0.0, 0.0, 0.5);

    // set graphic properties
    object2->m_texture = new cTexture2d();
    fileload = object2->m_texture->loadFromFile(RESOURCE_PATH("resources/images/stone.bmp"));
    if (!fileload)
    {
        #if defined(_MSVC)
        fileload = object2->m_texture->loadFromFile("../../../bin/resources/images/stone.bmp");
        #endif
    }
    if (!fileload)
    {
        printf("Error - Texture image failed to load correctly.\n");
        close();
        return (-1);
    }

	object2->m_texture->setSphericalMappingEnabled(true);
    object2->m_material->m_ambient.setPurpleIndigo();
    object2->setTransparencyLevel(0.5);
    object2->setUseTexture(true);

    // set haptic properties
    object2->m_material->setStickSlipForceMax(0.5 * maxForce);      // 50% of maximum linear force
    object2->m_material->setStickSlipStiffness(0.7 * maxStiffness); // 70% of maximum linear stiffness

    // create a haptic stick-slip effect
    newEffect = new cEffectStickSlip(object2);
    object2->addEffect(newEffect);


    /////////////////////////////////////////////////////////////////////////
    // OBJECT 3: "VIBRATIONS"
    ////////////////////////////////////////////////////////////////////////

    // create a sphere and define its radius
    object3 = new cShapeSphere(0.3);

    // add object to world
    world->addChild(object3);

    // set the position of the object at the center of the world
    object3->setPos(0.0, 0.0, -0.5);

    // set graphic properties
    object3->m_texture = new cTexture2d();
    fileload = object3->m_texture->loadFromFile(RESOURCE_PATH("resources/images/plastic.bmp"));
    if (!fileload)
    {
        #if defined(_MSVC)
        fileload = object3->m_texture->loadFromFile("../../../bin/resources/images/plastic.bmp");
        #endif
    }
    if (!fileload)
    {
        printf("Error - Texture image failed to load correctly.\n");
        close();
        return (-1);
    }

    object3->m_texture->setSphericalMappingEnabled(true);
    object3->setUseTexture(true);
    object3->m_material->setRed();
    object3->setTransparencyLevel(0.7);

    // set haptic properties
    object3->m_material->setVibrationFrequency(50);
    object3->m_material->setVibrationAmplitude(0.1 * maxForce); // 10% of maximum linear force
    object3->m_material->setStiffness(0.1 * maxStiffness);      // 10% of maximum linear stiffness

    // create a haptic vibration effect
    newEffect = new cEffectVibration(object3);
    object3->addEffect(newEffect);

    // create a haptic surface effect
    newEffect = new cEffectSurface(object3);
    object3->addEffect(newEffect);
    
    
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
    displayWidth = w;
    displayHeight = h;
    glViewport(0, 0, displayWidth, displayHeight);
}

//---------------------------------------------------------------------------

void keySelect(unsigned char key, int x, int y)
{
    if ((key == 27) || (key == 'x'))
    {
        // close everything
        close();

        // exit application
        exit(0);
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
    // main haptic simulation loop
    while(simulationRunning)
    {
        // compute global reference frames for each object
        world->computeGlobalPositions(true);

        // update position and orientation of tool
        tool->updatePose();

        // compute interaction forces
        tool->computeInteractionForces();

        // send forces to device
        tool->applyForces();

        // update frequency counter
        frequencyCounter.signal(1);
    }
    
    // exit haptics thread
    simulationFinished = true;
}

//---------------------------------------------------------------------------
