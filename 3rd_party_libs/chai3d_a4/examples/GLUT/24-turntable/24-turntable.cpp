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
#include "bass.h"
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
int displayW  = 0;
int displayH  = 0;

// a world that contains all objects of the virtual environment
cWorld* world;

// a camera that renders the world in a window display
cCamera* camera;

// a light source to illuminate the objects in the virtual scene
cSpotLight *light;

// a haptic device handler
cHapticDeviceHandler* handler;

// a pointer to the current haptic device
cGenericHapticDevice* hapticDevice;

// a virtual tool representing the haptic device in the scene
cToolCursor* tool;

// a label to display the rate [Hz] at which the simulation is running
cLabel* labelHapticRate;

// a virtual turntble object
cMultiMesh* turntable;

// a virtual vinyl
cMesh* record;

// texture image
cTexture2d* recordImage;

// status of the main simulation haptics loop
bool simulationRunning = false;

// BASS library
const int MAX_VAL_SLIDERS_P = 100000;
const int MAX_VAL_SLIDERS_I = 10000000;
const int FREQ = 1000;
const double chartClock = 50;
const double MESH_SCALE_SIZE = 0.35;

// Global variables for the audio stream
HSTREAM stream;
BASS_CHANNELINFO infoBass;
QWORD stream_length;
char *data;
int record_direction = 1;
unsigned int pos = 0;

// frequency counter to measure the simulation haptic rate
cFrequencyCounter frequencyCounter;

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

// build record player
void createRecord(cMesh *a_mesh, double radius);

// write the requested data from the loaded buffer to the sound card
DWORD CALLBACK MyStreamWriter(HSTREAM handle, void *buf, DWORD len, void *user);


//===========================================================================
/*
    DEMO:    turntable.cpp

    This example demonstrates the use of friction, animation, and
    sound effects.  Enable haptcs, and you can spin the record
    back and forth and at different speeds.
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
    printf ("Demo: 24-turntable\n");
    printf ("Copyright 2003-2012\n");
    printf ("-----------------------------------\n");
    printf ("\n\n");
    printf ("Instructions:\n\n");
    printf ("- Use the haptic device to spin the record \n");
    printf ("\n\n");
    printf ("Keyboard Options:\n\n");
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
    camera->set( cVector3d (2.5, 0.0, 1.2),    // camera position (eye)
                 cVector3d (0.0, 0.0, 0.0),    // lookat position (target)
                 cVector3d (0.0, 0.0, 1.0));   // direction of the (up) vector

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
    // CREATING SCENE
    //-----------------------------------------------------------------------

    // create a virtual mesh
    turntable = new cMultiMesh();

    // add object to world
    world->addChild(turntable);

    // set the position of the object at the center of the world
    turntable->setLocalPos(0.0, 0.0, 0.0);
    turntable->rotateAboutGlobalAxisDeg(cVector3d(0,0,1), 90);

    // load an object file
    bool fileload;
    fileload = turntable->loadFromFile(RESOURCE_PATH("resources/models/turntable/turntable.obj"));
    if (!fileload)
    {
        #if defined(_MSVC)
        fileload = turntable->loadFromFile("../../../bin/resources/models/turntable/turntable.obj");
        #endif
    }
    if (!fileload)
    {
        printf("Error - 3D Model failed to load correctly.\n");
        close();
        return (-1);
    }

    // compute a boundary box
    turntable->computeBoundaryBox(true);

    // get dimensions of object
    double size = cSub(turntable->getBoundaryMax(), turntable->getBoundaryMin()).length();

    // resize object to screen
    if (size > 0)
    {
        turntable->scale( 2.0 * tool->getWorkspaceRadius() / size);
    }

    // compute collision detection algorithm
    turntable->createAABBCollisionDetector(toolRadius);

    // define a default stiffness for the object
    turntable->setStiffness(0.8 * stiffnessMax, true);

    // Initialize sound device and create audio stream
    BASS_Init(1,44100,0,0,NULL);

		// Load the data from the specified file
    HSTREAM file_stream = 1;
    file_stream = BASS_StreamCreateFile(FALSE,RESOURCE_PATH("resources/sounds/classic.mp3"),0,0,BASS_STREAM_DECODE);
    if (file_stream == 0)
    {
        #if defined(_MSVC)
        file_stream = BASS_StreamCreateFile(FALSE,"../../../bin/resources/sounds/classic.mp3",0,0,BASS_STREAM_DECODE);
        #endif
    }
    if (!fileload)
    {
        printf("Error - MP3 audio file failed to load correctly.\n");
        close();
        return (-1);
    }

    // Get the length and header info from the loaded file
    //stream_length = BASS_StreamGetLength(file_stream);
    stream_length = BASS_ChannelGetLength(file_stream, 0);

    BASS_ChannelGetInfo(file_stream, &infoBass);

    // Get the audio samples from the loaded file
    data = new char[(unsigned int)stream_length];
    BASS_ChannelGetData(file_stream, data, (unsigned int)stream_length);

    // Set playing to begin at the beginning of the loaded data
    stream = BASS_StreamCreate(infoBass.freq, infoBass.chans, 0, &MyStreamWriter, 0);

    // create new record
    record = new cMesh();
    createRecord(record, 0.48);
    recordImage = new cTexture2d();

    fileload = recordImage->loadFromFile(RESOURCE_PATH("resources/images/record.bmp"));
    if (!fileload)
    {
        #if defined(_MSVC)
        fileload = recordImage->loadFromFile("../../../bin/resources/images/record.bmp");
        #endif
    }
    if (!fileload)
    {
        printf("Error - Texture image failed to load correctly.\n");
        close();
        return (-1);
    }

    record->m_material->setWhite();
    record->setTexture(recordImage);
    record->setUseTexture(true, true);
	record->setUseMaterial(true, true);
    record->createAABBCollisionDetector(toolRadius);

    // add object to world and translate
    world->addChild(record);
    record->translate(-0.0, 0, 0.02);

    // set stiffness properties of record
    record->setStiffness(stiffnessMax, true);

    // set static and dynamic friction
    double staticFriction = (double)100 / 100.0;
    double dynamicFriction = (double)100 / 100.0;
    record->setFriction(staticFriction, dynamicFriction, true);
    

    //-----------------------------------------------------------------------
    // WIDGETS
    //-----------------------------------------------------------------------

    // create a font
    cFont *font = NEW_CFONTCALIBRI20();
    
    // create a label to display the haptic rate of the simulation
    labelHapticRate = new cLabel(font);
    labelHapticRate->m_fontColor.setWhite();
    camera->m_frontLayer->addChild(labelHapticRate);

    // create a background
    cBackground* background = new cBackground();
    camera->m_backLayer->addChild(background);

    // set background properties
    background->setCornerColors(cColorf(0.2, 0.2, 0.2),
                                cColorf(0.2, 0.2, 0.2),
                                cColorf(0.0, 0.0, 0.0),
                                cColorf(0.0, 0.0, 0.0));


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
    // angular velocity of the record
    double angVel = 0;

    // angular position of the record
    double angPos = 0;

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
     //   frequencyCounter.signal(1);


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


        /////////////////////////////////////////////////////////////////////
        // ANIMATION
        /////////////////////////////////////////////////////////////////////

        // init
        cVector3d torque(0,0,0);

        // figure out if we're touching the record
        if (tool->isInContact(record))
        {
            // get position of cursor in global coordinates
            cVector3d toolPos = tool->getDeviceGlobalPos();

            // get position of object in global coordinates
            cVector3d objectPos = record->getGlobalPos();

            // compute a vector from the center of mass of the object (point of rotation) to the tool
            cVector3d vObjectCMToTool = cSub(toolPos, objectPos);

            if (vObjectCMToTool.length() > 0)
            {
                // get the last force applied to the cursor in global coordinates
                // we negate the result to obtain the opposite force that is applied on the
                // object
                cVector3d toolForce = cNegate(tool->m_lastComputedGlobalForce);

                // compute effective force to take into account the fact the object
                // can only rotate arround a its center mass and not translate
                cVector3d effectiveForce = toolForce - cProject(toolForce, vObjectCMToTool);

                // compute the resulting torque
                torque = cMul(vObjectCMToTool.length(), cCross( cNormalize(vObjectCMToTool), effectiveForce));
            }
        }

        // update rotational acceleration
        const double OBJECT_INERTIA = 0.02;
        double angAcc = (1.0 / OBJECT_INERTIA) * torque.z();

        // update rotattional velocity
        angVel = angVel + timeInterval * angAcc;

        // set a threshold on the rotational velocity term
        const double ANG_VEL_MAX = 20.0;

        if (angVel > ANG_VEL_MAX)
        {
            angVel = ANG_VEL_MAX;
        }
        else if (angVel < -ANG_VEL_MAX)
        {
            angVel = -ANG_VEL_MAX;
        }

        // compute the next rotation of the torus
        angPos = angPos + timeInterval * angVel;

        // update position of record
        cMatrix3d rot;
        rot.identity();
        rot.rotateAboutGlobalAxisRad(cVector3d(0,0,1), angPos);
        record->setLocalRot(rot);

        // update audio
        // set audio direction and frequency based on rotational velocity
        if ((fabs(angVel)) > 0.0)
        {
            if (angVel < 0.0) record_direction = 1;
            else record_direction = -1;

            BASS_ChannelSetAttribute(stream, BASS_ATTRIB_FREQ, (int)(infoBass.freq*(0.05+fabs(angVel))/6.5));

            if (!(BASS_ChannelPlay(stream,FALSE)))
            {

            }
        }
        else
        {
            BASS_ChannelStop(stream);
        }
    }

    // exit haptics thread
    simulationFinished = true;
}

//---------------------------------------------------------------------------

DWORD CALLBACK MyStreamWriter(HSTREAM handle, void *buf, DWORD len, void *user)
{
    // Cast the buffer to a character array
	char *d=(char*)buf;

    frequencyCounter.signal(1);

    // Loop the file when it reaches the beginning or end
    if ((pos >= stream_length) && (record_direction == 1))
		    pos = 0;
	if ((pos <= 0) && (record_direction == -1))
		    pos = (unsigned int)stream_length;

	// If record is spinning in positive direction, write requested
	// amount of data from current position forwards
	if (record_direction == 1)
	{
		int up = len + pos;
		if (up > stream_length)
			up = (unsigned int)stream_length;

    for (int i=pos; i<up; i+=1)
			d[(i-pos)] = data[i];

		int amt = (up-pos);
		pos += amt;
		return amt;
	 }

    // If record is spinning in negative direction, write requested
	// amount of data from current position backwards
	if (record_direction == -1)
	{
		int up = pos - len;

		if (up < 0)
			up = 0;

	    int cnt = 0;
        for (int i=pos; i>up; i-=1)
                d[cnt++] = data[i];

		int amt = cnt;
		pos -= amt;

		return amt;
	 }

	 return 0;
}

//---------------------------------------------------------------------------

void createRecord(cMesh *a_mesh, double radius)
{
    const double HEIGHT = 0.04;
    double vectorY1, originY, vectorX1, originX;
    vectorY1 = 0; originY = 0;
    vectorX1 = 0; originX = 0;
    cVector3d vector1b, vector2b, originb;
    cVector3d vector1t, vector2t, origint;
    originb.set(originX, originY, 0);
    origint.set(originX, originY, HEIGHT);

    vector2b.set(originX, originY, 0);
    vector2t.set(originX, originY, HEIGHT);

    int num = 28;
    double divisore = num / (2*C_PI);

    for(int i=0; i<=num; i++)
    {
        double angle=(float)(((double)i)/divisore);
        vector1b.set(originX+(radius*(float)sin((double)angle)), originY+(radius*(float)cos((double)angle)), 0);
        vector1t.set(originX+(radius*(float)sin((double)angle)), originY+(radius*(float)cos((double)angle)), HEIGHT);
        a_mesh->newTriangle(origint, vector1t, vector2t);
        a_mesh->newTriangle(originb, vector2b, vector1b);

        a_mesh->newTriangle(vector1b, vector2b, vector2t);
        a_mesh->newTriangle(vector1b, vector2t, vector1t);

        vector2b = vector1b;
        vector2t = vector1t;
    }

    for(unsigned int n=0; n<a_mesh->getNumVertices(); n++)
    {
        cVertex* curVertex = a_mesh->getVertex(n);
        curVertex->setTexCoord(
            (curVertex->getLocalPos().x() + radius) / (2.0 * radius),
            (curVertex->getLocalPos().y() + radius) / (2.0 * radius)
        );
        curVertex->setNormal(0,0,1);
    }

    // compute normals
    a_mesh->computeAllNormals();

    // compute boudary box
    a_mesh->computeBoundaryBox();
}

