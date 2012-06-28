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
    \version   $MAJOR.$MINOR.$RELEASE $Rev: 845 $
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

// state machine states
const int STATE_IDLE            = 1;
const int STATE_MODIFY_MAP      = 2;
const int STATE_MOVE_CAMERA     = 3;


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
int displayW = 0;
int displayH = 0;

// a haptic device handler
cHapticDeviceHandler* handler;

// a pointer to the current haptic device
cGenericHapticDevice* hapticDevice;

// a virtual tool representing the haptic device in the scene
cToolCursor* tool;

// radius of cursor proxy
double proxyRadius;

// a virtual mesh like object
cMesh* object;

// a small magnetic line used to constrain the tool alon the vertical axis
cShapeLine* magneticLine;

// two sphere position at the end of the magnetic line
cShapeSphere* sphereA;
cShapeSphere* sphereB;

// a label to display the rate [Hz] at which the simulation is running
cLabel* labelHapticRate;

// indicates if the haptic simulation currently running
bool simulationRunning = false;

// indicates if the haptic simulation has terminated
bool simulationFinished = false;

// frequency counter to measure the simulation haptic rate
cFrequencyCounter frequencyCounter;

// state machine 
int state = STATE_IDLE;

// mouse position and button status
int mouseX, mouseY;
int mouseButton;

// camera position and orientation is spherical coordinates
double cameraAngleH;
double cameraAngleV;
double cameraDistance;
cVector3d cameraPosition;

// camera status
bool flagCameraInMotion = false;

// informs the graphic display callback to update the display list
// when the topology of the 3D height map is changed.
bool flagUpdateDisplayList = false;

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

// callback when a keyboard key is pressed
void keySelect(unsigned char key, int x, int y);

// callback when the right mouse button is pressed to select a menu item
void menuSelect(int value);

// callback to handle mouse click
void mouseClick(int button, int state, int x, int y);

// callback to handle mouse motion
void mouseMove(int x, int y);

// function called before exiting the application
void close(void);

// main graphics timer and callback
void graphicsTimer(int data);
void updateGraphics(void);

// main haptics loop
void updateHaptics(void);

// update camera settings
void updateCameraPosition();

// loads a bitmap file and create 3D height map based on pixel color
int loadHeightMap();


//===========================================================================
/*
    DEMO:    20-map.cpp

    This example illustrates the construction a triangle based object.
    The application first loads a bitmap texture image. For each pixel,
    we then define a height by computing the gray-scale value. A vertex
    is created for each pixel and triangles are then generated to connect
    the array of vertices together. This example also demonstrates the
    use of mouse callback commands to allow the operator to control the
    position of the virtual camera. The operator can also use the haptic
    device (user switch command) to move the camera or grasp a point on the 
    surface and deform the terrain.

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
    printf ("Demo: 20-map\n");
    printf ("Copyright 2003-2012\n");
    printf ("-----------------------------------\n");
    printf ("\n\n");
    printf ("Instructions:\n\n");
    printf ("- Use left mouse button to rotate camera view. \n");
    printf ("- Use right mouse button to control camera zoom. \n");
    printf ("- Use haptic device and user switch to rotate \n");
    printf ("  camera or deform terrain. \n");
    printf ("\n\n");
    printf ("Keyboard Options:\n\n");
    printf ("[1] - Haptic Shading (ON/OFF)\n");
    printf ("[2] - Wireframe (ON/OFF)\n");
    printf ("[x] - Exit application\n");
    printf ("\n\n>\r");


    // parse first arg to try and locate resources
    resourceRoot = string(argv[0]).substr(0,string(argv[0]).find_last_of("/\\")+1);

    //-----------------------------------------------------------------------
    // WORLD - CAMERA - LIGHTING
    //-----------------------------------------------------------------------

    // create a new world.
    world = new cWorld();

    // set the background color of the environment
    // the color is defined by its (R,G,B) components.
    world->m_backgroundColor.setWhite();

    // create a camera and insert it into the virtual world
    camera = new cCamera(world);
    world->addChild(camera);

    // define a default position of the camera (described in spherical coordinates)
    cameraAngleH = 5;
    cameraAngleV = 40;
    cameraDistance = 3.5;
    updateCameraPosition();

    // set the near and far clipping planes of the camera
    // anything in front/behind these clipping planes will not be rendered
    camera->setClippingPlanes(0.01, 10.0);

    // enable shadow casting
    camera->setUseShadowCasting(true);

    // create a light source
    light = new cSpotLight(world);

    // attach light to camera
    world->addChild(light);    

    // enable light source
    light->setEnabled(true);                   

    // position the light source
    light->setLocalPos( 2.0, 0.0, 2.0);             

    // define the direction of the light beam
    light->setDir(-1.0, 0.0,-1.0);             

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

	// if the device has a gripper, then enable it to behave like a user switch
	hapticDevice->setEnableGripperUserSwitch(true);

    // create a 3D tool and add it to the camera
    tool = new cToolCursor(world);
    camera->addChild(tool);

    // position tool workspace in front of camera (x-axis of camera reference 
    // pointing towards the viewer)
    tool->setLocalPos(-cameraDistance, 0.0, 0.0);

    // connect the haptic device to the tool
    tool->setHapticDevice(hapticDevice);

    // initialize tool by connecting to haptic device
    tool->start();

    // map the physical workspace of the haptic device to a larger virtual workspace.
    tool->setWorkspaceRadius(1.0);

    // hide the device sphere. only show proxy.
    tool->setShowContactPoints(true, false);

    // set color of tool
    tool->m_hapticPoint->m_sphereProxy->m_material->setWhiteAliceBlue();

    // set the physical readius of the proxy.
    proxyRadius = 0.0;
    tool->setRadius(0.05, proxyRadius);

    // read the scale factor between the physical workspace of the haptic
    // device and the virtual workspace defined for the tool
    double workspaceScaleFactor = tool->getWorkspaceScaleFactor();

    // define a maximum stiffness that can be handled by the current
    // haptic device. The value is scaled to take into account the
    // workspace scale factor
    double stiffnessMax = info.m_maxLinearStiffness / workspaceScaleFactor;


    //-----------------------------------------------------------------------
    // CREATING OBJECTS
    //-----------------------------------------------------------------------

    /////////////////////////////////////////////////////////////////////////
    // MAP
    /////////////////////////////////////////////////////////////////////////

    // create a virtual mesh
    object = new cMesh();

    // add object to world
    world->addChild(object);

    // set the position of the object at the center of the world
    object->setLocalPos(0.0, 0.0, 0.0);

    // Since we want to see our polygons from both sides, we disable culling.
    object->setUseCulling(false);

    // load default map
    loadHeightMap();

    // set color properties
    object->m_material->setBlueCornflower();

    // set stiffness
    object->m_material->setStiffness(0.5 * stiffnessMax);

    // enable haptic shading
    object->m_material->setUseHapticShading(true);

    // use display list to increase graphical rendering performance
    object->setUseDisplayList(true);

	// draw edges arround map
	object->computeAllEdges(180);
	object->setEdgeProperties(1.0, cColorf(0,0,0));


    /////////////////////////////////////////////////////////////////////////
    // MAGNETIC LINE
    /////////////////////////////////////////////////////////////////////////

    // create a small vertical white magnetic line that will be activated when the
    // user deforms the mesh.
    magneticLine = new cShapeLine(cVector3d(0,0,0), cVector3d(0,0,0));

    // add line to world
    world->addChild(magneticLine);

    // set color of line
    magneticLine->m_colorPointA.setGrayDark();
    magneticLine->m_colorPointB.setGrayDark();

    // line is not yet enabled
    magneticLine->setHapticEnabled(false);
    magneticLine->setShowEnabled(false);

    // set haptic properties
    magneticLine->m_material->setStiffness(0.05 * stiffnessMax);
    magneticLine->m_material->setMagnetMaxForce(0.2 * info.m_maxLinearForce);
    magneticLine->m_material->setMagnetMaxDistance(0.25);
    magneticLine->m_material->setViscosity(0.05 * info.m_maxLinearDamping);

    // create a haptic magnetic effect
    cEffectMagnet* newEffect = new cEffectMagnet(magneticLine);
    magneticLine->addEffect(newEffect);

    // create two sphere that will be added at both ends of the line
    sphereA = new cShapeSphere(0.02);
    sphereB = new cShapeSphere(0.02);

    // add spheres to world
    world->addChild(sphereA);
    world->addChild(sphereB);

    // disable spheres for now
    sphereA->setShowEnabled(false);
    sphereB->setShowEnabled(false);

    // define some material properties for spheres
    cMaterial matSphere;
    matSphere.setWhiteAliceBlue();

    // assign material properties to both spheres
    sphereA->setMaterial(matSphere);
    sphereB->setMaterial(matSphere);


    //-----------------------------------------------------------------------
    // WIDGETS
    //-----------------------------------------------------------------------

    // create a font
    cFont *font = NEW_CFONTCALIBRI20();
    
    // create a label to display the haptic rate of the simulation
    labelHapticRate = new cLabel(font);
    camera->m_frontLayer->addChild(labelHapticRate);

    // set font color
    labelHapticRate->m_fontColor.setBlack();


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
    glutMouseFunc(mouseClick);
    glutMotionFunc(mouseMove);
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
        close();
        exit(0);
    }

    // option 1:
    if (key == '1')
    {
        bool useHapticShading = !object->m_material->getUseHapticShading();
        object->m_material->setUseHapticShading(useHapticShading);
        if (useHapticShading)
            printf ("> Haptic shading enabled     \r");    
        else
            printf ("> Haptic shading disabled    \r");    
    }

    // option 2:
    if (key == '2')
    {
        bool useWireMode = !object->getWireMode();
        object->setWireMode(useWireMode);
        if (useWireMode)
            printf ("> Wire mode enabled         \r");    
        else
            printf ("> Wire mode disabled        \r");   
    }
}

//---------------------------------------------------------------------------

void mouseClick(int button, int state, int x, int y)
{
     // mouse button down
    if (state == GLUT_DOWN)
    {
        flagCameraInMotion = true;
        mouseX = x;
        mouseY = y;
        mouseButton = button;
    }

    // mouse button u
    else if (state == GLUT_UP)
    {
        flagCameraInMotion = false;
    }
}

//---------------------------------------------------------------------------

void mouseMove(int x, int y)
{
    if (flagCameraInMotion)
    {
        if (mouseButton == GLUT_RIGHT_BUTTON)
        {
            cameraDistance = cameraDistance - 0.01 * (y - mouseY);
        }

        else if (mouseButton == GLUT_LEFT_BUTTON)
        {
            cameraAngleH = cameraAngleH - (x - mouseX);
            cameraAngleV = cameraAngleV + (y - mouseY);
        }
    }

    updateCameraPosition();

    mouseX = x;
    mouseY = y;
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

    int px = (int)(0.5 * (displayW - labelHapticRate->getString().size()));
    labelHapticRate->setLocalPos(px, 10, 0);

    // update object normals
    if (state == STATE_MODIFY_MAP)
    {
        object->computeAllNormals();
    }

    // if the mesh has been modified we update the display list
    if (flagUpdateDisplayList)
    {
        object->invalidateDisplayList();
        flagUpdateDisplayList = false;
    }

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
    // initialize state to idle
    state = STATE_IDLE;  

    // current tool position
    cVector3d toolGlobalPos;        // global world coordinates
    cVector3d toolLocalPos;         // local coordinates

    // previous tool position
    cVector3d prevToolGlobalPos;    // global world coordinates
    cVector3d prevToolLocalPos;     // local coordinates

    // main haptic simulation loop
    while(simulationRunning)
    {
        // compute global reference frames for each object
        world->computeGlobalPositions(true);

        // update position and orientation of tool
        tool->updatePose();

        // compute interaction forces
        tool->computeInteractionForces();

        // read user switch
        bool userSwitch = tool->getUserSwitch(0);

        // update tool position
        toolGlobalPos = tool->getDeviceGlobalPos();
        toolLocalPos  = tool->getDeviceLocalPos();

        if ((state == STATE_MOVE_CAMERA) && (!userSwitch))
        {
            state = STATE_IDLE;

            // enable haptic interaction with map
            object->setHapticEnabled(true, true);
        }

        else if (((state == STATE_MODIFY_MAP) && (!userSwitch)) ||
                 ((state == STATE_MODIFY_MAP) && (!tool->isInContact(magneticLine))))
        {
            state = STATE_IDLE;

            // disable magnetic line
            magneticLine->setHapticEnabled(false);
            magneticLine->setShowEnabled(false);

            // disable spheres
            sphereA->setShowEnabled(false);
            sphereB->setShowEnabled(false);

            // enable haptic interaction with map
            object->setHapticEnabled(true, true);

            // disable forces
            tool->setForcesOFF();

            // update bounding box (can take a little time)
            object->createAABBCollisionDetector(1.01 * proxyRadius);
            
            // enable forces again
            tool->setForcesON();
        }

        // user clicks with the mouse
        else if ((state == STATE_IDLE) && (userSwitch))
        {
            // start deforming object
            if (tool->isInContact(object))
            {
                state = STATE_MODIFY_MAP;

                // update position of line
                cVector3d posA = toolGlobalPos;
                posA.z(-0.7);

                cVector3d posB = toolGlobalPos;
                posB.z(0.7);

                magneticLine->m_pointA = posA;
                magneticLine->m_pointB = posB;

                // update position of spheres
                sphereA->setLocalPos(posA);
                sphereB->setLocalPos(posB);

                // enable spheres
                sphereA->setShowEnabled(true);
                sphereB->setShowEnabled(true);

                // enable magnetic line
                magneticLine->setHapticEnabled(true);
                magneticLine->setShowEnabled(true);

                // disable haptic interaction with map
                object->setHapticEnabled(false, true);
            }

            // start moving camera
            else
            {
                state = STATE_MOVE_CAMERA;
                
                // disable haptic interaction with map
                object->setHapticEnabled(false, true);
            }
        }

        // modify map
        else if (state == STATE_MODIFY_MAP)
        {
            // compute tool offset
            cVector3d offset = toolGlobalPos - prevToolGlobalPos;

            // map offset on z axis
            double offsetHeight = offset.z();

            // apply offset to all vertices through a weighted function
            int numVertices = object->getNumVertices();
            for (int i=0; i<numVertices; i++)
            {
                // get next vertex
                cVertex* vertex = object->getVertex(i);

                // compute distance between vertex and tool
                cVector3d posTool = tool->m_hapticPoint->getGlobalPosProxy();
                cVector3d posVertex = vertex->getLocalPos();
                double distance = cDistance(posTool, posVertex);

                // compute factor
                double RADIUS = 0.4;
                double relativeDistance = distance / RADIUS;
                double clampedRelativeDistance = cClamp01(relativeDistance);
                double w = 0.5 + 0.5 * cos(clampedRelativeDistance * C_PI);

                // apply offset
                double offsetVertexHeight = w * offsetHeight;
                posVertex.z(posVertex.z() + offsetVertexHeight);
                vertex->setLocalPos(posVertex);

                // mesh has been modified, inform the graphic rendering call back to
                // update the display list of the map.
                flagUpdateDisplayList = true;
            }
        }

        // move camera
        else if (state == STATE_MOVE_CAMERA)
        {
            // compute tool offset
            cVector3d offset = toolLocalPos - prevToolLocalPos;

            // apply camera motion
            cameraDistance = cameraDistance - 2 * offset.x();
            cameraAngleH = cameraAngleH - 40 * offset.y();
            cameraAngleV = cameraAngleV - 40 * offset.z();

            updateCameraPosition();   
        }

        // store tool position
        prevToolLocalPos  = toolLocalPos;
        prevToolGlobalPos = toolGlobalPos;

        // send forces to device
        tool->applyForces();

        // update frequency counter
        frequencyCounter.signal(1);
    }
    
    // exit haptics thread
    simulationFinished = true;
}

//---------------------------------------------------------------------------

int loadHeightMap()
{
    // create an image
    cImage* image = new cImage();

    // load a file
    bool fileload = image->loadFromFile(RESOURCE_PATH("resources/images/map.bmp"));
    if (!fileload)
    {
        #if defined(_MSVC)
        fileload = image->loadFromFile("../../../bin/resources/images/map.bmp");
        #endif
    }
    if (!fileload)
    {
        printf("Error - Texture image failed to load correctly.\n");
        close();
        return (-1);
    }

    // get the size of the image
    int sizeX = image->getWidth();
    int sizeY = image->getHeight();

    // check size of image
    if ((sizeX < 1) || (sizeY < 1)) { return (false); }

    // we look for the largest side
    int largestSide = cMax(sizeX, sizeY);

    // scale the image to fit the world
    double scale = 1.0 / (double)largestSide;

    // we will create an triangle based object. For centering puposes we
    // compute an offset for axis X and Y corresponding to the half size
    // of the image map.
    double offsetX = 0.5 * (double)sizeX * scale;
    double offsetY = 0.5 * (double)sizeY * scale;

    // For each pixel of the image, create a vertex
    int x,y;
    for (y=0; y<sizeY; y++)
    {
        for (x=0; x<sizeX; x++)
        {
            // get color of image pixel
            cColorb color;
            image->getPixelColor(x, y, color);

            // compute vertex height by averaging the color components RGB and scaling the value.
            const double HEIGHT_SCALE = 0.03;

            double height = HEIGHT_SCALE * (color.getLuminance() / 255.0);

            // compute the position of the vertex
            double px = scale * (double)x - offsetX;
            double py = scale * (double)y - offsetY;

            // create new vertex
            unsigned int index = object->newVertex(px, py, height);
            cVertex* vertex = object->getVertex(index);
        }
    }

    // Create a triangle based map using the above pixels
     for (x=0; x<(sizeX-1); x++)
    {
        for (y=0; y<(sizeY-1); y++)
        {
            // get the indexing numbers of the next four vertices
            unsigned int index00 = ((y + 0) * sizeX) + (x + 0);
            unsigned int index01 = ((y + 0) * sizeX) + (x + 1);
            unsigned int index10 = ((y + 1) * sizeX) + (x + 0);
            unsigned int index11 = ((y + 1) * sizeX) + (x + 1);

            // create two new triangles
            object->newTriangle(index00, index01, index10);
            object->newTriangle(index10, index01, index11);
        }
    }

    // compute normals
    object->computeAllNormals();

    // compute bounding box
    object->computeBoundaryBox(true);
    cVector3d min = object->getBoundaryMin();
    cVector3d max = object->getBoundaryMax();

    // compute size of object (largest side)
    cVector3d span = cSub(max, min);
    double size = cMax(span.x(), cMax(span.y(), span.z()));

    // scale object
    const double DESIRED_MESH_SIZE = 2.0;
    double scaleFactor = DESIRED_MESH_SIZE / size;
    object->scale(scaleFactor);

    // compute boundary box again
    object->computeBoundaryBox(true);

    // create collision detector for haptics interaction
    object->createAABBCollisionDetector(1.01 * proxyRadius);

    // success
    return (0);
}

//---------------------------------------------------------------------------

void updateCameraPosition()
{
    // check values
    if (cameraDistance < 0.1) { cameraDistance = 0.1; }
    if (cameraAngleV > 89) { cameraAngleV = 89; }
    if (cameraAngleV < -89) { cameraAngleV = -89; }

    // compute position of camera in space
    cVector3d pos = cAdd(
                        cameraPosition,
                        cVector3d(
                            cameraDistance * cCosDeg(cameraAngleH) * cCosDeg(cameraAngleV),
                            cameraDistance * cSinDeg(cameraAngleH) * cCosDeg(cameraAngleV),
                            cameraDistance * cSinDeg(cameraAngleV)
                        )
                    );

    // compute lookat position
    cVector3d lookat = cameraPosition;

    // define role orientation of camera
    cVector3d up(0.0, 0.0, 1.0);

    // set new position to camera
    camera->set(pos, lookat, up);

    // recompute global positions
    world->computeGlobalPositions(true);

    // update tool position
    if (tool != NULL)
    tool->setLocalPos(-cameraDistance, 0.0, 0.0);
}

//---------------------------------------------------------------------------
