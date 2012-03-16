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

//---------------------------------------------------------------------------
// DECLARED MACROS
//---------------------------------------------------------------------------
// convert to resource path
#define RESOURCE_PATH(p)    (char*)((resourceRoot+string(p)).c_str())

//---------------------------------------------------------------------------
// DECLARED BAD BAD BAD GLOBAL VARIABLES
//---------------------------------------------------------------------------

bool is_drawing = false;
string resourceRoot;
bool use_simple_dynamics = false;
double toolRadius = 0.05; // define the radius of the tool (sphere)
double stiffnessMax = 0.0;

double viscous_damping = 0.1;
const double MAX_DAMPING = 1.0, MIN_DAMPING = -0.5;

double gravity = 1.0;
cVector3d gravity_accel(0, 0, -gravity);
const double MIN_GRAVITY = 0, MAX_GRAVITY = 10;

const double DENSITY_MAX = 30, DENSITY_MIN = 1; // kg/m^3


struct RigidBody {
  RigidBody()
    : dims(1,1,1), density(10.0), mass(1.0), fixed(true)
  {
    updateProperties();
  }

  void initializeObject(bool from_scratch = true)
  {
    //printf("Initializing object!\n");
    // set the position of the object at the center of the world
    object->setPos(0.0, 0.0, 0.0);

    // create object
    object->clear();
    cCreateBox(object, dims.x(), dims.y(), dims.z());
    updateProperties();

    // compute collision detection algorithm
    object->createAABBCollisionDetector(1.01 * toolRadius);

    if(from_scratch)
    {
      // create a texture
      cTexture2d* texture = new cTexture2d();

      bool error = texture->loadFromFile(RESOURCE_PATH("resources/images/test1.bmp"));
      if(!error) printf("Failed to load texture!\n");

      object->setTexture(texture);
      object->setUseTexture(true);
      object->setUseCulling(false);
      object->m_material->setWhite();
      object->m_material->setStiffness(0.5 * stiffnessMax);
      object->m_material->setStaticFriction(0.2);
      object->m_material->setDynamicFriction(0.1);
      object->m_material->setTextureLevel(0.4);
      object->m_material->setRenderTriangles(true, false);
    }
    update_me = false;
  }

  void updateProperties()
  {

    mass = density*dims.x()*dims.y()*dims.z();
    I = mass/12.0*cVector3d( (dims.y()*dims.y() + dims.z()*dims.z()) ,
                                (dims.x()*dims.x() + dims.z()*dims.z()) ,
                                (dims.y()*dims.y() + dims.x()*dims.x()));

    printf("Object has mass %.2f, inertia terms %.2f, %.2f, %.2f \n", mass, I.x(), I.y(), I.z());
  }

  cMesh *object;
  cVector3d dims;
  double density, mass;
  cVector3d I;
  bool fixed;

  cVector3d velocity; // This should be taken to be in the world, expressed in world coordinates.
  cVector3d angular_velocity; // This should be taken to be in the world, expressed in world coordinates.

  bool update_me;
};

RigidBody body;


//---------------------------------------------------------------------------
// DECLARED VARIABLES
//---------------------------------------------------------------------------

// a world that contains all objects of the virtual environment
cWorld* world;

// a camera that renders the world in the window display
cCamera* camera;

// two light sources to illuminate the objects in the world
cSpotLight *light1;
cSpotLight *light2;

// width and height of the current window display
int displayWidth  = 0;
int displayHeight = 0;

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
    DEMO:    12-cubic.cpp

    This example illustrates how to build a small cube by assembling a
    cloud of triangles together. The applications also presents the
    use of texture properties by defining a texture image and defining
    texture coordinates at each of the vertices of the object.
    The texture image is produced and updated by copying the image
    buffer of the virtual camera at each graphical rendering cycle.
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
    printf ("Demo: 27-projectile\n");
    printf ("Copyright 2003-2012\n");
    printf ("-----------------------------------\n");
    printf ("\n\n");
    printf ("Keyboard Options:\n\n");
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
    world->setBackgroundColor(1.0, 1.0, 1.0);

    // create a camera and insert it into the virtual world
    camera = new cCamera(world);
    world->addChild(camera);

    // position and oriente the camera
    camera->set( cVector3d (2.0, 0.0, 0.0),    // camera position (eye)
                 cVector3d (0.0, 0.0, 0.0),    // lookat position (target)
                 cVector3d (0.0, 0.0, 1.0));   // direction of the "up" vector

    // set the near and far clipping planes of the camera
    // anything in front/behind these clipping planes will not be rendered
    camera->setClippingPlanes(0.01, 20.0);

    // set camera field of view 
    camera->setFieldViewAngle(80);

    // enable shadow casting
    camera->setUseShadowCasting(true);


    /////////////////////////////////////////////////////////////////////////
    // SPOTLIGHT 1
    /////////////////////////////////////////////////////////////////////////

    // create a light source
    light1 = new cSpotLight(world);

    // add light to world
    world->addChild(light1);    

    // enable light source
    light1->setEnabled(true);                   

    // position the light source
    light1->setPos( 0.5, 2.0, 0.0);             

    // define the direction of the light beam
    light1->setDir(-0.5, -2.0, 0.0);             

    // enable shadow casting
    light1->setShadowMapEnabled(true);

    // set light properties of spotlight 
    light1->setCutOffAngleDEG(30);
    light1->setSpotExponent(1.0);

    // set shadowmap near and far clipping planes
    light1->setShadowMapProperties(0.1, 4.0);


    /////////////////////////////////////////////////////////////////////////
    // SPOTLIGHT 2
    /////////////////////////////////////////////////////////////////////////

    // create a light source
    light2 = new cSpotLight(world);

    // add light to world
    world->addChild(light2);    

    // enable light source
    light2->setEnabled(true);                   

    // position the light source
    light2->setPos( 0.5,-2.0, 0.0);             

    // define the direction of the light beam
    light2->setDir(-0.5, 2.0, 0.0);             

    // enable shadow casting
    light2->setShadowMapEnabled(true);

    // set light properties of spotlight 
    light2->setCutOffAngleDEG(30);
    light2->setSpotExponent(1.0);

    // set shadowmap near and far clipping planes
    light2->setShadowMapProperties(0.1, 4.0);

    
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
    cQuaternion q;
    cMatrix3d m;
    q.fromAxisAngle(cVector3d(0,1,0),M_PI_2);
    q.toRotMat(m);
    cCreateCylinder(tool->m_image, 0.3, 0.75*toolRadius, 8, 3, true, true, cVector3d(0, 0, 0), m);
    cCreateBox(tool->m_image, 0.4, 0.5*toolRadius, 0.5*toolRadius, cVector3d(0.25,0,0) );
    cCreateSphere(tool->m_image, toolRadius);
    world->addChild(tool);

    // connect the haptic device to the tool
    tool->setHapticDevice(hapticDevice);

    // initialize tool by connecting to haptic device
    tool->start();

    // map the physical workspace of the haptic device to a larger virtual workspace.
    tool->setWorkspaceRadius(0.9);

    // define a radius for the tool
    tool->setRadius(toolRadius);

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
    stiffnessMax = info.m_maxLinearStiffness / workspaceScaleFactor;


    //-----------------------------------------------------------------------
    // COMPOSE THE VIRTUAL SCENE
    //-----------------------------------------------------------------------

    // create a virtual mesh
    //object = new cMesh(world);
    body.object = new cMesh(world);

    // add object to world
    world->addChild(body.object);

    body.dims.set(0.2, 1.0, 1.5);
    body.initializeObject(true);

//    cShapeBox* box = new cShapeBox(0.3, 0.3, 0.3);
//    world->addChild(box);
//    box->setPos(0,0,-0.5);

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

    switch(key)
    {
    case('d'):
      use_simple_dynamics ^= 1;
      if(use_simple_dynamics)
        printf("Using simple dynamics!\n");
      else
        printf("Using real dynamics! \n");
      break;
    case('1'):
      body.dims.x( std::max(body.dims.x() - 0.1, 0.1) );
      printf("Object dimensions are now: %.2f %.2f, %.2f \n", body.dims.x(), body.dims.y(), body.dims.z());
      body.update_me = true;
      break;
    case('2'):
      body.dims.x( std::min(body.dims.x() + 0.1, 1.5) );
      printf("Object dimensions are now: %.2f %.2f, %.2f \n", body.dims.x(), body.dims.y(), body.dims.z());
      body.update_me = true;
      break;
    case('q'):
      body.density = std::min(body.density + 1.0, DENSITY_MAX);
      printf("Object density is %.2f kg\n", body.density);
      body.updateProperties();
      break;
    case('a'):
      body.density = std::max(body.density - 1.0, DENSITY_MIN);
      printf("Object density is %.2f kg\n", body.density);
      body.updateProperties();
      break;
    case('w'):
      viscous_damping = std::min(viscous_damping + 0.1, MAX_DAMPING);
      printf("Damping is %.2f N*s/rad\n", viscous_damping);
      break;
    case('s'):
      viscous_damping = std::max(viscous_damping - 0.1, MIN_DAMPING);
      printf("Damping is %.2f N*s/rad\n", viscous_damping);
      break;
    case('r'):
    {
      cMatrix3d mat;
      mat.identity();
      body.object->setRot(mat);
      body.object->setPos(0,0,0);
      break;
    }
    case('t'):
      gravity = std::min(gravity + 1.0, MAX_GRAVITY);
      gravity_accel.set(0,0,-gravity);
      printf("Gravity is -%.2f m/s^2 z>\n", gravity);
      break;
    case('g'):
      gravity = std::max(gravity - 1.0, MIN_GRAVITY);
      gravity_accel.set(0,0,-gravity);
      printf("Gravity is -%.2f m/s^2 z>\n", gravity);
      break;
    case('f'):
      body.fixed ^= 1;
      if(body.fixed)
        printf("Body center is fixed!\n");
      else
        printf("Body center is free! \n");
      break;
    default:
      break;

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
  while(body.update_me)
  {
    cSleepMs(10);
  }

  is_drawing = true;

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
    is_drawing = false;
}

//---------------------------------------------------------------------------

void updateHaptics(void)
{
    // angular velocity of object
    cVector3d angVel(0,0,0);

    double wx = 0.0, wy = 0.0, wz = 0.0;
    double wx_dot = 0.0, wy_dot = 0.0, wz_dot = 0.0;


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
        // NON_THREAD_SAFE SCENE UPDATES
        /////////////////////////////////////////////////////////////////////
        if(body.update_me && !is_drawing)
        {
          printf("Refreshing scene geometry...\n");
          body.initializeObject(true);
          printf("Done!\n");
        }


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
        cVector3d objectPos = body.object->getGlobalPos();

        // compute a vector from the center of mass of the object (point of rotation) to the tool
        cVector3d v = cSub(toolPos, objectPos);


        // compute angular acceleration based on the interaction forces
        // between the tool and the object
        cVector3d angAcc(0,0,0);
        cVector3d torque(0,0,0);
        if (v.length() > 0.0)
        {
            // get the last force applied to the cursor in global coordinates
            // we negate the result to obtain the opposite force that is applied on the
            // object
            cVector3d toolForce = cNegate(tool->m_lastComputedGlobalForce);

            // compute the effective force that contributes to rotating the object.
            cVector3d force = toolForce - cProject(toolForce, v);

            // compute the resulting torque
            torque = cMul(v.length(), cCross( cNormalize(v), force));
        }

        if(!use_simple_dynamics)
        {
          cVector3d Tb = cMul(cTrans(body.object->getGlobalRot()), torque);
          double b = viscous_damping;
          wx_dot = ((body.I.y() - body.I.z())*wy*wz + Tb.x() - b*wx)/body.I.x();
          wy_dot = ((body.I.z() - body.I.x())*wx*wz + Tb.y() - b*wy)/body.I.y();
          wz_dot = ((body.I.x() - body.I.y())*wy*wx + Tb.z() - b*wz)/body.I.z();

          wx += timeInterval * wx_dot;
          wy += timeInterval * wy_dot;
          wz += timeInterval * wz_dot;

          angVel = cMul(body.object->getGlobalRot(), cVector3d(wx, wy, wz));

          // if user switch is pressed, set velocity to zero
          if (tool->getUserSwitch(0) == 1)
          {
              angVel.zero();
              wx = wy = wz = 0;
          }

          // compute the next rotation configuration of the object
          if (angVel.length() > C_SMALL)
          {
              body.object->rotate(cNormalize(angVel), timeInterval * angVel.length());
          }
        }

        if(use_simple_dynamics)
        {
          // update rotational acceleration
          const double INERTIA = 0.4*body.mass;
          angAcc = (1.0 / INERTIA) * torque;

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
              body.object->rotate(cNormalize(angVel), timeInterval * angVel.length());
          }
        }

        // Now let's do Newton for position:
        if(!body.fixed)
        {
          // get the last force applied to the cursor in global coordinates
          // we negate the result to obtain the opposite force that is applied on the
          // object
          cVector3d tool_force = cNegate(tool->m_lastComputedGlobalForce);

          cVector3d body_forces = tool_force + gravity_accel*body.mass;
          body.velocity += (body_forces / body.mass) * timeInterval;

          // if user switch is pressed, set velocity to zero
          if (tool->getUserSwitch(0) == 1)
          {
              body.velocity.zero();
          }

          body.object->translate(body.velocity*timeInterval);
        }


    }
    
    // exit haptics thread
    simulationFinished = true;
}

//---------------------------------------------------------------------------
