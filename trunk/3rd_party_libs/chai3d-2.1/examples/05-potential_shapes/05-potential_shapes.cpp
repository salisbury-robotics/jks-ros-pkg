//===========================================================================
/*
	Homework #1 CS 277
	Arne Bech
 
 
    CS277 - Experimental Haptics
    Winter 2010, Stanford University

    You may use this program as a boilerplate for starting your homework
    assignments.  Use CMake (www.cmake.org) on the CMakeLists.txt file to
    generate project files for the development tool of your choice.  The
    CHAI3D library directory (chai3d-2.1.0) should be installed as a sibling
    directory to the one containing this project.

    These files are meant to be helpful should you encounter difficulties
    setting up a working CHAI3D project.  However, you are not required to
    use them for your homework -- you may start from anywhere you'd like.

    \author    Arne Bech, Francois Conti & Sonny Chan 
    \date      January 2010
*/
//===========================================================================

//---------------------------------------------------------------------------
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
//---------------------------------------------------------------------------
#include "chai3d.h"
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
// DECLARED CONSTANTS
//---------------------------------------------------------------------------

// initial size (width/height) in pixels of the display window
const int WINDOW_SIZE_W         = 600;
const int WINDOW_SIZE_H         = 600;

// mouse menu options (right button)
const int OPTION_FULLSCREEN     = 1;
const int OPTION_WINDOWDISPLAY  = 2;

// maximum number of haptic devices supported in this demo
const int MAX_DEVICES           = 8;


//---------------------------------------------------------------------------
// DECLARED VARIABLES
//---------------------------------------------------------------------------

// a world that contains all objects of the virtual environment
cWorld* world;

// a camera that renders the world in a window display
cCamera* camera;

// a light source to illuminate the objects in the virtual scene
cLight *light;

// width and height of the current window display
int displayW  = 0;
int displayH  = 0;

// a haptic device handler
cHapticDeviceHandler* handler;

// a pointer to the first haptic device detected on this computer
cGenericHapticDevice* hapticDevice = 0;

// a 3D cursor for the haptic device
cShapeSphere* cursor = 0;

// a line to display velocity of the haptic interface
cShapeLine* velocityVector;

// labels to show haptic device position and update rate
cLabel* positionLabel;
cLabel* rateLabel;
double rateEstimate = 0;

// material properties used to render the color of the cursors
cMaterial matCursorButtonON;
cMaterial matCursorButtonOFF;

// status of the main simulation haptics loop
bool simulationRunning = false;

// has exited haptics simulation thread
bool simulationFinished = false;

//Adding a sphere and a cube

int scene = 0;

//scene 0
cShapeSphere *sphere;
cMesh* cube;
cTexture2D* texture;
int vertices[6][4];

//scene 1
cShapeSphere *sphere2;
cShapeLine *line2;

//scene 2
cShapeSphere *sphere3;
cPrecisionClock *timer;
cLabel *gameInfoLabel;
cLabel *gameInfoLabel2;
cLabel *gameInfoLabel3;
cCamera *cam2;
cBitmap *bitmap;

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
    This application illustrates the use of the haptic device handler
    "cHapticDevicehandler" to access haptic devices connected to the computer.

    In this example the application opens an OpenGL window and displays a
    3D cursor for the first device found. If the operator presses the device
    user button, the color of the cursor changes accordingly.

    In the main haptics loop function  "updateHaptics()" , the position and 
    user switch status of the device are retrieved at each simulation iteration.
    This information is then used to update the position and color of the
    cursor. A force is then commanded to the haptic device to attract the 
    end-effector towards the device origin.
*/
//===========================================================================

int main(int argc, char* argv[])
{
    //-----------------------------------------------------------------------
    // INITIALIZATION
    //-----------------------------------------------------------------------

    printf ("\n");
    printf ("-----------------------------------\n");
    printf ("CS277 - Experimental Haptics\n");
    printf ("Homework # 1 -- Part 1\n");
	printf ("Arne Bech\n");
    printf ("January 2010, Stanford University\n");
    printf ("-----------------------------------\n");
	printf ("Press key [n] to change scenes!");
    printf ("\n\n");

    //-----------------------------------------------------------------------
    // 3D - SCENEGRAPH
    //-----------------------------------------------------------------------

    // create a new world.
    world = new cWorld();

    // set the background color of the environment
    // the color is defined by its (R,G,B) components.
    world->setBackgroundColor(0.2, 0.2, 0.2);

    // create a camera and insert it into the virtual world
    camera = new cCamera(world);
    world->addChild(camera);

    // position and oriente the camera
    camera->set( cVector3d (0.2, 0.0, 0.0),    // camera position (eye)
                 cVector3d (0.0, 0.0, 0.0),    // lookat position (target)
                 cVector3d (0.0, 0.0, 1.0));   // direction of the "up" vector

    // set the near and far clipping planes of the camera
    // anything in front/behind these clipping planes will not be rendered
    camera->setClippingPlanes(0.01, 10.0);

    // create a light source and attach it to the camera
    light = new cLight(world);
    camera->addChild(light);                   // attach light to camera
    light->setEnabled(true);                   // enable light source
    light->setPos(cVector3d( 2.0, 0.5, 1.0));  // position the light source
    light->setDir(cVector3d(-2.0, 0.5, 1.0));  // define the direction of the light beam

    // create a label that shows the haptic loop update rate
    rateLabel = new cLabel();
    rateLabel->setPos(8, 24, 0);
    camera->m_front_2Dscene.addChild(rateLabel);
	
	

    //-----------------------------------------------------------------------
    // HAPTIC DEVICES / TOOLS
    //-----------------------------------------------------------------------

    // create a haptic device handler
    handler = new cHapticDeviceHandler();

    // read the number of haptic devices currently connected to the computer
    int numHapticDevices = handler->getNumDevices();

    // if there is at least one haptic device detected...
	cHapticDeviceInfo info;
    if (numHapticDevices)
    {
        // get a handle to the first haptic device
        handler->getDevice(hapticDevice);

        // open connection to haptic device
        hapticDevice->open();

		// initialize haptic device
		hapticDevice->initialize();

        // retrieve information about the current haptic device
       info = hapticDevice->getSpecifications();
		
		//printf("max force: %lf",info.m_maxForce);

        // create a cursor with its radius set
        cursor = new cShapeSphere(0.003);

        // add cursor to the world
        world->addChild(cursor);

        // create a small line to illustrate velocity
        velocityVector = new cShapeLine(cVector3d(0,0,0), cVector3d(0,0,0));

        // add line to the world
        world->addChild(velocityVector);

        positionLabel = new cLabel();
        positionLabel->setPos(8, 8, 0);
        camera->m_front_2Dscene.addChild(positionLabel);
    }
	
//	 double workspaceScaleFactor = tool->getWorkspaceScaleFactor();
//	//maxStiffness
//	double stiffnessMax = info.m_maxForceStiffness / workspaceScaleFactor;
	
	
    // here we define the material properties of the cursor when the
    // user button of the device end-effector is engaged (ON) or released (OFF)

    // a light orange material color
    matCursorButtonOFF.m_ambient.set(0.5, 0.2, 0.0);
    matCursorButtonOFF.m_diffuse.set(1.0, 0.5, 0.0);
    matCursorButtonOFF.m_specular.set(1.0, 1.0, 1.0);

    // a blue material color
    matCursorButtonON.m_ambient.set(0.1, 0.1, 0.4);
    matCursorButtonON.m_diffuse.set(0.3, 0.3, 0.8);
    matCursorButtonON.m_specular.set(1.0, 1.0, 1.0);

	//-----------------------------------------------------------------------
    // Scene 0
    //-----------------------------------------------------------------------
	
	sphere = new cShapeSphere(0.015);
	world->addChild(sphere);
	sphere->setPos(0.0, 0.00, 0.004);
	
	//square
	cube = new cMesh(world);
	world->addChild(cube);
	
	cube->setPos(0.0,0.0,-0.024);
	
	
	
	const double HALFSIZE = 0.016;
	
    // face -x
    vertices[0][0] = cube->newVertex(-HALFSIZE,  HALFSIZE, -HALFSIZE);
    vertices[0][1] = cube->newVertex(-HALFSIZE, -HALFSIZE, -HALFSIZE);
    vertices[0][2] = cube->newVertex(-HALFSIZE, -HALFSIZE,  HALFSIZE);
    vertices[0][3] = cube->newVertex(-HALFSIZE,  HALFSIZE,  HALFSIZE);
	
    // face +x
    vertices[1][0] = cube->newVertex( HALFSIZE, -HALFSIZE, -HALFSIZE);
    vertices[1][1] = cube->newVertex( HALFSIZE,  HALFSIZE, -HALFSIZE);
    vertices[1][2] = cube->newVertex( HALFSIZE,  HALFSIZE,  HALFSIZE);
    vertices[1][3] = cube->newVertex( HALFSIZE, -HALFSIZE,  HALFSIZE);
	
    // face -y
    vertices[2][0] = cube->newVertex(-HALFSIZE,  -HALFSIZE, -HALFSIZE);
    vertices[2][1] = cube->newVertex( HALFSIZE,  -HALFSIZE, -HALFSIZE);
    vertices[2][2] = cube->newVertex( HALFSIZE,  -HALFSIZE,  HALFSIZE);
    vertices[2][3] = cube->newVertex(-HALFSIZE,  -HALFSIZE,  HALFSIZE);
	
    // face +y
    vertices[3][0] = cube->newVertex( HALFSIZE,   HALFSIZE, -HALFSIZE);
    vertices[3][1] = cube->newVertex(-HALFSIZE,   HALFSIZE, -HALFSIZE);
    vertices[3][2] = cube->newVertex(-HALFSIZE,   HALFSIZE,  HALFSIZE);
    vertices[3][3] = cube->newVertex( HALFSIZE,   HALFSIZE,  HALFSIZE);
	
    // face -z
    vertices[4][0] = cube->newVertex(-HALFSIZE,  -HALFSIZE, -HALFSIZE);
    vertices[4][1] = cube->newVertex(-HALFSIZE,   HALFSIZE, -HALFSIZE);
    vertices[4][2] = cube->newVertex( HALFSIZE,   HALFSIZE, -HALFSIZE);
    vertices[4][3] = cube->newVertex( HALFSIZE,  -HALFSIZE, -HALFSIZE);
	
    // face +z
    vertices[5][0] = cube->newVertex( HALFSIZE,  -HALFSIZE,  HALFSIZE);
    vertices[5][1] = cube->newVertex( HALFSIZE,   HALFSIZE,  HALFSIZE);
    vertices[5][2] = cube->newVertex(-HALFSIZE,   HALFSIZE,  HALFSIZE);
    vertices[5][3] = cube->newVertex(-HALFSIZE,  -HALFSIZE,  HALFSIZE);
	
    // create triangles
    for (int i=0; i<6; i++)
    {
        cube->newTriangle(vertices[i][0], vertices[i][1], vertices[i][2]);
        cube->newTriangle(vertices[i][0], vertices[i][2], vertices[i][3]);
    }
	
//	texture = new cTexture2D();
//    cube->setTexture(texture);
//    cube->setUseTexture(true);
	
	
	
    cube->m_material.m_ambient.set(0.5f, 0.5f, 0.5f, 1.0f);
    cube->m_material.m_diffuse.set(0.7f, 0.7f, 0.7f, 1.0f);
    cube->m_material.m_specular.set(1.0f, 1.0f, 1.0f, 1.0f);
    cube->m_material.m_emission.set(0.0f, 0.0f, 0.0f, 1.0f);
	
	cube->computeAllNormals();

	//cube->rotate(cVector3d(0,0,1), 0.24);
	cube->computeBoundaryBox(true);
	
	
	//-----------------------------------------------------------------------
    // Scene 1
    //-----------------------------------------------------------------------
	
	sphere2 = new cShapeSphere(0.001);
	world->addChild(sphere2);
	sphere2->setPos(0.0, 0.00, 0.004);
	sphere2->setShowEnabled(false, true);
	
	line2 = new cShapeLine(cVector3d(0,0.00,0.02),cVector3d(0,0.00,-0.03));
	world->addChild(line2);
	line2->setShowEnabled(false, true);
	line2->setPos(0.0, 0.02, 0.0);
	
	//-----------------------------------------------------------------------
    // Scene 2
    //-----------------------------------------------------------------------
	
	sphere3 = new cShapeSphere(0.03);
	world->addChild(sphere3);
	sphere3->setPos(-0.005, 0.00, 0.00);
	sphere3->setShowEnabled(false, true);
	sphere3->setWireMode(true, true);
	
	timer = new cPrecisionClock();
	
	gameInfoLabel = new cLabel();
    gameInfoLabel->setPos(WINDOW_SIZE_W / 2 - 150, WINDOW_SIZE_H * 4/ 5, 0);
    camera->m_front_2Dscene.addChild(gameInfoLabel);
	gameInfoLabel->setShowEnabled(false, true);
	//gameInfoLabel->scale(3.0, true);
	
	gameInfoLabel2 = new cLabel();
    gameInfoLabel2->setPos(20, 50, 0);
    camera->m_front_2Dscene.addChild(gameInfoLabel2);
	gameInfoLabel2->setShowEnabled(false, true);
	
	
	//other view
	texture = new cTexture2D();
    cube->setTexture(texture);
    cube->setUseTexture(false);
	
	bitmap = new cBitmap();
	camera->m_back_2Dscene.addChild(bitmap);
	double scalingFactor = 0.35;
	bitmap->setPos(0,(double)WINDOW_SIZE_H - (double)WINDOW_SIZE_H * scalingFactor,0);
	bitmap->setZoomHV(scalingFactor,scalingFactor);
	bitmap->setShowEnabled(false,true);
	
	//cam
	cam2 = new cCamera(world);
	cam2->set( cVector3d (0, 0.0, 0.08),    // camera position (eye)
				cVector3d (0.0, 0.0, 0.0),    // lookat position (target)
				cVector3d (-1.0, 0.0, 0.0));   // direction of the "up" vector
	cam2->setClippingPlanes(0.01, 10.0);
	
	

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
    glutSetWindowTitle("CHAI 3D");

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
    hapticsThread->set(updateHaptics, CHAI_THREAD_PRIORITY_HAPTICS);

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
    displayW = w;
    displayH = h;
	
	//printf("resize");
	gameInfoLabel->setPos(w / 2 - 50, h * 4/ 5, 0);
	
	double scalingFactor = 0.38;
	bitmap->setPos(0,(double)h - (double) h * scalingFactor,0);
	
    glViewport(0, 0, displayW, displayH);
	
	
	double txMin, txMax, tyMin, tyMax;
    if (displayW >= displayH)
    {
        double ratio = (double)displayW / (double)displayH;
        txMin = 0.5 * (ratio - 1.0) / ratio;
        txMax = 1.0 - txMin;
        tyMin = 0.0;
        tyMax = 1.0;
    }
    else
    {
        double ratio = (double)displayH / (double)displayW;
        txMin = 0.0;
        txMax = 1.0;
        tyMin = 0.5 * (ratio - 1.0) / ratio;
        tyMax = 1.0 - tyMin;
    }
	
    // update texture coordinates
    for (int i=0; i<6; i++)
    {
        cube->getVertex(vertices[i][0])->setTexCoord(txMin, tyMin);
        cube->getVertex(vertices[i][1])->setTexCoord(txMax, tyMin);
        cube->getVertex(vertices[i][2])->setTexCoord(txMax, tyMax);
        cube->getVertex(vertices[i][3])->setTexCoord(txMin, tyMax);
    }
	
	
	
	
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
	
	if (key == 'n') {
		
		scene++;
		
		scene = scene % 3;

		
		switch (scene) {
			case 0:
				
				camera->set( cVector3d (0.2, 0.0, 0.0),    // camera position (eye)
							cVector3d (0.0, 0.0, 0.0),    // lookat position (target)
							cVector3d (0.0, 0.0, 1.0));   // direction of the "up" vector
				
				sphere->setShowEnabled(true, true);
				cube->setShowEnabled(true, true);
				cube->setUseTexture(false);
				cube->setPos(0.0,0.0,-0.024);
				
				sphere2->setShowEnabled(false, true);
				line2->setShowEnabled(false, true);
				
				sphere3->setShowEnabled(false, true);
				bitmap->setShowEnabled(false,true);
				break;
			case 1:
				sphere->setShowEnabled(false, true);
				cube->setShowEnabled(false, true);
				
				sphere2->setPos(0.0, 0.00, 0.004);
				line2->setPos(0.0, 0.02, 0.0);
				sphere2->setShowEnabled(true, true);
				line2->setShowEnabled(true, true);
				
				sphere3->setShowEnabled(false, true);
				break;
				
			case 2:
				
				camera->set( cVector3d (0.15, 0.0, 0.0),    // camera position (eye)
							cVector3d (0.0, 0.0, 0.01),    // lookat position (target)
							cVector3d (0.0, 0.0, 1.0));   // direction of the "up" vector
				
				
				sphere->setShowEnabled(false, true);
				//cube->setShowEnabled(true, true);
				//cube->setUseTexture(true);
				//cube->setPos(0.0,-0.035,0.05);
				
				sphere2->setShowEnabled(true, true);
				line2->setShowEnabled(true, true);
				
				sphere3->setShowEnabled(true, true);
				bitmap->setShowEnabled(true,true);
				
				printf("Game rules: Don't touch the magnetic sphere or the magnetic line. \n");
				printf("Stay inside the big sphere at all times! \n");
				printf("Start game by pressing the main falcon button.\n");
				printf("Use the top view (top left corner) to help with orientation.\n");
				break;


			default:
				break;
		}
		
		
		
	}
}

//---------------------------------------------------------------------------

void menuSelect(int value)
{
    switch (value)
    {
        // enable full screen display
        case OPTION_FULLSCREEN:
            glutFullScreen();
            break;

        // reshape window to original size
        case OPTION_WINDOWDISPLAY:
            glutReshapeWindow(WINDOW_SIZE_W, WINDOW_SIZE_H);
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

    // close the haptic devices
    if (hapticDevice)
    {
        hapticDevice->close();
    }
}

//---------------------------------------------------------------------------

void updateGraphics(void)
{
    // update the label showing the position of the haptic device
    if (cursor)
    {
        cVector3d position = cursor->getPos() * 1000.0; // convert to mm
        char buffer[128];
        sprintf(buffer, "device position: (%.2lf, %.2lf, %.2lf) mm",
            position.x, position.y, position.z);
        positionLabel->m_string = buffer;
    }

    // update the label with the haptic refresh rate
    char buffer[128];
    sprintf(buffer, "haptic rate: %.0lf Hz", rateEstimate);
    rateLabel->m_string = buffer;
	
	//********** game *****************
	
	//gameInfoLabel->m_string = "TESTING TESTING";
	
	double currentTime = timer->getCurrentTimeSeconds();
	bool isGameOn = timer->on();
	
	gameInfoLabel->setShowEnabled(false, true);
	
	if (currentTime < 2 && currentTime > 0) {
		gameInfoLabel->setShowEnabled(true, true);
		gameInfoLabel->m_string = "Beware the magnetic killers!!!";
		
		timer->setTimeoutPeriodSeconds(0.02 + currentTime);
		
		
	} else {
		
	}

	static bool hasPrintedMeOnce = false;
	
	if (currentTime > 2 && isGameOn) { //game is going on
		
		gameInfoLabel2->setShowEnabled(true, true);
		
		char buffer[128];
		sprintf(buffer, "Score %.1lf", currentTime - 2);
		gameInfoLabel2->m_string = buffer;
		hasPrintedMeOnce = false;
		
	} else {
		gameInfoLabel2->setShowEnabled(false, true);
	}

	if (currentTime > 0 && !isGameOn) {
		//game ended
		gameInfoLabel->setShowEnabled(true, true);
		char buffer[128];
		sprintf(buffer, "You scored %.1lf points. Better luck next time!", currentTime - 2);
		gameInfoLabel->m_string = buffer;
		
		if (!hasPrintedMeOnce) {
			hasPrintedMeOnce = true;
		
			printf("You scored %.1lf points. Better luck next time!\n", currentTime - 2);
		}
		
	}
	
	if (scene == 2) {
		
		cam2->renderView(displayW, displayH);
		cam2->copyImageData(&bitmap->m_image);
		
	}

	
	
	
	//********** game end *************

    // render world

    texture->markForUpdate();
    camera->renderView(displayW, displayH);
	


    // Swap buffers
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
    // a clock to estimate the haptic simulation loop update rate
    cPrecisionClock pclock;
    pclock.setTimeoutPeriodSeconds(1.0);
    pclock.start(true);
    int counter = 0;

    // main haptic simulation loop
    while(simulationRunning)
    {
		if (!hapticDevice) continue;
		
		world->computeGlobalPositions(true);
		
        // read position of haptic device
        cVector3d newPosition;
        hapticDevice->getPosition(newPosition);

        // update position and orientation of cursor
        cursor->setPos(newPosition);

        // read linear velocity from device
        cVector3d linearVelocity;
        hapticDevice->getLinearVelocity(linearVelocity);

        // update the line showing velocity
        velocityVector->m_pointA = newPosition;
        velocityVector->m_pointB = newPosition + linearVelocity;

        // read user button status
        bool buttonStatus;
        hapticDevice->getUserSwitch(0, buttonStatus);

        // adjust the color of the cursor according to the status of
        // the user switch (ON = TRUE / OFF = FALSE)
        cursor->m_material = buttonStatus ? matCursorButtonON : matCursorButtonOFF;

        // compute a reaction force (a spring that pulls the device to the origin)
//        const double stiffness = 100; // [N/m]
//        cVector3d force = -stiffness * newPosition;
		cVector3d force = cVector3d();
		force.zero();
		
		if (scene == 0) {
			//************************* sphere
			//inside or outside?
			//world->computeGlobalPositions(true);
			cVector3d c_pos = newPosition;
			cVector3d s_pos	= sphere->getGlobalPos();
			
			double lengthFromSphere = s_pos.distance(c_pos) - cursor->getRadius();
			
			
			
			if (lengthFromSphere < sphere->getRadius()) {
				
				//printf("len: %lf rad: %lf \n",lengthFromSphere, sphere->getRadius());
				
				double strength = pow((sphere->getRadius() - lengthFromSphere) * 8000,2);
				
				//printf("str %lf \n",strength);
				
				force = (c_pos - s_pos) * strength;
			}
			//************************* cube
			
			cVector3d max = cube->getBoundaryMax();
			
			//printf("max x: %lf y: %lf z: %lf \n",max.x,max.y,max.z);
			
			cVector3d min = cube->getBoundaryMin();
			
			//printf("min x: %lf y: %lf z: %lf \n",min.x,min.y,min.z);
			
			//set min to zero point (origin)
			max = max - min;
			
			
			//printf("max x: %lf y: %lf z: %lf \n",max.x,max.y,max.z);
			
			
			
			cVector3d c = c_pos - cube->getGlobalPos();
			c = c - min;
			
			//printf("c x: %lf y: %lf z: %lf \n",c.x,c.y,c.z);
			
			double dxz = max.z / max.x;
			double dyz = max.z / max.y;
			double dzx = max.x / max.z;
			double dyx = max.x / max.y;
			double dzy = max.y / max.z;
			double dxy = max.y / max.x;
			double cr = cursor->getRadius();
			
			//inside cube
			cVector3d cubeForce;
			cubeForce.zero();
			
			if ((c.x < max.x +cr && c.y < max.y +cr && c.z < max.z +cr) && (c.x > - cr && c.y > - cr && c.z > - cr)) {
				
				double forceFactor = 900;
				
				if ((c.z > c.x * dxz) && (c.z > (max.z - c.x * dxz)) && (c.z > c.y * dyz) && (c.z > (max.z - c.y * dyz))) {
					
					//upper surface
					
					forceFactor = pow(forceFactor * (max.z - c.z+cr), 2);
					cubeForce = cVector3d(0,0,1) * forceFactor;
					
				} else if ((c.z <= c.x * dxz) && (c.z <= (max.z - c.x * dxz)) && (c.z <= c.y * dyz) && (c.z <= (max.z - c.y * dyz))) {
					//bottom surface
					
					forceFactor = pow(forceFactor * (c.z + cr) , 2);
					cubeForce = cVector3d(0,0,-1) * forceFactor;
					
				} else if ((c.x > c.z * dzx) && (c.x > (max.x - c.z * dzx)) && (c.x > c.y * dyx) && (c.x > (max.x - c.y * dyx))) {
					
					//front
					
					forceFactor = pow(forceFactor * (max.x - c.x +cr), 2);
					
					cubeForce = cVector3d(1,0,0) *forceFactor; 
					
				} else if ((c.x <= c.z * dzx) && (c.x <= (max.x - c.z * dzx)) && (c.x <= c.y * dyx) && (c.x <= (max.x - c.y * dyx))) {
					
					//back
					forceFactor = pow(forceFactor * (c.x + cr), 2);
					cubeForce = cVector3d(-1,0,0) * forceFactor;
					
				} else if ((c.y > c.z * dzy) && (c.y > (max.y - c.z * dzy)) && (c.y > c.x * dxy) && (c.y > (max.y - c.x * dxy))) {
					
					//right side
					forceFactor = pow(forceFactor * (max.y - c.y+cr), 2);
					cubeForce = cVector3d(0,1,0) * forceFactor ;
					
				} else if ((c.y <= c.z * dzy) && (c.y <= (max.y - c.z * dzy)) && (c.y <= c.x * dxy) && (c.y <= (max.y - c.x * dxy))) {
					
					//leftside
					forceFactor = pow(forceFactor * (c.y + cr), 2);
					cubeForce = cVector3d(0,-1,0) * forceFactor;
					
				}
				
				
			}
			
			force = force + cubeForce;
		}
		else if (scene == 1 || scene == 2) {
			
			
			cVector3d c_pos = newPosition;
			
			//printf("x %lf y %lf z %lf \n",c_pos.x,c_pos.y,c_pos.z);
			
			cVector3d r_pos = c_pos - sphere2->getGlobalPos(); //relative to center of sphere
			
			double	distance = r_pos.length();
			cVector3d sphereForce;
			sphereForce.zero();
			cVector3d lineForce;
			lineForce.zero();
			
			if (distance < 0.023) {
			
				//we can feel the magnetic pull from sphere
				
				double inv_dist = 0.023 - distance;
				
				//
				double forceStrength = - pow(100 * inv_dist, 3);
				
				
				//don't want a singularity so fade off - almost zero force in center
				if (distance < 0.003) {
					forceStrength = - pow(667 * (0.023 - inv_dist), 3);
					
					if (timer->on() && timer->getCurrentTimeSeconds() > 2) {
						timer->stop();
					}
					
				}
				
				
				sphereForce = r_pos;
				sphereForce.normalize();
				sphereForce = sphereForce * (forceStrength);
				
				
				
			}
			
			//line = field of cylinder + 2 domes?
			//Assume line is constant x,y. 
			

			cVector3d l_pos = line2->getPos();
			cVector3d pA = line2->m_pointA + l_pos;
			cVector3d pB = line2->m_pointB + l_pos;
			
#define MIN(X,Y) ((X) < (Y) ? (X) : (Y))
#define MAX(X,Y) ((X) > (Y) ? (X) : (Y))
			
			double z_min = MIN(pA.z,pB.z);
			double z_max = MAX(pA.z,pB.z);
			
			cVector3d r_posA = c_pos - pA; //relative to enf o line
			cVector3d r_posB = c_pos - pB; 
			
			double	distanceA = r_posA.length();
			double	distanceB = r_posB.length();
			
			
			if (c_pos.z > z_min && c_pos.z < z_max) {
			
				pA.z = c_pos.z;
				
				pA = c_pos - pA; 
				
				double distance = pA.length();
				
				if (distance < 0.023) {
					
					//we can feel the magnetic pull from sphere
					
					double inv_dist = 0.023 - distance;
					
					//
					double forceStrength = - pow(100 * inv_dist, 3);
					
					
					//don't want a singularity so fade off - almost zero force in center
					if (distance < 0.003) {
						forceStrength = - pow(667 * (0.023 - inv_dist), 3);
						
						if (timer->on() && timer->getCurrentTimeSeconds() > 2) {
							timer->stop();
						}
						
					}
					
					
					lineForce = pA;
					lineForce.normalize();
					lineForce = lineForce * (forceStrength);
					
					
					
				} 
				
				
			} else if (distanceA < 0.023) { //top half sphere
				
				//we can feel the magnetic pull from sphere
				
				double inv_dist = 0.023 - distanceA;
				
				//
				double forceStrength = - pow(100 * inv_dist, 3);
				
				
				//don't want a singularity so fade off - almost zero force in center
				if (distanceA < 0.003) {
					forceStrength = - pow(667 * (0.023 - inv_dist), 3);
					
					if (timer->on() && timer->getCurrentTimeSeconds() > 2) {
						timer->stop();
					}
				}
				
				
				lineForce = r_posA;
				lineForce.normalize();
				lineForce = lineForce * (forceStrength);
				
				
				
			} else  if (distanceB < 0.023) { //top half sphere
				
				//we can feel the magnetic pull from sphere
				
				double inv_dist = 0.023 - distanceB;
				
				//
				double forceStrength = - pow(100 * inv_dist, 3);
				
				
				//don't want a singularity so fade off - almost zero force in center
				if (distanceB < 0.003) {
					forceStrength = - pow(667 * (0.023 - inv_dist), 3);
					
					if (timer->on() && timer->getCurrentTimeSeconds() > 2) {
						timer->stop();
					}
				}
				
				
				lineForce = r_posB;
				lineForce.normalize();
				lineForce = lineForce * (forceStrength);
				
				
				
			} 
			

			
			force = sphereForce + lineForce;
		}
		
		if (scene == 2) {
			
			cVector3d c_pos = newPosition;
			cVector3d s_pos	= sphere3->getGlobalPos();
			
			double lengthFromSphere = s_pos.distance(c_pos);
			
			
			//warning, about to leave wireframe sphere (arena)
			if (lengthFromSphere < sphere3->getRadius() +  cursor->getRadius() && lengthFromSphere > sphere3->getRadius() -  cursor->getRadius()) {
			
				//touching sphere surface
				cursor->m_material = matCursorButtonON;
				
			} else {
				cursor->m_material = matCursorButtonOFF;
			}
			
			
			if (lengthFromSphere > sphere3->getRadius() +  cursor->getRadius()) {
			
				//outside of arena
				if (timer->on() && timer->getCurrentTimeSeconds() > 2) {
					timer->stop();
				}
				
				
			}
			
			
			if (lengthFromSphere < sphere3->getRadius() -  cursor->getRadius()) {
				
				//fully inside  of arena
				//change color depending on placement from sphere
				
				double percentage = lengthFromSphere / (sphere3->getRadius() -  cursor->getRadius());
				
				cMaterial mat;//  = new cMaterial();
				mat.m_ambient.set(0.1, 0.4, 0.4);
				mat.m_diffuse.set(percentage, 0.3, 0.2);
				mat.m_specular.set(1.0, 1.0, 1.0);
				
				cursor->m_material = mat;
				
			}
			
			
			

			if (buttonStatus) {
				
				timer->start(true); //start and reset the clock
				
				
			}
			
			if (timer->on() && timer->getCurrentTimeSeconds() > 2) { //game is going on
				
				double gametime = timer->getCurrentTimeSeconds() - 2;
				
				if (timer->timeoutOccurred()) {
					
					timer->setTimeoutPeriodSeconds(gametime + 2.02);
				
					
					//move sphere towards cursor
					cVector3d s_pos = sphere2->getGlobalPos();
					cVector3d direction = c_pos - s_pos;
					direction.normalize();
					
					double	 speed = pow(gametime,1.5) / 25000.0;
					
					direction = direction * speed;
					s_pos = s_pos + direction;
					
					sphere2->setPos(s_pos);
					
					speed = gametime / 50000.0;
					
					cVector3d l_pos = line2->getGlobalPos();
					direction = s_pos - l_pos;
					direction.normalize();
					direction = direction * speed;
					l_pos = l_pos + direction;
					line2->setPos(l_pos);
					
				}
				
			}
			
			
			
		}
		
        // send computed force to haptic device
		
		double temp = force.length();
		
		static int displayCounter = 0;
		displayCounter++;
		
		//if (displayCounter % 100 == 0)
			//printf("f: %lf \n",temp);
		
        hapticDevice->setForce(force);

        // estimate the refresh rate
        ++counter;
        if (pclock.timeoutOccurred()) {
            pclock.stop();
            rateEstimate = counter;
            counter = 0;
            pclock.start(true);
        }
    }
    
    // exit haptics thread
    simulationFinished = true;
}

//---------------------------------------------------------------------------
