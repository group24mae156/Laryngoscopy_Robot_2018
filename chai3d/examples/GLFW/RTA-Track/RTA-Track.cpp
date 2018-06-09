#include <chai3d.h>
#include <devices/CWoodenDevice.h>
#include <devices/CTeachingDevice.h>
#include <stdio.h>
#include <list>
#include <GLFW/glfw3.h>
#include <iostream>
#include <fstream> 
#include <sstream>
#include <unistd.h>
#include <sys/types.h>
#include <pwd.h>
#include <map>
#include <cmath>

using namespace chai3d;
using namespace std;

//------------------------------------------------------------------------------
// GENERAL SETTINGS
//------------------------------------------------------------------------------

// stereo Mode
/*
    C_STEREO_DISABLED:            Stereo is disabled 
    C_STEREO_ACTIVE:              Active stereo for OpenGL NVDIA QUADRO cards
    C_STEREO_PASSIVE_LEFT_RIGHT:  Passive stereo where L/R images are rendered next to each other
    C_STEREO_PASSIVE_TOP_BOTTOM:  Passive stereo where L/R images are rendered above each other
*/
cStereoMode stereoMode = C_STEREO_DISABLED;

// fullscreen mode
bool fullscreen = false;

// mirrored display
bool mirroredDisplay = false;

//------------------------------------------------------------------------------
// DECLARED VARIABLES
//------------------------------------------------------------------------------

// a world that contains all objects of the virtual environment
cWorld* world;

// a camera to render the world in the window display
cCamera* camera;

// a light source to illuminate the objects in the world
cSpotLight *light;

// a haptic device handler
cHapticDeviceHandler* handler;

// a pointer to the current haptic device
// cGenericHapticDevicePtr hapticDevice;
cWoodenDevicePtr armA;
cWoodenDevicePtr armB;
cTeachingDevicePtr hapticDevice;

// a line representing the force vector of the haptic device
cShapeLine* forceVector;

// a line representing the torque vector of the haptic device
cShapeLine* torqueVector;

// a small sphere (cursor) representing the haptic device 
cShapeSphere* cursorA; 
cShapeSphere* cursorB;
cShapeSphere* cursor;
cShapeSphere* targetCursor;

// a font for rendering text
cFontPtr font;

// a label to display the rate [Hz] at which the simulation is running
cLabel* labelRates;

// audio device to play sound
cAudioDevice* audioDevice;

// audio buffer to store feedback sound
cAudioBuffer* audioBufferFeedback;

// audio source of feedback
cAudioSource* audioSourceFeedback;

// a flag that indicates if the haptic simulation is currently running
bool simulationRunning = false;

// a flag that indicates if the haptic simulation has terminated
bool simulationFinished = true;

// a frequency counter to measure the simulation graphic rate
cFrequencyCounter freqCounterGraphics;

// a frequency counter to measure the simulation haptic rate
cFrequencyCounter freqCounterHaptics;

// haptic thread
cThread* hapticsThread;

// a handle to window display context
GLFWwindow* window = NULL;

// current width of window
int width = 0;

// current height of window
int height = 0;

// swap interval for the display context (vertical synchronization)
int swapInterval = 1;

// trajectories
int numTrajectories = 0;

// root resource path
string resourceRoot;

// trajectory file
FILE * rtaFile;

// trajectory data point
struct rtaPoint {
    cVector3d position;          // position in 3D space
    cVector3d orientation;       // orientation of tool

        // Set values
    rtaPoint(cVector3d a_position, cVector3d a_orientation):
      position(a_position), orientation(a_orientation) {}

    rtaPoint(){}
    
    void writeToFile( FILE * fp )
    {
    	if ( fp == NULL ) return;
    
    	double pos[3] = {position.x(), position.y(), position.z()};
    	double ori[3] = {orientation.x(), orientation.y(), orientation.z()};
    	
    	fwrite( pos, sizeof *pos, 3, fp );
    	fwrite( ori, sizeof *ori, 3, fp );
    }
    
    void readFromFile( FILE * fp )
    {
    	if ( fp == NULL ) return;
    
    	double pos[3];
    	double ori[3];
    	
    	fread( pos, sizeof *pos, 3, fp );
    	fread( ori, sizeof *ori, 3, fp );
    	
    	position.set( pos[0], pos[1], pos[2] );
    	orientation.set( ori[0], ori[1], ori[2] );
    }
};

// trajectory data
map< int, list<rtaPoint> > rtaData;


const double pi = 3.14159265358979323846;


//------------------------------------------------------------------------------
// DECLARED MACROS
//------------------------------------------------------------------------------
// convert to resource path
#define RESOURCE_PATH(p)    (char*)((resourceRoot+string(p)).c_str())


//------------------------------------------------------------------------------
// DECLARED FUNCTIONS
//------------------------------------------------------------------------------

// callback when the window display is resized
void windowSizeCallback(GLFWwindow* a_window, int a_width, int a_height);

// callback when an error GLFW occurs
void errorCallback(int error, const char* a_description);

// callback when a key is pressed
void keyCallback(GLFWwindow* a_window, int a_key, int a_scancode, int a_action, int a_mods);

// this function renders the scene
void updateGraphics(void);

// this function contains the main haptics simulation loop
void updateHaptics(void);

// this function closes the application
void close(void);

// this function computes a restoring force to get back to a specified point
cVector3d getRestoringForce(cVector3d& position, cVector3d& target, double threshold);

// this function computes a restoring torque to get back to a specified orientation
cVector3d getRestoringTorque(cVector3d& orientation, cVector3d& target, double radianThresholdX,
	double radianThresholdY, double radianThresholdZ);

int main(int argc, char* argv[])
{
    //--------------------------------------------------------------------------
    // INITIALIZATION
    //--------------------------------------------------------------------------

    cout << endl;
    cout << "-----------------------------------" << endl;
    cout << "CHAI3D" << endl;
    cout << "Team 22" << endl;
    cout << "Laryngoscopy Training Simulation" << endl;
    cout << "-----------------------------------" << endl << endl << endl;
    cout << "Keyboard Options:" << endl << endl;
    cout << "[f] - Enable/Disable full screen mode" << endl;
    cout << "[m] - Enable/Disable vertical mirroring" << endl;
    cout << "[q] - Exit application" << endl;
    cout << endl << endl;

    // parse first arg to try and locate resources
    resourceRoot = string(argv[0]).substr(0,string(argv[0]).find_last_of("/\\")+1);

    //--------------------------------------------------------------------------
    // OPEN TRAJECTORY FILE
    //--------------------------------------------------------------------------

	// query user for filename
	string filename;
	cout << "Enter the name of the file to write to (without extensions): ";
	cin >> filename;
	
	// open file
    const char *homedir;

    if ((homedir = getenv("HOME")) == NULL) {

        homedir = getpwuid(getuid())->pw_dir;

    }
    
    filename = string(homedir) + "/" + filename + ".bin";
	rtaFile = fopen( filename.c_str(), "rb" );
	
	if (rtaFile == NULL)
	{
		perror( filename.c_str() );
	}
	
	// import contents
	int c;
	
	while ( (c = fgetc( rtaFile )) != EOF )
	{
		// put character back
		ungetc( c, rtaFile );
	
		// debug
		std::cout << "Trajectory " << std::to_string(numTrajectories) << std::endl;
	
		// read list size
		unsigned int size;
		fread( &size, sizeof size, 1, rtaFile);
		
		// debug
		std::cout << "Size: " << std::to_string(size) << std::endl;
	
		for ( long int i = 0; i < size; i++ )
		{
			// read in point
			rtaPoint point = rtaPoint();
			point.readFromFile( rtaFile );
			
			// write to map
			rtaData[ numTrajectories ].push_front(point);
		}
		
		numTrajectories++;
	}
	
	
	// close file
	fclose(rtaFile);
	
	// debug
	std::cout << "Done!" << std::endl;
	
	

    //--------------------------------------------------------------------------
    // OPEN GL - WINDOW DISPLAY
    //--------------------------------------------------------------------------

    // initialize GLFW library
    if (!glfwInit())
    {
        cout << "failed initialization" << endl;
        cSleepMs(1000);
        return 1;
    }

    // set error callback
    glfwSetErrorCallback(errorCallback);

    // compute desired size of window
    const GLFWvidmode* mode = glfwGetVideoMode(glfwGetPrimaryMonitor());
    int w = 0.8 * mode->height;
    int h = 0.5 * mode->height;
    int x = 0.5 * (mode->width - w);
    int y = 0.5 * (mode->height - h);

    // set OpenGL version
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);

    // set active stereo mode
    if (stereoMode == C_STEREO_ACTIVE)
    {
        glfwWindowHint(GLFW_STEREO, GL_TRUE);
    }
    else
    {
        glfwWindowHint(GLFW_STEREO, GL_FALSE);
    }

    // create display context
    window = glfwCreateWindow(w, h, "CHAI3D", NULL, NULL);
    if (!window)
    {
        cout << "failed to create window" << endl;
        cSleepMs(1000);
        glfwTerminate();
        return 1;
    }

    // get width and height of window
    glfwGetWindowSize(window, &width, &height);

    // set position of window
    glfwSetWindowPos(window, x, y);

    // set key callback
    glfwSetKeyCallback(window, keyCallback);

    // set resize callback
    glfwSetWindowSizeCallback(window, windowSizeCallback);

    // set current display context
    glfwMakeContextCurrent(window);

    // sets the swap interval for the current display context
    glfwSwapInterval(swapInterval);

#ifdef GLEW_VERSION
    // initialize GLEW library
    if (glewInit() != GLEW_OK)
    {
        cout << "failed to initialize GLEW library" << endl;
        glfwTerminate();
        return 1;
    }
#endif


    //--------------------------------------------------------------------------
    // WORLD - CAMERA - LIGHTING
    //--------------------------------------------------------------------------

    // create a new world.
    world = new cWorld();

    // set the background color of the environment
    world->m_backgroundColor.setWhite();

    // create a camera and insert it into the virtual world
    camera = new cCamera(world);
    world->addChild(camera);

    // position and orient the camera
    camera->set(cVector3d(-1.0, 0.0, 0.375),    // camera position (eye)
                cVector3d(1.0, 0.0, 0.375),    // lookat position (target)
                cVector3d(0.0, 0.0, 1.0));   // direction of the (up) vector

    // set the near and far clipping planes of the camera
    // anything in front or behind these clipping planes will not be rendered
    camera->setClippingPlanes(0.01, 10.0);

    // set stereo mode
    camera->setStereoMode(stereoMode);

    // set stereo eye separation and focal length (applies only if stereo is enabled)
    camera->setStereoEyeSeparation(0.03);
    camera->setStereoFocalLength(3.0);

    // set vertical mirrored display mode
    camera->setMirrorVertical(mirroredDisplay);

    // create a light source
    light = new cSpotLight(world);

    // add light to world
    world->addChild(light);

    // enable light source
    light->setEnabled(true);                   

    // define the direction of the light beam
    light->setDir(1.0, 0.0, -0.2); 
    light->setLocalPos(-2.0, 0.0, 0.0); 
    
    // enable this light source to generate shadows
    light->setShadowMapEnabled(true);

    // set the resolution of the shadow map
    //light->m_shadowMap->setQualityLow();
    light->m_shadowMap->setQualityHigh();

    // set shadow factor
    world->setShadowIntensity(0.3);

    // set light cone half angle
    light->setCutOffAngleDeg(30);
    
    // create a sphere (cursor) to represent the haptic device
    cursorA = new cShapeSphere(0.03);
    cursorB = new cShapeSphere(0.03);
    cursor = new cShapeSphere(0.03);
    targetCursor = new cShapeSphere(0.03);

    // insert cursor inside world
    world->addChild(cursorA);
    world->addChild(cursorB);
    world->addChild(cursor);
    world->addChild(targetCursor);
    
    // set cursor colors
    cursorA->m_material->setGreenMediumAquamarine(); 
    cursorB->m_material->setOrangeCoral();
    cursor->m_material->setBlueRoyal();
    targetCursor->m_material->setRed();

    //--------------------------------------------------------------------------
    // HAPTIC DEVICES / TOOLS
    //--------------------------------------------------------------------------
    // create a haptic device handler
    handler = new cHapticDeviceHandler();

    // get access to the first available haptic device found
    // handler->getDevice(hapticDevice, 0);
	cGenericHapticDevicePtr ptrRetrieval;
	handler->getDevice(ptrRetrieval, 0);
	armA = dynamic_pointer_cast<cWoodenDevice>(ptrRetrieval);
	handler->getDevice(ptrRetrieval, 1);
	armB = dynamic_pointer_cast<cWoodenDevice>(ptrRetrieval);
	handler->getDevice(ptrRetrieval, 2);
	hapticDevice = dynamic_pointer_cast<cTeachingDevice>(ptrRetrieval);
	
	// pairs armA and armB
	hapticDevice->pair(armA, armB);

	// start haptic devices
	armA->open();
	armB->open();
	hapticDevice->open();
    
    
    //--------------------------------------------------------------------------
    // SETUP AUDIO
    //--------------------------------------------------------------------------
    
    
    // create an audio device to play sounds
    audioDevice = new cAudioDevice();
    
    // attach audio device to camera
    camera->attachAudioDevice(audioDevice);
    
    audioBufferFeedback = new cAudioBuffer();
    bool fileload5 = audioBufferFeedback->loadFromFile(RESOURCE_PATH("../resources/sounds/drill.wav"));
    if (!fileload5)
    {
        #if defined(_MSVC)
        fileload5 = audioBufferFeedback->loadFromFile("../../../bin/resources/sounds/drill.wav");
        #endif
    }
    if (!fileload5)
    {
        cout << "Error - Sound file failed to load or initialize correctly." << endl;
        close();
        return (-1);
    }

    // create audio source
    audioSourceFeedback = new cAudioSource();

    // assign auio buffer to audio source
    audioSourceFeedback->setAudioBuffer(audioBufferFeedback);

    // loop playing of sound
    audioSourceFeedback->setLoop(true);

    // turn off sound for now
    audioSourceFeedback->setGain(4.0);

    // set pitch
    audioSourceFeedback->setPitch(0.2);

    // play sound
    audioSourceFeedback->play();
    

    //--------------------------------------------------------------------------
    // CREATE PLANE
    //--------------------------------------------------------------------------
	
    // create mesh
    cMesh* plane = new cMesh();

    // add mesh to world
    world->addChild(plane);

    // create plane primitive
    cCreateMap(plane, 3.0, 3.0, 20, 20);

    // compile object
    plane->setUseDisplayList(true);

    // set color properties
    plane->m_material->setWhite();
	
    //--------------------------------------------------------------------------
    // CREATING OBJECTS
    //--------------------------------------------------------------------------

    // read the scale factor between the physical workspace of the haptic
    // device and the virtual workspace defined for the tool
    // double workspaceScaleFactor = tool->getWorkspaceScaleFactor();

    // get properties of haptic device
    // double maxLinearForce = cMin(hapticDeviceInfo.m_maxLinearForce, 7.0);
    // double maxStiffness = hapticDeviceInfo.m_maxLinearStiffness / workspaceScaleFactor;
    // double maxDamping   = hapticDeviceInfo.m_maxLinearDamping / workspaceScaleFactor; 
   
    // create small line to illustrate the force and torque of the haptic device
    forceVector = new cShapeLine(cVector3d(0,0,0), 
                              cVector3d(0,0,0));
    torqueVector = new cShapeLine(cVector3d(0,0,0), 
                              cVector3d(0,0,0));
                              
    forceVector->setLineWidth( 0.01 );
    torqueVector->setLineWidth( 0.01 );
    
    forceVector->m_colorPointA.setRed();
	forceVector->m_colorPointB.setRed();
	torqueVector->m_colorPointA.setGreen();
	torqueVector->m_colorPointB.setGreen();
	
    // insert line inside world
    world->addChild(forceVector);
    world->addChild(torqueVector);
   
    //--------------------------------------------------------------------------
    // WIDGETS
    //--------------------------------------------------------------------------

    // create a font
    font = NEW_CFONTCALIBRI20();
    
    // create a label to display the haptic and graphic rate of the simulation
    labelRates = new cLabel(font);
    camera->m_frontLayer->addChild(labelRates);


    //--------------------------------------------------------------------------
    // START SIMULATION
    //--------------------------------------------------------------------------

    // create a thread which starts the main haptics rendering loop
    hapticsThread = new cThread();
    hapticsThread->start(updateHaptics, CTHREAD_PRIORITY_HAPTICS);

    // setup callback when application exits
    atexit(close);


    //--------------------------------------------------------------------------
    // MAIN GRAPHIC LOOP
    //--------------------------------------------------------------------------

    // call window size callback at initialization
    windowSizeCallback(window, width, height);

    // main graphic loop
    while (!glfwWindowShouldClose(window))
    {
        // get width and height of window
        glfwGetWindowSize(window, &width, &height);

        // render graphics
        updateGraphics();

        // swap buffers
        glfwSwapBuffers(window);

        // process events
        glfwPollEvents();

        // signal frequency counter
        freqCounterGraphics.signal(1);
    }


    // close window
    glfwDestroyWindow(window);

    // terminate GLFW library
    glfwTerminate();

    // exit
    return (0);
}

//------------------------------------------------------------------------------

void windowSizeCallback(GLFWwindow* a_window, int a_width, int a_height)
{
    // update window size
    width = a_width;
    height = a_height;
}

//------------------------------------------------------------------------------

void errorCallback(int a_error, const char* a_description)
{
    cout << "Error: " << a_description << endl;
}

//------------------------------------------------------------------------------

void keyCallback(GLFWwindow* a_window, int a_key, int a_scancode, int a_action, int a_mods)
{
    // filter calls that only include a key press
    if ((a_action != GLFW_PRESS) && (a_action != GLFW_REPEAT))
    {
        return;
    }

    // option - exit
    else if ((a_key == GLFW_KEY_ESCAPE) || (a_key == GLFW_KEY_Q))
    {
        glfwSetWindowShouldClose(a_window, GLFW_TRUE);
    }

    // option - toggle fullscreen
    else if (a_key == GLFW_KEY_F)
    {
        // toggle state variable
        fullscreen = !fullscreen;

        // get handle to monitor
        GLFWmonitor* monitor = glfwGetPrimaryMonitor();

        // get information about monitor
        const GLFWvidmode* mode = glfwGetVideoMode(monitor);

        // set fullscreen or window mode
        if (fullscreen)
        {
            glfwSetWindowMonitor(window, monitor, 0, 0, mode->width, mode->height, mode->refreshRate);
            glfwSwapInterval(swapInterval);
        }
        else
        {
            int w = 0.8 * mode->height;
            int h = 0.5 * mode->height;
            int x = 0.5 * (mode->width - w);
            int y = 0.5 * (mode->height - h);
            glfwSetWindowMonitor(window, NULL, x, y, w, h, mode->refreshRate);
            glfwSwapInterval(swapInterval);
        }
    }

    // option - toggle vertical mirroring
    else if (a_key == GLFW_KEY_M)
    {
        mirroredDisplay = !mirroredDisplay;
        camera->setMirrorVertical(mirroredDisplay);
    }
}

//------------------------------------------------------------------------------

void close(void)
{
    // stop the simulation
    simulationRunning = false;

    // wait for graphics and haptics loops to terminate
    while (!simulationFinished) { cSleepMs(100); }

    // close haptic device
    armA->close();
    armB->close();
    hapticDevice->close();

    // delete resources
    delete hapticsThread;
    delete world;
    delete handler;
}

//------------------------------------------------------------------------------

void updateGraphics(void)
{
    /////////////////////////////////////////////////////////////////////
    // UPDATE WIDGETS
    /////////////////////////////////////////////////////////////////////

    // update haptic and graphic rate data
    labelRates->setText(cStr(freqCounterGraphics.getFrequency(), 0) + " Hz / " +
                        cStr(freqCounterHaptics.getFrequency(), 0) + " Hz");

    // update position of label
    labelRates->setLocalPos((int)(0.5 * (width - labelRates->getWidth())), 15);


    /////////////////////////////////////////////////////////////////////
    // RENDER SCENE
    /////////////////////////////////////////////////////////////////////

    // update shadow maps (if any)
    world->updateShadowMaps(false, mirroredDisplay);

    // render world
    camera->renderView(width, height);

    // wait until all GL commands are completed
    glFinish();

    // check for any OpenGL errors
    GLenum err = glGetError();
    if (err != GL_NO_ERROR) cout << "Error: " << gluErrorString(err) << endl;
}

//------------------------------------------------------------------------------

void updateHaptics(void)
{
    // simulation in now running
    simulationRunning  = true;
    simulationFinished = false;

    // main haptic simulation loop
    while(simulationRunning)
    {
		
		// display tool position
		cVector3d position, orientation;
		armA->getPosition(position);
		cursorA->setLocalPos(position);
		armB->getPosition(position);
		cursorB->setLocalPos(position);
		hapticDevice->getPosition(position);
		cursor->setLocalPos(position);

		// get tool orientation
		hapticDevice->getRollPitchYaw(orientation);

		// find nearest points
		rtaPoint nearestPoints[numTrajectories]; double maxDist; double dist;
		for (int i = 0; i < numTrajectories; i++)
		{
			maxDist = 1000000;
			for ( rtaPoint point : rtaData[i] )
			{
				if ( ( dist = position.distance( point.position ) ) < maxDist )
				{
					maxDist = dist;
					nearestPoints[i] = point;
				}
			}
		}
		
		// find center
		cVector3d center(0,0,0), centerTorque(0,0,0);
		for (int i = 0; i < numTrajectories; i++)
		{
			center += nearestPoints[i].position;
			
			//centerTorque += nearestPoints[i].orientation;
		}
		center = center / numTrajectories;
		//centerTorque = centerTorque / numTrajectories;
		
		// find deviation
		double stDev = 0, stRoll = 0, stPitch = 0, stYaw = 0;
		for (int i = 0; i < numTrajectories; i++)
		{
			dist = nearestPoints[i].position.distance( center );
			stDev += dist * dist;
			
			//dist     = abs( nearestPoints[i].orientation(0) - centerTorque(0) );
			//stRoll  += dist * dist;
			//dist     = abs( nearestPoints[i].orientation(1) - centerTorque(1) );
			//stPitch += dist * dist;
			//dist     = abs( nearestPoints[i].orientation(2) - centerTorque(2) );
			//stYaw   += dist * dist;
		}
		stDev   = sqrt( stDev / numTrajectories );
		//stRoll  = sqrt( stRoll / numTrajectories );
		//stPitch = sqrt( stPitch / numTrajectories );
		//stYaw   = sqrt( stYaw / numTrajectories );
		
		
		/*
		// find torque at center
		cVector3d centerTorque(0,0,0); double term, termSum = 0;
		for (int i = 0; i < numTrajectories; i++)
		{
			term = 1/( nearestPoints[i].position.distance( center ) );
			centerTorque += term * nearestPoints[i].orientation;
			termSum += term;
		}
		centerTorque = centerTorque / termSum;
		
		// find torque deviation
		double stRoll = 0, stPitch = 0, stYaw = 0, rollDist, pitchDist, yawDist;
		for (int i = 0; i < numTrajectories; i++)
		{
			// option 1
			term = 1/( nearestPoints[i].position.distance( center ) );
			rollDist = abs( ( centerTorque(0) * termSum / numTrajectories ) - 
				( nearestPoints[i].orientation(0) * term ) );
				
			pitchDist = abs( ( centerTorque(1) * termSum / numTrajectories ) - 
				( nearestPoints[i].orientation(1) * term ) ); 
				
			yawDist = abs( ( centerTorque(2) * termSum / numTrajectories ) - 
				( nearestPoints[i].orientation(2) * term ) );
				
				
			stRoll = rollDist * rollDist;
			stPitch = pitchDist * pitchDist;
			stYaw = yawDist * yawDist;
			
			
			// option 2
			rollDist  = abs( centerTorque(0) - nearestPoints[i].orientation(0) );
			pitchDist = abs( centerTorque(1) - nearestPoints[i].orientation(1) );
			yawDist   = abs( centerTorque(2) - nearestPoints[i].orientation(2) );
			
			stRoll  += (rollDist * term) * (rollDist * term);
			stPitch += (pitchDist * term) * (pitchDist * term);
			stYaw   += (yawDist * term) * (yawDist * term);
			
			
		}
		// option 1
		
		stRoll = sqrt( stRoll / numTrajectories ) / termSum;
		stPitch = sqrt( stPitch / numTrajectories ) / termSum;
		stYaw = sqrt( stYaw / numTrajectories ) / termSum;
		

		// option 2
		stRoll = sqrt( stRoll / numTrajectories );
		stPitch = sqrt( stPitch / numTrajectories );
		stYaw = sqrt( stYaw / numTrajectories );
		*/
		
		
		
		// get restoring forces and torques
		cVector3d restForce, restTorque;
		
		restForce = getRestoringForce( position, center, stDev );
		restTorque = getRestoringTorque( orientation, centerTorque, stRoll,
			stPitch, stYaw );
			
		// apply forces and torques
		// hapticDevice->setForceAndTorque( restForce, restTorque );
		forceVector->m_pointA = position;
        forceVector->m_pointB = cAdd(position, restForce / 50);
		torqueVector->m_pointA = position;
        torqueVector->m_pointB = cAdd(position, restTorque / 100);
        //std::cout << std::to_string( restForce.length() ) << " N || " << 
        //	std::to_string( restTorque.length() ) << "Nm || " << 
        //	std::to_string( stDev ) << " || " << std::to_string( stRoll ) << " || " <<
        //	std::to_string( stPitch ) << " || " << std:: to_string( stYaw ) << std::endl;
        std::cout << std::to_string( position.distance( center ) ) << std::endl;
        
        // determine feedback pitch
        audioSourceFeedback->setPitch(0.02 * restForce.length() );
        
        // set target cursor
        targetCursor->setLocalPos( center );

        // signal frequency counter
        freqCounterHaptics.signal(1);
    }
    
    // exit haptics thread
    simulationFinished = true;
}

//------------------------------------------------------------------------------

cVector3d getRestoringForce(cVector3d& position, cVector3d& target, double threshold)
{
	// return value
	cVector3d force(0,0,0);
	
	// proportional constant
	double Kd = 1;
	
	// determine if threshold is met
	//if (position.distance(target) < threshold) return force;
	
	// apply restoring forces
	(threshold) ? force.x( (target.x() - position.x()) * Kd / threshold ) : force.x(0);
	(threshold) ? force.y( (target.y() - position.y()) * Kd / threshold ) : force.y(0);
	(threshold) ? force.z( (target.z() - position.z()) * Kd / threshold ) : force.z(0);
	
	return force;
}

//------------------------------------------------------------------------------

cVector3d getRestoringTorque(cVector3d& orientation, cVector3d& target, double radianThresholdX,
	double radianThresholdY, double radianThresholdZ)
{
	// return value
	cVector3d force(0,0,0);
	
	// proportional constant
	double Kd = 1;
	
	// determine if threshold is met
	//if ( abs( orientation.x() - orientation.x() ) < radianThresholdX && 
	//	abs( orientation.y() - orientation.y() ) < radianThresholdY &&
	//	abs( orientation.z() - orientation.z() ) < radianThresholdZ ) return force;
	
	// apply restoring forces
	(radianThresholdX) ? force.x( (target.x() - orientation.x()) * Kd / radianThresholdX ) : force.x(0);
	(radianThresholdY) ? force.y( (target.y() - orientation.y()) * Kd / radianThresholdY ) : force.y(0);
	(radianThresholdZ) ? force.z( (target.z() - orientation.z()) * Kd / radianThresholdZ ) : force.y(0);
	
	return force;
}
//------------------------------------------------------------------------------
