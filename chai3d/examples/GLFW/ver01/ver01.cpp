//==============================================================================
/*
    Software License Agreement (BSD License)
    Copyright (c) 2003-2016, CHAI3D.
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
    COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,z
    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
    BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
    LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
    ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
    POSSIBILITY OF SUCH DAMAGE. 
    \author   <http://www.chai3d.org>
    \author   Francois Conti
    \author    Michael Berger
*/
//==============================================================================

//------------------------------------------------------------------------------
#include "chai3d.h"
#if defined(C_ENABLE_ALUMINUM_DEVICE_SUPPORT)
#include "devices/CAluminumDevice.h"
#endif
//------------------------------------------------------------------------------
#include <GLFW/glfw3.h>
//------------------------------------------------------------------------------
using namespace chai3d;
using namespace std;

#include <iostream>
#include <fstream> 
#include <sstream>
#include <unistd.h>
#include <sys/types.h>
#include <pwd.h>
#include <string>
#include <vector>

#include <stdio.h>
#include <sys/types.h>
#include <dirent.h>
//------------------------------------------------------------------------------

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

// show points other than the tip of the arm
bool showJoints = true;

// have safety shutoff for too high velocity reading
bool useVelocitySafety = false;

//------------------------------------------------------------------------------
// DECLARED VARIABLES
//------------------------------------------------------------------------------

// a world that contains all objects of the virtual environment
cWorld* world;

// a camera to render the world in the window display
cCamera* camera;

// a light source to illuminate the objects in the world
cDirectionalLight *light;

// a small sphere (cursor) representing parts of the haptic device 
cShapeSphere* cursor_1;
cShapeSphere* cursor_2; 
cShapeSphere* cursor_3;
cShapeSphere* cursor_4;

// a transparent sphere representing the trajectory start point
cShapeSphere* startPoint;

// a haptic device handler
cHapticDeviceHandler* handler;

// a line segment object to represent target trajectory
cMultiSegment* guidePath = new cMultiSegment();

// a line sgement object to connect joints of arms
cMultiSegment* jointRelations = new cMultiSegment();

// a pointer to the current haptic device
cAluminumDevicePtr hapticDevice;

// a label to display the haptic device model
cLabel* labelHapticDeviceModel;

// a label to display the position [m] of the haptic device
cLabel* labelHapticDevicePosition;

// a global variable to store the position [m] of the haptic device
cVector3d hapticDevicePosition;

// a label to display the linear velocity [m] of the haptic device
cLabel* labelHapticDeviceLinearVelocity;

// a global variable to store the linear velocity [m] of the haptic device
string hapticeDeviceLinearVelocity;

// a font for rendering text
cFontPtr font;

// a label to display the rate [Hz] at which the simulation is running
cLabel* labelRates;

// a label to display position axes
cLabel* labelPosAxes;

// a global variable to store the forces [N] on the haptic device
cVector3d hapticDeviceForces;

// a label to display forces at joint
cLabel* labelForces;

//a label to display force axes
cLabel* labelForcesAxes;

// a flag for using damping (ON/OFF)
bool useDamping = false;

// a flag for using force field (ON/OFF)
bool useForceField = false;

// a flag to indicate if the haptic simulation currently running
bool simulationRunning = false;

// a flag to indicate if the haptic simulation has terminated
bool simulationFinished = true;

// a flag for trajectory creation (ON/OFF)
bool useRecording = false;

// a flag for trajectory pathway (ON/OFF)
bool useTrajectory = true;

// a frequency counter to measure the simulation graphic rate
cFrequencyCounter freqCounterGraphics;

// a frequency counter to measure the simulation haptic rate
cFrequencyCounter freqCounterHaptics;

// haptic thread
cThread* hapticsThread;

// a handle to window display context
GLFWwindow* window = NULL;

// current width of window
int width  = 0;

// current height of window
int height = 0;

// swap interval for the display context (vertical synchronization)
int swapInterval = 1;

// global coordinate vectors used in trajectory reading and writing
std::vector<double> ave_x = {0};
std::vector<double> ave_y = {0};
std::vector<double> ave_z = {0};
std::vector<double> stddev_x = {0};
std::vector<double> stddev_y = {0};
std::vector<double> stddev_z = {0};

// variable for trajectory input file name
std::vector<string> inputFileName;

// variable for trajectory output file name
std::string outputFileName;

// variable for proportional force feedback
double proportionalInput;

// variable for how many points taken from trajectory file
int length;
int lines;

// variable for number of trajectories loaded
int numTrajectories;

// save_log variables
std::vector<cVector3d> positions_unique;
cVector3d a_position;

// parameter for max linear velocity allowed
double maxLinearVelocity = 1;

// parameter for where haptic feedback kicks in
double distanceTolerance = 0.01;

// global position vectors of the tip of arm and it's join
cVector3d position, position_2, position_3, position_4;

// variable to store location of home directory
const char *homedir;
//------------------------------------------------------------------------------
// DECLARED FUNCTIONS
//------------------------------------------------------------------------------

// function used to read trajectory data
void trajectoryRead(void);

// function used to record trajectory data
void trajectoryWrite(void);

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


//==============================================================================
/*
    Laryngoscopy Tracking Device
    This application illustrates how to program forces, torques and gripper
    forces to your haptic device.
    An OpenGL window is opened and displays 3D cursors representing the device connected to your computer. 
    In the main haptics loop function  "updateHaptics()" , the position,
    orientation and user switch status are read at each haptic cycle. 
    Force and torque vectors are computed and sent back to the haptic device.
*/
//==============================================================================

int main(int argc, char* argv[])
{
    //--------------------------------------------------------------------------
    // INITIALIZATION
    //--------------------------------------------------------------------------

    cout << endl;
    cout << "-----------------------------------" << endl;
    cout << "CHAI3D" << endl;
    cout << "Laryngoscopy Tracker" << endl;
    cout << "ver01" << endl;
    cout << "Team 24" << endl;
    cout << "-----------------------------------" << endl << endl << endl;
    cout << "Keyboard Options:" << endl << endl;
    cout << "[1] - Enable/Disable potential field" << endl;
    cout << "[2] - Enable/Disable damping" << endl;
    cout << "[t] - Enable/Disable guide path" << endl;
    cout << "[r] - Enable/Disable recording a trajectory" << endl;
    cout << "[f] - Enable/Disable full screen mode" << endl;
    cout << "[m] - Enable/Disable vertical mirroring" << endl;
    cout << "[q] - Exit application" << endl;
    cout << endl << endl;
    
    // finds home directory location
    if ((homedir = getenv("HOME")) == NULL) {

        homedir = getpwuid(getuid())->pw_dir;

    }
    // inputs user mode
    string userMode;
    string space;
    cout << "See mode instructions in chai3d/trajectory files/README.txt" << endl;
    cout << "Enter project mode {Options: test, multi, else}: ";
    cin >> userMode;

    // for easy compiling and testing
    if (userMode == "test"){
        numTrajectories = 1;
        inputFileName.push_back("please");
        outputFileName = "trajectory1";
        proportionalInput = 75;
    }
    
    // for loading multiple trajectories with integer name differencce in same folder
    else if (userMode == "multi"){
        // finds number of files in trajectory folder to be loaded
        DIR *dp;
        int fileCount;
        struct dirent *ep;
        string directoryName;
        cout << "Enter trajectory folder: ";
        cin >> directoryName;
        string directoryPath = string(homedir) + "/chai3d/trajectory files/" + directoryName;
        dp = opendir (directoryPath.c_str());

        if (dp != NULL)
        {
            while (ep = readdir(dp))
            fileCount++;
            closedir(dp);
        }
        else{
            perror ("Couldn't open the directory");
            exit(1);
        }
        //printf("There's %d files in the current directory.\n", fileCount);

        // shifted to account for hidden "." and ".." file  in directory
        for (int i=1; i<(fileCount-1); i++){
            inputFileName.push_back(directoryName + "/" + directoryName + std::to_string(i));
            //cout << inputFileName[i-1] +"\n";
            numTrajectories = i;
        }
        // query user for output fileName
        cout << "Enter the name of the file to record trajectory to (without extensions): ";
        cin >> outputFileName;

        // // query user for proportional feedback constant
        cout << "Enter proportional force constant (from 0 to 100) ";
        cin >> proportionalInput;
    }

    // for loading any number of trajectories from generic trajectory folder
    else{
        cout << "Enter number of trajectories to be loaded: ";
        cin >> numTrajectories;

        if (numTrajectories != 0){
            for (int i = 0; i < numTrajectories; i++) {
            // // query user for input inputFileName
            cout << "Enter the name of the file to read trajectory from (without extensions): ";
            string space;
            std::cin >> space;
            inputFileName.push_back(space);
            }
        }

        // query user for output fileName
        cout << "Enter the name of the file to record trajectory to (without extensions)" << endl;
        cin >> outputFileName;

        // // query user for proportional feedback constant
        cout << "Enter proportional force constant (from 0 to 150) ";
        cin >> proportionalInput;
    }


    //--------------------------------------------------------------------------
    // OPENGL - WINDOW DISPLAY
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

    // position and orient the camera side view
    camera->set( cVector3d (0.5, -0.8, 0.375),    // camera position (eye)
                 cVector3d (0.5, 0.0, 0.375),    // look at position (target)
                 cVector3d (0.0, 0.0, 1.0));   // direction of the (up) vector

    // set the near and far clipping planes of the camera
    camera->setClippingPlanes(0.01, 10.0);

    // set stereo mode
    camera->setStereoMode(stereoMode);

    // set stereo eye separation and focal length (applies only if stereo is enabled)
    camera->setStereoEyeSeparation(0.01);
    camera->setStereoFocalLength(0.5);

    // set vertical mirrored display mode
    camera->setMirrorVertical(mirroredDisplay);

    // create a directional light source
    light = new cDirectionalLight(world);

    // insert light source inside world
    world->addChild(light);

    // enable light source
    light->setEnabled(true);

    // define direction of light beam
    light->setDir(-1.0, 0.0, 0.0);

    // create a sphere (cursor) to represent the haptic device
    double cursorScaler = 1.5;
    cursor_1 = new cShapeSphere(0.015 * cursorScaler);
    cursor_2 = new cShapeSphere(0.012 * cursorScaler);
    cursor_3 = new cShapeSphere(0.012 * cursorScaler);
    cursor_4 = new cShapeSphere(0.02 * cursorScaler);

    // create a sphere to represent the start point
    startPoint = new cShapeSphere(0.02 * cursorScaler);

    // insert cursor into world
    world->addChild(cursor_1);
    world->addChild(cursor_2);
    world->addChild(cursor_3);
    world->addChild(cursor_4);

    // insert start point into world
    world->addChild(startPoint);

    // add guide path trajectory into world
    world->addChild(guidePath);

    // add joint relations object into world
    world->addChild(jointRelations);

    // assign line width of multiSegment objects
    guidePath->setLineWidth(3.0);
    jointRelations->setLineWidth(3.0);

    // use display list for faster rendering
    guidePath->setUseDisplayList(true);
    //jointRelations->setUseDisplayList(true);

  	// set cursor colors
    cursor_1->m_material->setBlue();
    cursor_2->m_material->setYellow();
    cursor_3->m_material->setRed();
    cursor_4->m_material->setOrange(); 

    // set start point color and transparency
    startPoint->m_material->setGreen();
    startPoint->setTransparencyLevel(0.4);

   
    //--------------------------------------------------------------------------
    // HAPTIC DEVICE
    //--------------------------------------------------------------------------

    // create a haptic device handler
    handler = new cHapticDeviceHandler();

    // get a handle to the first haptic device
    cGenericHapticDevicePtr ptrRetrieval;
	handler->getDevice(ptrRetrieval, 0);
    hapticDevice = dynamic_pointer_cast<cAluminumDevice>(ptrRetrieval);
    
    // open a connection to haptic device
    hapticDevice->open();

    // calibrate device (if necessary)
    hapticDevice->calibrate();

    // retrieve information about the current haptic device
    cHapticDeviceInfo info = hapticDevice->getSpecifications();

    // display a reference frame if haptic device supports orientations
    if (info.m_sensedRotation == true)
    {
        // display reference frame
        cursor_1->setShowFrame(true,true);

        // set the size of the reference frame
        cursor_1->setFrameSize(0.05);

        // display reference frame
        cursor_4->setShowFrame(true);

        // set the size of the reference frame
        cursor_4->setFrameSize(0.05);
    }
   
    //--------------------------------------------------------------------------
    // CREATE PLANE
    //--------------------------------------------------------------------------
	
    // create mesh
    cMesh* plane = new cMesh();

    // add mesh to world
    world->addChild(plane);

    // create plane primitive
    cCreateMap(plane, 5, 5, 10, 10);

    // compile object
    plane->setUseDisplayList(true);

    // set color properties
    plane->m_material->setGreen();


    //--------------------------------------------------------------------------
    // WIDGETS
    //--------------------------------------------------------------------------

    // create a font
    font = NEW_CFONTCALIBRI32();

    // create a label to display the haptic device model
    labelHapticDeviceModel = new cLabel(font);
    labelHapticDeviceModel->m_fontColor.setBlack();
    labelHapticDeviceModel->setText(info.m_modelName);
    camera->m_frontLayer->addChild(labelHapticDeviceModel);
    
    // create a label for pos axes
    labelPosAxes = new cLabel(font);
    labelPosAxes->m_fontColor.setBlack();
    labelPosAxes->setText("X Pos  Y Pos   Z Pos");
    camera->m_frontLayer->addChild(labelPosAxes);

    // create a label for pos axes
    labelForcesAxes = new cLabel(font);
    labelForcesAxes->m_fontColor.setBlack();
    labelForcesAxes->setText("X Force Y Force Z Force");
    camera->m_frontLayer->addChild(labelForcesAxes);

    // create a label to display the forces on tip of haptic device
    labelForces = new cLabel(font);
    labelForces->m_fontColor.setBlack();
    camera->m_frontLayer->addChild(labelForces);

    // create a label to display the position of haptic device
    labelHapticDevicePosition = new cLabel(font);
    labelHapticDevicePosition->m_fontColor.setBlack();
    camera->m_frontLayer->addChild(labelHapticDevicePosition);

    // create a label to display the linearVelocity of haptic device
    labelHapticDeviceLinearVelocity = new cLabel(font);
    labelHapticDeviceLinearVelocity->m_fontColor.setBlack();
    camera->m_frontLayer->addChild(labelHapticDeviceLinearVelocity);
    
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

    // reads trajectory data
    if (useTrajectory && (numTrajectories != 0)){
        trajectoryRead();

        // set start point position
        startPoint->setLocalPos(ave_x[0], ave_y[0], ave_z[0]);

        // Create guidePath line segment object
        double index0;
        double index1;
        for (int i=0;i<(lines-2);i++){
            // create vertex 0
            index0 = guidePath->newVertex(ave_x[i], ave_y[i], ave_z[i]);
                
            // create vertex 1
            index1 = guidePath->newVertex(ave_x[i+1], ave_y[i+1], ave_z[i+1]);

            // create segment by connecting both vertices together
            guidePath->newSegment(index0, index1);
        } 
    
    // sets guidePath object line color to green
    cColorf color;
    color.setYellowGold();
    guidePath->setLineColor(color);

    }

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

    // save log
    if (outputFileName != " "){
    trajectoryWrite();
    }

    // exit
    return 0;
}

//------------------------------------------------------------------------------
void trajectoryRead(void)
{
    // This is the global position vector which holds all x, y, and z positions 
    // of each trajectory like:
    // x1 y1 z1 x2 y2 z2 ... xn yn zn
    // .  .  .  .  .  .  ... .  .  .
    // .  .  .  .  .  .  ... .  .  .
    std::vector<vector<double>> pos_matrix(1000, vector<double>(numTrajectories*3));
    
    // opens trajectory file
    for (int i = 0; i < numTrajectories; i++) {
        ifstream trajectoryFile;
        trajectoryFile.open(string(homedir) + "/chai3d/trajectory files/" + inputFileName[i] + ".m", ios::in);
        if (trajectoryFile.fail()) {
            cerr << "Error Opening Trajectory File, Check File Name" <<endl;
            exit(1);
        }
        
        // this variable indicates the final size of all vectors after resizing
        lines = 1000; 
        double point;
        std::vector<double> all_points = {0};

        // reads data in until end of file
        while (trajectoryFile.peek()!=EOF) {
            trajectoryFile >> point;
            //cout << point;
            all_points.push_back(point);
        }

        // removes last point
        all_points.erase(all_points.begin()+all_points.size()-1);

        // removes first point
        all_points.erase(all_points.begin());
        
        //debugging
        //for (int i = 0; i < all_points.size(); i++) {

        //    cout << all_points[i];

        //}

        int third = all_points.size() / 3;

        std::vector<double> x_vec = {0};
        std::vector<double> y_vec = {0};
        std::vector<double> z_vec = {0};

        // Loading values into x_vec
        for (int i = 0; i < third; i++) {
            x_vec.push_back(all_points[i]);
            //cout << x_vec[i];
        }
        x_vec.erase(x_vec.begin());
        // cout << x_vec[x_vec.size()-1];
        // Loading values into y_vec
        for (int i = third; i < 2*third; i++) {
            y_vec.push_back(all_points[i]);
            //cout << y_vec[i-third];
        }
        y_vec.erase(y_vec.begin());

        // Loading values into z_vec
        for (int i = 2*third; i < 3*third; i++) {
            z_vec.push_back(all_points[i]);
            //cout << z_vec[i-2*third];
        }
        z_vec.erase(z_vec.begin());

        /**** VECTOR RESIZING ****/
        std::vector<double> x_temp = {0};
        std::vector<double> y_temp = {0};
        std::vector<double> z_temp = {0};

        double division = x_vec.size() / lines;
        int multiplier = 0;

        for (int i = 0; i < lines; i++) {
            x_temp.push_back(x_vec[round(multiplier * division)]);
            y_temp.push_back(y_vec[round(multiplier * division)]);
            z_temp.push_back(z_vec[round(multiplier * division)]);
            //cout << x_temp[i];
            multiplier++;
        }

        x_temp.erase(x_temp.begin());
        y_temp.erase(y_temp.begin());
        z_temp.erase(z_temp.begin());

        // clearing original position vectors and inputting temporary values
        x_vec.clear();
        y_vec.clear();
        z_vec.clear();

        for (int i = 0; i < lines; i++) {
            x_vec.push_back(x_temp[i]);
            y_vec.push_back(y_temp[i]);
            z_vec.push_back(z_temp[i]);
            //cout << x_vec.size();
        }
        /**** END OF VECTOR RESIZING ****/

        trajectoryFile.close();
        //std::cout << "Number of points in trajectory input file " << third << std::endl;

    // Creating position matrix
        for (int j = 0; j < lines; j++) {
            pos_matrix[j][3*i] = x_vec[j];
            pos_matrix[j][3*i + 1] = y_vec[j];
            pos_matrix[j][3*i + 2] = z_vec[j];
        }
    }
    // End of trajectory-adding loop

    /***** Creating average and stddev trajectories *****/
    ave_x.erase(ave_x.begin());
    ave_y.erase(ave_y.begin());
    ave_z.erase(ave_z.begin());
    
    for (int j = 0; j < lines; j++) {
        double xsum = 0;
        double ysum = 0;
        double zsum = 0;
        for (int i = 0; i < numTrajectories; i++) {
            xsum = xsum + pos_matrix[j][3*i];
            ysum = ysum + pos_matrix[j][3*i + 1];
            zsum = zsum + pos_matrix[j][3*i + 2];
        }
        ave_x.push_back(xsum/numTrajectories);
        ave_y.push_back(ysum/numTrajectories);
        ave_z.push_back(zsum/numTrajectories);
    } 

    // Creating stddev vectors
    stddev_x.erase(stddev_x.begin());
    stddev_y.erase(stddev_y.begin());
    stddev_z.erase(stddev_z.begin());

    for (int j = 0; j < lines; j++) {
        double xsum = 0;
        double ysum = 0;
        double zsum = 0;
        for (int i = 0; i < numTrajectories; i++) {
            xsum = xsum + pow(pos_matrix[j][3*i] - ave_x[j],2);
            ysum = ysum + pow(pos_matrix[j][3*i + 1] - ave_y[j],2);
            zsum = zsum + pow(pos_matrix[j][3*i + 2] - ave_z[j],2);
        }
        stddev_x.push_back(sqrt(xsum/numTrajectories));
        stddev_y.push_back(sqrt(ysum/numTrajectories));
        stddev_z.push_back(sqrt(zsum/numTrajectories));
    }
    // cout << ave_x[0] << endl;
    // cout << ave_y[0] << endl;
    // cout << ave_z[0] << endl;
    // cout << ave_x[999] << endl;
    // cout << ave_y[999] << endl;
    // cout << ave_z[999] << endl;
}

void trajectoryWrite(void)
{
    using namespace std;
    std::ofstream myfile;
    myfile.open (std::string(homedir) + "/chai3d/trajectory files/" + outputFileName + ".m");
    length = positions_unique.size(); 
    std::cout << "Number of points in trajectory output file " << length << std::endl;
    string channel[3] = {"Test1","Test2","Test3"}; 
    for(int c=4;c<7;++c){
        myfile << channel[c];
        for(int i=0;i<length;++i){
            if(c==4) myfile << positions_unique[i].x();
            if(c==5) myfile << positions_unique[i].y();
            if(c==6) myfile << positions_unique[i].z();
            myfile << " ";
        }
        myfile << "\n";
    }
    myfile.close();
}

void windowSizeCallback(GLFWwindow* a_window, int a_width, int a_height)
{
    // update window size
    width  = a_width;
    height = a_height;

    // update position of label
    labelHapticDeviceModel->setLocalPos(20, height - 40, 0);

    // update position of pos axes label
    labelPosAxes->setLocalPos(20, height - 70, 0);

    // update position of label
    labelHapticDevicePosition->setLocalPos(20, height - 100, 0);

    // update position of label
    labelHapticDeviceLinearVelocity->setLocalPos(20, height - 130, 0);
  
    //update position of force axes label
    labelForcesAxes->setLocalPos(width - 260, height - 70, 0);

    // update position of force label
    labelForces->setLocalPos(width - 260, height - 100, 0);
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

    // option - enable/disable force field
    else if (a_key == GLFW_KEY_1)
    {
        useForceField = !useForceField;
        if (useForceField){
            cout << "> Enable force field     \n";
        }
        else{
            cout << "> Disable force field    \n";
        }
    }

    // option - enable/disable trajectory recording
    else if (a_key == GLFW_KEY_R)
    {
        useRecording = !useRecording;
        if (useRecording){
            cout << "> Enable trajectory recording     \n";
            startPoint->setTransparencyLevel(0);
        }
        else{
            cout << "> Disable trajectory recording    \n";
            startPoint->setTransparencyLevel(0.4);

        }
    }

    // option - enable/disable trajectory on screen
    else if (a_key == GLFW_KEY_T)
    {
        useTrajectory = !useTrajectory;
        if (useTrajectory)
            cout << "> Enable trajectory      \n";
        else
            cout << "> Disable trajectory     \n";
    }

    // option - enable/disable damping
    else if (a_key == GLFW_KEY_2)
    {
        useDamping = !useDamping;
        if (useDamping)
            cout << "> Enable damping         \n";
        else
            cout << "> Disable damping        \n";
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

    // update position data
    labelHapticDevicePosition->setText(hapticDevicePosition.str(3));

    // update velocity data
    labelHapticDeviceLinearVelocity->setText("Linear Velocity: " + hapticeDeviceLinearVelocity);

    // update haptic and graphic rate data
    labelRates->setText(cStr(freqCounterGraphics.getFrequency(), 0) + " Hz / " +
                        cStr(freqCounterHaptics.getFrequency(), 0) + " Hz");

    // update position of label
    labelRates->setLocalPos((int)(0.5 * (width - labelRates->getWidth())), 15);

    // update force data
    labelForces->setText(hapticDeviceForces.str(3));

    if (useTrajectory){
         guidePath->setTransparencyLevel(1.0,useTrajectory,useTrajectory,useTrajectory);
    }
    else {
         guidePath->setTransparencyLevel(0,!useTrajectory,!useTrajectory,!useTrajectory);
    }
    /////////////////////////////////////////////////////////////////////
    // AluminumHAPTICS DEBUG INFO
    /////////////////////////////////////////////////////////////////////

#if defined(C_ENABLE_ALUMINUM_DEVICE_SUPPORT)
    if(cAluminumDevice* w = dynamic_cast<cAluminumDevice*>(hapticDevice.get())){
        cAluminumDevice::aluminumhaptics_status s = w->getStatus();
        //std::cout << s.toJSON() << std::endl;
    }
#endif





    /////////////////////////////////////////////////////////////////////
    // RENDER SCENE
    /////////////////////////////////////////////////////////////////////

    // update shadow maps (if any)
    world->updateShadowMaps(false, mirroredDisplay);

    // render world
    camera->renderView(width, height);

    // wait until all OpenGL commands are completed
    glFinish();

    // check for any OpenGL errors
    GLenum err;
    err = glGetError();
    if (err != GL_NO_ERROR) cout << "Error:  %s\n" << gluErrorString(err);
}

//------------------------------------------------------------------------------

void updateHaptics(void)
{
    // simulation in now running
    simulationRunning  = true;
    simulationFinished = false;
    int loopCount = 0;
    // main haptic simulation loop
    while(simulationRunning)
    {
        /////////////////////////////////////////////////////////////////////
        // READ HAPTIC DEVICE
        /////////////////////////////////////////////////////////////////////

        // grabs position and rotation data and creates vectors to assign it to  
        hapticDevice->getPosition(position, position_2, position_3, position_4);
        cMatrix3d rotation, rotation_2, rotation_3, rotation_4;
        hapticDevice->getRotation(rotation, rotation_2, rotation_3, rotation_4);

        // update position and orientation of cursor_1 (arm tip)
        cursor_1->setLocalPos(position);
        cursor_1->setLocalRot(rotation);

        if (showJoints){
        // update position and orientation of cursor_2 (last joint)
            cursor_2->setLocalPos(position_2);
            // cursor_2->setLocalRot(rotation_2);

            // update position and orientation of middle joint
            cursor_3->setLocalPos(position_3);
            // cursor_3->setLocalRot(rotation_3);

            // update position and orientation of base 
            cursor_4->setLocalPos(position_4);
            // cursor_4->setLocalRot(rotation_4);

            // call to function which creates and updates jointRelations object at slower tick rate than updateHaptics loop
            if (loopCount % 16 == 0){
                int n = jointRelations->getNumSegments();;
                
                //Create jointRelations line segment object
                if (n > 0){
                    jointRelations->clear();
                }
                jointRelations->newSegment(position, position_2);
                jointRelations->newSegment(position_2, position_3);
                jointRelations->newSegment(position_3, position_4);
                
            }
        }
        // sets jointRelations object to yellow
        cColorf color;
        color.setBlack();
        jointRelations->setLineColor(color);
        //read linear velocity 
        cVector3d linearVelocity; 
        hapticDevice->getLinearVelocity(linearVelocity);       
        double currentVelocity = pow(linearVelocity.x(), 2) + pow(linearVelocity.y(), 2) +pow(linearVelocity.z(),2);
        hapticeDeviceLinearVelocity = std::to_string(currentVelocity);

        // safety shutdown
        if (useVelocitySafety && (loopCount > 1000) && (currentVelocity > maxLinearVelocity)) {
        cerr << "Linear Velocity Too High, System Shutdown" <<endl;
        GLFWwindow* a_window;
        glfwSetWindowShouldClose(a_window, GLFW_TRUE);
        exit(1);
        }

        // read angular velocity
        cVector3d angularVelocity;
        hapticDevice->getAngularVelocity(angularVelocity);
     
        // /////////////////////////////////////////////////////////////////////
        // // UPDATE 3D CURSOR MODEL
        // /////////////////////////////////////////////////////////////////////
       
        // update global variable for graphic display update
        hapticDevicePosition = position;


        /////////////////////////////////////////////////////////////////////
        // CONTINUOUSLY UPDATING NEAREST POINT
        ////////////////////////////////////////////////////////////////////

        // initializing minimum distance from haptic device
        double min = sqrt(pow(ave_x[0]-position.x(),2) + pow(ave_y[0]-position.y(),2) + pow(ave_z[0]-position.z(),2));
        // index of mininum-distance point
        int minIndex = 0;

        // Loop through every point on trajectory and change min and minIndex if a smaller distance is found
        for (int i=1; i<lines; i++) {
                double nextDistance = sqrt(pow(ave_x[i]-position.x(),2) + pow(ave_y[i]-position.y(),2) + pow(ave_z[i]-position.z(),2));
                if (nextDistance < min) {
                    min = nextDistance;
                    minIndex = i;
                    
                }
        }
        double stdDevMag = sqrt(pow(stddev_x[minIndex],2) + pow(stddev_y[minIndex],2) + pow(stddev_z[minIndex],2));

            //cVector3d currentPosition (position.x(),position.y(),position.z());
            double x_hold = position.x();
            double y_hold = position.y();
            double z_hold = position.z();
            a_position.set(x_hold,y_hold,z_hold);

            // only record
            if (useRecording){
                positions_unique.push_back(a_position);
            }
           
       
        /////////////////////////////////////////////////////////////////////
        // COMPUTE AND APPLY FORCES
        /////////////////////////////////////////////////////////////////////

        // desired position
        cVector3d desiredPosition;
        double kpScaler = 1;
        if (min < distanceTolerance || min < (0.7 * stdDevMag)){
            desiredPosition.set(position.x(),position.y(),position.z());
        }
        else {
            
            desiredPosition.set(ave_x[minIndex], ave_y[minIndex], ave_z[minIndex]);
        }
    
        // desired orientation
        //cMatrix3d desiredRotation;
        //desiredRotation.identity();
        
        // variables for forces, torque, and gripper force
        cVector3d force (0,0,0);
        cVector3d torque (0,0,0);
        double gripperForce = 0;

        //apply force field
        if (useForceField && numTrajectories !=0)
        {
            if (numTrajectories > 1){
                if (min < 1 * stdDevMag){
                    kpScaler = 0.5;
                }
                if (min < 1.5 * stdDevMag){
                    kpScaler = 0.8;
                }
                if (min > 2 * stdDevMag){
                    kpScaler = 1.5;
                }
                if (min > 3 * stdDevMag){
                    kpScaler = 2;
                }
            }
            // compute linear force
            double Kp = proportionalInput * kpScaler; // [N/m]
            cVector3d forceField = Kp * (desiredPosition - position);
            force.add(forceField);

            // compute angular torque
            //double Kr = 0.05; // [N/m.rad]
            //cVector3d axis;
            //double angle;
            //cMatrix3d deltaRotation = cTranspose(rotation) * desiredRotation;
            //deltaRotation.toAxisAngle(axis, angle);
            //torque = rotation * ((Kr * angle) * axis);
        }
    
        // apply damping term
        if (useDamping)
        {
            cHapticDeviceInfo info = hapticDevice->getSpecifications();

            // compute linear damping force
            double Kv = 1.0 * info.m_maxLinearDamping;
            cVector3d forceDamping = -Kv * linearVelocity;
            //force.add(forceDamping);

            // compute angular damping force
            double Kvr = 1.0 * info.m_maxAngularDamping;
            cVector3d torqueDamping = -Kvr * angularVelocity;
            //torque.add(torqueDamping);

        }
       
        // update global variable for graphic display update
         hapticDeviceForces = force;

        // send computed force, torque, and gripper force to haptic device
        hapticDevice->setForceAndTorqueAndGripperForce(force, torque, gripperForce);

        // signal frequency counter
        freqCounterHaptics.signal(1);
        loopCount = loopCount + 1;

        // sleep to set update rate at approximately 1000Hz
         usleep(100);
    }
    
    // exit haptics thread
    simulationFinished = true;
}


//------------------------------------------------------------------------------
