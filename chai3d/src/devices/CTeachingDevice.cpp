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
    COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
    BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
    LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
    ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
    POSSIBILITY OF SUCH DAMAGE. 

    \author    <http://www.chai3d.org>
    \author    Alex Bertino
    \version   3.2.0 $Rev: 1869 $
*/
//==============================================================================


//------------------------------------------------------------------------------
#include "system/CGlobals.h"
#include "devices/CTeachingDevice.h"
#include "math/CTransform.h"
#include <math.h>
#include <string>

// Following includes are only used for reading/writing config file and to find 
// the user's home directory (where the config file will be stored)
#include <iostream>
#include <fstream> 
#include <sstream>
#include <unistd.h>
#include <sys/types.h>
#include <pwd.h>

// For USB HID version
#include "hidapi.h"
#include <stdio.h>
#include <wchar.h>
#include <string.h>
#include <stdlib.h>
#include "hidapi.h"
//------------------------------------------------------------------------------
#if defined(C_ENABLE_TEACHING_DEVICE_SUPPORT)
//------------------------------------------------------------------------------

////////////////////////////////////////////////////////////////////////////////
/*
    INSTRUCTION TO IMPLEMENT YOUR OWN CUSTOM DEVICE:

    Please review header file CMyCustomDevice.h for some initial 
    guidelines about how to implement your own haptic device using this
    template.

    When ready, simply completed the next 11 documented steps described here
    below.
*/
////////////////////////////////////////////////////////////////////////////////

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------


//==============================================================================
// WoodenHaptics configuration helper files.
//==============================================================================

cTeachingDevice::configuration default_teaching(){
    double data[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    return cTeachingDevice::configuration(data); 
}

double v2(const std::string& json, const std::string& key){
    int p = json.find(":", json.find(key));
    return atof(json.substr(p+1).c_str());
}

cTeachingDevice::configuration fromJSON2(std::string json){
    double d[]= {
        v2(json,"length"),
        v2(json,"length_b"),
        v2(json,"length_blade"),      
        v2(json,"height_blade_drop"),      
        v2(json,"workspace_radius"),           
        v2(json,"torque_constant_motor"),           
        v2(json,"current_for_10_v_signal"),           
        v2(json,"cpr_encoder"),         
        v2(json,"max_linear_force"),         
        v2(json,"max_linear_stiffness"),         
        v2(json,"max_linear_damping"),      
        v2(json,"max_rotational_stiffness"),      
        v2(json,"max_rotational_damping"),      
        v2(json,"encoder_serial_number"),
		v2(json,"length_com"),
		v2(json,"height_com"),
		v2(json,"mass"),
		v2(json,"inertia_x"),
		v2(json,"inertia_y"),
		v2(json,"inertia_z")
    }; 
    return cTeachingDevice::configuration(d);
}

std::string j2(const std::string& key, const double& value){
   std::stringstream s;
   s << "    \"" << key << "\":";
   while(s.str().length()<32) s<< " ";
   s << value << "," << std::endl;
   return s.str();
}
std::string toJSON(const cTeachingDevice::configuration& c){
   using namespace std;
   stringstream json;
   json << "{" << endl
        << j2("length",c.length)
        << j2("length_b",c.length_b)
        << j2("length_blade",c.length_blade)
        << j2("height_blade_drop",c.height_blade_drop)
        << j2("workspace_radius",c.workspace_radius)
        << j2("torque_constant_motor",c.torque_constant_motor)
        << j2("current_for_10_v_signal",c.current_for_10_v_signal)
        << j2("cpr_encoder",c.cpr_encoder)
        << j2("max_linear_force",c.max_linear_force)
        << j2("max_linear_stiffness",c.max_linear_stiffness)
        << j2("max_linear_damping",c.max_linear_damping)
        << j2("max_rotational_stiffness",c.max_rotational_stiffness)
        << j2("max_rotational_damping",c.max_rotational_damping)
        << j2("encoder_serial_number",c.encoder_serial_number)
        << j2("length_com",c.length_com)
        << j2("height_com",c.height_com)
		<< j2("mass",c.mass)
		<< j2("inertia_x",c.inertia_x)
		<< j2("inertia_y",c.inertia_y)
		<< j2("inertia_z",c.inertia_z)
        << "}" << endl;
   return json.str();
}

void write_config_file(const cTeachingDevice::configuration& config){
    const char *homedir;
    if ((homedir = getenv("HOME")) == NULL) {
        homedir = getpwuid(getuid())->pw_dir;
    }

    std::cout << "Writing configuration to: "<< homedir 
              << "/teaching_device.json" << std::endl;
    std::ofstream ofile;
    ofile.open(std::string(homedir) + "/teaching_device.json");
    ofile << toJSON(config);
    ofile.close();
}

cTeachingDevice::configuration read_config_file(){
    const char *homedir;
    if ((homedir = getenv("HOME")) == NULL) {
        homedir = getpwuid(getuid())->pw_dir;
    }

    std::cout << "Trying loading configuration from: "<< homedir 
              << "/teaching_device.json" << std::endl;

    std::ifstream ifile;
    ifile.open(std::string(homedir) + "/teaching_device.json");
    if(ifile.is_open()){
        std::stringstream buffer;
        buffer << ifile.rdbuf();
        ifile.close();
        std::cout << "Success. " << std::endl;
        return fromJSON2(buffer.str());    
    } else {
        std::cout << "File not found. We will write one "
                  << "based on default configuration values." << std::endl;

        write_config_file(default_teaching());
        return default_teaching();
    }
}


//==============================================================================
/*!
    Constructor of cTeachingDevice.
*/
//==============================================================================
cTeachingDevice::cTeachingDevice(unsigned int a_deviceNumber):
    m_config(read_config_file())
{
	

    // the connection to your device has not yet been established.
    m_deviceReady = false;


    ////////////////////////////////////////////////////////////////////////////
    /*
        STEP 1:

        Here you should define the specifications of your device.
        These values only need to be estimates. Since haptic devices often perform
        differently depending of their configuration withing their workspace,
        simply use average values.
    */
    ////////////////////////////////////////////////////////////////////////////

    //--------------------------------------------------------------------------
    // NAME:
    //--------------------------------------------------------------------------
	
    // If we have a config file, use its values, otherwise, 
    // use standard values (and write them to config file)
    std::cout << std::endl << "WoodenHaptics configuration used: " << std::endl 
              << toJSON(m_config) << std::endl; 
	
	
    // haptic device model (see file "CGenericHapticDevice.h")
    m_specifications.m_model                         = C_HAPTIC_DEVICE_CUSTOM;

    // name of the device manufacturer, research lab, university.
    m_specifications.m_manufacturerName              = "Team 22";

    // name of your device
    m_specifications.m_modelName                     = "Robotic Teaching Adjunct";


    //--------------------------------------------------------------------------
    // CHARACTERISTICS: (The following values must be positive or equal to zero)
    //--------------------------------------------------------------------------

    // the maximum force [N] the device can produce along the x,y,z axis.
    m_specifications.m_maxLinearForce                = m_config.max_linear_force;     // [N]

    // the maximum amount of torque your device can provide arround its
    // rotation degrees of freedom.
    m_specifications.m_maxAngularTorque              = m_config.max_linear_force * m_config.length_b;     // [N*m]


    // the maximum amount of torque which can be provided by your gripper
    m_specifications.m_maxGripperForce                = 3.0;     // [N]

    // the maximum closed loop linear stiffness in [N/m] along the x,y,z axis
    m_specifications.m_maxLinearStiffness             = m_config.max_linear_stiffness; // [N/m]

    // the maximum amount of angular stiffness
    m_specifications.m_maxAngularStiffness            = m_config.max_rotational_stiffness;    // [N*m/Rad]

    // the maximum amount of stiffness supported by the gripper
    m_specifications.m_maxGripperLinearStiffness      = 1000;   // [N*m]

    // the radius of the physical workspace of the device (x,y,z axis)
    m_specifications.m_workspaceRadius                = m_config.workspace_radius;     // [m]

    // the maximum opening angle of the gripper
    m_specifications.m_gripperMaxAngleRad             = cDegToRad(30.0);


    ////////////////////////////////////////////////////////////////////////////
    /*
        DAMPING PROPERTIES:

        Start with small values as damping terms can be high;y sensitive to 
        the quality of your velocity signal and the spatial resolution of your
        device. Try gradually increasing the values by using example "01-devices" 
        and by enabling viscosity with key command "2".
    */
    ////////////////////////////////////////////////////////////////////////////
    
    // Maximum recommended linear damping factor Kv
    m_specifications.m_maxLinearDamping             = m_config.max_linear_damping;   // [N/(m/s)]

    //! Maximum recommended angular damping factor Kv (if actuated torques are available)
    m_specifications.m_maxAngularDamping            = m_config.max_rotational_damping;    // [N*m/(Rad/s)]

    //! Maximum recommended angular damping factor Kv for the force gripper. (if actuated gripper is available)
    m_specifications.m_maxGripperAngularDamping     = 0.0;    // [N*m/(Rad/s)]


    //--------------------------------------------------------------------------
    // CHARACTERISTICS: (The following are of boolean type: (true or false)
    //--------------------------------------------------------------------------

    // does your device provide sensed position (x,y,z axis)?
    m_specifications.m_sensedPosition                = true;

    // does your device provide sensed rotations (i.e stylus)?
    m_specifications.m_sensedRotation                = true;

    // does your device provide a gripper which can be sensed?
    m_specifications.m_sensedGripper                 = false;

    // is you device actuated on the translation degrees of freedom?
    m_specifications.m_actuatedPosition              = true;

    // is your device actuated on the rotation degrees of freedom?
    m_specifications.m_actuatedRotation              = true;

    // is the gripper of your device actuated?
    m_specifications.m_actuatedGripper               = false;

    // can the device be used with the left hand?
    m_specifications.m_leftHand                      = true;

    // can the device be used with the right hand?
    m_specifications.m_rightHand                     = true;


    ////////////////////////////////////////////////////////////////////////////
    /*
        STEP 2:

        Here, you shall  implement code which tells the application if your
        device is actually connected to your computer and can be accessed.
        In practice this may be consist in checking if your I/O board
        is active or if your drivers are available.

        If your device can be accessed, set:
        m_systemAvailable = true;

        Otherwise set:
        m_systemAvailable = false;

        Your actual code may look like:

        bool result = checkIfMyDeviceIsAvailable()
        m_systemAvailable = result;

        If want to support multiple devices, using the method argument
        a_deviceNumber to know which device to setup
    */  
    ////////////////////////////////////////////////////////////////////////////
       
	m_deviceAvailable = false; // this value should become 'true' when the device is available.
}


//==============================================================================
/*!
    Destructor of cTeachingDevice.
*/
//==============================================================================
cTeachingDevice::~cTeachingDevice()
{
    // close connection to device
    if (m_deviceReady)
    {
        close();
    }
}

//==============================================================================
/*!
    This method pairs two CWoodenDevice objects to the Teaching Device

    \return __true__ if the operation succeeds, __false__ otherwise.
*/
//==============================================================================
bool cTeachingDevice::pair( cWoodenDevicePtr deviceA, cWoodenDevicePtr deviceB)
{
	if (!deviceA || !deviceB) return (C_ERROR);
	
	armA = deviceA;
	armB = deviceB;
	
    m_deviceAvailable = true; // this value should become 'true' when the device is available.
	return C_SUCCESS;
}

//==============================================================================
/*!
    This method opens a connection to your device.

    \return __true__ if the operation succeeds, __false__ otherwise.
*/
//==============================================================================
bool cTeachingDevice::open()
{
    // check if the system is available
    if (!m_deviceAvailable) return (C_ERROR);

    // if system is already opened then return
    if (m_deviceReady) return (C_ERROR);
	
	if (!armA->isDeviceReady()) armA->open();
	if (!armB->isDeviceReady()) armB->open();

    ////////////////////////////////////////////////////////////////////////////
    /*
        STEP 3:

        Here you shall implement to open a connection to your
        device. This may include opening a connection to an interface board
        for instance or a USB port.

        If the connection succeeds, set the variable 'result' to true.
        otherwise, set the variable 'result' to false.

        Verify that your device is calibrated. If your device 
        needs calibration then call method calibrate() for wich you will 
        provide code in STEP 5 further below.
    */
    ////////////////////////////////////////////////////////////////////////////

    bool result = C_ERROR; // this value will need to become "C_SUCCESS" for the device to be marked as ready.

    // *** INSERT YOUR CODE HERE ***
    // result = openConnectionToMyDevice();

	result = C_SUCCESS;

    // update device status
    if (result)
    {
        m_deviceReady = true;
        return (C_SUCCESS);
    }
    else
    {
        m_deviceReady = false;
        return (C_ERROR);
    }
}


//==============================================================================
/*!
    This method closes the connection to your device.

    \return __true__ if the operation succeeds, __false__ otherwise.
*/
//==============================================================================
bool cTeachingDevice::close()
{
    // check if the system has been opened previously
    if (!m_deviceReady) return (C_ERROR);

    ////////////////////////////////////////////////////////////////////////////
    /*
        STEP 4:

        Here you shall implement code that closes the connection to your
        device.

        If the operation fails, simply set the variable 'result' to C_ERROR   .
        If the connection succeeds, set the variable 'result' to C_SUCCESS.
    */
    ////////////////////////////////////////////////////////////////////////////

    bool result = C_SUCCESS; // if the operation fails, set value to C_ERROR.
	
	if (armA->isDeviceReady()) result = result && armA->close();
	if (armB->isDeviceReady()) result = result && armB->close();

    // *** INSERT YOUR CODE HERE ***
    // result = closeConnectionToMyDevice()

    // update status
    m_deviceReady = false;

    return (result);
}


//==============================================================================
/*!
    This method calibrates your device.

    \return __true__ if the operation succeeds, __false__ otherwise.
*/
//==============================================================================
bool cTeachingDevice::calibrate(bool a_forceCalibration)
{
    // check if the device is read. See step 3.
    if (!m_deviceReady) return (C_ERROR);

    ////////////////////////////////////////////////////////////////////////////
    /*
        STEP 5:
        
        Here you shall implement code that handles a calibration procedure of the 
        device. In practice this may include initializing the registers of the
        encoder counters for instance. 

        If the device is already calibrated and  a_forceCalibration == false,
        the method may immediately return without further action.
        If a_forceCalibration == true, then the calibrartion procedure
        shall be executed even if the device has already been calibrated.
 
        If the calibration procedure succeeds, the method returns C_SUCCESS,
        otherwise return C_ERROR.
    */
    ////////////////////////////////////////////////////////////////////////////

    bool result = C_SUCCESS;

    // *** INSERT YOUR CODE HERE ***

    // error = calibrateMyDevice()

    return (result);
}


//==============================================================================
/*!
    This method returns the number of devices available from this class of device.

    \return __true__ if the operation succeeds, __false__ otherwise.
*/
//==============================================================================
unsigned int cTeachingDevice::getNumDevices()
{
    ////////////////////////////////////////////////////////////////////////////
    /*
        STEP 6:

        Here you shall implement code that returns the number of available
        haptic devices of type "cTeachingDevice" which are currently connected
        to your computer.

        In practice you will often have either 0 or 1 device. In which case
        the code here below is already implemented for you.

        If you have support more than 1 devices connected at the same time,
        then simply modify the code accordingly so that "numberOfDevices" takes
        the correct value.
    */
    ////////////////////////////////////////////////////////////////////////////

    // *** INSERT YOUR CODE HERE, MODIFY CODE below ACCORDINGLY ***

    int numberOfDevices = 1;  // At least set to 1 if a device is available.

    // numberOfDevices = getNumberOfDevicesConnectedToTheComputer();

    return (numberOfDevices);
}

//==============================================================================
/*!
    This method returns the pitch, yaw, and roll of the device. 
	Units are radians [rad].

    \param   a_pyr  Return value.

    \return __true__ if the operation succeeds, __false__ otherwise.
*/
//==============================================================================
bool cTeachingDevice::getPitchYawRoll(cVector3d& a_pyr)
{
    cVector3d posA, posB;
	double dX, dY, dZ, pitch, yaw, roll;
	armA->getPosition(posA, false); armB->getPosition(posB, false);
	
	dX = posB.x() - posA.x(); dY = posB.y() - posA.y(); dZ = posB.z() - posA.z();
	pitch = -atan2(dZ,dX); yaw = atan2(dY, sqrt(dX*dX + dZ*dZ) ); roll = 0;
	//! set roll
	a_pyr.set(pitch, yaw, roll);
	
	return C_SUCCESS;
}

//==============================================================================
/*!
    This method returns the roll, pitch, and yaw of the device. 
	Units are radians [rad].

    \param   a_pyr  Return value.

    \return __true__ if the operation succeeds, __false__ otherwise.
*/
//==============================================================================
bool cTeachingDevice::getRollPitchYaw(cVector3d& a_rpy)
{
    cVector3d posA, posB;
	double dX, dY, dZ, pitch, yaw, roll;
	armA->getPosition(posA, false); armB->getPosition(posB, false);
	
	dX = posB.x() - posA.x(); dY = posB.y() - posA.y(); dZ = posB.z() - posA.z();
	pitch = -atan2(dZ,dX); yaw = atan2(dY, sqrt(dX*dX + dZ*dZ) ); roll = 0;
	//! set roll
	a_rpy.set(roll, pitch, yaw);
	
	return C_SUCCESS;
}

//==============================================================================
/*!
    This method returns the position of your device. Units are meters [m].

    \param   a_position  Return value.

    \return __true__ if the operation succeeds, __false__ otherwise.
*/
//==============================================================================
bool cTeachingDevice::getPosition(cVector3d& a_position)
{
    // check if the device is read. See step 3.
    if (!m_deviceReady) return (C_ERROR);

    ////////////////////////////////////////////////////////////////////////////
    /*
        STEP 7:

        Here you shall implement code that reads the position (X,Y,Z) from
        your haptic device. Read the values from your device and modify
        the local variable (x,y,z) accordingly.
        If the operation fails return an C_ERROR, C_SUCCESS otherwise

        Note:
        For consistency, units must be in meters.
        If your device is located in front of you, the x-axis is pointing
        towards you (the operator). The y-axis points towards your right
        hand side and the z-axis points up towards the sky. 
    */
    ////////////////////////////////////////////////////////////////////////////

    bool result = C_SUCCESS;
    double Lx,Lz;
	cMatrix3d rotation; cVector3d posA; cVector3d posB;

    // *** INSERT YOUR CODE HERE, MODIFY CODE below ACCORDINGLY ***
	Lx = m_config.length - m_config.height_blade_drop - m_config.length_b/2;
	Lz = m_config.length_blade;
	
	armA->getPosition(posA, autoUpdate);
	armB->getPosition(posB, autoUpdate);
	
	getRotation(rotation);
	a_position = rotation * cVector3d(Lx,0,Lz) + (posA + posB)/2;

    // estimate linear velocity
    estimateLinearVelocity(a_position);

    // exit
    return (result);
}


//==============================================================================
/*!
    This method returns the orientation frame of your device end-effector

    \param   a_rotation  Return value.

    \return __true__ if the operation succeeds, __false__ otherwise.
*/
//==============================================================================
bool cTeachingDevice::getRotation(cMatrix3d& a_rotation)
{
    // check if the device is read. See step 3.
    if (!m_deviceReady) return (C_ERROR);

    ////////////////////////////////////////////////////////////////////////////
    /*
        STEP 8:

        Here you shall implement code which reads the orientation frame from
        your haptic device. The orientation frame is expressed by a 3x3
        rotation matrix. The 1st column of this matrix corresponds to the
        x-axis, the 2nd column to the y-axis and the 3rd column to the z-axis.
        The length of each column vector should be of length 1 and vectors need
        to be orthogonal to each other.

        Note:
        If your device is located in front of you, the x-axis is pointing
        towards you (the operator). The y-axis points towards your right
        hand side and the z-axis points up towards the sky.

        If your device has a stylus, make sure that you set the reference frame
        so that the x-axis corresponds to the axis of the stylus.
    */
    ////////////////////////////////////////////////////////////////////////////

    bool result = C_SUCCESS;

    // variables that describe the rotation matrix
    cMatrix3d frame; cVector3d pyr;
	getPitchYawRoll(pyr);
    frame.identity();

    // *** INSERT YOUR CODE HERE, MODIFY CODE below ACCORDINGLY ***

    frame.setIntrinsicEulerRotationRad(pyr.x(), pyr.y(), pyr.z(), C_EULER_ORDER_YZX );

    // store new rotation matrix
    a_rotation = frame;

    // estimate angular velocity
    estimateAngularVelocity(a_rotation);

    // exit
    return (result);
}


//==============================================================================
/*!
    This method returns the gripper angle in radian.

    \param   a_angle  Return value.

    \return __true__ if the operation succeeds, __false__ otherwise.
*/
//==============================================================================
bool cTeachingDevice::getGripperAngleRad(double& a_angle)
{
    // check if the device is read. See step 3.
    if (!m_deviceReady) return (C_ERROR);

    ////////////////////////////////////////////////////////////////////////////
    /*
        STEP 9:
        Here you may implement code which reads the position angle of your
        gripper. The result must be returned in radian.

        If the operation fails return an error code such as C_ERROR for instance.
    */
    ////////////////////////////////////////////////////////////////////////////

    bool result = C_SUCCESS;

    // *** INSERT YOUR CODE HERE, MODIFY CODE below ACCORDINGLY ***

    // return gripper angle in radian
    a_angle = 0.0;  // a_angle = getGripperAngleInRadianFromMyDevice();

    // estimate gripper velocity
    estimateGripperVelocity(a_angle);

    // exit
    return (result);
}


//==============================================================================
/*!
    This method sends a force [N] and a torque [N*m] and gripper torque [N*m] 
    to your haptic device.

    \param   a_force  Force command.
    \param   a_torque  Torque command.
    \param   a_gripperForce  Gripper force command.

    \return __true__ if the operation succeeds, __false__ otherwise.
*/
//==============================================================================
bool cTeachingDevice::setForceAndTorqueAndGripperForce(const cVector3d& a_force,
                                                       const cVector3d& a_torque,
                                                       const double a_gripperForce)
{
    // check if the device is read. See step 3.
    if (!m_deviceReady) return (C_ERROR);

    ////////////////////////////////////////////////////////////////////////////
    /*
        STEP 10:
        
        Here you may implement code which sends a force (fx,fy,fz),
        torque (tx, ty, tz) and/or gripper force (gf) command to your haptic device.

        If your device does not support one of more of the force, torque and 
        gripper force capabilities, you can simply ignore them. 

        Note:
        For consistency, units must be in Newtons and Newton-meters
        If your device is placed in front of you, the x-axis is pointing
        towards you (the operator). The y-axis points towards your right
        hand side and the z-axis points up towards the sky.

        For instance: if the force = (1,0,0), the device should move towards
        the operator, if the force = (0,0,1), the device should move upwards.
        A torque (1,0,0) would rotate the handle counter clock-wise around the 
        x-axis.
    */
    ////////////////////////////////////////////////////////////////////////////

    bool result = C_SUCCESS;

    // store new force value.
    m_prevForce = a_force;
    m_prevTorque = a_torque;
    m_prevGripperForce = a_gripperForce;
	
    // retrieve force, torque, and gripper force components in individual variables
    cVector3d F;
    F = a_force;

    double Tx = a_torque(0);
    double Ty = a_torque(1);
    double Tz = a_torque(2);
	
    double Fxa,Fya,Fza,Fxb,Fyb,Fzb;
    cVector3d F1,F2;

    // *** INSERT YOUR CODE HERE ***

	double pitch,yaw,roll,Mx,Lb,torqueY,torqueZ;
	cVector3d pyr;
	cMatrix3d frame;
	cMatrix3d transpose;
	
	// set terms
	getPitchYawRoll(pyr);
	pitch = pyr.x(); yaw = pyr.y(); roll = pyr.z();
	Mx = m_config.length_com; Lb = m_config.length_b;
	frame.setIntrinsicEulerRotationRad(pitch, yaw, 0, C_EULER_ORDER_YZX );
	frame.transr(transpose);
	
	// transfer to local coordinates
	F = transpose * F;
	
	// x
	Fxa = F(0)/2; Fxb = Fxa;
	
	// y
	Fya = F(1)/2; Fyb = Fya;
	torqueY = Tz + Fya * Mx + Fyb * (Mx - Lb);
	Fya = Fya - torqueY/Lb;
	Fyb = Fyb + torqueY/Lb;
	
	// z
	Fza = F(2)/2; Fzb = Fza;
	torqueZ = Ty - Fza * Mx - Fzb * (Mx - Lb);
	Fza = Fza + torqueZ/Lb;
	Fzb = Fzb - torqueZ/Lb;
	
	// return to global coordinates
	F1.set(Fxa,Fya,Fza); F2.set(Fxb,Fyb,Fzb);
	F1 = frame * F1; F2 = frame * F2;
	Fxa = F1(0); Fya = F1(1); Fza = F1(2);
	Fxb = F2(0); Fyb = F2(1); Fzb = F2(2);
	
	// reduce excessive forces
	double Fmax = m_config.max_linear_force/2;
	if(Fxa > Fmax) Fmax = Fxa;
	if(Fya > Fmax) Fmax = Fya;
	if(Fza > Fmax) Fmax = Fza;
	if(Fxb > Fmax) Fmax = Fxb;
	if(Fyb > Fmax) Fmax = Fyb;
	if(Fzb > Fmax) Fmax = Fzb;
	
	Fxa = Fxa * (m_config.max_linear_force/(Fmax*2));
	Fya = Fya * (m_config.max_linear_force/(Fmax*2));
	Fza = Fza * (m_config.max_linear_force/(Fmax*2));
	Fxb = Fxb * (m_config.max_linear_force/(Fmax*2));
	Fyb = Fyb * (m_config.max_linear_force/(Fmax*2));
	Fzb = Fzb * (m_config.max_linear_force/(Fmax*2));
	
	// transmit forces
	armA->setForceAndTorqueAndGripperForce(cVector3d(Fxa,Fya,Fza), cVector3d(0,0,0), 0);
	armB->setForceAndTorqueAndGripperForce(cVector3d(Fxb,Fyb,Fzb), cVector3d(0,0,0), 0);
	//! implement roll


    // exit
    return (result);
}


//==============================================================================
/*!
    This method returns status of all user switches 
    [__true__ = __ON__ / __false__ = __OFF__].

    \param  a_userSwitches  Return the 32-bit binary mask of the device buttons.

    \return __true__ if the operation succeeds, __false__ otherwise.
*/
//==============================================================================
bool cTeachingDevice::getUserSwitches(unsigned int& a_userSwitches)
{
    // check if the device is read. See step 3.
    if (!m_deviceReady) return (C_ERROR);

    ////////////////////////////////////////////////////////////////////////////
    /*
        STEP 11:

        Here you shall implement code that reads the status all user switches 
        on your device. For each user switch, set the associated bit on variable
        a_userSwitches. If your device only has one user switch, then set 
        a_userSwitches to 1, when the user switch is engaged, and 0 otherwise.
    */
    ////////////////////////////////////////////////////////////////////////////

    // *** INSERT YOUR CODE HERE ***
    a_userSwitches = 0;

    return (C_SUCCESS);
}

bool cTeachingDevice::enableAutoUpdate()
{
	autoUpdate = true;
}

bool cTeachingDevice::disableAutoUpdate()
{
	autoUpdate = false;
}

//------------------------------------------------------------------------------
}       // namespace chai3d
//------------------------------------------------------------------------------
#endif  // C_ENABLE_TEACHING_DEVICE_SUPPORT
//------------------------------------------------------------------------------
