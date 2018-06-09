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

    \author    Alex Bertino
    \version   3.2.0 $Rev: 1875 $
*/
//==============================================================================

//------------------------------------------------------------------------------
#ifndef CTeachingDeviceH
#define CTeachingDeviceH
//------------------------------------------------------------------------------
#if defined(C_ENABLE_TEACHING_DEVICE_SUPPORT)
//------------------------------------------------------------------------------
#include "devices/CGenericHapticDevice.h"
#include "hidapi.h"
#include <chrono>
#include <thread>
#include <vector>
#include "devices/CWoodenDevice.h"
#include "math/CTransform.h"
#include <math.h>
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

//==============================================================================
/*!
    \file       CTeachingDevice.h

    \brief
    Implements support for a dual-arm 6 DOF haptic device.
*/
//==============================================================================

//------------------------------------------------------------------------------
class cTeachingDevice;
typedef std::shared_ptr<cTeachingDevice> cTeachingDevicePtr;

/*
struct hid_to_pc_message { // 2*2 = 4 bytes
    short encoder;
    short debug;
};

struct pc_to_hid_message {  // 2*2 = 4 bytes
    short current_motor_mA;
    short debug;
};
*/

//------------------------------------------------------------------------------

//==============================================================================
/*!
    \class      cTeachingDevice
    \ingroup    devices  

    \brief
    This class is a interface to support custom haptic devices (template).

    \details
    This class provides the basics to easily interface CHAI3D to your 
    own custom haptic device. \n\n

    Simply follow the 11 commented step in file CMyCustomDevice.cpp 
    and complete the code accordingly.
    Depending of the numbers of degrees of freedom of your device, not
    all methods need to be implemented. For instance, if your device
    does not provide any rotation degrees of freedom, simply ignore
    the getRotation() method. Default values will be returned correctly
    if these are not implemented on your device. In the case of rotations
    for instance, the identity matrix is returned.\n\n

    You may also rename this class in which case you will also want to
    customize the haptic device handler to automatically detect your device.
    Please consult method update() of the cHapticDeviceHandler class
    that is located in file CHapticDeviceHandler.cpp .
    Simply see how the haptic device handler already looks for
    device of type cMyCustomDevice.\n\n

    If you are encountering any problems with your implementation, check 
    for instance file cDeltaDevices.cpp which implement supports for the 
    Force Dimension series of haptic devices. In order to verify the implementation
    use the 01-device example to get started. Example 11-effects is a great
    demo to verify how basic haptic effects may behave with you haptic
    devices. If you do encounter vibrations or instabilities, try reducing
    the maximum stiffness and/or damping values supported by your device. 
    (see STEP-1 in file CMyCustomDevice.cpp).\n
    
    Make  sure that your device is also communicating fast enough with 
    your computer. Ideally the communication period should take less 
    than 1 millisecond in order to reach a desired update rate of at least 1000Hz.
    Problems can typically occur when using a slow serial port (RS232) for
    instance.\n
*/
//==============================================================================
class cTeachingDevice : public cGenericHapticDevice
{
    //--------------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //--------------------------------------------------------------------------

public:

    //! Constructor of cMyCustomDevice.
    cTeachingDevice(unsigned int a_deviceNumber = 0);

    //! Destructor of cMyCustomDevice.
    virtual ~cTeachingDevice();

    //! Shared cMyCustomDevice allocator.
    static cTeachingDevicePtr create(unsigned int a_deviceNumber = 0) { return (std::make_shared<cTeachingDevice>(a_deviceNumber)); }


    //--------------------------------------------------------------------------
    // PUBLIC METHODS:
    //--------------------------------------------------------------------------

public:

	//! This method pairs the teaching device with two CWoodenDevice objects
	virtual bool pair( cWoodenDevicePtr deviceA, cWoodenDevicePtr deviceB);

    //! This method opens a connection to the haptic device.
    virtual bool open();

    //! This method closes the connection to the haptic device.
    virtual bool close();

    //! This method calibrates the haptic device.
    virtual bool calibrate(bool a_forceCalibration = false);

    //! This method returns the position of the device.
    virtual bool getPosition(cVector3d& a_position);

    //! This method returns the orientation frame of the device end-effector.
    virtual bool getRotation(cMatrix3d& a_rotation);

    //! This method returns the gripper angle in radian [rad].
    virtual bool getGripperAngleRad(double& a_angle);

    //! This method returns the status of all user switches [__true__ = __ON__ / __false__ = __OFF__].
    virtual bool getUserSwitches(unsigned int& a_userSwitches); 

    //! This method sends a force [N] and a torque [N*m] and gripper force [N] to the haptic device.
    virtual bool setForceAndTorqueAndGripperForce(const cVector3d& a_force, const cVector3d& a_torque, double a_gripperForce);
    
    //! This method returns the pitch, yaw, and roll of the device
    virtual bool getPitchYawRoll(cVector3d& a_pyr);
    
    //! This method returns the roll (x), pitch (y), and yaw (z) of the device
    virtual bool getRollPitchYaw(cVector3d& a_rpy);
    
    //! This method enables the device to update the position of connected devices
    virtual bool enableAutoUpdate();
    
    //! This method disables the device to update the position of connected devices
    virtual bool disableAutoUpdate();


    //--------------------------------------------------------------------------
    // PUBLIC STATIC METHODS:
    //--------------------------------------------------------------------------
	
protected:
	
	//--------------------------------------------------------------------------
    // PROTECTED METHODS:
    //--------------------------------------------------------------------------
	
	
	//--------------------------------------------------------------------------
    // PROTECTED STATIC METHODS:
    //--------------------------------------------------------------------------

public: 

    //! This method returns the number of devices available from this class of device.
    static unsigned int getNumDevices();
	
    //! A collection of variables that can be set in ~/teaching_device.json 
    struct configuration {
        double length;                  // m (from point A to Laryngoscope attachment)
        double length_b;                // m (from point A to point B)
        double length_blade;            // m (length of Laryngoscope blade)
        double height_blade_drop;       // m (height drop of Laryngoscope)
        double workspace_radius;        // m (for application information)
        double torque_constant_motor;   // Nm/A
        double current_for_10_v_signal; // A
        double cpr_encoder;             // quadrupled counts per revolution
        double max_linear_force;        // N (Should be double of individual arm force)
        double max_linear_stiffness;    // N/m (Should be the same as individual arm)
        double max_linear_damping;      // N/(m/s) (Should be the same as individual arm)
		double max_rotational_stiffness;// N*m/rad
		double max_rotational_damping;  // N*m/(Rad/s)
		double encoder_serial_number;   // Serial number for motor
		double length_com;              // m (from point A to center of mass)
		double height_com;              // m (from point A to center of mass)
		double mass;                    // kg (weight of device)
		double inertia_x;               // kg m^2 (inertia in x direction)
		double inertia_y;               // kg m^2 (inertia in y direction)
		double inertia_z;               // kg m^2 (inertia in z direction)

        // Set values
        configuration(const double* k):
          length(k[0]), length_b(k[1]), length_blade(k[2]), height_blade_drop(k[3]), 
		  workspace_radius(k[4]), torque_constant_motor(k[5]), current_for_10_v_signal(k[6]), 
		  cpr_encoder(k[7]), max_linear_force(k[8]), max_linear_stiffness(k[9]), 
		  max_linear_damping(k[10]), max_rotational_stiffness(k[11]), max_rotational_damping(k[12]), 
		  encoder_serial_number(k[13]), length_com(k[14]), height_com(k[15]), mass(k[16]),
		  inertia_x(k[17]), inertia_y(k[18]), inertia_z(k[19]) {}

        configuration(){}
    };


    //--------------------------------------------------------------------------
    // PROTECTED MEMBERS:
    //--------------------------------------------------------------------------

    ////////////////////////////////////////////////////////////////////////////
    /*
        INTERNAL VARIABLES:

        If you need to declare any local variables or methods for your device,
        you may do it here below. 
    */
    ////////////////////////////////////////////////////////////////////////////

protected:

	// true if device should update the position of connected devices (rather than the user)
	bool autoUpdate = false;
	
    const configuration m_config;
	cWoodenDevicePtr armA;
	cWoodenDevicePtr armB;
    hid_to_pc_message hid_to_pc;
    pc_to_hid_message pc_to_hid;
    hid_device *handle;
    struct hid_device_info *devs, *cur_dev;
    int res;
    unsigned char buf[5];// 1 extra byte for the report ID
};

//------------------------------------------------------------------------------
}       // namespace chai3d
//------------------------------------------------------------------------------
#endif  // C_ENABLE_TEACHING_DEVICE_SUPPORT
//------------------------------------------------------------------------------
#endif
//------------------------------------------------------------------------------
