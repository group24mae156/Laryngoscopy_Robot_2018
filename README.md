# Laryngoscopy_Robot_2018

CHAI3D-WoodenHaptics-Robotic Teaching Adjunct 2017-2018
Software Designers: Michael Berger and Kevin Anderson
------------------------------------------------

Quick installation instructions:
 1. Install Ubuntu 16.04
 2. Install the proprietary nvidia graphics drivers via the "additional driver" tool
 3. sudo apt install git
 4. git clone https://github.com/group24mae156/Laryngoscopy_Robot_2018.git
 5. sudo apt install libhidapi-dev libudev-dev libusb-1.0-0-dev libasound2-dev freeglut3-dev \
    build-essential cmake libudev-dev libxcursor-dev libxrandr-dev libxinerama-dev \
    qt5-default 
 6. Place chai3d in user directory
 7. cd chai3d
 8. cmake .
 9. make -j5
 
Config Files:
1. aluminumhapticarms.json
2. This config will be found inside the chai3d folder.

Microcontroller Code:
1. Find in microcontroller folder
2. Connect each microcontroller to pc
3. Microcontroller info avaliable at https://developer.mbed.org/platforms/mbed-LPC1768/
4. Microcontroller code avaliable at https://developer.mbed.org/users/jofo/code/WoodenHapticsHID/
5. The serial number of armA is '9876543210', armB is '0123456789'

Usage instructions:
1. cd bin
2. cd lin-x86_64
3. sudo ./ver01
4. Enter mode
5. Input data requested
6. Press 1 to activate forces after checking positional allignment
7. See trajectory files README.txt for more detailed instructions

Software Additions
///////////////////////////
Source Files
1. Put .cpp and .h into relevant folders within src
2. run cmake .

Main Files (executable)
1. Create folder in examples/GLFW, name it the same as the main file.
2. Put main file in said folder
3. Add folder name to CMakeLists foreach function (the CMakeLists in the GLFW folder)

File Relations:
Files edited to create own device and code are outlined in apendix of project report, highly suggested reading.
https://docs.google.com/document/d/1_s5preTpMC40lnCXLAwxoUjiM2-6q4_4RU3nG2HKbPw/edit?usp=sharing

Questions?
Email: michaelberger2000@gmail.com; kmanderson003@gmail.com

Original WoodenHaptics API extension software can be found at https://github.com/WoodenHaptics/TEI_2015/
WoodenHaptics software written by Jonas Forsslund, KTH, 2014-12-08


CHAI3D-WoodenHaptics-Robotic Teaching Adjunct 2016-2017
Software Designer: Alex Bertino
------------------------------------------------

Quick installation instructions:
 1. Install Ubuntu 16.04
 2. Install the proprietary nvidia graphics drivers via the "additional driver" tool
 3. sudo apt install git
 4. git clone https://github.com/RoboticTrainingAdjunct/LRTA.git
 5. sudo apt install libhidapi-dev libudev-dev libusb-1.0-0-dev libasound2-dev freeglut3-dev \
    build-essential cmake libudev-dev libxcursor-dev libxrandr-dev libxinerama-dev \
    qt5-default 
 6. cd chai3d
 7. cmake .
 8. make -j5
 
Config Files:
1. woodenhapticsArm0.json
2. woodenhapticsArm1.json
3. teaching_device.json
4. make sure these files are in your home directory before running.

Microcontroller Code:
1. Find in microcontroller folder
2. armA is the arm with the higher attachment point, armB is the lower
3. Connect each microcontroller to pc
4. Drag and drop the .bin file into the micorocontroller
5. Press microcontroller reset button
6. Microcontroller ready!
7. Microcontroller info avaliable at https://developer.mbed.org/platforms/mbed-LPC1768/
8. Microcontroller code avaliable at https://developer.mbed.org/users/jofo/code/WoodenHapticsHID/
9. The serial number of armA is '9876543210', armB is '0123456789'

Usage instructions:
1. cd bin
2. cd lin-x86_64
3. sudo ./RTA-Build
4. enter your trajectory name
5. collect data
6. repeat steps 3-5 until enough runs are taken
7. sudo ./RTA-Track
8. track your trajectory

Software Additions
///////////////////////////
Source Files
1. Put .cpp and .h into relevant folders within src
2. run cmake .
///////////////////////////
Main Files (executable)
1. Create folder in examples/GLFW, name it the same as the main file.
2. Put main file in said folder
3. Add folder name to CMakeLists foreach function (the CMakeLists in the GLFW folder)
///////////////////////////


Questions?
Email alex.bertino@yahoo.com

Original WoodenHaptics API extension software can be found at https://github.com/WoodenHaptics/TEI_2015/
WoodenHaptics software written by Jonas Forsslund, KTH, 2014-12-08










CHAI3D - The Open Source Haptic Framework
-----------------------------------------


Introduction
============


First launched in 2003 at the Robotics and Artificial Intelligence 
Laboratory at Stanford University, CHAI3D is a powerful cross-platform 
C++ simulation framework with over 100+ industries and research 
institutions developing CHAI3D based applications all around the world 
in segments such as automotive, aerospace, medical, entertainment and 
industrial robotics.

Designed as a platform agnostic framework for computer haptics, 
visualization and interactive real-time simulation, CHAI3D is an open 
source framework that supports a variety of commercially-available 
three-, six- and seven-degree-of-freedom haptic devices, and makes it 
simple to support new custom force feedback devices.

CHAI3D's modular capabilities allows for the creation of highly-performing 
native haptic applications as well as for hybrid development where you 
can choose which components provide the best haptic and visual user 
experience.



Learning Resources
==================

* General information is available at the CHAI3D Homepage.

  http://www.chai3d.org


* Community discussion takes place on the CHAI3D Forum.

  http://www.chai3d.org/forum/index


* Commercial support is available from Force Dimension.

  http://www.forcedimension.com


* Doxygen-generated reference documentation is available online.

  http://www.chai3d.org/download/doc/html/wrapper-overview.html


* Instructions on how to get started can be found in the 
  documentation folder.

  doc/getting-started.html



Modules
=======

Support and examples for extension frameworks can be found in the 
dedicated module folders:

  modules/BULLET
  modules/GEL
  modules/OCULUS   (Visual Studio 2015 only)
  modules/ODE
  modules/V-REP 



Haptic Devices and Trackers
===========================

CHAI3D supports a selection of commercial haptic devices and trackers.
Information about their installation and support within CHAI3D can be 
found in the documentation folder.

  doc/html/chapter2-installation.html



Contributing
============

If you have developed a new module or improved any of the capabilities of
CHAI3D and would like to share them with the community, please contact our 
development team at developers@chai3d.org



Reporting Bugs
==============

If you have found a bug:

1. Please join the CHAI3D forum and ask about the expected and observed 
   behaviors to determine if it is really a bug

2. If you have identified a problem and managed to document it, you may
   also submit a report at support@chai3d.org



License
=======

CHAI3D is open source software and is licensed under the Revised BSD 
License (3-clause). The license allows unlimited redistribution for 
any purpose as long as its copyright notices and the license's 
disclaimers of warranty are maintained. The license also contains a 
clause restricting use of the names of contributors for endorsement of 
a derived work without specific permission. 

See copyright.txt for details.



Reference
=========

For scientific publications, please reference CHAI3D:

@INPROCEEDINGS{Conti03,
  author    = {Conti, F. and Barbagli, F. and Balaniuk, R. 
               and Halg, M. and Lu, C. and Morris, D. 
               and Sentis, L. and Warren, J. and Khatib, O. 
               and Salisbury, K.},
  title     = {The CHAI libraries},
  booktitle = {Proceedings of Eurohaptics 2003},
  year      = {2003},
  pages     = {496--500},
  address   = {Dublin, Ireland}
}



________________________
(C) 2003-2016 by CHAI3D
All Rights Reserved.
