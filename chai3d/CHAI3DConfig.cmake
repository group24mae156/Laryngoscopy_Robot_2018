#  Software License Agreement (BSD License)
#  Copyright (c) 2003-2016, CHAI3D.
#  (www.chai3d.org)
#
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions
#  are met:
#
#  * Redistributions of source code must retain the above copyright
#  notice, this list of conditions and the following disclaimer.
#
#  * Redistributions in binary form must reproduce the above
#  copyright notice, this list of conditions and the following
#  disclaimer in the documentation and/or other materials provided
#  with the distribution.
#
#  * Neither the name of CHAI3D nor the names of its contributors may
#  be used to endorse or promote products derived from this software
#  without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  POSSIBILITY OF SUCH DAMAGE.
#
#  $Author: conti $
#  $Date: 2015-12-17 06:26:05 +0100 (Thu, 17 Dec 2015) $
#  $Rev: 1869 $


# options
get_filename_component(CHAI3D_CMAKE_DIR "${CMAKE_CURRENT_LIST_FILE}" PATH)
set(CHAI3D_INCLUDE_DIRS "/home/michael/chai3d/src;/home/michael/chai3d/external/Eigen;/home/michael/chai3d/external/glew/include;/usr/include")
set(CHAI3D_LIBRARIES "chai3d;/usr/lib/x86_64-linux-gnu/libGLU.so;/usr/lib/x86_64-linux-gnu/libGL.so;drd;usb-1.0;rt;pthread;dl;udev;826_64;remotehaptics;Qt5Core;Qt5Network")
set(CHAI3D_LIBRARY_DIRS "/home/michael/chai3d/external/DHD/lib/lin-x86_64;/home/michael/chai3d/external/s826/lib;/home/michael/chai3d/external/libhid;/home/michael/chai3d/external/libremotehaptics")
set(CHAI3D_DEFINITIONS )
set(CHAI3D_SOURCE_DIR /home/michael/chai3d)

# library dependencies (contains definitions for IMPORTED targets)
if(NOT TARGET chai3d AND NOT CHAI3D_BINARY_DIR)
  include("${CHAI3D_CMAKE_DIR}/CHAI3DTargets.cmake")
endif()
