====================================
Building the SofaROSConnector plugin
====================================

The top-level directory of your SOFA source tree is referred to as $SOFA_ROOT in the following instructions.

The least intrusive alternative to add the SofaROSConnector plugin to a SOFA build is to clone this repository as a git submodule into your SOFA source tree.
The ROS connector plugin also depends the SoftRobots and STLIB plugins at the time of writing. These can also be added as git submodules.

To do so, run these commands from your $SOFA_ROOT directory:

git submodule add https://github.com/SofaDefrost/STLIB.git applications/plugins/STLIB
git submodule add https://github.com/faichele/SoftRobots.git applications/plugins/SoftRobots
git submodule add https://github.com/faichele/SofaROSConnector.git applications/plugins/SofaROSConnector

After git has finished cloning the repositories, please add the following lines at the end of $SOFA_ROOT/applications/plugins/CMakeLists.txt:

set(SOFA_BUILD_METIS ON CACHE BOOL "Compile SOFA with Metis support by default as per dependency for the SoftRobots plugin." FORCE)
set(PLUGIN_SOFAPYTHON ON CACHE BOOL "Compile the SofaPython plugin by default as per dependency for the SoftRobots plugin." FORCE)

sofa_add_plugin(STLIB STLIB ON)
sofa_add_plugin(SoftRobots SoftRobots ON)
sofa_add_plugin(SofaROSConnector ZyROSConnector ON)

Please make sure that the following CMake options are set to ON: 
- SOFA_BUILD_METIS
- PLUGIN_STLIB
- PLUGIN_SOFTROBOTS
- PLUGIN_ZYROSCONNECTOR 

The lines you added to $SOFA_ROOT/applications/plugins/CMakeLists.txt should already have ensured that, but please double-check in case you should run into problems during the compilation of SOFA.

=================================
Using the SofaROSConnector plugin
=================================
The SofaROSConnector plugin currently has three different examples that illustrate how to use it from a SOFA XML scene description, via the SofaPython bindings, and from your own code directly.

- An example scene that shows how to add the required components to a SOFA XML scene description is located in: 
  $SOFA_ROOT/applications/plugins/SofaROSConnector/examples/Demos/UR10/ur10_withEGP_noKinematics_zyComponents.scn
  This example also includes a COLLADA model (a Universal Robotics UR10) and a Python script for manual control of the robot model's six degrees of freedom.
- Instantiating the SofaROSConnector components via SofaPython is demonstrated in: 
  $SOFA_ROOT/applications/plugins/SoftRobots/docs/tutorials/CableGripper/details/step7_rosconnector.pyscn
- Using ROS topics and services (both as client and server) from C++ code is demonstrated in:
  $SOFA_ROOT/applications/plugins/SofaROSConnector/ZyROSConnectionManager/ZyROSConnector_test/ZyROSConnectorTest.cpp
  
Please make sure that you have set the ROS_MASTER_URI and the ROS_IP variables in your enviroment before you start SOFA.
The ROS_MASTER_URI is the URL of the ROS core process the connector plugin will connect to, in the format: "http://<IP or hostname>:11311"
The ROS_IP is the IP address of the ROS client, i. e. the machine SOFA runs on, in usual IPv4 address format: www.xxx.yyy.zzz

Scene setup: For loading and controlling a robot arm model in SOFA, use the following plugins in your scene:
<RequiredPlugin name="ZyROSConnectionManager"/>
<RequiredPlugin name="ZyROSKinematics"/>
<RequiredPlugin name="ZyColladaLoader"/>

The ZyROSConnectionManager plugin is responsible for ROS message processing.
The ZyROSKinematics plugin controls the motion of a robot model according to ROS joint_state messages.
The ZyColladaLoader plugin is a modified version of the original ColladaLoader plugin.
It adds support for position-based control of kinematics with rotational degrees of freedom.
