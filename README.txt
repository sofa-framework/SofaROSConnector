====================================
Building the SofaROSConnector plugin
====================================
Please make sure that the CMake options:
- BUILD_ZYROSCONNECTOR
- BUILD_SOFAPYTHON

are set to ON.
SofaPython is not strictly necessary for the ROS connector plugin to work.
It is however helpful to control a robot arm model manually (in addition to ROS messages).

=================================
Using the SofaROSConnector plugin
=================================
You can find an example scene in $SOFA_HOME/examples/Demos/SofaROSConnector/ur10_withEGP_noKinematics_zyComponents.scn
This example also includes an example for a COLLADA model (a Universal Robotics UR10) and a Python script for manual control of the robot model's six degrees of freedom.

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

=================================
Current limitations of the plugin
=================================
There currently are two limitations for the connector plugin:
- The types of ROS messages that can be processed is still "hard-coded" in the ZyROSConnectionManager.
- It is currently only possible to subscribe and publish to ROS topics, not ROS services.

Both limitations will be migitated as the next steps in the further development of the plugin.
