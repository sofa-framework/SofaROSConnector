/***********************************************************************
ROS message definition headers and ROS connector template instantiations.
This file is AUTO-GENERATED during the CMake run.
Please do not modify it by hand.
The contents will be overwritten and re-generated.
************************************************************************/



#include "ZyROSConnectorTopicSubscriber.inl"

using namespace Zyklio::ROSConnector;

SOFA_DECL_CLASS(ZyROSConnectorTopicSubscriberIface)

ZyROSListener::ZyROSListener()
    : m_uuid(boost::uuids::random_generator()())
    , messageType("")
{

}

ZyROSListener::ZyROSListener(const ZyROSListener& other)
{
        if (this != &other)
        {
                m_uuid = other.m_uuid;
                m_rosTopic = other.m_rosTopic;
        }
}

ZyROSListener& ZyROSListener::operator=(const ZyROSListener& other)
{
        if (this != &other)
        {
                m_uuid = other.m_uuid;
                m_rosTopic = other.m_rosTopic;
        }
        return *this;
}

void ZyROSConnectorTopicSubscriberIface::onMessageReceived()
{
        m_sig();
}
        

#include <ZyROS_MessageType_Instantiations_Subscribers.h>


using namespace Zyklio::ROSConnector;
// Publisher and subscriber proxy class instantiation for ROS message type: actionlib/TestAction
template class ZyROSConnectorTopicSubscriber<actionlib::TestAction>;

// Publisher and subscriber proxy class instantiation for ROS message type: actionlib/TestActionFeedback
template class ZyROSConnectorTopicSubscriber<actionlib::TestActionFeedback>;

// Publisher and subscriber proxy class instantiation for ROS message type: actionlib/TestActionGoal
template class ZyROSConnectorTopicSubscriber<actionlib::TestActionGoal>;

// Publisher and subscriber proxy class instantiation for ROS message type: actionlib/TestActionResult
template class ZyROSConnectorTopicSubscriber<actionlib::TestActionResult>;

// Publisher and subscriber proxy class instantiation for ROS message type: actionlib/TestFeedback
template class ZyROSConnectorTopicSubscriber<actionlib::TestFeedback>;

// Publisher and subscriber proxy class instantiation for ROS message type: actionlib/TestGoal
template class ZyROSConnectorTopicSubscriber<actionlib::TestGoal>;

// Publisher and subscriber proxy class instantiation for ROS message type: actionlib/TestRequestAction
template class ZyROSConnectorTopicSubscriber<actionlib::TestRequestAction>;

// Publisher and subscriber proxy class instantiation for ROS message type: actionlib/TestRequestActionFeedback
template class ZyROSConnectorTopicSubscriber<actionlib::TestRequestActionFeedback>;

// Publisher and subscriber proxy class instantiation for ROS message type: actionlib/TestRequestActionGoal
template class ZyROSConnectorTopicSubscriber<actionlib::TestRequestActionGoal>;

// Publisher and subscriber proxy class instantiation for ROS message type: actionlib/TestRequestActionResult
template class ZyROSConnectorTopicSubscriber<actionlib::TestRequestActionResult>;

// Publisher and subscriber proxy class instantiation for ROS message type: actionlib/TestRequestFeedback
template class ZyROSConnectorTopicSubscriber<actionlib::TestRequestFeedback>;

// Publisher and subscriber proxy class instantiation for ROS message type: actionlib/TestRequestGoal
template class ZyROSConnectorTopicSubscriber<actionlib::TestRequestGoal>;

// Publisher and subscriber proxy class instantiation for ROS message type: actionlib/TestRequestResult
template class ZyROSConnectorTopicSubscriber<actionlib::TestRequestResult>;

// Publisher and subscriber proxy class instantiation for ROS message type: actionlib/TestResult
template class ZyROSConnectorTopicSubscriber<actionlib::TestResult>;

// Publisher and subscriber proxy class instantiation for ROS message type: actionlib/TwoIntsAction
template class ZyROSConnectorTopicSubscriber<actionlib::TwoIntsAction>;

// Publisher and subscriber proxy class instantiation for ROS message type: actionlib/TwoIntsActionFeedback
template class ZyROSConnectorTopicSubscriber<actionlib::TwoIntsActionFeedback>;

// Publisher and subscriber proxy class instantiation for ROS message type: actionlib/TwoIntsActionGoal
template class ZyROSConnectorTopicSubscriber<actionlib::TwoIntsActionGoal>;

// Publisher and subscriber proxy class instantiation for ROS message type: actionlib/TwoIntsActionResult
template class ZyROSConnectorTopicSubscriber<actionlib::TwoIntsActionResult>;

// Publisher and subscriber proxy class instantiation for ROS message type: actionlib/TwoIntsFeedback
template class ZyROSConnectorTopicSubscriber<actionlib::TwoIntsFeedback>;

// Publisher and subscriber proxy class instantiation for ROS message type: actionlib/TwoIntsGoal
template class ZyROSConnectorTopicSubscriber<actionlib::TwoIntsGoal>;

// Publisher and subscriber proxy class instantiation for ROS message type: actionlib/TwoIntsResult
template class ZyROSConnectorTopicSubscriber<actionlib::TwoIntsResult>;

// Publisher and subscriber proxy class instantiation for ROS message type: actionlib_msgs/GoalID
template class ZyROSConnectorTopicSubscriber<actionlib_msgs::GoalID>;

// Publisher and subscriber proxy class instantiation for ROS message type: actionlib_msgs/GoalStatus
template class ZyROSConnectorTopicSubscriber<actionlib_msgs::GoalStatus>;

// Publisher and subscriber proxy class instantiation for ROS message type: actionlib_msgs/GoalStatusArray
template class ZyROSConnectorTopicSubscriber<actionlib_msgs::GoalStatusArray>;

// Publisher and subscriber proxy class instantiation for ROS message type: bond/Constants
template class ZyROSConnectorTopicSubscriber<bond::Constants>;

// Publisher and subscriber proxy class instantiation for ROS message type: bond/Status
template class ZyROSConnectorTopicSubscriber<bond::Status>;

// Publisher and subscriber proxy class instantiation for ROS message type: control_msgs/FollowJointTrajectoryAction
template class ZyROSConnectorTopicSubscriber<control_msgs::FollowJointTrajectoryAction>;

// Publisher and subscriber proxy class instantiation for ROS message type: control_msgs/FollowJointTrajectoryActionFeedback
template class ZyROSConnectorTopicSubscriber<control_msgs::FollowJointTrajectoryActionFeedback>;

// Publisher and subscriber proxy class instantiation for ROS message type: control_msgs/FollowJointTrajectoryActionGoal
template class ZyROSConnectorTopicSubscriber<control_msgs::FollowJointTrajectoryActionGoal>;

// Publisher and subscriber proxy class instantiation for ROS message type: control_msgs/FollowJointTrajectoryActionResult
template class ZyROSConnectorTopicSubscriber<control_msgs::FollowJointTrajectoryActionResult>;

// Publisher and subscriber proxy class instantiation for ROS message type: control_msgs/FollowJointTrajectoryFeedback
template class ZyROSConnectorTopicSubscriber<control_msgs::FollowJointTrajectoryFeedback>;

// Publisher and subscriber proxy class instantiation for ROS message type: control_msgs/FollowJointTrajectoryGoal
template class ZyROSConnectorTopicSubscriber<control_msgs::FollowJointTrajectoryGoal>;

// Publisher and subscriber proxy class instantiation for ROS message type: control_msgs/FollowJointTrajectoryResult
template class ZyROSConnectorTopicSubscriber<control_msgs::FollowJointTrajectoryResult>;

// Publisher and subscriber proxy class instantiation for ROS message type: control_msgs/GripperCommand
template class ZyROSConnectorTopicSubscriber<control_msgs::GripperCommand>;

// Publisher and subscriber proxy class instantiation for ROS message type: control_msgs/GripperCommandAction
template class ZyROSConnectorTopicSubscriber<control_msgs::GripperCommandAction>;

// Publisher and subscriber proxy class instantiation for ROS message type: control_msgs/GripperCommandActionFeedback
template class ZyROSConnectorTopicSubscriber<control_msgs::GripperCommandActionFeedback>;

// Publisher and subscriber proxy class instantiation for ROS message type: control_msgs/GripperCommandActionGoal
template class ZyROSConnectorTopicSubscriber<control_msgs::GripperCommandActionGoal>;

// Publisher and subscriber proxy class instantiation for ROS message type: control_msgs/GripperCommandActionResult
template class ZyROSConnectorTopicSubscriber<control_msgs::GripperCommandActionResult>;

// Publisher and subscriber proxy class instantiation for ROS message type: control_msgs/GripperCommandFeedback
template class ZyROSConnectorTopicSubscriber<control_msgs::GripperCommandFeedback>;

// Publisher and subscriber proxy class instantiation for ROS message type: control_msgs/GripperCommandGoal
template class ZyROSConnectorTopicSubscriber<control_msgs::GripperCommandGoal>;

// Publisher and subscriber proxy class instantiation for ROS message type: control_msgs/GripperCommandResult
template class ZyROSConnectorTopicSubscriber<control_msgs::GripperCommandResult>;

// Publisher and subscriber proxy class instantiation for ROS message type: control_msgs/JointControllerState
template class ZyROSConnectorTopicSubscriber<control_msgs::JointControllerState>;

// Publisher and subscriber proxy class instantiation for ROS message type: control_msgs/JointJog
template class ZyROSConnectorTopicSubscriber<control_msgs::JointJog>;

// Publisher and subscriber proxy class instantiation for ROS message type: control_msgs/JointTolerance
template class ZyROSConnectorTopicSubscriber<control_msgs::JointTolerance>;

// Publisher and subscriber proxy class instantiation for ROS message type: control_msgs/JointTrajectoryAction
template class ZyROSConnectorTopicSubscriber<control_msgs::JointTrajectoryAction>;

// Publisher and subscriber proxy class instantiation for ROS message type: control_msgs/JointTrajectoryActionFeedback
template class ZyROSConnectorTopicSubscriber<control_msgs::JointTrajectoryActionFeedback>;

// Publisher and subscriber proxy class instantiation for ROS message type: control_msgs/JointTrajectoryActionGoal
template class ZyROSConnectorTopicSubscriber<control_msgs::JointTrajectoryActionGoal>;

// Publisher and subscriber proxy class instantiation for ROS message type: control_msgs/JointTrajectoryActionResult
template class ZyROSConnectorTopicSubscriber<control_msgs::JointTrajectoryActionResult>;

// Publisher and subscriber proxy class instantiation for ROS message type: control_msgs/JointTrajectoryControllerState
template class ZyROSConnectorTopicSubscriber<control_msgs::JointTrajectoryControllerState>;

// Publisher and subscriber proxy class instantiation for ROS message type: control_msgs/JointTrajectoryFeedback
template class ZyROSConnectorTopicSubscriber<control_msgs::JointTrajectoryFeedback>;

// Publisher and subscriber proxy class instantiation for ROS message type: control_msgs/JointTrajectoryGoal
template class ZyROSConnectorTopicSubscriber<control_msgs::JointTrajectoryGoal>;

// Publisher and subscriber proxy class instantiation for ROS message type: control_msgs/JointTrajectoryResult
template class ZyROSConnectorTopicSubscriber<control_msgs::JointTrajectoryResult>;

// Publisher and subscriber proxy class instantiation for ROS message type: control_msgs/PidState
template class ZyROSConnectorTopicSubscriber<control_msgs::PidState>;

// Publisher and subscriber proxy class instantiation for ROS message type: control_msgs/PointHeadAction
template class ZyROSConnectorTopicSubscriber<control_msgs::PointHeadAction>;

// Publisher and subscriber proxy class instantiation for ROS message type: control_msgs/PointHeadActionFeedback
template class ZyROSConnectorTopicSubscriber<control_msgs::PointHeadActionFeedback>;

// Publisher and subscriber proxy class instantiation for ROS message type: control_msgs/PointHeadActionGoal
template class ZyROSConnectorTopicSubscriber<control_msgs::PointHeadActionGoal>;

// Publisher and subscriber proxy class instantiation for ROS message type: control_msgs/PointHeadActionResult
template class ZyROSConnectorTopicSubscriber<control_msgs::PointHeadActionResult>;

// Publisher and subscriber proxy class instantiation for ROS message type: control_msgs/PointHeadFeedback
template class ZyROSConnectorTopicSubscriber<control_msgs::PointHeadFeedback>;

// Publisher and subscriber proxy class instantiation for ROS message type: control_msgs/PointHeadGoal
template class ZyROSConnectorTopicSubscriber<control_msgs::PointHeadGoal>;

// Publisher and subscriber proxy class instantiation for ROS message type: control_msgs/PointHeadResult
template class ZyROSConnectorTopicSubscriber<control_msgs::PointHeadResult>;

// Publisher and subscriber proxy class instantiation for ROS message type: control_msgs/SingleJointPositionAction
template class ZyROSConnectorTopicSubscriber<control_msgs::SingleJointPositionAction>;

// Publisher and subscriber proxy class instantiation for ROS message type: control_msgs/SingleJointPositionActionFeedback
template class ZyROSConnectorTopicSubscriber<control_msgs::SingleJointPositionActionFeedback>;

// Publisher and subscriber proxy class instantiation for ROS message type: control_msgs/SingleJointPositionActionGoal
template class ZyROSConnectorTopicSubscriber<control_msgs::SingleJointPositionActionGoal>;

// Publisher and subscriber proxy class instantiation for ROS message type: control_msgs/SingleJointPositionActionResult
template class ZyROSConnectorTopicSubscriber<control_msgs::SingleJointPositionActionResult>;

// Publisher and subscriber proxy class instantiation for ROS message type: control_msgs/SingleJointPositionFeedback
template class ZyROSConnectorTopicSubscriber<control_msgs::SingleJointPositionFeedback>;

// Publisher and subscriber proxy class instantiation for ROS message type: control_msgs/SingleJointPositionGoal
template class ZyROSConnectorTopicSubscriber<control_msgs::SingleJointPositionGoal>;

// Publisher and subscriber proxy class instantiation for ROS message type: control_msgs/SingleJointPositionResult
template class ZyROSConnectorTopicSubscriber<control_msgs::SingleJointPositionResult>;

// Publisher and subscriber proxy class instantiation for ROS message type: diagnostic_msgs/DiagnosticArray
template class ZyROSConnectorTopicSubscriber<diagnostic_msgs::DiagnosticArray>;

// Publisher and subscriber proxy class instantiation for ROS message type: diagnostic_msgs/DiagnosticStatus
template class ZyROSConnectorTopicSubscriber<diagnostic_msgs::DiagnosticStatus>;

// Publisher and subscriber proxy class instantiation for ROS message type: diagnostic_msgs/KeyValue
template class ZyROSConnectorTopicSubscriber<diagnostic_msgs::KeyValue>;

// Publisher and subscriber proxy class instantiation for ROS message type: dynamic_reconfigure/BoolParameter
template class ZyROSConnectorTopicSubscriber<dynamic_reconfigure::BoolParameter>;

// Publisher and subscriber proxy class instantiation for ROS message type: dynamic_reconfigure/Config
template class ZyROSConnectorTopicSubscriber<dynamic_reconfigure::Config>;

// Publisher and subscriber proxy class instantiation for ROS message type: dynamic_reconfigure/ConfigDescription
template class ZyROSConnectorTopicSubscriber<dynamic_reconfigure::ConfigDescription>;

// Publisher and subscriber proxy class instantiation for ROS message type: dynamic_reconfigure/DoubleParameter
template class ZyROSConnectorTopicSubscriber<dynamic_reconfigure::DoubleParameter>;

// Publisher and subscriber proxy class instantiation for ROS message type: dynamic_reconfigure/Group
template class ZyROSConnectorTopicSubscriber<dynamic_reconfigure::Group>;

// Publisher and subscriber proxy class instantiation for ROS message type: dynamic_reconfigure/GroupState
template class ZyROSConnectorTopicSubscriber<dynamic_reconfigure::GroupState>;

// Publisher and subscriber proxy class instantiation for ROS message type: dynamic_reconfigure/IntParameter
template class ZyROSConnectorTopicSubscriber<dynamic_reconfigure::IntParameter>;

// Publisher and subscriber proxy class instantiation for ROS message type: dynamic_reconfigure/ParamDescription
template class ZyROSConnectorTopicSubscriber<dynamic_reconfigure::ParamDescription>;

// Publisher and subscriber proxy class instantiation for ROS message type: dynamic_reconfigure/SensorLevels
template class ZyROSConnectorTopicSubscriber<dynamic_reconfigure::SensorLevels>;

// Publisher and subscriber proxy class instantiation for ROS message type: dynamic_reconfigure/StrParameter
template class ZyROSConnectorTopicSubscriber<dynamic_reconfigure::StrParameter>;

// Publisher and subscriber proxy class instantiation for ROS message type: geometry_msgs/Accel
template class ZyROSConnectorTopicSubscriber<geometry_msgs::Accel>;

// Publisher and subscriber proxy class instantiation for ROS message type: geometry_msgs/AccelStamped
template class ZyROSConnectorTopicSubscriber<geometry_msgs::AccelStamped>;

// Publisher and subscriber proxy class instantiation for ROS message type: geometry_msgs/AccelWithCovariance
template class ZyROSConnectorTopicSubscriber<geometry_msgs::AccelWithCovariance>;

// Publisher and subscriber proxy class instantiation for ROS message type: geometry_msgs/AccelWithCovarianceStamped
template class ZyROSConnectorTopicSubscriber<geometry_msgs::AccelWithCovarianceStamped>;

// Publisher and subscriber proxy class instantiation for ROS message type: geometry_msgs/Inertia
template class ZyROSConnectorTopicSubscriber<geometry_msgs::Inertia>;

// Publisher and subscriber proxy class instantiation for ROS message type: geometry_msgs/InertiaStamped
template class ZyROSConnectorTopicSubscriber<geometry_msgs::InertiaStamped>;

// Publisher and subscriber proxy class instantiation for ROS message type: geometry_msgs/Point
template class ZyROSConnectorTopicSubscriber<geometry_msgs::Point>;

// Publisher and subscriber proxy class instantiation for ROS message type: geometry_msgs/Point32
template class ZyROSConnectorTopicSubscriber<geometry_msgs::Point32>;

// Publisher and subscriber proxy class instantiation for ROS message type: geometry_msgs/PointStamped
template class ZyROSConnectorTopicSubscriber<geometry_msgs::PointStamped>;

// Publisher and subscriber proxy class instantiation for ROS message type: geometry_msgs/Polygon
template class ZyROSConnectorTopicSubscriber<geometry_msgs::Polygon>;

// Publisher and subscriber proxy class instantiation for ROS message type: geometry_msgs/PolygonStamped
template class ZyROSConnectorTopicSubscriber<geometry_msgs::PolygonStamped>;

// Publisher and subscriber proxy class instantiation for ROS message type: geometry_msgs/Pose
template class ZyROSConnectorTopicSubscriber<geometry_msgs::Pose>;

// Publisher and subscriber proxy class instantiation for ROS message type: geometry_msgs/Pose2D
template class ZyROSConnectorTopicSubscriber<geometry_msgs::Pose2D>;

// Publisher and subscriber proxy class instantiation for ROS message type: geometry_msgs/PoseArray
template class ZyROSConnectorTopicSubscriber<geometry_msgs::PoseArray>;

// Publisher and subscriber proxy class instantiation for ROS message type: geometry_msgs/PoseStamped
template class ZyROSConnectorTopicSubscriber<geometry_msgs::PoseStamped>;

// Publisher and subscriber proxy class instantiation for ROS message type: geometry_msgs/PoseWithCovariance
template class ZyROSConnectorTopicSubscriber<geometry_msgs::PoseWithCovariance>;

// Publisher and subscriber proxy class instantiation for ROS message type: geometry_msgs/PoseWithCovarianceStamped
template class ZyROSConnectorTopicSubscriber<geometry_msgs::PoseWithCovarianceStamped>;

// Publisher and subscriber proxy class instantiation for ROS message type: geometry_msgs/Quaternion
template class ZyROSConnectorTopicSubscriber<geometry_msgs::Quaternion>;

// Publisher and subscriber proxy class instantiation for ROS message type: geometry_msgs/QuaternionStamped
template class ZyROSConnectorTopicSubscriber<geometry_msgs::QuaternionStamped>;

// Publisher and subscriber proxy class instantiation for ROS message type: geometry_msgs/Transform
template class ZyROSConnectorTopicSubscriber<geometry_msgs::Transform>;

// Publisher and subscriber proxy class instantiation for ROS message type: geometry_msgs/TransformStamped
template class ZyROSConnectorTopicSubscriber<geometry_msgs::TransformStamped>;

// Publisher and subscriber proxy class instantiation for ROS message type: geometry_msgs/Twist
template class ZyROSConnectorTopicSubscriber<geometry_msgs::Twist>;

// Publisher and subscriber proxy class instantiation for ROS message type: geometry_msgs/TwistStamped
template class ZyROSConnectorTopicSubscriber<geometry_msgs::TwistStamped>;

// Publisher and subscriber proxy class instantiation for ROS message type: geometry_msgs/TwistWithCovariance
template class ZyROSConnectorTopicSubscriber<geometry_msgs::TwistWithCovariance>;

// Publisher and subscriber proxy class instantiation for ROS message type: geometry_msgs/TwistWithCovarianceStamped
template class ZyROSConnectorTopicSubscriber<geometry_msgs::TwistWithCovarianceStamped>;

// Publisher and subscriber proxy class instantiation for ROS message type: geometry_msgs/Vector3
template class ZyROSConnectorTopicSubscriber<geometry_msgs::Vector3>;

// Publisher and subscriber proxy class instantiation for ROS message type: geometry_msgs/Vector3Stamped
template class ZyROSConnectorTopicSubscriber<geometry_msgs::Vector3Stamped>;

// Publisher and subscriber proxy class instantiation for ROS message type: geometry_msgs/Wrench
template class ZyROSConnectorTopicSubscriber<geometry_msgs::Wrench>;

// Publisher and subscriber proxy class instantiation for ROS message type: geometry_msgs/WrenchStamped
template class ZyROSConnectorTopicSubscriber<geometry_msgs::WrenchStamped>;

// Publisher and subscriber proxy class instantiation for ROS message type: nav_msgs/GetMapAction
template class ZyROSConnectorTopicSubscriber<nav_msgs::GetMapAction>;

// Publisher and subscriber proxy class instantiation for ROS message type: nav_msgs/GetMapActionFeedback
template class ZyROSConnectorTopicSubscriber<nav_msgs::GetMapActionFeedback>;

// Publisher and subscriber proxy class instantiation for ROS message type: nav_msgs/GetMapActionGoal
template class ZyROSConnectorTopicSubscriber<nav_msgs::GetMapActionGoal>;

// Publisher and subscriber proxy class instantiation for ROS message type: nav_msgs/GetMapActionResult
template class ZyROSConnectorTopicSubscriber<nav_msgs::GetMapActionResult>;

// Publisher and subscriber proxy class instantiation for ROS message type: nav_msgs/GetMapFeedback
template class ZyROSConnectorTopicSubscriber<nav_msgs::GetMapFeedback>;

// Publisher and subscriber proxy class instantiation for ROS message type: nav_msgs/GetMapGoal
template class ZyROSConnectorTopicSubscriber<nav_msgs::GetMapGoal>;

// Publisher and subscriber proxy class instantiation for ROS message type: nav_msgs/GetMapResult
template class ZyROSConnectorTopicSubscriber<nav_msgs::GetMapResult>;

// Publisher and subscriber proxy class instantiation for ROS message type: nav_msgs/GridCells
template class ZyROSConnectorTopicSubscriber<nav_msgs::GridCells>;

// Publisher and subscriber proxy class instantiation for ROS message type: nav_msgs/MapMetaData
template class ZyROSConnectorTopicSubscriber<nav_msgs::MapMetaData>;

// Publisher and subscriber proxy class instantiation for ROS message type: nav_msgs/OccupancyGrid
template class ZyROSConnectorTopicSubscriber<nav_msgs::OccupancyGrid>;

// Publisher and subscriber proxy class instantiation for ROS message type: nav_msgs/Odometry
template class ZyROSConnectorTopicSubscriber<nav_msgs::Odometry>;

// Publisher and subscriber proxy class instantiation for ROS message type: nav_msgs/Path
template class ZyROSConnectorTopicSubscriber<nav_msgs::Path>;

// Publisher and subscriber proxy class instantiation for ROS message type: roscpp/Logger
template class ZyROSConnectorTopicSubscriber<roscpp::Logger>;

// Publisher and subscriber proxy class instantiation for ROS message type: rosgraph_msgs/Clock
template class ZyROSConnectorTopicSubscriber<rosgraph_msgs::Clock>;

// Publisher and subscriber proxy class instantiation for ROS message type: rosgraph_msgs/Log
template class ZyROSConnectorTopicSubscriber<rosgraph_msgs::Log>;

// Publisher and subscriber proxy class instantiation for ROS message type: rosgraph_msgs/TopicStatistics
template class ZyROSConnectorTopicSubscriber<rosgraph_msgs::TopicStatistics>;

// Publisher and subscriber proxy class instantiation for ROS message type: rospy_tutorials/Floats
template class ZyROSConnectorTopicSubscriber<rospy_tutorials::Floats>;

// Publisher and subscriber proxy class instantiation for ROS message type: rospy_tutorials/HeaderString
template class ZyROSConnectorTopicSubscriber<rospy_tutorials::HeaderString>;

// Publisher and subscriber proxy class instantiation for ROS message type: sensor_msgs/BatteryState
template class ZyROSConnectorTopicSubscriber<sensor_msgs::BatteryState>;

// Publisher and subscriber proxy class instantiation for ROS message type: sensor_msgs/CameraInfo
template class ZyROSConnectorTopicSubscriber<sensor_msgs::CameraInfo>;

// Publisher and subscriber proxy class instantiation for ROS message type: sensor_msgs/ChannelFloat32
template class ZyROSConnectorTopicSubscriber<sensor_msgs::ChannelFloat32>;

// Publisher and subscriber proxy class instantiation for ROS message type: sensor_msgs/CompressedImage
template class ZyROSConnectorTopicSubscriber<sensor_msgs::CompressedImage>;

// Publisher and subscriber proxy class instantiation for ROS message type: sensor_msgs/FluidPressure
template class ZyROSConnectorTopicSubscriber<sensor_msgs::FluidPressure>;

// Publisher and subscriber proxy class instantiation for ROS message type: sensor_msgs/Illuminance
template class ZyROSConnectorTopicSubscriber<sensor_msgs::Illuminance>;

// Publisher and subscriber proxy class instantiation for ROS message type: sensor_msgs/Image
template class ZyROSConnectorTopicSubscriber<sensor_msgs::Image>;

// Publisher and subscriber proxy class instantiation for ROS message type: sensor_msgs/Imu
template class ZyROSConnectorTopicSubscriber<sensor_msgs::Imu>;

// Publisher and subscriber proxy class instantiation for ROS message type: sensor_msgs/JointState
template class ZyROSConnectorTopicSubscriber<sensor_msgs::JointState>;

// Publisher and subscriber proxy class instantiation for ROS message type: sensor_msgs/Joy
template class ZyROSConnectorTopicSubscriber<sensor_msgs::Joy>;

// Publisher and subscriber proxy class instantiation for ROS message type: sensor_msgs/JoyFeedback
template class ZyROSConnectorTopicSubscriber<sensor_msgs::JoyFeedback>;

// Publisher and subscriber proxy class instantiation for ROS message type: sensor_msgs/JoyFeedbackArray
template class ZyROSConnectorTopicSubscriber<sensor_msgs::JoyFeedbackArray>;

// Publisher and subscriber proxy class instantiation for ROS message type: sensor_msgs/LaserEcho
template class ZyROSConnectorTopicSubscriber<sensor_msgs::LaserEcho>;

// Publisher and subscriber proxy class instantiation for ROS message type: sensor_msgs/LaserScan
template class ZyROSConnectorTopicSubscriber<sensor_msgs::LaserScan>;

// Publisher and subscriber proxy class instantiation for ROS message type: sensor_msgs/MagneticField
template class ZyROSConnectorTopicSubscriber<sensor_msgs::MagneticField>;

// Publisher and subscriber proxy class instantiation for ROS message type: sensor_msgs/MultiDOFJointState
template class ZyROSConnectorTopicSubscriber<sensor_msgs::MultiDOFJointState>;

// Publisher and subscriber proxy class instantiation for ROS message type: sensor_msgs/MultiEchoLaserScan
template class ZyROSConnectorTopicSubscriber<sensor_msgs::MultiEchoLaserScan>;

// Publisher and subscriber proxy class instantiation for ROS message type: sensor_msgs/NavSatFix
template class ZyROSConnectorTopicSubscriber<sensor_msgs::NavSatFix>;

// Publisher and subscriber proxy class instantiation for ROS message type: sensor_msgs/NavSatStatus
template class ZyROSConnectorTopicSubscriber<sensor_msgs::NavSatStatus>;

// Publisher and subscriber proxy class instantiation for ROS message type: sensor_msgs/PointCloud
template class ZyROSConnectorTopicSubscriber<sensor_msgs::PointCloud>;

// Publisher and subscriber proxy class instantiation for ROS message type: sensor_msgs/PointCloud2
template class ZyROSConnectorTopicSubscriber<sensor_msgs::PointCloud2>;

// Publisher and subscriber proxy class instantiation for ROS message type: sensor_msgs/PointField
template class ZyROSConnectorTopicSubscriber<sensor_msgs::PointField>;

// Publisher and subscriber proxy class instantiation for ROS message type: sensor_msgs/Range
template class ZyROSConnectorTopicSubscriber<sensor_msgs::Range>;

// Publisher and subscriber proxy class instantiation for ROS message type: sensor_msgs/RegionOfInterest
template class ZyROSConnectorTopicSubscriber<sensor_msgs::RegionOfInterest>;

// Publisher and subscriber proxy class instantiation for ROS message type: sensor_msgs/RelativeHumidity
template class ZyROSConnectorTopicSubscriber<sensor_msgs::RelativeHumidity>;

// Publisher and subscriber proxy class instantiation for ROS message type: sensor_msgs/Temperature
template class ZyROSConnectorTopicSubscriber<sensor_msgs::Temperature>;

// Publisher and subscriber proxy class instantiation for ROS message type: sensor_msgs/TimeReference
template class ZyROSConnectorTopicSubscriber<sensor_msgs::TimeReference>;

// Publisher and subscriber proxy class instantiation for ROS message type: shape_msgs/Mesh
template class ZyROSConnectorTopicSubscriber<shape_msgs::Mesh>;

// Publisher and subscriber proxy class instantiation for ROS message type: shape_msgs/MeshTriangle
template class ZyROSConnectorTopicSubscriber<shape_msgs::MeshTriangle>;

// Publisher and subscriber proxy class instantiation for ROS message type: shape_msgs/Plane
template class ZyROSConnectorTopicSubscriber<shape_msgs::Plane>;

// Publisher and subscriber proxy class instantiation for ROS message type: shape_msgs/SolidPrimitive
template class ZyROSConnectorTopicSubscriber<shape_msgs::SolidPrimitive>;

// Publisher and subscriber proxy class instantiation for ROS message type: smach_msgs/SmachContainerInitialStatusCmd
template class ZyROSConnectorTopicSubscriber<smach_msgs::SmachContainerInitialStatusCmd>;

// Publisher and subscriber proxy class instantiation for ROS message type: smach_msgs/SmachContainerStatus
template class ZyROSConnectorTopicSubscriber<smach_msgs::SmachContainerStatus>;

// Publisher and subscriber proxy class instantiation for ROS message type: smach_msgs/SmachContainerStructure
template class ZyROSConnectorTopicSubscriber<smach_msgs::SmachContainerStructure>;

// Publisher and subscriber proxy class instantiation for ROS message type: sofa_softrobots_msgs/BodyTransforms
template class ZyROSConnectorTopicSubscriber<sofa_softrobots_msgs::BodyTransforms>;

// Publisher and subscriber proxy class instantiation for ROS message type: std_msgs/Bool
template class ZyROSConnectorTopicSubscriber<std_msgs::Bool>;

// Publisher and subscriber proxy class instantiation for ROS message type: std_msgs/Byte
template class ZyROSConnectorTopicSubscriber<std_msgs::Byte>;

// Publisher and subscriber proxy class instantiation for ROS message type: std_msgs/ByteMultiArray
template class ZyROSConnectorTopicSubscriber<std_msgs::ByteMultiArray>;

// Publisher and subscriber proxy class instantiation for ROS message type: std_msgs/Char
template class ZyROSConnectorTopicSubscriber<std_msgs::Char>;

// Publisher and subscriber proxy class instantiation for ROS message type: std_msgs/ColorRGBA
template class ZyROSConnectorTopicSubscriber<std_msgs::ColorRGBA>;

// Publisher and subscriber proxy class instantiation for ROS message type: std_msgs/Duration
template class ZyROSConnectorTopicSubscriber<std_msgs::Duration>;

// Publisher and subscriber proxy class instantiation for ROS message type: std_msgs/Empty
template class ZyROSConnectorTopicSubscriber<std_msgs::Empty>;

// Publisher and subscriber proxy class instantiation for ROS message type: std_msgs/Float32
template class ZyROSConnectorTopicSubscriber<std_msgs::Float32>;

// Publisher and subscriber proxy class instantiation for ROS message type: std_msgs/Float32MultiArray
template class ZyROSConnectorTopicSubscriber<std_msgs::Float32MultiArray>;

// Publisher and subscriber proxy class instantiation for ROS message type: std_msgs/Float64
template class ZyROSConnectorTopicSubscriber<std_msgs::Float64>;

// Publisher and subscriber proxy class instantiation for ROS message type: std_msgs/Float64MultiArray
template class ZyROSConnectorTopicSubscriber<std_msgs::Float64MultiArray>;

// Publisher and subscriber proxy class instantiation for ROS message type: std_msgs/Header
template class ZyROSConnectorTopicSubscriber<std_msgs::Header>;

// Publisher and subscriber proxy class instantiation for ROS message type: std_msgs/Int16
template class ZyROSConnectorTopicSubscriber<std_msgs::Int16>;

// Publisher and subscriber proxy class instantiation for ROS message type: std_msgs/Int16MultiArray
template class ZyROSConnectorTopicSubscriber<std_msgs::Int16MultiArray>;

// Publisher and subscriber proxy class instantiation for ROS message type: std_msgs/Int32
template class ZyROSConnectorTopicSubscriber<std_msgs::Int32>;

// Publisher and subscriber proxy class instantiation for ROS message type: std_msgs/Int32MultiArray
template class ZyROSConnectorTopicSubscriber<std_msgs::Int32MultiArray>;

// Publisher and subscriber proxy class instantiation for ROS message type: std_msgs/Int64
template class ZyROSConnectorTopicSubscriber<std_msgs::Int64>;

// Publisher and subscriber proxy class instantiation for ROS message type: std_msgs/Int64MultiArray
template class ZyROSConnectorTopicSubscriber<std_msgs::Int64MultiArray>;

// Publisher and subscriber proxy class instantiation for ROS message type: std_msgs/Int8
template class ZyROSConnectorTopicSubscriber<std_msgs::Int8>;

// Publisher and subscriber proxy class instantiation for ROS message type: std_msgs/Int8MultiArray
template class ZyROSConnectorTopicSubscriber<std_msgs::Int8MultiArray>;

// Publisher and subscriber proxy class instantiation for ROS message type: std_msgs/MultiArrayDimension
template class ZyROSConnectorTopicSubscriber<std_msgs::MultiArrayDimension>;

// Publisher and subscriber proxy class instantiation for ROS message type: std_msgs/MultiArrayLayout
template class ZyROSConnectorTopicSubscriber<std_msgs::MultiArrayLayout>;

// Publisher and subscriber proxy class instantiation for ROS message type: std_msgs/String
template class ZyROSConnectorTopicSubscriber<std_msgs::String>;

// Publisher and subscriber proxy class instantiation for ROS message type: std_msgs/Time
template class ZyROSConnectorTopicSubscriber<std_msgs::Time>;

// Publisher and subscriber proxy class instantiation for ROS message type: std_msgs/UInt16
template class ZyROSConnectorTopicSubscriber<std_msgs::UInt16>;

// Publisher and subscriber proxy class instantiation for ROS message type: std_msgs/UInt16MultiArray
template class ZyROSConnectorTopicSubscriber<std_msgs::UInt16MultiArray>;

// Publisher and subscriber proxy class instantiation for ROS message type: std_msgs/UInt32
template class ZyROSConnectorTopicSubscriber<std_msgs::UInt32>;

// Publisher and subscriber proxy class instantiation for ROS message type: std_msgs/UInt32MultiArray
template class ZyROSConnectorTopicSubscriber<std_msgs::UInt32MultiArray>;

// Publisher and subscriber proxy class instantiation for ROS message type: std_msgs/UInt64
template class ZyROSConnectorTopicSubscriber<std_msgs::UInt64>;

// Publisher and subscriber proxy class instantiation for ROS message type: std_msgs/UInt64MultiArray
template class ZyROSConnectorTopicSubscriber<std_msgs::UInt64MultiArray>;

// Publisher and subscriber proxy class instantiation for ROS message type: std_msgs/UInt8
template class ZyROSConnectorTopicSubscriber<std_msgs::UInt8>;

// Publisher and subscriber proxy class instantiation for ROS message type: std_msgs/UInt8MultiArray
template class ZyROSConnectorTopicSubscriber<std_msgs::UInt8MultiArray>;

// Publisher and subscriber proxy class instantiation for ROS message type: stereo_msgs/DisparityImage
template class ZyROSConnectorTopicSubscriber<stereo_msgs::DisparityImage>;

// Publisher and subscriber proxy class instantiation for ROS message type: tf/tfMessage
template class ZyROSConnectorTopicSubscriber<tf::tfMessage>;

// Publisher and subscriber proxy class instantiation for ROS message type: tf2_msgs/LookupTransformAction
template class ZyROSConnectorTopicSubscriber<tf2_msgs::LookupTransformAction>;

// Publisher and subscriber proxy class instantiation for ROS message type: tf2_msgs/LookupTransformActionFeedback
template class ZyROSConnectorTopicSubscriber<tf2_msgs::LookupTransformActionFeedback>;

// Publisher and subscriber proxy class instantiation for ROS message type: tf2_msgs/LookupTransformActionGoal
template class ZyROSConnectorTopicSubscriber<tf2_msgs::LookupTransformActionGoal>;

// Publisher and subscriber proxy class instantiation for ROS message type: tf2_msgs/LookupTransformActionResult
template class ZyROSConnectorTopicSubscriber<tf2_msgs::LookupTransformActionResult>;

// Publisher and subscriber proxy class instantiation for ROS message type: tf2_msgs/LookupTransformFeedback
template class ZyROSConnectorTopicSubscriber<tf2_msgs::LookupTransformFeedback>;

// Publisher and subscriber proxy class instantiation for ROS message type: tf2_msgs/LookupTransformGoal
template class ZyROSConnectorTopicSubscriber<tf2_msgs::LookupTransformGoal>;

// Publisher and subscriber proxy class instantiation for ROS message type: tf2_msgs/LookupTransformResult
template class ZyROSConnectorTopicSubscriber<tf2_msgs::LookupTransformResult>;

// Publisher and subscriber proxy class instantiation for ROS message type: tf2_msgs/TF2Error
template class ZyROSConnectorTopicSubscriber<tf2_msgs::TF2Error>;

// Publisher and subscriber proxy class instantiation for ROS message type: tf2_msgs/TFMessage
template class ZyROSConnectorTopicSubscriber<tf2_msgs::TFMessage>;

// Publisher and subscriber proxy class instantiation for ROS message type: trajectory_msgs/JointTrajectory
template class ZyROSConnectorTopicSubscriber<trajectory_msgs::JointTrajectory>;

// Publisher and subscriber proxy class instantiation for ROS message type: trajectory_msgs/JointTrajectoryPoint
template class ZyROSConnectorTopicSubscriber<trajectory_msgs::JointTrajectoryPoint>;

// Publisher and subscriber proxy class instantiation for ROS message type: trajectory_msgs/MultiDOFJointTrajectory
template class ZyROSConnectorTopicSubscriber<trajectory_msgs::MultiDOFJointTrajectory>;

// Publisher and subscriber proxy class instantiation for ROS message type: trajectory_msgs/MultiDOFJointTrajectoryPoint
template class ZyROSConnectorTopicSubscriber<trajectory_msgs::MultiDOFJointTrajectoryPoint>;

// Publisher and subscriber proxy class instantiation for ROS message type: visualization_msgs/ImageMarker
template class ZyROSConnectorTopicSubscriber<visualization_msgs::ImageMarker>;

// Publisher and subscriber proxy class instantiation for ROS message type: visualization_msgs/InteractiveMarker
template class ZyROSConnectorTopicSubscriber<visualization_msgs::InteractiveMarker>;

// Publisher and subscriber proxy class instantiation for ROS message type: visualization_msgs/InteractiveMarkerControl
template class ZyROSConnectorTopicSubscriber<visualization_msgs::InteractiveMarkerControl>;

// Publisher and subscriber proxy class instantiation for ROS message type: visualization_msgs/InteractiveMarkerFeedback
template class ZyROSConnectorTopicSubscriber<visualization_msgs::InteractiveMarkerFeedback>;

// Publisher and subscriber proxy class instantiation for ROS message type: visualization_msgs/InteractiveMarkerInit
template class ZyROSConnectorTopicSubscriber<visualization_msgs::InteractiveMarkerInit>;

// Publisher and subscriber proxy class instantiation for ROS message type: visualization_msgs/InteractiveMarkerPose
template class ZyROSConnectorTopicSubscriber<visualization_msgs::InteractiveMarkerPose>;

// Publisher and subscriber proxy class instantiation for ROS message type: visualization_msgs/InteractiveMarkerUpdate
template class ZyROSConnectorTopicSubscriber<visualization_msgs::InteractiveMarkerUpdate>;

// Publisher and subscriber proxy class instantiation for ROS message type: visualization_msgs/Marker
template class ZyROSConnectorTopicSubscriber<visualization_msgs::Marker>;

// Publisher and subscriber proxy class instantiation for ROS message type: visualization_msgs/MarkerArray
template class ZyROSConnectorTopicSubscriber<visualization_msgs::MarkerArray>;

// Publisher and subscriber proxy class instantiation for ROS message type: visualization_msgs/MenuEntry
template class ZyROSConnectorTopicSubscriber<visualization_msgs::MenuEntry>;

boost::shared_ptr<ZyROSListener> ZyROSConnectorMessageSubscriberFactory::createTopicSubscriber(ros::NodeHandlePtr rosNode, const std::string& topicURI, const std::string& messageType)
{
	bool supported = false;
	boost::shared_ptr<ZyROSListener> topicListener;
	// Subscriber instance for ROS message type: actionlib/TestAction
	if (messageType == "actionlib::TestAction")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<actionlib::TestAction>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: actionlib/TestActionFeedback
	if (messageType == "actionlib::TestActionFeedback")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<actionlib::TestActionFeedback>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: actionlib/TestActionGoal
	if (messageType == "actionlib::TestActionGoal")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<actionlib::TestActionGoal>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: actionlib/TestActionResult
	if (messageType == "actionlib::TestActionResult")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<actionlib::TestActionResult>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: actionlib/TestFeedback
	if (messageType == "actionlib::TestFeedback")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<actionlib::TestFeedback>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: actionlib/TestGoal
	if (messageType == "actionlib::TestGoal")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<actionlib::TestGoal>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: actionlib/TestRequestAction
	if (messageType == "actionlib::TestRequestAction")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<actionlib::TestRequestAction>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: actionlib/TestRequestActionFeedback
	if (messageType == "actionlib::TestRequestActionFeedback")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<actionlib::TestRequestActionFeedback>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: actionlib/TestRequestActionGoal
	if (messageType == "actionlib::TestRequestActionGoal")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<actionlib::TestRequestActionGoal>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: actionlib/TestRequestActionResult
	if (messageType == "actionlib::TestRequestActionResult")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<actionlib::TestRequestActionResult>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: actionlib/TestRequestFeedback
	if (messageType == "actionlib::TestRequestFeedback")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<actionlib::TestRequestFeedback>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: actionlib/TestRequestGoal
	if (messageType == "actionlib::TestRequestGoal")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<actionlib::TestRequestGoal>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: actionlib/TestRequestResult
	if (messageType == "actionlib::TestRequestResult")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<actionlib::TestRequestResult>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: actionlib/TestResult
	if (messageType == "actionlib::TestResult")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<actionlib::TestResult>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: actionlib/TwoIntsAction
	if (messageType == "actionlib::TwoIntsAction")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<actionlib::TwoIntsAction>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: actionlib/TwoIntsActionFeedback
	if (messageType == "actionlib::TwoIntsActionFeedback")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<actionlib::TwoIntsActionFeedback>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: actionlib/TwoIntsActionGoal
	if (messageType == "actionlib::TwoIntsActionGoal")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<actionlib::TwoIntsActionGoal>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: actionlib/TwoIntsActionResult
	if (messageType == "actionlib::TwoIntsActionResult")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<actionlib::TwoIntsActionResult>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: actionlib/TwoIntsFeedback
	if (messageType == "actionlib::TwoIntsFeedback")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<actionlib::TwoIntsFeedback>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: actionlib/TwoIntsGoal
	if (messageType == "actionlib::TwoIntsGoal")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<actionlib::TwoIntsGoal>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: actionlib/TwoIntsResult
	if (messageType == "actionlib::TwoIntsResult")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<actionlib::TwoIntsResult>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: actionlib_msgs/GoalID
	if (messageType == "actionlib_msgs::GoalID")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<actionlib_msgs::GoalID>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: actionlib_msgs/GoalStatus
	if (messageType == "actionlib_msgs::GoalStatus")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<actionlib_msgs::GoalStatus>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: actionlib_msgs/GoalStatusArray
	if (messageType == "actionlib_msgs::GoalStatusArray")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<actionlib_msgs::GoalStatusArray>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: bond/Constants
	if (messageType == "bond::Constants")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<bond::Constants>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: bond/Status
	if (messageType == "bond::Status")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<bond::Status>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: control_msgs/FollowJointTrajectoryAction
	if (messageType == "control_msgs::FollowJointTrajectoryAction")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<control_msgs::FollowJointTrajectoryAction>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: control_msgs/FollowJointTrajectoryActionFeedback
	if (messageType == "control_msgs::FollowJointTrajectoryActionFeedback")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<control_msgs::FollowJointTrajectoryActionFeedback>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: control_msgs/FollowJointTrajectoryActionGoal
	if (messageType == "control_msgs::FollowJointTrajectoryActionGoal")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<control_msgs::FollowJointTrajectoryActionGoal>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: control_msgs/FollowJointTrajectoryActionResult
	if (messageType == "control_msgs::FollowJointTrajectoryActionResult")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<control_msgs::FollowJointTrajectoryActionResult>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: control_msgs/FollowJointTrajectoryFeedback
	if (messageType == "control_msgs::FollowJointTrajectoryFeedback")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<control_msgs::FollowJointTrajectoryFeedback>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: control_msgs/FollowJointTrajectoryGoal
	if (messageType == "control_msgs::FollowJointTrajectoryGoal")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<control_msgs::FollowJointTrajectoryGoal>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: control_msgs/FollowJointTrajectoryResult
	if (messageType == "control_msgs::FollowJointTrajectoryResult")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<control_msgs::FollowJointTrajectoryResult>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: control_msgs/GripperCommand
	if (messageType == "control_msgs::GripperCommand")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<control_msgs::GripperCommand>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: control_msgs/GripperCommandAction
	if (messageType == "control_msgs::GripperCommandAction")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<control_msgs::GripperCommandAction>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: control_msgs/GripperCommandActionFeedback
	if (messageType == "control_msgs::GripperCommandActionFeedback")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<control_msgs::GripperCommandActionFeedback>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: control_msgs/GripperCommandActionGoal
	if (messageType == "control_msgs::GripperCommandActionGoal")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<control_msgs::GripperCommandActionGoal>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: control_msgs/GripperCommandActionResult
	if (messageType == "control_msgs::GripperCommandActionResult")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<control_msgs::GripperCommandActionResult>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: control_msgs/GripperCommandFeedback
	if (messageType == "control_msgs::GripperCommandFeedback")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<control_msgs::GripperCommandFeedback>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: control_msgs/GripperCommandGoal
	if (messageType == "control_msgs::GripperCommandGoal")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<control_msgs::GripperCommandGoal>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: control_msgs/GripperCommandResult
	if (messageType == "control_msgs::GripperCommandResult")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<control_msgs::GripperCommandResult>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: control_msgs/JointControllerState
	if (messageType == "control_msgs::JointControllerState")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<control_msgs::JointControllerState>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: control_msgs/JointJog
	if (messageType == "control_msgs::JointJog")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<control_msgs::JointJog>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: control_msgs/JointTolerance
	if (messageType == "control_msgs::JointTolerance")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<control_msgs::JointTolerance>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: control_msgs/JointTrajectoryAction
	if (messageType == "control_msgs::JointTrajectoryAction")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<control_msgs::JointTrajectoryAction>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: control_msgs/JointTrajectoryActionFeedback
	if (messageType == "control_msgs::JointTrajectoryActionFeedback")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<control_msgs::JointTrajectoryActionFeedback>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: control_msgs/JointTrajectoryActionGoal
	if (messageType == "control_msgs::JointTrajectoryActionGoal")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<control_msgs::JointTrajectoryActionGoal>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: control_msgs/JointTrajectoryActionResult
	if (messageType == "control_msgs::JointTrajectoryActionResult")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<control_msgs::JointTrajectoryActionResult>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: control_msgs/JointTrajectoryControllerState
	if (messageType == "control_msgs::JointTrajectoryControllerState")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<control_msgs::JointTrajectoryControllerState>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: control_msgs/JointTrajectoryFeedback
	if (messageType == "control_msgs::JointTrajectoryFeedback")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<control_msgs::JointTrajectoryFeedback>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: control_msgs/JointTrajectoryGoal
	if (messageType == "control_msgs::JointTrajectoryGoal")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<control_msgs::JointTrajectoryGoal>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: control_msgs/JointTrajectoryResult
	if (messageType == "control_msgs::JointTrajectoryResult")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<control_msgs::JointTrajectoryResult>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: control_msgs/PidState
	if (messageType == "control_msgs::PidState")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<control_msgs::PidState>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: control_msgs/PointHeadAction
	if (messageType == "control_msgs::PointHeadAction")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<control_msgs::PointHeadAction>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: control_msgs/PointHeadActionFeedback
	if (messageType == "control_msgs::PointHeadActionFeedback")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<control_msgs::PointHeadActionFeedback>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: control_msgs/PointHeadActionGoal
	if (messageType == "control_msgs::PointHeadActionGoal")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<control_msgs::PointHeadActionGoal>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: control_msgs/PointHeadActionResult
	if (messageType == "control_msgs::PointHeadActionResult")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<control_msgs::PointHeadActionResult>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: control_msgs/PointHeadFeedback
	if (messageType == "control_msgs::PointHeadFeedback")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<control_msgs::PointHeadFeedback>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: control_msgs/PointHeadGoal
	if (messageType == "control_msgs::PointHeadGoal")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<control_msgs::PointHeadGoal>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: control_msgs/PointHeadResult
	if (messageType == "control_msgs::PointHeadResult")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<control_msgs::PointHeadResult>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: control_msgs/SingleJointPositionAction
	if (messageType == "control_msgs::SingleJointPositionAction")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<control_msgs::SingleJointPositionAction>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: control_msgs/SingleJointPositionActionFeedback
	if (messageType == "control_msgs::SingleJointPositionActionFeedback")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<control_msgs::SingleJointPositionActionFeedback>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: control_msgs/SingleJointPositionActionGoal
	if (messageType == "control_msgs::SingleJointPositionActionGoal")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<control_msgs::SingleJointPositionActionGoal>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: control_msgs/SingleJointPositionActionResult
	if (messageType == "control_msgs::SingleJointPositionActionResult")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<control_msgs::SingleJointPositionActionResult>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: control_msgs/SingleJointPositionFeedback
	if (messageType == "control_msgs::SingleJointPositionFeedback")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<control_msgs::SingleJointPositionFeedback>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: control_msgs/SingleJointPositionGoal
	if (messageType == "control_msgs::SingleJointPositionGoal")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<control_msgs::SingleJointPositionGoal>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: control_msgs/SingleJointPositionResult
	if (messageType == "control_msgs::SingleJointPositionResult")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<control_msgs::SingleJointPositionResult>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: diagnostic_msgs/DiagnosticArray
	if (messageType == "diagnostic_msgs::DiagnosticArray")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<diagnostic_msgs::DiagnosticArray>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: diagnostic_msgs/DiagnosticStatus
	if (messageType == "diagnostic_msgs::DiagnosticStatus")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<diagnostic_msgs::DiagnosticStatus>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: diagnostic_msgs/KeyValue
	if (messageType == "diagnostic_msgs::KeyValue")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<diagnostic_msgs::KeyValue>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: dynamic_reconfigure/BoolParameter
	if (messageType == "dynamic_reconfigure::BoolParameter")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<dynamic_reconfigure::BoolParameter>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: dynamic_reconfigure/Config
	if (messageType == "dynamic_reconfigure::Config")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<dynamic_reconfigure::Config>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: dynamic_reconfigure/ConfigDescription
	if (messageType == "dynamic_reconfigure::ConfigDescription")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<dynamic_reconfigure::ConfigDescription>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: dynamic_reconfigure/DoubleParameter
	if (messageType == "dynamic_reconfigure::DoubleParameter")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<dynamic_reconfigure::DoubleParameter>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: dynamic_reconfigure/Group
	if (messageType == "dynamic_reconfigure::Group")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<dynamic_reconfigure::Group>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: dynamic_reconfigure/GroupState
	if (messageType == "dynamic_reconfigure::GroupState")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<dynamic_reconfigure::GroupState>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: dynamic_reconfigure/IntParameter
	if (messageType == "dynamic_reconfigure::IntParameter")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<dynamic_reconfigure::IntParameter>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: dynamic_reconfigure/ParamDescription
	if (messageType == "dynamic_reconfigure::ParamDescription")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<dynamic_reconfigure::ParamDescription>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: dynamic_reconfigure/SensorLevels
	if (messageType == "dynamic_reconfigure::SensorLevels")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<dynamic_reconfigure::SensorLevels>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: dynamic_reconfigure/StrParameter
	if (messageType == "dynamic_reconfigure::StrParameter")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<dynamic_reconfigure::StrParameter>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: geometry_msgs/Accel
	if (messageType == "geometry_msgs::Accel")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<geometry_msgs::Accel>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: geometry_msgs/AccelStamped
	if (messageType == "geometry_msgs::AccelStamped")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<geometry_msgs::AccelStamped>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: geometry_msgs/AccelWithCovariance
	if (messageType == "geometry_msgs::AccelWithCovariance")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<geometry_msgs::AccelWithCovariance>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: geometry_msgs/AccelWithCovarianceStamped
	if (messageType == "geometry_msgs::AccelWithCovarianceStamped")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<geometry_msgs::AccelWithCovarianceStamped>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: geometry_msgs/Inertia
	if (messageType == "geometry_msgs::Inertia")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<geometry_msgs::Inertia>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: geometry_msgs/InertiaStamped
	if (messageType == "geometry_msgs::InertiaStamped")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<geometry_msgs::InertiaStamped>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: geometry_msgs/Point
	if (messageType == "geometry_msgs::Point")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<geometry_msgs::Point>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: geometry_msgs/Point32
	if (messageType == "geometry_msgs::Point32")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<geometry_msgs::Point32>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: geometry_msgs/PointStamped
	if (messageType == "geometry_msgs::PointStamped")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<geometry_msgs::PointStamped>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: geometry_msgs/Polygon
	if (messageType == "geometry_msgs::Polygon")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<geometry_msgs::Polygon>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: geometry_msgs/PolygonStamped
	if (messageType == "geometry_msgs::PolygonStamped")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<geometry_msgs::PolygonStamped>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: geometry_msgs/Pose
	if (messageType == "geometry_msgs::Pose")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<geometry_msgs::Pose>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: geometry_msgs/Pose2D
	if (messageType == "geometry_msgs::Pose2D")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<geometry_msgs::Pose2D>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: geometry_msgs/PoseArray
	if (messageType == "geometry_msgs::PoseArray")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<geometry_msgs::PoseArray>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: geometry_msgs/PoseStamped
	if (messageType == "geometry_msgs::PoseStamped")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<geometry_msgs::PoseStamped>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: geometry_msgs/PoseWithCovariance
	if (messageType == "geometry_msgs::PoseWithCovariance")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<geometry_msgs::PoseWithCovariance>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: geometry_msgs/PoseWithCovarianceStamped
	if (messageType == "geometry_msgs::PoseWithCovarianceStamped")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<geometry_msgs::PoseWithCovarianceStamped>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: geometry_msgs/Quaternion
	if (messageType == "geometry_msgs::Quaternion")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<geometry_msgs::Quaternion>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: geometry_msgs/QuaternionStamped
	if (messageType == "geometry_msgs::QuaternionStamped")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<geometry_msgs::QuaternionStamped>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: geometry_msgs/Transform
	if (messageType == "geometry_msgs::Transform")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<geometry_msgs::Transform>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: geometry_msgs/TransformStamped
	if (messageType == "geometry_msgs::TransformStamped")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<geometry_msgs::TransformStamped>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: geometry_msgs/Twist
	if (messageType == "geometry_msgs::Twist")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<geometry_msgs::Twist>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: geometry_msgs/TwistStamped
	if (messageType == "geometry_msgs::TwistStamped")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<geometry_msgs::TwistStamped>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: geometry_msgs/TwistWithCovariance
	if (messageType == "geometry_msgs::TwistWithCovariance")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<geometry_msgs::TwistWithCovariance>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: geometry_msgs/TwistWithCovarianceStamped
	if (messageType == "geometry_msgs::TwistWithCovarianceStamped")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<geometry_msgs::TwistWithCovarianceStamped>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: geometry_msgs/Vector3
	if (messageType == "geometry_msgs::Vector3")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<geometry_msgs::Vector3>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: geometry_msgs/Vector3Stamped
	if (messageType == "geometry_msgs::Vector3Stamped")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<geometry_msgs::Vector3Stamped>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: geometry_msgs/Wrench
	if (messageType == "geometry_msgs::Wrench")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<geometry_msgs::Wrench>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: geometry_msgs/WrenchStamped
	if (messageType == "geometry_msgs::WrenchStamped")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<geometry_msgs::WrenchStamped>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: nav_msgs/GetMapAction
	if (messageType == "nav_msgs::GetMapAction")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<nav_msgs::GetMapAction>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: nav_msgs/GetMapActionFeedback
	if (messageType == "nav_msgs::GetMapActionFeedback")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<nav_msgs::GetMapActionFeedback>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: nav_msgs/GetMapActionGoal
	if (messageType == "nav_msgs::GetMapActionGoal")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<nav_msgs::GetMapActionGoal>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: nav_msgs/GetMapActionResult
	if (messageType == "nav_msgs::GetMapActionResult")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<nav_msgs::GetMapActionResult>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: nav_msgs/GetMapFeedback
	if (messageType == "nav_msgs::GetMapFeedback")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<nav_msgs::GetMapFeedback>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: nav_msgs/GetMapGoal
	if (messageType == "nav_msgs::GetMapGoal")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<nav_msgs::GetMapGoal>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: nav_msgs/GetMapResult
	if (messageType == "nav_msgs::GetMapResult")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<nav_msgs::GetMapResult>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: nav_msgs/GridCells
	if (messageType == "nav_msgs::GridCells")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<nav_msgs::GridCells>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: nav_msgs/MapMetaData
	if (messageType == "nav_msgs::MapMetaData")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<nav_msgs::MapMetaData>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: nav_msgs/OccupancyGrid
	if (messageType == "nav_msgs::OccupancyGrid")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<nav_msgs::OccupancyGrid>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: nav_msgs/Odometry
	if (messageType == "nav_msgs::Odometry")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<nav_msgs::Odometry>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: nav_msgs/Path
	if (messageType == "nav_msgs::Path")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<nav_msgs::Path>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: roscpp/Logger
	if (messageType == "roscpp::Logger")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<roscpp::Logger>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: rosgraph_msgs/Clock
	if (messageType == "rosgraph_msgs::Clock")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<rosgraph_msgs::Clock>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: rosgraph_msgs/Log
	if (messageType == "rosgraph_msgs::Log")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<rosgraph_msgs::Log>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: rosgraph_msgs/TopicStatistics
	if (messageType == "rosgraph_msgs::TopicStatistics")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<rosgraph_msgs::TopicStatistics>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: rospy_tutorials/Floats
	if (messageType == "rospy_tutorials::Floats")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<rospy_tutorials::Floats>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: rospy_tutorials/HeaderString
	if (messageType == "rospy_tutorials::HeaderString")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<rospy_tutorials::HeaderString>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: sensor_msgs/BatteryState
	if (messageType == "sensor_msgs::BatteryState")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<sensor_msgs::BatteryState>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: sensor_msgs/CameraInfo
	if (messageType == "sensor_msgs::CameraInfo")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<sensor_msgs::CameraInfo>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: sensor_msgs/ChannelFloat32
	if (messageType == "sensor_msgs::ChannelFloat32")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<sensor_msgs::ChannelFloat32>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: sensor_msgs/CompressedImage
	if (messageType == "sensor_msgs::CompressedImage")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<sensor_msgs::CompressedImage>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: sensor_msgs/FluidPressure
	if (messageType == "sensor_msgs::FluidPressure")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<sensor_msgs::FluidPressure>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: sensor_msgs/Illuminance
	if (messageType == "sensor_msgs::Illuminance")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<sensor_msgs::Illuminance>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: sensor_msgs/Image
	if (messageType == "sensor_msgs::Image")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<sensor_msgs::Image>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: sensor_msgs/Imu
	if (messageType == "sensor_msgs::Imu")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<sensor_msgs::Imu>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: sensor_msgs/JointState
	if (messageType == "sensor_msgs::JointState")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<sensor_msgs::JointState>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: sensor_msgs/Joy
	if (messageType == "sensor_msgs::Joy")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<sensor_msgs::Joy>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: sensor_msgs/JoyFeedback
	if (messageType == "sensor_msgs::JoyFeedback")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<sensor_msgs::JoyFeedback>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: sensor_msgs/JoyFeedbackArray
	if (messageType == "sensor_msgs::JoyFeedbackArray")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<sensor_msgs::JoyFeedbackArray>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: sensor_msgs/LaserEcho
	if (messageType == "sensor_msgs::LaserEcho")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<sensor_msgs::LaserEcho>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: sensor_msgs/LaserScan
	if (messageType == "sensor_msgs::LaserScan")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<sensor_msgs::LaserScan>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: sensor_msgs/MagneticField
	if (messageType == "sensor_msgs::MagneticField")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<sensor_msgs::MagneticField>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: sensor_msgs/MultiDOFJointState
	if (messageType == "sensor_msgs::MultiDOFJointState")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<sensor_msgs::MultiDOFJointState>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: sensor_msgs/MultiEchoLaserScan
	if (messageType == "sensor_msgs::MultiEchoLaserScan")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<sensor_msgs::MultiEchoLaserScan>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: sensor_msgs/NavSatFix
	if (messageType == "sensor_msgs::NavSatFix")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<sensor_msgs::NavSatFix>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: sensor_msgs/NavSatStatus
	if (messageType == "sensor_msgs::NavSatStatus")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<sensor_msgs::NavSatStatus>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: sensor_msgs/PointCloud
	if (messageType == "sensor_msgs::PointCloud")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<sensor_msgs::PointCloud>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: sensor_msgs/PointCloud2
	if (messageType == "sensor_msgs::PointCloud2")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<sensor_msgs::PointCloud2>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: sensor_msgs/PointField
	if (messageType == "sensor_msgs::PointField")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<sensor_msgs::PointField>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: sensor_msgs/Range
	if (messageType == "sensor_msgs::Range")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<sensor_msgs::Range>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: sensor_msgs/RegionOfInterest
	if (messageType == "sensor_msgs::RegionOfInterest")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<sensor_msgs::RegionOfInterest>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: sensor_msgs/RelativeHumidity
	if (messageType == "sensor_msgs::RelativeHumidity")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<sensor_msgs::RelativeHumidity>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: sensor_msgs/Temperature
	if (messageType == "sensor_msgs::Temperature")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<sensor_msgs::Temperature>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: sensor_msgs/TimeReference
	if (messageType == "sensor_msgs::TimeReference")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<sensor_msgs::TimeReference>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: shape_msgs/Mesh
	if (messageType == "shape_msgs::Mesh")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<shape_msgs::Mesh>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: shape_msgs/MeshTriangle
	if (messageType == "shape_msgs::MeshTriangle")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<shape_msgs::MeshTriangle>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: shape_msgs/Plane
	if (messageType == "shape_msgs::Plane")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<shape_msgs::Plane>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: shape_msgs/SolidPrimitive
	if (messageType == "shape_msgs::SolidPrimitive")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<shape_msgs::SolidPrimitive>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: smach_msgs/SmachContainerInitialStatusCmd
	if (messageType == "smach_msgs::SmachContainerInitialStatusCmd")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<smach_msgs::SmachContainerInitialStatusCmd>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: smach_msgs/SmachContainerStatus
	if (messageType == "smach_msgs::SmachContainerStatus")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<smach_msgs::SmachContainerStatus>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: smach_msgs/SmachContainerStructure
	if (messageType == "smach_msgs::SmachContainerStructure")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<smach_msgs::SmachContainerStructure>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: sofa_softrobots_msgs/BodyTransforms
	if (messageType == "sofa_softrobots_msgs::BodyTransforms")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<sofa_softrobots_msgs::BodyTransforms>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: std_msgs/Bool
	if (messageType == "std_msgs::Bool")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<std_msgs::Bool>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: std_msgs/Byte
	if (messageType == "std_msgs::Byte")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<std_msgs::Byte>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: std_msgs/ByteMultiArray
	if (messageType == "std_msgs::ByteMultiArray")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<std_msgs::ByteMultiArray>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: std_msgs/Char
	if (messageType == "std_msgs::Char")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<std_msgs::Char>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: std_msgs/ColorRGBA
	if (messageType == "std_msgs::ColorRGBA")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<std_msgs::ColorRGBA>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: std_msgs/Duration
	if (messageType == "std_msgs::Duration")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<std_msgs::Duration>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: std_msgs/Empty
	if (messageType == "std_msgs::Empty")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<std_msgs::Empty>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: std_msgs/Float32
	if (messageType == "std_msgs::Float32")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<std_msgs::Float32>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: std_msgs/Float32MultiArray
	if (messageType == "std_msgs::Float32MultiArray")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<std_msgs::Float32MultiArray>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: std_msgs/Float64
	if (messageType == "std_msgs::Float64")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<std_msgs::Float64>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: std_msgs/Float64MultiArray
	if (messageType == "std_msgs::Float64MultiArray")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<std_msgs::Float64MultiArray>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: std_msgs/Header
	if (messageType == "std_msgs::Header")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<std_msgs::Header>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: std_msgs/Int16
	if (messageType == "std_msgs::Int16")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<std_msgs::Int16>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: std_msgs/Int16MultiArray
	if (messageType == "std_msgs::Int16MultiArray")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<std_msgs::Int16MultiArray>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: std_msgs/Int32
	if (messageType == "std_msgs::Int32")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<std_msgs::Int32>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: std_msgs/Int32MultiArray
	if (messageType == "std_msgs::Int32MultiArray")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<std_msgs::Int32MultiArray>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: std_msgs/Int64
	if (messageType == "std_msgs::Int64")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<std_msgs::Int64>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: std_msgs/Int64MultiArray
	if (messageType == "std_msgs::Int64MultiArray")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<std_msgs::Int64MultiArray>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: std_msgs/Int8
	if (messageType == "std_msgs::Int8")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<std_msgs::Int8>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: std_msgs/Int8MultiArray
	if (messageType == "std_msgs::Int8MultiArray")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<std_msgs::Int8MultiArray>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: std_msgs/MultiArrayDimension
	if (messageType == "std_msgs::MultiArrayDimension")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<std_msgs::MultiArrayDimension>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: std_msgs/MultiArrayLayout
	if (messageType == "std_msgs::MultiArrayLayout")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<std_msgs::MultiArrayLayout>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: std_msgs/String
	if (messageType == "std_msgs::String")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<std_msgs::String>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: std_msgs/Time
	if (messageType == "std_msgs::Time")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<std_msgs::Time>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: std_msgs/UInt16
	if (messageType == "std_msgs::UInt16")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<std_msgs::UInt16>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: std_msgs/UInt16MultiArray
	if (messageType == "std_msgs::UInt16MultiArray")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<std_msgs::UInt16MultiArray>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: std_msgs/UInt32
	if (messageType == "std_msgs::UInt32")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<std_msgs::UInt32>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: std_msgs/UInt32MultiArray
	if (messageType == "std_msgs::UInt32MultiArray")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<std_msgs::UInt32MultiArray>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: std_msgs/UInt64
	if (messageType == "std_msgs::UInt64")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<std_msgs::UInt64>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: std_msgs/UInt64MultiArray
	if (messageType == "std_msgs::UInt64MultiArray")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<std_msgs::UInt64MultiArray>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: std_msgs/UInt8
	if (messageType == "std_msgs::UInt8")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<std_msgs::UInt8>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: std_msgs/UInt8MultiArray
	if (messageType == "std_msgs::UInt8MultiArray")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<std_msgs::UInt8MultiArray>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: stereo_msgs/DisparityImage
	if (messageType == "stereo_msgs::DisparityImage")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<stereo_msgs::DisparityImage>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: tf/tfMessage
	if (messageType == "tf::tfMessage")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<tf::tfMessage>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: tf2_msgs/LookupTransformAction
	if (messageType == "tf2_msgs::LookupTransformAction")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<tf2_msgs::LookupTransformAction>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: tf2_msgs/LookupTransformActionFeedback
	if (messageType == "tf2_msgs::LookupTransformActionFeedback")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<tf2_msgs::LookupTransformActionFeedback>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: tf2_msgs/LookupTransformActionGoal
	if (messageType == "tf2_msgs::LookupTransformActionGoal")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<tf2_msgs::LookupTransformActionGoal>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: tf2_msgs/LookupTransformActionResult
	if (messageType == "tf2_msgs::LookupTransformActionResult")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<tf2_msgs::LookupTransformActionResult>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: tf2_msgs/LookupTransformFeedback
	if (messageType == "tf2_msgs::LookupTransformFeedback")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<tf2_msgs::LookupTransformFeedback>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: tf2_msgs/LookupTransformGoal
	if (messageType == "tf2_msgs::LookupTransformGoal")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<tf2_msgs::LookupTransformGoal>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: tf2_msgs/LookupTransformResult
	if (messageType == "tf2_msgs::LookupTransformResult")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<tf2_msgs::LookupTransformResult>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: tf2_msgs/TF2Error
	if (messageType == "tf2_msgs::TF2Error")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<tf2_msgs::TF2Error>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: tf2_msgs/TFMessage
	if (messageType == "tf2_msgs::TFMessage")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<tf2_msgs::TFMessage>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: trajectory_msgs/JointTrajectory
	if (messageType == "trajectory_msgs::JointTrajectory")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<trajectory_msgs::JointTrajectory>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: trajectory_msgs/JointTrajectoryPoint
	if (messageType == "trajectory_msgs::JointTrajectoryPoint")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<trajectory_msgs::JointTrajectoryPoint>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: trajectory_msgs/MultiDOFJointTrajectory
	if (messageType == "trajectory_msgs::MultiDOFJointTrajectory")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<trajectory_msgs::MultiDOFJointTrajectory>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: trajectory_msgs/MultiDOFJointTrajectoryPoint
	if (messageType == "trajectory_msgs::MultiDOFJointTrajectoryPoint")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<trajectory_msgs::MultiDOFJointTrajectoryPoint>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: visualization_msgs/ImageMarker
	if (messageType == "visualization_msgs::ImageMarker")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<visualization_msgs::ImageMarker>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: visualization_msgs/InteractiveMarker
	if (messageType == "visualization_msgs::InteractiveMarker")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<visualization_msgs::InteractiveMarker>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: visualization_msgs/InteractiveMarkerControl
	if (messageType == "visualization_msgs::InteractiveMarkerControl")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<visualization_msgs::InteractiveMarkerControl>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: visualization_msgs/InteractiveMarkerFeedback
	if (messageType == "visualization_msgs::InteractiveMarkerFeedback")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<visualization_msgs::InteractiveMarkerFeedback>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: visualization_msgs/InteractiveMarkerInit
	if (messageType == "visualization_msgs::InteractiveMarkerInit")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<visualization_msgs::InteractiveMarkerInit>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: visualization_msgs/InteractiveMarkerPose
	if (messageType == "visualization_msgs::InteractiveMarkerPose")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<visualization_msgs::InteractiveMarkerPose>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: visualization_msgs/InteractiveMarkerUpdate
	if (messageType == "visualization_msgs::InteractiveMarkerUpdate")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<visualization_msgs::InteractiveMarkerUpdate>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: visualization_msgs/Marker
	if (messageType == "visualization_msgs::Marker")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<visualization_msgs::Marker>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: visualization_msgs/MarkerArray
	if (messageType == "visualization_msgs::MarkerArray")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<visualization_msgs::MarkerArray>(rosNode, topicURI, 50, true));
	}
	// Subscriber instance for ROS message type: visualization_msgs/MenuEntry
	if (messageType == "visualization_msgs::MenuEntry")
	{
		supported = true;
		topicListener.reset(new ZyROSConnectorTopicSubscriber<visualization_msgs::MenuEntry>(rosNode, topicURI, 50, true));
	}
	if (supported)
	{
		msg_info("ZyROSConnectorMessageSubscriberFactory") << "ROS message type supported: " << messageType;
	}
	else
	{
		msg_warning("ZyROSConnectorMessageSubscriberFactory") << "ROS message type NOT supported: " << messageType;
	}	return topicListener;
}

