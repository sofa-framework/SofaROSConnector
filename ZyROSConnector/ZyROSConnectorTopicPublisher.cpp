/***********************************************************************
ROS message definition headers and ROS connector template instantiations.
This file is AUTO-GENERATED during the CMake run.
Please do not modify it by hand.
The contents will be overwritten and re-generated.
************************************************************************/

#include "ZyROSConnectorTopicPublisher.inl"

using namespace Zyklio::ROSConnector;

ZyROSPublisher::ZyROSPublisher() : m_uuid(boost::uuids::random_generator()())
{

}

ZyROSPublisher::ZyROSPublisher(const ZyROSPublisher& other)
{
    if (this != &other)
    {
        m_uuid = other.m_uuid;
        messageType = other.messageType;
    }
}

ZyROSPublisher& ZyROSPublisher::operator=(const ZyROSPublisher& other)
{
    if (this != &other)
    {
        m_uuid = other.m_uuid;
        messageType = other.messageType;
    }
    return *this;
}

template <class MessageType>
void ZyROSConnectorTopicPublisher<MessageType>::publishMessageQueue()
{
    boost::mutex::scoped_lock lock(m_mutex);

    if (!m_messageQueue.empty())
    {
        msg_info("ZyROSConnectorTopicPublisher") << "publishMessageQueue of size " << m_messageQueue.size();
        while (!m_messageQueue.empty())
        {
            MessageType& msg = m_messageQueue.back();
            m_publisher.publish(msg);
            m_messageQueue.pop_back();
        }
    }
    lock.unlock();
}
#include <ZyROS_MessageType_Instantiations_Publishers.h>


using namespace Zyklio::ROSConnector;
// Publisher and subscriber proxy class instantiation for ROS message type: actionlib/TestAction
template class ZyROSConnectorTopicPublisher<actionlib::TestAction>;

// Publisher and subscriber proxy class instantiation for ROS message type: actionlib/TestActionFeedback
template class ZyROSConnectorTopicPublisher<actionlib::TestActionFeedback>;

// Publisher and subscriber proxy class instantiation for ROS message type: actionlib/TestActionGoal
template class ZyROSConnectorTopicPublisher<actionlib::TestActionGoal>;

// Publisher and subscriber proxy class instantiation for ROS message type: actionlib/TestActionResult
template class ZyROSConnectorTopicPublisher<actionlib::TestActionResult>;

// Publisher and subscriber proxy class instantiation for ROS message type: actionlib/TestFeedback
template class ZyROSConnectorTopicPublisher<actionlib::TestFeedback>;

// Publisher and subscriber proxy class instantiation for ROS message type: actionlib/TestGoal
template class ZyROSConnectorTopicPublisher<actionlib::TestGoal>;

// Publisher and subscriber proxy class instantiation for ROS message type: actionlib/TestRequestAction
template class ZyROSConnectorTopicPublisher<actionlib::TestRequestAction>;

// Publisher and subscriber proxy class instantiation for ROS message type: actionlib/TestRequestActionFeedback
template class ZyROSConnectorTopicPublisher<actionlib::TestRequestActionFeedback>;

// Publisher and subscriber proxy class instantiation for ROS message type: actionlib/TestRequestActionGoal
template class ZyROSConnectorTopicPublisher<actionlib::TestRequestActionGoal>;

// Publisher and subscriber proxy class instantiation for ROS message type: actionlib/TestRequestActionResult
template class ZyROSConnectorTopicPublisher<actionlib::TestRequestActionResult>;

// Publisher and subscriber proxy class instantiation for ROS message type: actionlib/TestRequestFeedback
template class ZyROSConnectorTopicPublisher<actionlib::TestRequestFeedback>;

// Publisher and subscriber proxy class instantiation for ROS message type: actionlib/TestRequestGoal
template class ZyROSConnectorTopicPublisher<actionlib::TestRequestGoal>;

// Publisher and subscriber proxy class instantiation for ROS message type: actionlib/TestRequestResult
template class ZyROSConnectorTopicPublisher<actionlib::TestRequestResult>;

// Publisher and subscriber proxy class instantiation for ROS message type: actionlib/TestResult
template class ZyROSConnectorTopicPublisher<actionlib::TestResult>;

// Publisher and subscriber proxy class instantiation for ROS message type: actionlib/TwoIntsAction
template class ZyROSConnectorTopicPublisher<actionlib::TwoIntsAction>;

// Publisher and subscriber proxy class instantiation for ROS message type: actionlib/TwoIntsActionFeedback
template class ZyROSConnectorTopicPublisher<actionlib::TwoIntsActionFeedback>;

// Publisher and subscriber proxy class instantiation for ROS message type: actionlib/TwoIntsActionGoal
template class ZyROSConnectorTopicPublisher<actionlib::TwoIntsActionGoal>;

// Publisher and subscriber proxy class instantiation for ROS message type: actionlib/TwoIntsActionResult
template class ZyROSConnectorTopicPublisher<actionlib::TwoIntsActionResult>;

// Publisher and subscriber proxy class instantiation for ROS message type: actionlib/TwoIntsFeedback
template class ZyROSConnectorTopicPublisher<actionlib::TwoIntsFeedback>;

// Publisher and subscriber proxy class instantiation for ROS message type: actionlib/TwoIntsGoal
template class ZyROSConnectorTopicPublisher<actionlib::TwoIntsGoal>;

// Publisher and subscriber proxy class instantiation for ROS message type: actionlib/TwoIntsResult
template class ZyROSConnectorTopicPublisher<actionlib::TwoIntsResult>;

// Publisher and subscriber proxy class instantiation for ROS message type: actionlib_msgs/GoalID
template class ZyROSConnectorTopicPublisher<actionlib_msgs::GoalID>;

// Publisher and subscriber proxy class instantiation for ROS message type: actionlib_msgs/GoalStatus
template class ZyROSConnectorTopicPublisher<actionlib_msgs::GoalStatus>;

// Publisher and subscriber proxy class instantiation for ROS message type: actionlib_msgs/GoalStatusArray
template class ZyROSConnectorTopicPublisher<actionlib_msgs::GoalStatusArray>;

// Publisher and subscriber proxy class instantiation for ROS message type: bond/Constants
template class ZyROSConnectorTopicPublisher<bond::Constants>;

// Publisher and subscriber proxy class instantiation for ROS message type: bond/Status
template class ZyROSConnectorTopicPublisher<bond::Status>;

// Publisher and subscriber proxy class instantiation for ROS message type: control_msgs/FollowJointTrajectoryAction
template class ZyROSConnectorTopicPublisher<control_msgs::FollowJointTrajectoryAction>;

// Publisher and subscriber proxy class instantiation for ROS message type: control_msgs/FollowJointTrajectoryActionFeedback
template class ZyROSConnectorTopicPublisher<control_msgs::FollowJointTrajectoryActionFeedback>;

// Publisher and subscriber proxy class instantiation for ROS message type: control_msgs/FollowJointTrajectoryActionGoal
template class ZyROSConnectorTopicPublisher<control_msgs::FollowJointTrajectoryActionGoal>;

// Publisher and subscriber proxy class instantiation for ROS message type: control_msgs/FollowJointTrajectoryActionResult
template class ZyROSConnectorTopicPublisher<control_msgs::FollowJointTrajectoryActionResult>;

// Publisher and subscriber proxy class instantiation for ROS message type: control_msgs/FollowJointTrajectoryFeedback
template class ZyROSConnectorTopicPublisher<control_msgs::FollowJointTrajectoryFeedback>;

// Publisher and subscriber proxy class instantiation for ROS message type: control_msgs/FollowJointTrajectoryGoal
template class ZyROSConnectorTopicPublisher<control_msgs::FollowJointTrajectoryGoal>;

// Publisher and subscriber proxy class instantiation for ROS message type: control_msgs/FollowJointTrajectoryResult
template class ZyROSConnectorTopicPublisher<control_msgs::FollowJointTrajectoryResult>;

// Publisher and subscriber proxy class instantiation for ROS message type: control_msgs/GripperCommand
template class ZyROSConnectorTopicPublisher<control_msgs::GripperCommand>;

// Publisher and subscriber proxy class instantiation for ROS message type: control_msgs/GripperCommandAction
template class ZyROSConnectorTopicPublisher<control_msgs::GripperCommandAction>;

// Publisher and subscriber proxy class instantiation for ROS message type: control_msgs/GripperCommandActionFeedback
template class ZyROSConnectorTopicPublisher<control_msgs::GripperCommandActionFeedback>;

// Publisher and subscriber proxy class instantiation for ROS message type: control_msgs/GripperCommandActionGoal
template class ZyROSConnectorTopicPublisher<control_msgs::GripperCommandActionGoal>;

// Publisher and subscriber proxy class instantiation for ROS message type: control_msgs/GripperCommandActionResult
template class ZyROSConnectorTopicPublisher<control_msgs::GripperCommandActionResult>;

// Publisher and subscriber proxy class instantiation for ROS message type: control_msgs/GripperCommandFeedback
template class ZyROSConnectorTopicPublisher<control_msgs::GripperCommandFeedback>;

// Publisher and subscriber proxy class instantiation for ROS message type: control_msgs/GripperCommandGoal
template class ZyROSConnectorTopicPublisher<control_msgs::GripperCommandGoal>;

// Publisher and subscriber proxy class instantiation for ROS message type: control_msgs/GripperCommandResult
template class ZyROSConnectorTopicPublisher<control_msgs::GripperCommandResult>;

// Publisher and subscriber proxy class instantiation for ROS message type: control_msgs/JointControllerState
template class ZyROSConnectorTopicPublisher<control_msgs::JointControllerState>;

// Publisher and subscriber proxy class instantiation for ROS message type: control_msgs/JointJog
template class ZyROSConnectorTopicPublisher<control_msgs::JointJog>;

// Publisher and subscriber proxy class instantiation for ROS message type: control_msgs/JointTolerance
template class ZyROSConnectorTopicPublisher<control_msgs::JointTolerance>;

// Publisher and subscriber proxy class instantiation for ROS message type: control_msgs/JointTrajectoryAction
template class ZyROSConnectorTopicPublisher<control_msgs::JointTrajectoryAction>;

// Publisher and subscriber proxy class instantiation for ROS message type: control_msgs/JointTrajectoryActionFeedback
template class ZyROSConnectorTopicPublisher<control_msgs::JointTrajectoryActionFeedback>;

// Publisher and subscriber proxy class instantiation for ROS message type: control_msgs/JointTrajectoryActionGoal
template class ZyROSConnectorTopicPublisher<control_msgs::JointTrajectoryActionGoal>;

// Publisher and subscriber proxy class instantiation for ROS message type: control_msgs/JointTrajectoryActionResult
template class ZyROSConnectorTopicPublisher<control_msgs::JointTrajectoryActionResult>;

// Publisher and subscriber proxy class instantiation for ROS message type: control_msgs/JointTrajectoryControllerState
template class ZyROSConnectorTopicPublisher<control_msgs::JointTrajectoryControllerState>;

// Publisher and subscriber proxy class instantiation for ROS message type: control_msgs/JointTrajectoryFeedback
template class ZyROSConnectorTopicPublisher<control_msgs::JointTrajectoryFeedback>;

// Publisher and subscriber proxy class instantiation for ROS message type: control_msgs/JointTrajectoryGoal
template class ZyROSConnectorTopicPublisher<control_msgs::JointTrajectoryGoal>;

// Publisher and subscriber proxy class instantiation for ROS message type: control_msgs/JointTrajectoryResult
template class ZyROSConnectorTopicPublisher<control_msgs::JointTrajectoryResult>;

// Publisher and subscriber proxy class instantiation for ROS message type: control_msgs/PidState
template class ZyROSConnectorTopicPublisher<control_msgs::PidState>;

// Publisher and subscriber proxy class instantiation for ROS message type: control_msgs/PointHeadAction
template class ZyROSConnectorTopicPublisher<control_msgs::PointHeadAction>;

// Publisher and subscriber proxy class instantiation for ROS message type: control_msgs/PointHeadActionFeedback
template class ZyROSConnectorTopicPublisher<control_msgs::PointHeadActionFeedback>;

// Publisher and subscriber proxy class instantiation for ROS message type: control_msgs/PointHeadActionGoal
template class ZyROSConnectorTopicPublisher<control_msgs::PointHeadActionGoal>;

// Publisher and subscriber proxy class instantiation for ROS message type: control_msgs/PointHeadActionResult
template class ZyROSConnectorTopicPublisher<control_msgs::PointHeadActionResult>;

// Publisher and subscriber proxy class instantiation for ROS message type: control_msgs/PointHeadFeedback
template class ZyROSConnectorTopicPublisher<control_msgs::PointHeadFeedback>;

// Publisher and subscriber proxy class instantiation for ROS message type: control_msgs/PointHeadGoal
template class ZyROSConnectorTopicPublisher<control_msgs::PointHeadGoal>;

// Publisher and subscriber proxy class instantiation for ROS message type: control_msgs/PointHeadResult
template class ZyROSConnectorTopicPublisher<control_msgs::PointHeadResult>;

// Publisher and subscriber proxy class instantiation for ROS message type: control_msgs/SingleJointPositionAction
template class ZyROSConnectorTopicPublisher<control_msgs::SingleJointPositionAction>;

// Publisher and subscriber proxy class instantiation for ROS message type: control_msgs/SingleJointPositionActionFeedback
template class ZyROSConnectorTopicPublisher<control_msgs::SingleJointPositionActionFeedback>;

// Publisher and subscriber proxy class instantiation for ROS message type: control_msgs/SingleJointPositionActionGoal
template class ZyROSConnectorTopicPublisher<control_msgs::SingleJointPositionActionGoal>;

// Publisher and subscriber proxy class instantiation for ROS message type: control_msgs/SingleJointPositionActionResult
template class ZyROSConnectorTopicPublisher<control_msgs::SingleJointPositionActionResult>;

// Publisher and subscriber proxy class instantiation for ROS message type: control_msgs/SingleJointPositionFeedback
template class ZyROSConnectorTopicPublisher<control_msgs::SingleJointPositionFeedback>;

// Publisher and subscriber proxy class instantiation for ROS message type: control_msgs/SingleJointPositionGoal
template class ZyROSConnectorTopicPublisher<control_msgs::SingleJointPositionGoal>;

// Publisher and subscriber proxy class instantiation for ROS message type: control_msgs/SingleJointPositionResult
template class ZyROSConnectorTopicPublisher<control_msgs::SingleJointPositionResult>;

// Publisher and subscriber proxy class instantiation for ROS message type: diagnostic_msgs/DiagnosticArray
template class ZyROSConnectorTopicPublisher<diagnostic_msgs::DiagnosticArray>;

// Publisher and subscriber proxy class instantiation for ROS message type: diagnostic_msgs/DiagnosticStatus
template class ZyROSConnectorTopicPublisher<diagnostic_msgs::DiagnosticStatus>;

// Publisher and subscriber proxy class instantiation for ROS message type: diagnostic_msgs/KeyValue
template class ZyROSConnectorTopicPublisher<diagnostic_msgs::KeyValue>;

// Publisher and subscriber proxy class instantiation for ROS message type: dynamic_reconfigure/BoolParameter
template class ZyROSConnectorTopicPublisher<dynamic_reconfigure::BoolParameter>;

// Publisher and subscriber proxy class instantiation for ROS message type: dynamic_reconfigure/Config
template class ZyROSConnectorTopicPublisher<dynamic_reconfigure::Config>;

// Publisher and subscriber proxy class instantiation for ROS message type: dynamic_reconfigure/ConfigDescription
template class ZyROSConnectorTopicPublisher<dynamic_reconfigure::ConfigDescription>;

// Publisher and subscriber proxy class instantiation for ROS message type: dynamic_reconfigure/DoubleParameter
template class ZyROSConnectorTopicPublisher<dynamic_reconfigure::DoubleParameter>;

// Publisher and subscriber proxy class instantiation for ROS message type: dynamic_reconfigure/Group
template class ZyROSConnectorTopicPublisher<dynamic_reconfigure::Group>;

// Publisher and subscriber proxy class instantiation for ROS message type: dynamic_reconfigure/GroupState
template class ZyROSConnectorTopicPublisher<dynamic_reconfigure::GroupState>;

// Publisher and subscriber proxy class instantiation for ROS message type: dynamic_reconfigure/IntParameter
template class ZyROSConnectorTopicPublisher<dynamic_reconfigure::IntParameter>;

// Publisher and subscriber proxy class instantiation for ROS message type: dynamic_reconfigure/ParamDescription
template class ZyROSConnectorTopicPublisher<dynamic_reconfigure::ParamDescription>;

// Publisher and subscriber proxy class instantiation for ROS message type: dynamic_reconfigure/SensorLevels
template class ZyROSConnectorTopicPublisher<dynamic_reconfigure::SensorLevels>;

// Publisher and subscriber proxy class instantiation for ROS message type: dynamic_reconfigure/StrParameter
template class ZyROSConnectorTopicPublisher<dynamic_reconfigure::StrParameter>;

// Publisher and subscriber proxy class instantiation for ROS message type: geometry_msgs/Accel
template class ZyROSConnectorTopicPublisher<geometry_msgs::Accel>;

// Publisher and subscriber proxy class instantiation for ROS message type: geometry_msgs/AccelStamped
template class ZyROSConnectorTopicPublisher<geometry_msgs::AccelStamped>;

// Publisher and subscriber proxy class instantiation for ROS message type: geometry_msgs/AccelWithCovariance
template class ZyROSConnectorTopicPublisher<geometry_msgs::AccelWithCovariance>;

// Publisher and subscriber proxy class instantiation for ROS message type: geometry_msgs/AccelWithCovarianceStamped
template class ZyROSConnectorTopicPublisher<geometry_msgs::AccelWithCovarianceStamped>;

// Publisher and subscriber proxy class instantiation for ROS message type: geometry_msgs/Inertia
template class ZyROSConnectorTopicPublisher<geometry_msgs::Inertia>;

// Publisher and subscriber proxy class instantiation for ROS message type: geometry_msgs/InertiaStamped
template class ZyROSConnectorTopicPublisher<geometry_msgs::InertiaStamped>;

// Publisher and subscriber proxy class instantiation for ROS message type: geometry_msgs/Point
template class ZyROSConnectorTopicPublisher<geometry_msgs::Point>;

// Publisher and subscriber proxy class instantiation for ROS message type: geometry_msgs/Point32
template class ZyROSConnectorTopicPublisher<geometry_msgs::Point32>;

// Publisher and subscriber proxy class instantiation for ROS message type: geometry_msgs/PointStamped
template class ZyROSConnectorTopicPublisher<geometry_msgs::PointStamped>;

// Publisher and subscriber proxy class instantiation for ROS message type: geometry_msgs/Polygon
template class ZyROSConnectorTopicPublisher<geometry_msgs::Polygon>;

// Publisher and subscriber proxy class instantiation for ROS message type: geometry_msgs/PolygonStamped
template class ZyROSConnectorTopicPublisher<geometry_msgs::PolygonStamped>;

// Publisher and subscriber proxy class instantiation for ROS message type: geometry_msgs/Pose
template class ZyROSConnectorTopicPublisher<geometry_msgs::Pose>;

// Publisher and subscriber proxy class instantiation for ROS message type: geometry_msgs/Pose2D
template class ZyROSConnectorTopicPublisher<geometry_msgs::Pose2D>;

// Publisher and subscriber proxy class instantiation for ROS message type: geometry_msgs/PoseArray
template class ZyROSConnectorTopicPublisher<geometry_msgs::PoseArray>;

// Publisher and subscriber proxy class instantiation for ROS message type: geometry_msgs/PoseStamped
template class ZyROSConnectorTopicPublisher<geometry_msgs::PoseStamped>;

// Publisher and subscriber proxy class instantiation for ROS message type: geometry_msgs/PoseWithCovariance
template class ZyROSConnectorTopicPublisher<geometry_msgs::PoseWithCovariance>;

// Publisher and subscriber proxy class instantiation for ROS message type: geometry_msgs/PoseWithCovarianceStamped
template class ZyROSConnectorTopicPublisher<geometry_msgs::PoseWithCovarianceStamped>;

// Publisher and subscriber proxy class instantiation for ROS message type: geometry_msgs/Quaternion
template class ZyROSConnectorTopicPublisher<geometry_msgs::Quaternion>;

// Publisher and subscriber proxy class instantiation for ROS message type: geometry_msgs/QuaternionStamped
template class ZyROSConnectorTopicPublisher<geometry_msgs::QuaternionStamped>;

// Publisher and subscriber proxy class instantiation for ROS message type: geometry_msgs/Transform
template class ZyROSConnectorTopicPublisher<geometry_msgs::Transform>;

// Publisher and subscriber proxy class instantiation for ROS message type: geometry_msgs/TransformStamped
template class ZyROSConnectorTopicPublisher<geometry_msgs::TransformStamped>;

// Publisher and subscriber proxy class instantiation for ROS message type: geometry_msgs/Twist
template class ZyROSConnectorTopicPublisher<geometry_msgs::Twist>;

// Publisher and subscriber proxy class instantiation for ROS message type: geometry_msgs/TwistStamped
template class ZyROSConnectorTopicPublisher<geometry_msgs::TwistStamped>;

// Publisher and subscriber proxy class instantiation for ROS message type: geometry_msgs/TwistWithCovariance
template class ZyROSConnectorTopicPublisher<geometry_msgs::TwistWithCovariance>;

// Publisher and subscriber proxy class instantiation for ROS message type: geometry_msgs/TwistWithCovarianceStamped
template class ZyROSConnectorTopicPublisher<geometry_msgs::TwistWithCovarianceStamped>;

// Publisher and subscriber proxy class instantiation for ROS message type: geometry_msgs/Vector3
template class ZyROSConnectorTopicPublisher<geometry_msgs::Vector3>;

// Publisher and subscriber proxy class instantiation for ROS message type: geometry_msgs/Vector3Stamped
template class ZyROSConnectorTopicPublisher<geometry_msgs::Vector3Stamped>;

// Publisher and subscriber proxy class instantiation for ROS message type: geometry_msgs/Wrench
template class ZyROSConnectorTopicPublisher<geometry_msgs::Wrench>;

// Publisher and subscriber proxy class instantiation for ROS message type: geometry_msgs/WrenchStamped
template class ZyROSConnectorTopicPublisher<geometry_msgs::WrenchStamped>;

// Publisher and subscriber proxy class instantiation for ROS message type: nav_msgs/GetMapAction
template class ZyROSConnectorTopicPublisher<nav_msgs::GetMapAction>;

// Publisher and subscriber proxy class instantiation for ROS message type: nav_msgs/GetMapActionFeedback
template class ZyROSConnectorTopicPublisher<nav_msgs::GetMapActionFeedback>;

// Publisher and subscriber proxy class instantiation for ROS message type: nav_msgs/GetMapActionGoal
template class ZyROSConnectorTopicPublisher<nav_msgs::GetMapActionGoal>;

// Publisher and subscriber proxy class instantiation for ROS message type: nav_msgs/GetMapActionResult
template class ZyROSConnectorTopicPublisher<nav_msgs::GetMapActionResult>;

// Publisher and subscriber proxy class instantiation for ROS message type: nav_msgs/GetMapFeedback
template class ZyROSConnectorTopicPublisher<nav_msgs::GetMapFeedback>;

// Publisher and subscriber proxy class instantiation for ROS message type: nav_msgs/GetMapGoal
template class ZyROSConnectorTopicPublisher<nav_msgs::GetMapGoal>;

// Publisher and subscriber proxy class instantiation for ROS message type: nav_msgs/GetMapResult
template class ZyROSConnectorTopicPublisher<nav_msgs::GetMapResult>;

// Publisher and subscriber proxy class instantiation for ROS message type: nav_msgs/GridCells
template class ZyROSConnectorTopicPublisher<nav_msgs::GridCells>;

// Publisher and subscriber proxy class instantiation for ROS message type: nav_msgs/MapMetaData
template class ZyROSConnectorTopicPublisher<nav_msgs::MapMetaData>;

// Publisher and subscriber proxy class instantiation for ROS message type: nav_msgs/OccupancyGrid
template class ZyROSConnectorTopicPublisher<nav_msgs::OccupancyGrid>;

// Publisher and subscriber proxy class instantiation for ROS message type: nav_msgs/Odometry
template class ZyROSConnectorTopicPublisher<nav_msgs::Odometry>;

// Publisher and subscriber proxy class instantiation for ROS message type: nav_msgs/Path
template class ZyROSConnectorTopicPublisher<nav_msgs::Path>;

// Publisher and subscriber proxy class instantiation for ROS message type: roscpp/Logger
template class ZyROSConnectorTopicPublisher<roscpp::Logger>;

// Publisher and subscriber proxy class instantiation for ROS message type: rosgraph_msgs/Clock
template class ZyROSConnectorTopicPublisher<rosgraph_msgs::Clock>;

// Publisher and subscriber proxy class instantiation for ROS message type: rosgraph_msgs/Log
template class ZyROSConnectorTopicPublisher<rosgraph_msgs::Log>;

// Publisher and subscriber proxy class instantiation for ROS message type: rosgraph_msgs/TopicStatistics
template class ZyROSConnectorTopicPublisher<rosgraph_msgs::TopicStatistics>;

// Publisher and subscriber proxy class instantiation for ROS message type: rospy_tutorials/Floats
template class ZyROSConnectorTopicPublisher<rospy_tutorials::Floats>;

// Publisher and subscriber proxy class instantiation for ROS message type: rospy_tutorials/HeaderString
template class ZyROSConnectorTopicPublisher<rospy_tutorials::HeaderString>;

// Publisher and subscriber proxy class instantiation for ROS message type: sensor_msgs/BatteryState
template class ZyROSConnectorTopicPublisher<sensor_msgs::BatteryState>;

// Publisher and subscriber proxy class instantiation for ROS message type: sensor_msgs/CameraInfo
template class ZyROSConnectorTopicPublisher<sensor_msgs::CameraInfo>;

// Publisher and subscriber proxy class instantiation for ROS message type: sensor_msgs/ChannelFloat32
template class ZyROSConnectorTopicPublisher<sensor_msgs::ChannelFloat32>;

// Publisher and subscriber proxy class instantiation for ROS message type: sensor_msgs/CompressedImage
template class ZyROSConnectorTopicPublisher<sensor_msgs::CompressedImage>;

// Publisher and subscriber proxy class instantiation for ROS message type: sensor_msgs/FluidPressure
template class ZyROSConnectorTopicPublisher<sensor_msgs::FluidPressure>;

// Publisher and subscriber proxy class instantiation for ROS message type: sensor_msgs/Illuminance
template class ZyROSConnectorTopicPublisher<sensor_msgs::Illuminance>;

// Publisher and subscriber proxy class instantiation for ROS message type: sensor_msgs/Image
template class ZyROSConnectorTopicPublisher<sensor_msgs::Image>;

// Publisher and subscriber proxy class instantiation for ROS message type: sensor_msgs/Imu
template class ZyROSConnectorTopicPublisher<sensor_msgs::Imu>;

// Publisher and subscriber proxy class instantiation for ROS message type: sensor_msgs/JointState
template class ZyROSConnectorTopicPublisher<sensor_msgs::JointState>;

// Publisher and subscriber proxy class instantiation for ROS message type: sensor_msgs/Joy
template class ZyROSConnectorTopicPublisher<sensor_msgs::Joy>;

// Publisher and subscriber proxy class instantiation for ROS message type: sensor_msgs/JoyFeedback
template class ZyROSConnectorTopicPublisher<sensor_msgs::JoyFeedback>;

// Publisher and subscriber proxy class instantiation for ROS message type: sensor_msgs/JoyFeedbackArray
template class ZyROSConnectorTopicPublisher<sensor_msgs::JoyFeedbackArray>;

// Publisher and subscriber proxy class instantiation for ROS message type: sensor_msgs/LaserEcho
template class ZyROSConnectorTopicPublisher<sensor_msgs::LaserEcho>;

// Publisher and subscriber proxy class instantiation for ROS message type: sensor_msgs/LaserScan
template class ZyROSConnectorTopicPublisher<sensor_msgs::LaserScan>;

// Publisher and subscriber proxy class instantiation for ROS message type: sensor_msgs/MagneticField
template class ZyROSConnectorTopicPublisher<sensor_msgs::MagneticField>;

// Publisher and subscriber proxy class instantiation for ROS message type: sensor_msgs/MultiDOFJointState
template class ZyROSConnectorTopicPublisher<sensor_msgs::MultiDOFJointState>;

// Publisher and subscriber proxy class instantiation for ROS message type: sensor_msgs/MultiEchoLaserScan
template class ZyROSConnectorTopicPublisher<sensor_msgs::MultiEchoLaserScan>;

// Publisher and subscriber proxy class instantiation for ROS message type: sensor_msgs/NavSatFix
template class ZyROSConnectorTopicPublisher<sensor_msgs::NavSatFix>;

// Publisher and subscriber proxy class instantiation for ROS message type: sensor_msgs/NavSatStatus
template class ZyROSConnectorTopicPublisher<sensor_msgs::NavSatStatus>;

// Publisher and subscriber proxy class instantiation for ROS message type: sensor_msgs/PointCloud
template class ZyROSConnectorTopicPublisher<sensor_msgs::PointCloud>;

// Publisher and subscriber proxy class instantiation for ROS message type: sensor_msgs/PointCloud2
template class ZyROSConnectorTopicPublisher<sensor_msgs::PointCloud2>;

// Publisher and subscriber proxy class instantiation for ROS message type: sensor_msgs/PointField
template class ZyROSConnectorTopicPublisher<sensor_msgs::PointField>;

// Publisher and subscriber proxy class instantiation for ROS message type: sensor_msgs/Range
template class ZyROSConnectorTopicPublisher<sensor_msgs::Range>;

// Publisher and subscriber proxy class instantiation for ROS message type: sensor_msgs/RegionOfInterest
template class ZyROSConnectorTopicPublisher<sensor_msgs::RegionOfInterest>;

// Publisher and subscriber proxy class instantiation for ROS message type: sensor_msgs/RelativeHumidity
template class ZyROSConnectorTopicPublisher<sensor_msgs::RelativeHumidity>;

// Publisher and subscriber proxy class instantiation for ROS message type: sensor_msgs/Temperature
template class ZyROSConnectorTopicPublisher<sensor_msgs::Temperature>;

// Publisher and subscriber proxy class instantiation for ROS message type: sensor_msgs/TimeReference
template class ZyROSConnectorTopicPublisher<sensor_msgs::TimeReference>;

// Publisher and subscriber proxy class instantiation for ROS message type: shape_msgs/Mesh
template class ZyROSConnectorTopicPublisher<shape_msgs::Mesh>;

// Publisher and subscriber proxy class instantiation for ROS message type: shape_msgs/MeshTriangle
template class ZyROSConnectorTopicPublisher<shape_msgs::MeshTriangle>;

// Publisher and subscriber proxy class instantiation for ROS message type: shape_msgs/Plane
template class ZyROSConnectorTopicPublisher<shape_msgs::Plane>;

// Publisher and subscriber proxy class instantiation for ROS message type: shape_msgs/SolidPrimitive
template class ZyROSConnectorTopicPublisher<shape_msgs::SolidPrimitive>;

// Publisher and subscriber proxy class instantiation for ROS message type: smach_msgs/SmachContainerInitialStatusCmd
template class ZyROSConnectorTopicPublisher<smach_msgs::SmachContainerInitialStatusCmd>;

// Publisher and subscriber proxy class instantiation for ROS message type: smach_msgs/SmachContainerStatus
template class ZyROSConnectorTopicPublisher<smach_msgs::SmachContainerStatus>;

// Publisher and subscriber proxy class instantiation for ROS message type: smach_msgs/SmachContainerStructure
template class ZyROSConnectorTopicPublisher<smach_msgs::SmachContainerStructure>;

// Publisher and subscriber proxy class instantiation for ROS message type: sofa_softrobots_msgs/BodyTransforms
template class ZyROSConnectorTopicPublisher<sofa_softrobots_msgs::BodyTransforms>;

// Publisher and subscriber proxy class instantiation for ROS message type: std_msgs/Bool
template class ZyROSConnectorTopicPublisher<std_msgs::Bool>;

// Publisher and subscriber proxy class instantiation for ROS message type: std_msgs/Byte
template class ZyROSConnectorTopicPublisher<std_msgs::Byte>;

// Publisher and subscriber proxy class instantiation for ROS message type: std_msgs/ByteMultiArray
template class ZyROSConnectorTopicPublisher<std_msgs::ByteMultiArray>;

// Publisher and subscriber proxy class instantiation for ROS message type: std_msgs/Char
template class ZyROSConnectorTopicPublisher<std_msgs::Char>;

// Publisher and subscriber proxy class instantiation for ROS message type: std_msgs/ColorRGBA
template class ZyROSConnectorTopicPublisher<std_msgs::ColorRGBA>;

// Publisher and subscriber proxy class instantiation for ROS message type: std_msgs/Duration
template class ZyROSConnectorTopicPublisher<std_msgs::Duration>;

// Publisher and subscriber proxy class instantiation for ROS message type: std_msgs/Empty
template class ZyROSConnectorTopicPublisher<std_msgs::Empty>;

// Publisher and subscriber proxy class instantiation for ROS message type: std_msgs/Float32
template class ZyROSConnectorTopicPublisher<std_msgs::Float32>;

// Publisher and subscriber proxy class instantiation for ROS message type: std_msgs/Float32MultiArray
template class ZyROSConnectorTopicPublisher<std_msgs::Float32MultiArray>;

// Publisher and subscriber proxy class instantiation for ROS message type: std_msgs/Float64
template class ZyROSConnectorTopicPublisher<std_msgs::Float64>;

// Publisher and subscriber proxy class instantiation for ROS message type: std_msgs/Float64MultiArray
template class ZyROSConnectorTopicPublisher<std_msgs::Float64MultiArray>;

// Publisher and subscriber proxy class instantiation for ROS message type: std_msgs/Header
template class ZyROSConnectorTopicPublisher<std_msgs::Header>;

// Publisher and subscriber proxy class instantiation for ROS message type: std_msgs/Int16
template class ZyROSConnectorTopicPublisher<std_msgs::Int16>;

// Publisher and subscriber proxy class instantiation for ROS message type: std_msgs/Int16MultiArray
template class ZyROSConnectorTopicPublisher<std_msgs::Int16MultiArray>;

// Publisher and subscriber proxy class instantiation for ROS message type: std_msgs/Int32
template class ZyROSConnectorTopicPublisher<std_msgs::Int32>;

// Publisher and subscriber proxy class instantiation for ROS message type: std_msgs/Int32MultiArray
template class ZyROSConnectorTopicPublisher<std_msgs::Int32MultiArray>;

// Publisher and subscriber proxy class instantiation for ROS message type: std_msgs/Int64
template class ZyROSConnectorTopicPublisher<std_msgs::Int64>;

// Publisher and subscriber proxy class instantiation for ROS message type: std_msgs/Int64MultiArray
template class ZyROSConnectorTopicPublisher<std_msgs::Int64MultiArray>;

// Publisher and subscriber proxy class instantiation for ROS message type: std_msgs/Int8
template class ZyROSConnectorTopicPublisher<std_msgs::Int8>;

// Publisher and subscriber proxy class instantiation for ROS message type: std_msgs/Int8MultiArray
template class ZyROSConnectorTopicPublisher<std_msgs::Int8MultiArray>;

// Publisher and subscriber proxy class instantiation for ROS message type: std_msgs/MultiArrayDimension
template class ZyROSConnectorTopicPublisher<std_msgs::MultiArrayDimension>;

// Publisher and subscriber proxy class instantiation for ROS message type: std_msgs/MultiArrayLayout
template class ZyROSConnectorTopicPublisher<std_msgs::MultiArrayLayout>;

// Publisher and subscriber proxy class instantiation for ROS message type: std_msgs/String
template class ZyROSConnectorTopicPublisher<std_msgs::String>;

// Publisher and subscriber proxy class instantiation for ROS message type: std_msgs/Time
template class ZyROSConnectorTopicPublisher<std_msgs::Time>;

// Publisher and subscriber proxy class instantiation for ROS message type: std_msgs/UInt16
template class ZyROSConnectorTopicPublisher<std_msgs::UInt16>;

// Publisher and subscriber proxy class instantiation for ROS message type: std_msgs/UInt16MultiArray
template class ZyROSConnectorTopicPublisher<std_msgs::UInt16MultiArray>;

// Publisher and subscriber proxy class instantiation for ROS message type: std_msgs/UInt32
template class ZyROSConnectorTopicPublisher<std_msgs::UInt32>;

// Publisher and subscriber proxy class instantiation for ROS message type: std_msgs/UInt32MultiArray
template class ZyROSConnectorTopicPublisher<std_msgs::UInt32MultiArray>;

// Publisher and subscriber proxy class instantiation for ROS message type: std_msgs/UInt64
template class ZyROSConnectorTopicPublisher<std_msgs::UInt64>;

// Publisher and subscriber proxy class instantiation for ROS message type: std_msgs/UInt64MultiArray
template class ZyROSConnectorTopicPublisher<std_msgs::UInt64MultiArray>;

// Publisher and subscriber proxy class instantiation for ROS message type: std_msgs/UInt8
template class ZyROSConnectorTopicPublisher<std_msgs::UInt8>;

// Publisher and subscriber proxy class instantiation for ROS message type: std_msgs/UInt8MultiArray
template class ZyROSConnectorTopicPublisher<std_msgs::UInt8MultiArray>;

// Publisher and subscriber proxy class instantiation for ROS message type: stereo_msgs/DisparityImage
template class ZyROSConnectorTopicPublisher<stereo_msgs::DisparityImage>;

// Publisher and subscriber proxy class instantiation for ROS message type: tf/tfMessage
template class ZyROSConnectorTopicPublisher<tf::tfMessage>;

// Publisher and subscriber proxy class instantiation for ROS message type: tf2_msgs/LookupTransformAction
template class ZyROSConnectorTopicPublisher<tf2_msgs::LookupTransformAction>;

// Publisher and subscriber proxy class instantiation for ROS message type: tf2_msgs/LookupTransformActionFeedback
template class ZyROSConnectorTopicPublisher<tf2_msgs::LookupTransformActionFeedback>;

// Publisher and subscriber proxy class instantiation for ROS message type: tf2_msgs/LookupTransformActionGoal
template class ZyROSConnectorTopicPublisher<tf2_msgs::LookupTransformActionGoal>;

// Publisher and subscriber proxy class instantiation for ROS message type: tf2_msgs/LookupTransformActionResult
template class ZyROSConnectorTopicPublisher<tf2_msgs::LookupTransformActionResult>;

// Publisher and subscriber proxy class instantiation for ROS message type: tf2_msgs/LookupTransformFeedback
template class ZyROSConnectorTopicPublisher<tf2_msgs::LookupTransformFeedback>;

// Publisher and subscriber proxy class instantiation for ROS message type: tf2_msgs/LookupTransformGoal
template class ZyROSConnectorTopicPublisher<tf2_msgs::LookupTransformGoal>;

// Publisher and subscriber proxy class instantiation for ROS message type: tf2_msgs/LookupTransformResult
template class ZyROSConnectorTopicPublisher<tf2_msgs::LookupTransformResult>;

// Publisher and subscriber proxy class instantiation for ROS message type: tf2_msgs/TF2Error
template class ZyROSConnectorTopicPublisher<tf2_msgs::TF2Error>;

// Publisher and subscriber proxy class instantiation for ROS message type: tf2_msgs/TFMessage
template class ZyROSConnectorTopicPublisher<tf2_msgs::TFMessage>;

// Publisher and subscriber proxy class instantiation for ROS message type: trajectory_msgs/JointTrajectory
template class ZyROSConnectorTopicPublisher<trajectory_msgs::JointTrajectory>;

// Publisher and subscriber proxy class instantiation for ROS message type: trajectory_msgs/JointTrajectoryPoint
template class ZyROSConnectorTopicPublisher<trajectory_msgs::JointTrajectoryPoint>;

// Publisher and subscriber proxy class instantiation for ROS message type: trajectory_msgs/MultiDOFJointTrajectory
template class ZyROSConnectorTopicPublisher<trajectory_msgs::MultiDOFJointTrajectory>;

// Publisher and subscriber proxy class instantiation for ROS message type: trajectory_msgs/MultiDOFJointTrajectoryPoint
template class ZyROSConnectorTopicPublisher<trajectory_msgs::MultiDOFJointTrajectoryPoint>;

// Publisher and subscriber proxy class instantiation for ROS message type: visualization_msgs/ImageMarker
template class ZyROSConnectorTopicPublisher<visualization_msgs::ImageMarker>;

// Publisher and subscriber proxy class instantiation for ROS message type: visualization_msgs/InteractiveMarker
template class ZyROSConnectorTopicPublisher<visualization_msgs::InteractiveMarker>;

// Publisher and subscriber proxy class instantiation for ROS message type: visualization_msgs/InteractiveMarkerControl
template class ZyROSConnectorTopicPublisher<visualization_msgs::InteractiveMarkerControl>;

// Publisher and subscriber proxy class instantiation for ROS message type: visualization_msgs/InteractiveMarkerFeedback
template class ZyROSConnectorTopicPublisher<visualization_msgs::InteractiveMarkerFeedback>;

// Publisher and subscriber proxy class instantiation for ROS message type: visualization_msgs/InteractiveMarkerInit
template class ZyROSConnectorTopicPublisher<visualization_msgs::InteractiveMarkerInit>;

// Publisher and subscriber proxy class instantiation for ROS message type: visualization_msgs/InteractiveMarkerPose
template class ZyROSConnectorTopicPublisher<visualization_msgs::InteractiveMarkerPose>;

// Publisher and subscriber proxy class instantiation for ROS message type: visualization_msgs/InteractiveMarkerUpdate
template class ZyROSConnectorTopicPublisher<visualization_msgs::InteractiveMarkerUpdate>;

// Publisher and subscriber proxy class instantiation for ROS message type: visualization_msgs/Marker
template class ZyROSConnectorTopicPublisher<visualization_msgs::Marker>;

// Publisher and subscriber proxy class instantiation for ROS message type: visualization_msgs/MarkerArray
template class ZyROSConnectorTopicPublisher<visualization_msgs::MarkerArray>;

// Publisher and subscriber proxy class instantiation for ROS message type: visualization_msgs/MenuEntry
template class ZyROSConnectorTopicPublisher<visualization_msgs::MenuEntry>;

boost::shared_ptr<ZyROSPublisher> ZyROSConnectorMessagePublisherFactory::createTopicPublisher(ros::NodeHandlePtr rosNode, const std::string& topicURI, const std::string& messageType)
{
	bool supported = false;
	boost::shared_ptr<ZyROSPublisher> topicPublisher;
	// Publisher instance for ROS message type: actionlib/TestAction
	if (messageType == "actionlib::TestAction")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<actionlib::TestAction>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: actionlib/TestActionFeedback
	if (messageType == "actionlib::TestActionFeedback")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<actionlib::TestActionFeedback>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: actionlib/TestActionGoal
	if (messageType == "actionlib::TestActionGoal")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<actionlib::TestActionGoal>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: actionlib/TestActionResult
	if (messageType == "actionlib::TestActionResult")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<actionlib::TestActionResult>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: actionlib/TestFeedback
	if (messageType == "actionlib::TestFeedback")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<actionlib::TestFeedback>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: actionlib/TestGoal
	if (messageType == "actionlib::TestGoal")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<actionlib::TestGoal>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: actionlib/TestRequestAction
	if (messageType == "actionlib::TestRequestAction")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<actionlib::TestRequestAction>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: actionlib/TestRequestActionFeedback
	if (messageType == "actionlib::TestRequestActionFeedback")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<actionlib::TestRequestActionFeedback>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: actionlib/TestRequestActionGoal
	if (messageType == "actionlib::TestRequestActionGoal")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<actionlib::TestRequestActionGoal>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: actionlib/TestRequestActionResult
	if (messageType == "actionlib::TestRequestActionResult")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<actionlib::TestRequestActionResult>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: actionlib/TestRequestFeedback
	if (messageType == "actionlib::TestRequestFeedback")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<actionlib::TestRequestFeedback>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: actionlib/TestRequestGoal
	if (messageType == "actionlib::TestRequestGoal")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<actionlib::TestRequestGoal>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: actionlib/TestRequestResult
	if (messageType == "actionlib::TestRequestResult")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<actionlib::TestRequestResult>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: actionlib/TestResult
	if (messageType == "actionlib::TestResult")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<actionlib::TestResult>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: actionlib/TwoIntsAction
	if (messageType == "actionlib::TwoIntsAction")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<actionlib::TwoIntsAction>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: actionlib/TwoIntsActionFeedback
	if (messageType == "actionlib::TwoIntsActionFeedback")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<actionlib::TwoIntsActionFeedback>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: actionlib/TwoIntsActionGoal
	if (messageType == "actionlib::TwoIntsActionGoal")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<actionlib::TwoIntsActionGoal>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: actionlib/TwoIntsActionResult
	if (messageType == "actionlib::TwoIntsActionResult")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<actionlib::TwoIntsActionResult>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: actionlib/TwoIntsFeedback
	if (messageType == "actionlib::TwoIntsFeedback")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<actionlib::TwoIntsFeedback>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: actionlib/TwoIntsGoal
	if (messageType == "actionlib::TwoIntsGoal")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<actionlib::TwoIntsGoal>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: actionlib/TwoIntsResult
	if (messageType == "actionlib::TwoIntsResult")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<actionlib::TwoIntsResult>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: actionlib_msgs/GoalID
	if (messageType == "actionlib_msgs::GoalID")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<actionlib_msgs::GoalID>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: actionlib_msgs/GoalStatus
	if (messageType == "actionlib_msgs::GoalStatus")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<actionlib_msgs::GoalStatus>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: actionlib_msgs/GoalStatusArray
	if (messageType == "actionlib_msgs::GoalStatusArray")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<actionlib_msgs::GoalStatusArray>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: bond/Constants
	if (messageType == "bond::Constants")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<bond::Constants>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: bond/Status
	if (messageType == "bond::Status")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<bond::Status>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: control_msgs/FollowJointTrajectoryAction
	if (messageType == "control_msgs::FollowJointTrajectoryAction")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<control_msgs::FollowJointTrajectoryAction>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: control_msgs/FollowJointTrajectoryActionFeedback
	if (messageType == "control_msgs::FollowJointTrajectoryActionFeedback")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<control_msgs::FollowJointTrajectoryActionFeedback>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: control_msgs/FollowJointTrajectoryActionGoal
	if (messageType == "control_msgs::FollowJointTrajectoryActionGoal")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<control_msgs::FollowJointTrajectoryActionGoal>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: control_msgs/FollowJointTrajectoryActionResult
	if (messageType == "control_msgs::FollowJointTrajectoryActionResult")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<control_msgs::FollowJointTrajectoryActionResult>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: control_msgs/FollowJointTrajectoryFeedback
	if (messageType == "control_msgs::FollowJointTrajectoryFeedback")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<control_msgs::FollowJointTrajectoryFeedback>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: control_msgs/FollowJointTrajectoryGoal
	if (messageType == "control_msgs::FollowJointTrajectoryGoal")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<control_msgs::FollowJointTrajectoryGoal>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: control_msgs/FollowJointTrajectoryResult
	if (messageType == "control_msgs::FollowJointTrajectoryResult")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<control_msgs::FollowJointTrajectoryResult>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: control_msgs/GripperCommand
	if (messageType == "control_msgs::GripperCommand")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<control_msgs::GripperCommand>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: control_msgs/GripperCommandAction
	if (messageType == "control_msgs::GripperCommandAction")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<control_msgs::GripperCommandAction>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: control_msgs/GripperCommandActionFeedback
	if (messageType == "control_msgs::GripperCommandActionFeedback")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<control_msgs::GripperCommandActionFeedback>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: control_msgs/GripperCommandActionGoal
	if (messageType == "control_msgs::GripperCommandActionGoal")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<control_msgs::GripperCommandActionGoal>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: control_msgs/GripperCommandActionResult
	if (messageType == "control_msgs::GripperCommandActionResult")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<control_msgs::GripperCommandActionResult>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: control_msgs/GripperCommandFeedback
	if (messageType == "control_msgs::GripperCommandFeedback")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<control_msgs::GripperCommandFeedback>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: control_msgs/GripperCommandGoal
	if (messageType == "control_msgs::GripperCommandGoal")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<control_msgs::GripperCommandGoal>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: control_msgs/GripperCommandResult
	if (messageType == "control_msgs::GripperCommandResult")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<control_msgs::GripperCommandResult>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: control_msgs/JointControllerState
	if (messageType == "control_msgs::JointControllerState")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<control_msgs::JointControllerState>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: control_msgs/JointJog
	if (messageType == "control_msgs::JointJog")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<control_msgs::JointJog>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: control_msgs/JointTolerance
	if (messageType == "control_msgs::JointTolerance")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<control_msgs::JointTolerance>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: control_msgs/JointTrajectoryAction
	if (messageType == "control_msgs::JointTrajectoryAction")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<control_msgs::JointTrajectoryAction>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: control_msgs/JointTrajectoryActionFeedback
	if (messageType == "control_msgs::JointTrajectoryActionFeedback")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<control_msgs::JointTrajectoryActionFeedback>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: control_msgs/JointTrajectoryActionGoal
	if (messageType == "control_msgs::JointTrajectoryActionGoal")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<control_msgs::JointTrajectoryActionGoal>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: control_msgs/JointTrajectoryActionResult
	if (messageType == "control_msgs::JointTrajectoryActionResult")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<control_msgs::JointTrajectoryActionResult>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: control_msgs/JointTrajectoryControllerState
	if (messageType == "control_msgs::JointTrajectoryControllerState")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<control_msgs::JointTrajectoryControllerState>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: control_msgs/JointTrajectoryFeedback
	if (messageType == "control_msgs::JointTrajectoryFeedback")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<control_msgs::JointTrajectoryFeedback>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: control_msgs/JointTrajectoryGoal
	if (messageType == "control_msgs::JointTrajectoryGoal")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<control_msgs::JointTrajectoryGoal>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: control_msgs/JointTrajectoryResult
	if (messageType == "control_msgs::JointTrajectoryResult")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<control_msgs::JointTrajectoryResult>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: control_msgs/PidState
	if (messageType == "control_msgs::PidState")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<control_msgs::PidState>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: control_msgs/PointHeadAction
	if (messageType == "control_msgs::PointHeadAction")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<control_msgs::PointHeadAction>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: control_msgs/PointHeadActionFeedback
	if (messageType == "control_msgs::PointHeadActionFeedback")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<control_msgs::PointHeadActionFeedback>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: control_msgs/PointHeadActionGoal
	if (messageType == "control_msgs::PointHeadActionGoal")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<control_msgs::PointHeadActionGoal>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: control_msgs/PointHeadActionResult
	if (messageType == "control_msgs::PointHeadActionResult")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<control_msgs::PointHeadActionResult>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: control_msgs/PointHeadFeedback
	if (messageType == "control_msgs::PointHeadFeedback")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<control_msgs::PointHeadFeedback>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: control_msgs/PointHeadGoal
	if (messageType == "control_msgs::PointHeadGoal")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<control_msgs::PointHeadGoal>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: control_msgs/PointHeadResult
	if (messageType == "control_msgs::PointHeadResult")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<control_msgs::PointHeadResult>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: control_msgs/SingleJointPositionAction
	if (messageType == "control_msgs::SingleJointPositionAction")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<control_msgs::SingleJointPositionAction>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: control_msgs/SingleJointPositionActionFeedback
	if (messageType == "control_msgs::SingleJointPositionActionFeedback")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<control_msgs::SingleJointPositionActionFeedback>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: control_msgs/SingleJointPositionActionGoal
	if (messageType == "control_msgs::SingleJointPositionActionGoal")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<control_msgs::SingleJointPositionActionGoal>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: control_msgs/SingleJointPositionActionResult
	if (messageType == "control_msgs::SingleJointPositionActionResult")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<control_msgs::SingleJointPositionActionResult>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: control_msgs/SingleJointPositionFeedback
	if (messageType == "control_msgs::SingleJointPositionFeedback")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<control_msgs::SingleJointPositionFeedback>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: control_msgs/SingleJointPositionGoal
	if (messageType == "control_msgs::SingleJointPositionGoal")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<control_msgs::SingleJointPositionGoal>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: control_msgs/SingleJointPositionResult
	if (messageType == "control_msgs::SingleJointPositionResult")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<control_msgs::SingleJointPositionResult>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: diagnostic_msgs/DiagnosticArray
	if (messageType == "diagnostic_msgs::DiagnosticArray")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<diagnostic_msgs::DiagnosticArray>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: diagnostic_msgs/DiagnosticStatus
	if (messageType == "diagnostic_msgs::DiagnosticStatus")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<diagnostic_msgs::DiagnosticStatus>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: diagnostic_msgs/KeyValue
	if (messageType == "diagnostic_msgs::KeyValue")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<diagnostic_msgs::KeyValue>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: dynamic_reconfigure/BoolParameter
	if (messageType == "dynamic_reconfigure::BoolParameter")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<dynamic_reconfigure::BoolParameter>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: dynamic_reconfigure/Config
	if (messageType == "dynamic_reconfigure::Config")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<dynamic_reconfigure::Config>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: dynamic_reconfigure/ConfigDescription
	if (messageType == "dynamic_reconfigure::ConfigDescription")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<dynamic_reconfigure::ConfigDescription>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: dynamic_reconfigure/DoubleParameter
	if (messageType == "dynamic_reconfigure::DoubleParameter")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<dynamic_reconfigure::DoubleParameter>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: dynamic_reconfigure/Group
	if (messageType == "dynamic_reconfigure::Group")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<dynamic_reconfigure::Group>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: dynamic_reconfigure/GroupState
	if (messageType == "dynamic_reconfigure::GroupState")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<dynamic_reconfigure::GroupState>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: dynamic_reconfigure/IntParameter
	if (messageType == "dynamic_reconfigure::IntParameter")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<dynamic_reconfigure::IntParameter>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: dynamic_reconfigure/ParamDescription
	if (messageType == "dynamic_reconfigure::ParamDescription")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<dynamic_reconfigure::ParamDescription>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: dynamic_reconfigure/SensorLevels
	if (messageType == "dynamic_reconfigure::SensorLevels")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<dynamic_reconfigure::SensorLevels>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: dynamic_reconfigure/StrParameter
	if (messageType == "dynamic_reconfigure::StrParameter")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<dynamic_reconfigure::StrParameter>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: geometry_msgs/Accel
	if (messageType == "geometry_msgs::Accel")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<geometry_msgs::Accel>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: geometry_msgs/AccelStamped
	if (messageType == "geometry_msgs::AccelStamped")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<geometry_msgs::AccelStamped>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: geometry_msgs/AccelWithCovariance
	if (messageType == "geometry_msgs::AccelWithCovariance")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<geometry_msgs::AccelWithCovariance>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: geometry_msgs/AccelWithCovarianceStamped
	if (messageType == "geometry_msgs::AccelWithCovarianceStamped")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<geometry_msgs::AccelWithCovarianceStamped>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: geometry_msgs/Inertia
	if (messageType == "geometry_msgs::Inertia")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<geometry_msgs::Inertia>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: geometry_msgs/InertiaStamped
	if (messageType == "geometry_msgs::InertiaStamped")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<geometry_msgs::InertiaStamped>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: geometry_msgs/Point
	if (messageType == "geometry_msgs::Point")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<geometry_msgs::Point>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: geometry_msgs/Point32
	if (messageType == "geometry_msgs::Point32")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<geometry_msgs::Point32>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: geometry_msgs/PointStamped
	if (messageType == "geometry_msgs::PointStamped")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<geometry_msgs::PointStamped>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: geometry_msgs/Polygon
	if (messageType == "geometry_msgs::Polygon")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<geometry_msgs::Polygon>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: geometry_msgs/PolygonStamped
	if (messageType == "geometry_msgs::PolygonStamped")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<geometry_msgs::PolygonStamped>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: geometry_msgs/Pose
	if (messageType == "geometry_msgs::Pose")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<geometry_msgs::Pose>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: geometry_msgs/Pose2D
	if (messageType == "geometry_msgs::Pose2D")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<geometry_msgs::Pose2D>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: geometry_msgs/PoseArray
	if (messageType == "geometry_msgs::PoseArray")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<geometry_msgs::PoseArray>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: geometry_msgs/PoseStamped
	if (messageType == "geometry_msgs::PoseStamped")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<geometry_msgs::PoseStamped>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: geometry_msgs/PoseWithCovariance
	if (messageType == "geometry_msgs::PoseWithCovariance")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<geometry_msgs::PoseWithCovariance>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: geometry_msgs/PoseWithCovarianceStamped
	if (messageType == "geometry_msgs::PoseWithCovarianceStamped")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<geometry_msgs::PoseWithCovarianceStamped>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: geometry_msgs/Quaternion
	if (messageType == "geometry_msgs::Quaternion")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<geometry_msgs::Quaternion>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: geometry_msgs/QuaternionStamped
	if (messageType == "geometry_msgs::QuaternionStamped")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<geometry_msgs::QuaternionStamped>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: geometry_msgs/Transform
	if (messageType == "geometry_msgs::Transform")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<geometry_msgs::Transform>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: geometry_msgs/TransformStamped
	if (messageType == "geometry_msgs::TransformStamped")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<geometry_msgs::TransformStamped>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: geometry_msgs/Twist
	if (messageType == "geometry_msgs::Twist")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<geometry_msgs::Twist>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: geometry_msgs/TwistStamped
	if (messageType == "geometry_msgs::TwistStamped")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<geometry_msgs::TwistStamped>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: geometry_msgs/TwistWithCovariance
	if (messageType == "geometry_msgs::TwistWithCovariance")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<geometry_msgs::TwistWithCovariance>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: geometry_msgs/TwistWithCovarianceStamped
	if (messageType == "geometry_msgs::TwistWithCovarianceStamped")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<geometry_msgs::TwistWithCovarianceStamped>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: geometry_msgs/Vector3
	if (messageType == "geometry_msgs::Vector3")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<geometry_msgs::Vector3>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: geometry_msgs/Vector3Stamped
	if (messageType == "geometry_msgs::Vector3Stamped")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<geometry_msgs::Vector3Stamped>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: geometry_msgs/Wrench
	if (messageType == "geometry_msgs::Wrench")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<geometry_msgs::Wrench>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: geometry_msgs/WrenchStamped
	if (messageType == "geometry_msgs::WrenchStamped")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<geometry_msgs::WrenchStamped>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: nav_msgs/GetMapAction
	if (messageType == "nav_msgs::GetMapAction")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<nav_msgs::GetMapAction>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: nav_msgs/GetMapActionFeedback
	if (messageType == "nav_msgs::GetMapActionFeedback")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<nav_msgs::GetMapActionFeedback>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: nav_msgs/GetMapActionGoal
	if (messageType == "nav_msgs::GetMapActionGoal")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<nav_msgs::GetMapActionGoal>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: nav_msgs/GetMapActionResult
	if (messageType == "nav_msgs::GetMapActionResult")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<nav_msgs::GetMapActionResult>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: nav_msgs/GetMapFeedback
	if (messageType == "nav_msgs::GetMapFeedback")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<nav_msgs::GetMapFeedback>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: nav_msgs/GetMapGoal
	if (messageType == "nav_msgs::GetMapGoal")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<nav_msgs::GetMapGoal>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: nav_msgs/GetMapResult
	if (messageType == "nav_msgs::GetMapResult")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<nav_msgs::GetMapResult>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: nav_msgs/GridCells
	if (messageType == "nav_msgs::GridCells")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<nav_msgs::GridCells>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: nav_msgs/MapMetaData
	if (messageType == "nav_msgs::MapMetaData")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<nav_msgs::MapMetaData>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: nav_msgs/OccupancyGrid
	if (messageType == "nav_msgs::OccupancyGrid")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<nav_msgs::OccupancyGrid>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: nav_msgs/Odometry
	if (messageType == "nav_msgs::Odometry")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<nav_msgs::Odometry>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: nav_msgs/Path
	if (messageType == "nav_msgs::Path")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<nav_msgs::Path>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: roscpp/Logger
	if (messageType == "roscpp::Logger")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<roscpp::Logger>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: rosgraph_msgs/Clock
	if (messageType == "rosgraph_msgs::Clock")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<rosgraph_msgs::Clock>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: rosgraph_msgs/Log
	if (messageType == "rosgraph_msgs::Log")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<rosgraph_msgs::Log>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: rosgraph_msgs/TopicStatistics
	if (messageType == "rosgraph_msgs::TopicStatistics")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<rosgraph_msgs::TopicStatistics>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: rospy_tutorials/Floats
	if (messageType == "rospy_tutorials::Floats")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<rospy_tutorials::Floats>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: rospy_tutorials/HeaderString
	if (messageType == "rospy_tutorials::HeaderString")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<rospy_tutorials::HeaderString>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: sensor_msgs/BatteryState
	if (messageType == "sensor_msgs::BatteryState")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<sensor_msgs::BatteryState>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: sensor_msgs/CameraInfo
	if (messageType == "sensor_msgs::CameraInfo")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<sensor_msgs::CameraInfo>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: sensor_msgs/ChannelFloat32
	if (messageType == "sensor_msgs::ChannelFloat32")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<sensor_msgs::ChannelFloat32>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: sensor_msgs/CompressedImage
	if (messageType == "sensor_msgs::CompressedImage")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<sensor_msgs::CompressedImage>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: sensor_msgs/FluidPressure
	if (messageType == "sensor_msgs::FluidPressure")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<sensor_msgs::FluidPressure>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: sensor_msgs/Illuminance
	if (messageType == "sensor_msgs::Illuminance")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<sensor_msgs::Illuminance>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: sensor_msgs/Image
	if (messageType == "sensor_msgs::Image")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<sensor_msgs::Image>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: sensor_msgs/Imu
	if (messageType == "sensor_msgs::Imu")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<sensor_msgs::Imu>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: sensor_msgs/JointState
	if (messageType == "sensor_msgs::JointState")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<sensor_msgs::JointState>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: sensor_msgs/Joy
	if (messageType == "sensor_msgs::Joy")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<sensor_msgs::Joy>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: sensor_msgs/JoyFeedback
	if (messageType == "sensor_msgs::JoyFeedback")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<sensor_msgs::JoyFeedback>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: sensor_msgs/JoyFeedbackArray
	if (messageType == "sensor_msgs::JoyFeedbackArray")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<sensor_msgs::JoyFeedbackArray>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: sensor_msgs/LaserEcho
	if (messageType == "sensor_msgs::LaserEcho")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<sensor_msgs::LaserEcho>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: sensor_msgs/LaserScan
	if (messageType == "sensor_msgs::LaserScan")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<sensor_msgs::LaserScan>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: sensor_msgs/MagneticField
	if (messageType == "sensor_msgs::MagneticField")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<sensor_msgs::MagneticField>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: sensor_msgs/MultiDOFJointState
	if (messageType == "sensor_msgs::MultiDOFJointState")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<sensor_msgs::MultiDOFJointState>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: sensor_msgs/MultiEchoLaserScan
	if (messageType == "sensor_msgs::MultiEchoLaserScan")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<sensor_msgs::MultiEchoLaserScan>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: sensor_msgs/NavSatFix
	if (messageType == "sensor_msgs::NavSatFix")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<sensor_msgs::NavSatFix>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: sensor_msgs/NavSatStatus
	if (messageType == "sensor_msgs::NavSatStatus")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<sensor_msgs::NavSatStatus>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: sensor_msgs/PointCloud
	if (messageType == "sensor_msgs::PointCloud")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<sensor_msgs::PointCloud>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: sensor_msgs/PointCloud2
	if (messageType == "sensor_msgs::PointCloud2")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<sensor_msgs::PointCloud2>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: sensor_msgs/PointField
	if (messageType == "sensor_msgs::PointField")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<sensor_msgs::PointField>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: sensor_msgs/Range
	if (messageType == "sensor_msgs::Range")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<sensor_msgs::Range>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: sensor_msgs/RegionOfInterest
	if (messageType == "sensor_msgs::RegionOfInterest")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<sensor_msgs::RegionOfInterest>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: sensor_msgs/RelativeHumidity
	if (messageType == "sensor_msgs::RelativeHumidity")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<sensor_msgs::RelativeHumidity>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: sensor_msgs/Temperature
	if (messageType == "sensor_msgs::Temperature")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<sensor_msgs::Temperature>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: sensor_msgs/TimeReference
	if (messageType == "sensor_msgs::TimeReference")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<sensor_msgs::TimeReference>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: shape_msgs/Mesh
	if (messageType == "shape_msgs::Mesh")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<shape_msgs::Mesh>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: shape_msgs/MeshTriangle
	if (messageType == "shape_msgs::MeshTriangle")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<shape_msgs::MeshTriangle>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: shape_msgs/Plane
	if (messageType == "shape_msgs::Plane")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<shape_msgs::Plane>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: shape_msgs/SolidPrimitive
	if (messageType == "shape_msgs::SolidPrimitive")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<shape_msgs::SolidPrimitive>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: smach_msgs/SmachContainerInitialStatusCmd
	if (messageType == "smach_msgs::SmachContainerInitialStatusCmd")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<smach_msgs::SmachContainerInitialStatusCmd>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: smach_msgs/SmachContainerStatus
	if (messageType == "smach_msgs::SmachContainerStatus")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<smach_msgs::SmachContainerStatus>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: smach_msgs/SmachContainerStructure
	if (messageType == "smach_msgs::SmachContainerStructure")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<smach_msgs::SmachContainerStructure>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: sofa_softrobots_msgs/BodyTransforms
	if (messageType == "sofa_softrobots_msgs::BodyTransforms")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<sofa_softrobots_msgs::BodyTransforms>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: std_msgs/Bool
	if (messageType == "std_msgs::Bool")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<std_msgs::Bool>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: std_msgs/Byte
	if (messageType == "std_msgs::Byte")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<std_msgs::Byte>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: std_msgs/ByteMultiArray
	if (messageType == "std_msgs::ByteMultiArray")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<std_msgs::ByteMultiArray>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: std_msgs/Char
	if (messageType == "std_msgs::Char")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<std_msgs::Char>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: std_msgs/ColorRGBA
	if (messageType == "std_msgs::ColorRGBA")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<std_msgs::ColorRGBA>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: std_msgs/Duration
	if (messageType == "std_msgs::Duration")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<std_msgs::Duration>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: std_msgs/Empty
	if (messageType == "std_msgs::Empty")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<std_msgs::Empty>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: std_msgs/Float32
	if (messageType == "std_msgs::Float32")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<std_msgs::Float32>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: std_msgs/Float32MultiArray
	if (messageType == "std_msgs::Float32MultiArray")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<std_msgs::Float32MultiArray>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: std_msgs/Float64
	if (messageType == "std_msgs::Float64")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<std_msgs::Float64>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: std_msgs/Float64MultiArray
	if (messageType == "std_msgs::Float64MultiArray")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<std_msgs::Float64MultiArray>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: std_msgs/Header
	if (messageType == "std_msgs::Header")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<std_msgs::Header>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: std_msgs/Int16
	if (messageType == "std_msgs::Int16")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<std_msgs::Int16>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: std_msgs/Int16MultiArray
	if (messageType == "std_msgs::Int16MultiArray")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<std_msgs::Int16MultiArray>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: std_msgs/Int32
	if (messageType == "std_msgs::Int32")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<std_msgs::Int32>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: std_msgs/Int32MultiArray
	if (messageType == "std_msgs::Int32MultiArray")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<std_msgs::Int32MultiArray>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: std_msgs/Int64
	if (messageType == "std_msgs::Int64")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<std_msgs::Int64>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: std_msgs/Int64MultiArray
	if (messageType == "std_msgs::Int64MultiArray")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<std_msgs::Int64MultiArray>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: std_msgs/Int8
	if (messageType == "std_msgs::Int8")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<std_msgs::Int8>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: std_msgs/Int8MultiArray
	if (messageType == "std_msgs::Int8MultiArray")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<std_msgs::Int8MultiArray>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: std_msgs/MultiArrayDimension
	if (messageType == "std_msgs::MultiArrayDimension")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<std_msgs::MultiArrayDimension>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: std_msgs/MultiArrayLayout
	if (messageType == "std_msgs::MultiArrayLayout")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<std_msgs::MultiArrayLayout>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: std_msgs/String
	if (messageType == "std_msgs::String")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<std_msgs::String>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: std_msgs/Time
	if (messageType == "std_msgs::Time")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<std_msgs::Time>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: std_msgs/UInt16
	if (messageType == "std_msgs::UInt16")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<std_msgs::UInt16>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: std_msgs/UInt16MultiArray
	if (messageType == "std_msgs::UInt16MultiArray")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<std_msgs::UInt16MultiArray>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: std_msgs/UInt32
	if (messageType == "std_msgs::UInt32")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<std_msgs::UInt32>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: std_msgs/UInt32MultiArray
	if (messageType == "std_msgs::UInt32MultiArray")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<std_msgs::UInt32MultiArray>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: std_msgs/UInt64
	if (messageType == "std_msgs::UInt64")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<std_msgs::UInt64>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: std_msgs/UInt64MultiArray
	if (messageType == "std_msgs::UInt64MultiArray")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<std_msgs::UInt64MultiArray>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: std_msgs/UInt8
	if (messageType == "std_msgs::UInt8")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<std_msgs::UInt8>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: std_msgs/UInt8MultiArray
	if (messageType == "std_msgs::UInt8MultiArray")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<std_msgs::UInt8MultiArray>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: stereo_msgs/DisparityImage
	if (messageType == "stereo_msgs::DisparityImage")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<stereo_msgs::DisparityImage>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: tf/tfMessage
	if (messageType == "tf::tfMessage")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<tf::tfMessage>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: tf2_msgs/LookupTransformAction
	if (messageType == "tf2_msgs::LookupTransformAction")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<tf2_msgs::LookupTransformAction>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: tf2_msgs/LookupTransformActionFeedback
	if (messageType == "tf2_msgs::LookupTransformActionFeedback")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<tf2_msgs::LookupTransformActionFeedback>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: tf2_msgs/LookupTransformActionGoal
	if (messageType == "tf2_msgs::LookupTransformActionGoal")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<tf2_msgs::LookupTransformActionGoal>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: tf2_msgs/LookupTransformActionResult
	if (messageType == "tf2_msgs::LookupTransformActionResult")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<tf2_msgs::LookupTransformActionResult>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: tf2_msgs/LookupTransformFeedback
	if (messageType == "tf2_msgs::LookupTransformFeedback")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<tf2_msgs::LookupTransformFeedback>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: tf2_msgs/LookupTransformGoal
	if (messageType == "tf2_msgs::LookupTransformGoal")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<tf2_msgs::LookupTransformGoal>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: tf2_msgs/LookupTransformResult
	if (messageType == "tf2_msgs::LookupTransformResult")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<tf2_msgs::LookupTransformResult>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: tf2_msgs/TF2Error
	if (messageType == "tf2_msgs::TF2Error")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<tf2_msgs::TF2Error>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: tf2_msgs/TFMessage
	if (messageType == "tf2_msgs::TFMessage")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<tf2_msgs::TFMessage>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: trajectory_msgs/JointTrajectory
	if (messageType == "trajectory_msgs::JointTrajectory")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<trajectory_msgs::JointTrajectory>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: trajectory_msgs/JointTrajectoryPoint
	if (messageType == "trajectory_msgs::JointTrajectoryPoint")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<trajectory_msgs::JointTrajectoryPoint>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: trajectory_msgs/MultiDOFJointTrajectory
	if (messageType == "trajectory_msgs::MultiDOFJointTrajectory")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<trajectory_msgs::MultiDOFJointTrajectory>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: trajectory_msgs/MultiDOFJointTrajectoryPoint
	if (messageType == "trajectory_msgs::MultiDOFJointTrajectoryPoint")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<trajectory_msgs::MultiDOFJointTrajectoryPoint>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: visualization_msgs/ImageMarker
	if (messageType == "visualization_msgs::ImageMarker")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<visualization_msgs::ImageMarker>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: visualization_msgs/InteractiveMarker
	if (messageType == "visualization_msgs::InteractiveMarker")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<visualization_msgs::InteractiveMarker>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: visualization_msgs/InteractiveMarkerControl
	if (messageType == "visualization_msgs::InteractiveMarkerControl")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<visualization_msgs::InteractiveMarkerControl>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: visualization_msgs/InteractiveMarkerFeedback
	if (messageType == "visualization_msgs::InteractiveMarkerFeedback")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<visualization_msgs::InteractiveMarkerFeedback>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: visualization_msgs/InteractiveMarkerInit
	if (messageType == "visualization_msgs::InteractiveMarkerInit")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<visualization_msgs::InteractiveMarkerInit>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: visualization_msgs/InteractiveMarkerPose
	if (messageType == "visualization_msgs::InteractiveMarkerPose")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<visualization_msgs::InteractiveMarkerPose>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: visualization_msgs/InteractiveMarkerUpdate
	if (messageType == "visualization_msgs::InteractiveMarkerUpdate")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<visualization_msgs::InteractiveMarkerUpdate>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: visualization_msgs/Marker
	if (messageType == "visualization_msgs::Marker")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<visualization_msgs::Marker>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: visualization_msgs/MarkerArray
	if (messageType == "visualization_msgs::MarkerArray")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<visualization_msgs::MarkerArray>(rosNode, topicURI, 50));
	}
	// Publisher instance for ROS message type: visualization_msgs/MenuEntry
	if (messageType == "visualization_msgs::MenuEntry")
	{
		supported = true;
		topicPublisher.reset(new ZyROSConnectorTopicPublisher<visualization_msgs::MenuEntry>(rosNode, topicURI, 50));
	}
	if (supported)
	{
		msg_info("ZyROSConnectorMessagePublisherFactory") << "ROS message type supported: " << messageType;
	}
	else
	{
		msg_warning("ZyROSConnectorMessagePublisherFactory") << "ROS message type NOT supported: " << messageType;
	}	return topicPublisher;
}
