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
        m_rosTopic = other.m_rosTopic;
    }
}

ZyROSPublisher& ZyROSPublisher::operator=(const ZyROSPublisher& other)
{
    if (this != &other)
    {
        m_uuid = other.m_uuid;
        m_rosTopic = other.m_rosTopic;
    }
    return *this;
}

// Template specializations for common/most used ROS messages

#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Float32.h>

#include <std_msgs/Float32MultiArray.h>

// Include order and the #undef are important; ROS defines an enum named "ERROR".
#undef ERROR
#include <rosgraph_msgs/Log.h>

template class ZyROSConnectorTopicPublisher<sensor_msgs::JointState>;
template class ZyROSConnectorTopicPublisher<geometry_msgs::Pose>;
template class ZyROSConnectorTopicPublisher<std_msgs::Float32>;
template class ZyROSConnectorTopicPublisher<rosgraph_msgs::Log>;

template class ZyROSConnectorTopicPublisher<std_msgs::Float32MultiArray>;
