#include "ZyROSConnectorTopicSubscriber.inl"

using namespace Zyklio::ROSConnector;

SOFA_DECL_CLASS(TruRosConnectorTopicSubscriberIface)

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

void TruRosConnectorTopicSubscriberIface::onMessageReceived()
{
	m_sig();
}


// Template specializations for common/most used ROS messages

#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/String.h>

// Include order and the #undef are important; ROS defines an enum named "ERROR".
#undef ERROR
#include <rosgraph_msgs/Log.h>

template class ZyROSConnectorTopicSubscriber<sensor_msgs::JointState>;
template class ZyROSConnectorTopicSubscriber<geometry_msgs::Pose>;
template class ZyROSConnectorTopicSubscriber<std_msgs::Float32>;
template class ZyROSConnectorTopicSubscriber<rosgraph_msgs::Log>;
template class ZyROSConnectorTopicSubscriber<std_msgs::Bool>;
template class ZyROSConnectorTopicSubscriber<std_msgs::Float64MultiArray>;
template class ZyROSConnectorTopicSubscriber<std_msgs::String>;
