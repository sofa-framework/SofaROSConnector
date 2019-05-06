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

// Template specializations for common ROS messages - even with auto-generated template instantiations, these seem to be required to satisfy the linker?

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

#include <std_msgs/Bool.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/String.h>

template class ZyROSConnectorTopicPublisher<std_msgs::Bool>;
template class ZyROSConnectorTopicPublisher<std_msgs::Float64MultiArray>;
template class ZyROSConnectorTopicPublisher<std_msgs::String>;
