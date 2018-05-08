#ifndef TRUROSCONNECTOR_TOPIC_PUBLISHER_H
#define TRUROSCONNECTOR_TOPIC_PUBLISHER_H

#ifdef _WIN32
// Ensure that Winsock2.h is included before Windows.h, which can get
// pulled in by anybody (e.g., Boost).
#include <Winsock2.h>
#endif

#include "init_ZyROSConnector.h"

#include <ros/node_handle.h>

#include <boost/circular_buffer.hpp>
#include <boost/signals2/signal.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include "sofa/helper/logging/Messaging.h"

namespace Zyklio
{
namespace ROSConnector
{
class SOFA_ZY_ROS_CONNECTOR_API ZyROSPublisher
{
public:
    ZyROSPublisher();
    virtual ~ZyROSPublisher() {}

    ZyROSPublisher(const ZyROSPublisher&);
    ZyROSPublisher& operator=(const ZyROSPublisher&);

    virtual void cleanup() { std::cout << "WARNING:  cleanup() not implemented for a ZyROSPublisher." << std::endl; }
    virtual void publishMessageQueue() { std::cout << "WARNING:  publishMessageQueue() not implemented for a ZyROSPublisher." << std::endl; }

    const boost::uuids::uuid& getUuid() { return m_uuid; }

    std::string getMessageType() { return messageType; }

protected:
    boost::uuids::uuid m_uuid;
    std::string m_rosTopic;
    std::string messageType;
};

template <class MessageType>
class SOFA_ZY_ROS_CONNECTOR_API ZyROSConnectorTopicPublisher : public ZyROSPublisher
{
public:
    ZyROSConnectorTopicPublisher(ros::NodeHandlePtr rosNode, const std::string& topic, unsigned int messageQueueLength = 10) : m_rosTopic(topic), m_rosNodeHandle(rosNode), m_messageQueueLength(messageQueueLength)
    {
        m_messageQueue.resize(m_messageQueueLength);
        messageType = ros::message_traits::DataType<MessageType>::value();

        msg_info("ZyROSConnectorTopicPublisher<" + messageType + ">") << "Constructor";
        publishTopic();
    }

    ~ZyROSConnectorTopicPublisher() {}

    void publishTopic();
    void stopToPublishTopic();

    void publishMessage(const MessageType&);
    void publishMessageQueue();

protected:
    std::string m_rosTopic;
    ros::NodeHandlePtr m_rosNodeHandle;

    ros::Publisher m_publisher;

    boost::circular_buffer<MessageType> m_messageQueue;
    unsigned int m_messageQueueLength;

    boost::mutex m_mutex;
};
}
}

#endif //TRUROSCONNECTOR_TOPIC_PUBLISHER_H
