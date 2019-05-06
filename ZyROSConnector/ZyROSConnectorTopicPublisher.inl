#include "ZyROSConnectorTopicPublisher.h"

#include <sofa/helper/logging/Messaging.h>

using namespace Zyklio::ROSConnector;

template <class MessageType>
void ZyROSConnectorTopicPublisher<MessageType>::publishTopic()
{
    m_publisher = m_rosNodeHandle->advertise<MessageType>(m_rosTopic,25,false);
	msg_info("ZyROSConnectorTopicPublisher") << "Publishing topic " << m_publisher.getTopic();
}

template <class MessageType>
void ZyROSConnectorTopicPublisher<MessageType>::stopToPublishTopic()
{
	m_publisher.shutdown();
}

template <class MessageType>
void ZyROSConnectorTopicPublisher<MessageType>::publishMessage(const MessageType& msg)
{
    boost::mutex::scoped_lock lock(m_mutex);
    m_messageQueue.push_front(msg);
    lock.unlock();
}
