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
