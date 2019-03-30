#include "ZyROSConnectorTopicSubscriber.h"

#include <sofa/helper/logging/Messaging.h>

using namespace Zyklio::ROSConnector;

template <class MessageType>
void ZyROSConnectorTopicSubscriber<MessageType>::processMessage(const boost::shared_ptr<MessageType const>& msg)
{
    boost::mutex::scoped_lock lock(m_mutex);
    m_messageQueue.push_front(*(msg.get()));
    //msg_info("ZyROSConnectorTopicSubscriber") << "processMessage " << msg;
    lock.unlock();

    ZyROSConnectorTopicSubscriberIface::onMessageReceived();
}

template <class MessageType>
void ZyROSConnectorTopicSubscriber<MessageType>::subscribeToTopic()
{
    m_subscriber = m_rosNodeHandle->subscribe<MessageType>(m_rosTopic, 10, &ZyROSConnectorTopicSubscriber<MessageType>::processMessage, this);
    msg_info("ZyROSConnectorTopicSubscriber") << "Subscribed to topic " << m_subscriber.getTopic() << ", with " << m_subscriber.getNumPublishers() << " publishers.";
}

template <class MessageType>
void ZyROSConnectorTopicSubscriber<MessageType>::unsubscribeFromTopic()
{
    m_subscriber.shutdown();
}

template <class MessageType>
const MessageType& ZyROSConnectorTopicSubscriber<MessageType>::getMessage(size_t index)
{
    boost::mutex::scoped_lock lock(m_mutex);

    static MessageType empty_message;
    if (index < m_messageQueue.size())
        return m_messageQueue.at(index);

    return empty_message;
}

template <class MessageType>
const MessageType& ZyROSConnectorTopicSubscriber<MessageType>::getLatestMessage()
{
    boost::mutex::scoped_lock lock(m_mutex);

    static MessageType empty_message;
    if (m_messageQueue.size() > 0)
        return m_messageQueue.front();

    return empty_message;
}

template <class MessageType>
unsigned int ZyROSConnectorTopicSubscriber<MessageType>::getMessageCount() const
{
    return m_messageQueue.size();
}

template <class MessageType>
void ZyROSConnectorTopicSubscriber<MessageType>::clearMessages()
{
    boost::mutex::scoped_lock lock(m_mutex);
    m_messageQueue.clear();
}


template <class MessageType>
ZyROSConnectorTopicSubscriber<MessageType>::ZyROSConnectorTopicSubscriber()
  : m_rosNodeHandle(NULL)
  , m_messageQueueLength(10)
{
    m_rosTopic = "";
}

template <class MessageType>
ZyROSConnectorTopicSubscriber<MessageType>::ZyROSConnectorTopicSubscriber(ros::NodeHandlePtr rosNode, const std::string& topic, unsigned int messageQueueLength, bool useGenericMessageHandling)
    : m_rosNodeHandle(rosNode)
    , m_messageQueueLength(messageQueueLength)
    //, m_rosTopic(topic)
{
    m_rosTopic = topic;

    messageType = ros::message_traits::DataType<MessageType>::value();
    msg_info("ZyROSConnectorTopicSubscriber<" + messageType + ">") << "Constructor";
    m_messageQueue.resize(m_messageQueueLength);

    subscribeToTopic();

    if (useGenericMessageHandling)
    {
        m_topicConnection = getSignal().connect(boost::bind(&ZyROSConnectorTopicSubscriber<MessageType>::handleGenericMessage, this));
    }
}

template <class MessageType>
ZyROSConnectorTopicSubscriber<MessageType>::~ZyROSConnectorTopicSubscriber()
{
    msg_info("ZyROSConnectorTopicSubscriber<" + messageType + ">") << "Destructor";
    unsubscribeFromTopic();
}

template <class MessageType>
ZyROSConnectorTopicSubscriber<MessageType>::ZyROSConnectorTopicSubscriber(const ZyROSConnectorTopicSubscriber<MessageType>& other): ZyROSListener(other)
{
    if (this != &other)
    {
        ZyROSConnectorTopicSubscriber<MessageType>(other);
    }
}

template <class MessageType>
ZyROSConnectorTopicSubscriber<MessageType>& ZyROSConnectorTopicSubscriber<MessageType>::operator=(const ZyROSConnectorTopicSubscriber<MessageType>& other)
{
    if (this != &other)
    {
        ZyROSListener::operator=(other);
    }
    return *this;
}

template <class MessageType>
void ZyROSConnectorTopicSubscriber<MessageType>::cleanup()
{
    if (m_topicConnection.connected())
        m_topicConnection.disconnect();

    if (getMessageCount() > 0)
    {
        clearMessages();
    }
}

template <class MessageType>
void ZyROSConnectorTopicSubscriber<MessageType>::handleGenericMessage()
{
    // const MessageType& msg = getLatestMessage();
    // msg_info("ZyROSConnectorTopicSubscriber<" + messageType + ">") << "Message received: " << msg;
}

