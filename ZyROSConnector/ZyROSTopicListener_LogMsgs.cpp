#include "ZyROSTopicListener_LogMsgs.h"

using namespace Zyklio::ROSConnector;

SOFA_DECL_CLASS(ZyROSLogListener)

ZyROSLogListener::ZyROSLogListener(ros::NodeHandlePtr nodeHandle, const std::string& rosTopic)
{
    msg_info("ZyROSLogListener") << "Constructor";
    m_topicSubscriber = new ZyROSConnectorTopicSubscriber<rosgraph_msgs::Log>(nodeHandle, rosTopic);
	m_topicSubscriber->subscribeToTopic();
    m_topicConnection = m_topicSubscriber->getSignal().connect(boost::bind(&ZyROSLogListener::handleLogMessage, this));
}

ZyROSLogListener::~ZyROSLogListener()
{
	if (m_topicSubscriber != NULL)
	{	
		m_topicSubscriber->unsubscribeFromTopic();

		delete m_topicSubscriber;
		m_topicSubscriber = NULL;
	}
}

ZyROSLogListener::ZyROSLogListener(const ZyROSLogListener& other)
{
	if (this != &other)
	{
        ZyROSListener(other);
	}
}

ZyROSLogListener& ZyROSLogListener::operator=(const ZyROSLogListener& other)
{
	if (this != &other)
	{
        ZyROSListener::operator=(other);
	}
	return *this;
}

void ZyROSLogListener::cleanup()
{
	if (m_topicConnection.connected())
		m_topicConnection.disconnect();

	if (m_topicSubscriber->getMessageCount() > 0)
	{
		m_topicSubscriber->clearMessages();
	}
}

void ZyROSLogListener::handleLogMessage()
{
	const rosgraph_msgs::Log& msg = m_topicSubscriber->getLatestMessage();
    msg_info("ZyROSLogListener") << "Log message received: " << msg;
}
