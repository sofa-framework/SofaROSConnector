#include "ZyROSTopicPublisher_Float32.h"

using namespace Zyklio::ROSConnector;

SOFA_DECL_CLASS(ZyROSFloat32Publisher)

ZyROSFloat32Publisher::ZyROSFloat32Publisher(ros::NodeHandlePtr nodeHandle, const std::string& rosTopic)
{
    msg_info("ZyROSFloat32Publisher") << "Constructor";
    m_topicPublisher = new ZyROSConnectorTopicPublisher<std_msgs::Float32>(nodeHandle, rosTopic);
    m_topicPublisher->publishTopic();
    //m_topicConnection = m_topicPublisher->getSignal().connect(boost::bind(&ZyROSFloat32Publisher::handleJointUpdateMessage, this));
}

ZyROSFloat32Publisher::~ZyROSFloat32Publisher()
{
	if (m_topicPublisher != NULL)
    {
        msg_info("ZyROSFloat32Publisher") << "Destructor";
        m_topicPublisher->stopToPublishTopic();
		delete m_topicPublisher;
		m_topicPublisher = NULL;
	}
}

ZyROSFloat32Publisher::ZyROSFloat32Publisher(const ZyROSFloat32Publisher& other)
{
	if (this != &other)
	{
        ZyROSPublisher(other);
	}
}

ZyROSFloat32Publisher& ZyROSFloat32Publisher::operator=(const ZyROSFloat32Publisher& other)
{
	if (this != &other)
	{
        ZyROSPublisher::operator=(other);
	}
	return *this;
}

void ZyROSFloat32Publisher::publishMessage(std_msgs::Float32 msg)
{
    m_topicPublisher->publishMessage(msg);
}

void ZyROSFloat32Publisher::publishMessageQueue()
{
    m_topicPublisher->publishMessageQueue();
}

void ZyROSFloat32Publisher::cleanup()
{
	/*if (m_topicConnection.connected())
		m_topicConnection.disconnect();

	if (m_topicPublisher->getMessageCount() > 0)
	{
		m_topicPublisher->clearMessages();
	}*/
}
