#include "ZyROSTopicPublisher_Float32MultiArray.h"

using namespace Zyklio::ROSConnector;

SOFA_DECL_CLASS(ZyROSFloat32MultiArrayPublisher)

ZyROSFloat32MultiArrayPublisher::ZyROSFloat32MultiArrayPublisher(ros::NodeHandlePtr nodeHandle, const std::string& rosTopic)
{
    msg_info("ZyROSFloat32MultiArrayPublisher") << "Constructor";
    m_topicPublisher = new ZyROSConnectorTopicPublisher<std_msgs::Float32MultiArray>(nodeHandle, rosTopic);
    m_topicPublisher->publishTopic();
    //m_topicConnection = m_topicPublisher->getSignal().connect(boost::bind(&ZyROSFloat32MultiArrayPublisher::handleJointUpdateMessage, this));
}

ZyROSFloat32MultiArrayPublisher::~ZyROSFloat32MultiArrayPublisher()
{
	if (m_topicPublisher != NULL)
    {
        msg_info("ZyROSFloat32MultiArrayPublisher") << "Destructor";
        m_topicPublisher->stopToPublishTopic();
		delete m_topicPublisher;
		m_topicPublisher = NULL;
	}
}

ZyROSFloat32MultiArrayPublisher::ZyROSFloat32MultiArrayPublisher(const ZyROSFloat32MultiArrayPublisher& other)
{
	if (this != &other)
	{
        ZyROSPublisher(other);
	}
}

ZyROSFloat32MultiArrayPublisher& ZyROSFloat32MultiArrayPublisher::operator=(const ZyROSFloat32MultiArrayPublisher& other)
{
	if (this != &other)
	{
        ZyROSPublisher::operator=(other);
	}
	return *this;
}

void ZyROSFloat32MultiArrayPublisher::publishMessage(std_msgs::Float32MultiArray msg)
{
    m_topicPublisher->publishMessage(msg);
}

void ZyROSFloat32MultiArrayPublisher::publishMessageQueue()
{
    m_topicPublisher->publishMessageQueue();
}

void ZyROSFloat32MultiArrayPublisher::cleanup()
{
	/*if (m_topicConnection.connected())
		m_topicConnection.disconnect();

	if (m_topicPublisher->getMessageCount() > 0)
	{
		m_topicPublisher->clearMessages();
	}*/
}
