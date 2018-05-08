#include "ZyROSTopicListener_JointState.h"

#include <ZyVelocityApproximator.h>

using namespace Zyklio::ROSConnector;

class Zyklio::ROSConnector::TruRosJointStateListenerPrivate
{
    public:
        Zyklio::VelocityApproximation::ZyVelocityApproximator* velocityApproximator;
};

SOFA_DECL_CLASS(ZyROSJointStateListener)

ZyROSJointStateListener::ZyROSJointStateListener(ros::NodeHandlePtr nodeHandle, const std::string& rosTopic)
: sceneGraphContext(NULL)
, privateData(NULL)
{
    msg_info("ZyROSJointStateListener") << "Constructor";
    m_topicSubscriber = new ZyROSConnectorTopicSubscriber<sensor_msgs::JointState>(nodeHandle, rosTopic);
    m_topicSubscriber->subscribeToTopic();
    m_topicConnection = m_topicSubscriber->getSignal().connect(boost::bind(&ZyROSJointStateListener::handleJointUpdateMessage, this));

    privateData = new TruRosJointStateListenerPrivate();
}

ZyROSJointStateListener::~ZyROSJointStateListener()
{
	if (m_topicSubscriber != NULL)
    {
        msg_info("ZyROSJointStateListener") << "Destructor";
        m_topicSubscriber->unsubscribeFromTopic();
		delete m_topicSubscriber;
		m_topicSubscriber = NULL;
	}

    if (privateData)
    {
        delete privateData;
    }
}

ZyROSJointStateListener::ZyROSJointStateListener(const ZyROSJointStateListener& other)
{
	if (this != &other)
	{
        ZyROSListener(other);
	}
}

ZyROSJointStateListener& ZyROSJointStateListener::operator=(const ZyROSJointStateListener& other)
{
	if (this != &other)
	{
        ZyROSListener::operator=(other);
	}
	return *this;
}

void ZyROSJointStateListener::cleanup()
{
	if (m_topicConnection.connected())
		m_topicConnection.disconnect();

	if (m_topicSubscriber->getMessageCount() > 0)
	{
		m_topicSubscriber->clearMessages();
	}
}

void ZyROSJointStateListener::setContext(BaseContext* bscon, bool searchForObjectHandler)
{
    sceneGraphContext = bscon;

    if (searchForObjectHandler)
    {
        privateData->velocityApproximator = sceneGraphContext->getRootContext()->get< Zyklio::VelocityApproximation::ZyVelocityApproximator >();
        
        if (privateData->velocityApproximator)
        {
            std::cout << "(ZyROSJointStateListener::setContext) Found ZyVelocityApproximator " << privateData->velocityApproximator->getName() << std::endl;
        }
        else
        {
            std::cout << "(ZyROSJointStateListener::setContext) Could not find ZyVelocityApproximator!" << std::endl;
        }
    }
};

void ZyROSJointStateListener::handleJointUpdateMessage()
{
	const sensor_msgs::JointState& msg = m_topicSubscriber->getLatestMessage();
    msg_info("ZyROSJointStateListener") << "Joint update message received: " << msg;

    if (privateData->velocityApproximator)
    {
        sofa::helper::vector<Zyklio::VelocityApproximation::ZyVelocityApproximator::jointData> jntDtVec;
        for (int k = 0; k < msg.name.size(); k++)
        {
            Zyklio::VelocityApproximation::ZyVelocityApproximator::jointData jntDt(msg.position[k], msg.name[k]);
            jntDtVec.push_back(jntDt);
        }

        privateData->velocityApproximator->pushJointMsg(Zyklio::VelocityApproximation::ZyVelocityApproximator::jointMsg(std::pair<unsigned int, double>(msg.header.seq, msg.header.stamp.toSec()), jntDtVec));
    }
    else
    {
        updateJointState(msg);
    }
}
