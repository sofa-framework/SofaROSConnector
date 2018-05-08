#include "ZyROSTopicListener_Bool.h"

#include <sofa/simulation/Node.h>

using namespace Zyklio::ROSConnector;

//class Zyklio::ROSConnector::TruRosBoolListenerPrivate
//{
//    public:
//        Zyklio::VelocityApproximation::TruVelocityApproximator* velocityApproximator;
//};

SOFA_DECL_CLASS(ZyROSBoolListener)

ZyROSBoolListener::ZyROSBoolListener(ros::NodeHandlePtr nodeHandle, const std::string& rosTopic)
: sceneGraphContext(NULL)
//, privateData(NULL)
//, currentGripperOpenSignal(false)
//, currentGripperCloseSignal(false)
{
    msg_info("ZyROSBoolListener") << "Constructor";
    m_topicSubscriber = new ZyROSConnectorTopicSubscriber<std_msgs::Bool>(nodeHandle, rosTopic);
    m_topicSubscriber->subscribeToTopic();
    m_topicConnection = m_topicSubscriber->getSignal().connect(boost::bind(&ZyROSBoolListener::handleBoolUpdateMessage, this));

    //privateData = new TruRosBoolListenerPrivate();
    // tst begin
    messageType = ros::message_traits::DataType<std_msgs::Bool>::value();//"std_msgs/Bool";
    // tst end
}

ZyROSBoolListener::~ZyROSBoolListener()
{
	if (m_topicSubscriber != NULL)
    {
        msg_info("ZyROSBoolListener") << "Destructor";
        m_topicSubscriber->unsubscribeFromTopic();
		delete m_topicSubscriber;
		m_topicSubscriber = NULL;
	}

    /*if (privateData)
    {
        delete privateData;
    }*/
}

ZyROSBoolListener::ZyROSBoolListener(const ZyROSBoolListener& other)
{
	if (this != &other)
	{
		ZyROSListener(other);
	}
}

ZyROSBoolListener& ZyROSBoolListener::operator=(const ZyROSBoolListener& other)
{
	if (this != &other)
	{
		ZyROSListener::operator=(other);
	}
	return *this;
}

void ZyROSBoolListener::cleanup()
{
	if (m_topicConnection.connected())
		m_topicConnection.disconnect();

	if (m_topicSubscriber->getMessageCount() > 0)
	{
		m_topicSubscriber->clearMessages();
	}
}

void ZyROSBoolListener::setContext(BaseContext* bscon)
{
    sceneGraphContext = bscon;
}

boost::signals2::signal<void()>& ZyROSBoolListener::getSignal()
{
    return m_topicSubscriber->getSignal();
}

void ZyROSBoolListener::handleBoolUpdateMessage()
{
    //const std_msgs::Bool& msg = m_topicSubscriber->getLatestMessage();
	//msg_info("ZyROSBoolListener") << "Bool update message received: " << msg;

    //handleGrippers(msg);
}

//void ZyROSBoolListener::handleGrippers(const std_msgs::Bool& gripperState)
//{
//    sofa::simulation::Node* root = dynamic_cast<sofa::simulation::Node*>(sceneGraphContext->getRootContext());
//    if (root == NULL) return;
//
//    std::vector< GripperHandling::ZyGripping* > gripperHandlers;
//    root->getTreeObjects< GripperHandling::ZyGripping >(&gripperHandlers);
//
//    bool gripperSignal = gripperState.data;
//    if (m_topicSubscriber->getTopic().compare("/gripper_state_open") == 0)
//    {
//        if (gripperSignal != currentGripperOpenSignal)
//        {
//            currentGripperOpenSignal = gripperSignal;
//            if (gripperSignal)
//            {
//                // open grippers
//                msg_info("ZyROSBoolListener") << "Opening grippers ";
//
//                for (std::vector< GripperHandling::ZyGripping* >::const_iterator it = gripperHandlers.begin(); it != gripperHandlers.end(); it++)
//                {
//                    //msg_info("ZyROSBoolListener") << (*it)->getName();
//                    (*it)->startOpening();
//                }
//            }
//        }
//    }
//
//    if (m_topicSubscriber->getTopic().compare("/gripper_state_close") == 0)
//    {
//        if (gripperSignal != currentGripperCloseSignal)
//        {
//            currentGripperCloseSignal = gripperSignal;
//            if (gripperSignal)
//            {
//                // close grippers
//                msg_info("ZyROSBoolListener") << "Closing grippers ";
//
//                for (std::vector< GripperHandling::ZyGripping* >::const_iterator it = gripperHandlers.begin(); it != gripperHandlers.end(); it++)
//                {
//                    //msg_info("ZyROSBoolListener") << (*it)->getName();
//                    (*it)->startClosing();
//                }
//            }
//        }
//    }
//}
