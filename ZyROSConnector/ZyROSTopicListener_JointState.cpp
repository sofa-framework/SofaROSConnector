#include "ZyROSTopicListener_JointState.h"

#include <ZyVelocityApproximator.h>

using namespace Zyklio::ROSConnector;

class Zyklio::ROSConnector::TruRosJointStateListenerPrivate
{
    public:
        Zyklio::VelocityApproximation::TruVelocityApproximator* velocityApproximator;
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
        privateData->velocityApproximator = sceneGraphContext->getRootContext()->get< Zyklio::VelocityApproximation::TruVelocityApproximator >();
        
        if (privateData->velocityApproximator)
        {
            std::cout << "(ZyROSJointStateListener::setContext) Found TruVelocityApproximator " << privateData->velocityApproximator->getName() << std::endl;
        }
        else
        {
            std::cout << "(ZyROSJointStateListener::setContext) Could not find TruVelocityApproximator!" << std::endl;
        }
    }
};

void ZyROSJointStateListener::handleJointUpdateMessage()
{
	const sensor_msgs::JointState& msg = m_topicSubscriber->getLatestMessage();
    //msg_info("ZyROSJointStateListener") << "Joint update message received: " << msg;

    if (privateData->velocityApproximator)
    {
        sofa::helper::vector<Zyklio::VelocityApproximation::TruVelocityApproximator::jointData> jntDtVec;
        for (int k = 0; k < msg.name.size(); k++)
        {
            Zyklio::VelocityApproximation::TruVelocityApproximator::jointData jntDt(msg.position[k], msg.name[k]);
            jntDtVec.push_back(jntDt);
        }

        privateData->velocityApproximator->pushJointMsg(Zyklio::VelocityApproximation::TruVelocityApproximator::jointMsg(std::pair<unsigned int, double>(msg.header.seq, msg.header.stamp.toSec()), jntDtVec));
    }
    else
    {
        updateJointState(msg);
    }
}

//void ZyROSJointStateListener::updateJointState(const sensor_msgs::JointState::ConstPtr& joint_state)
void ZyROSJointStateListener::updateJointState(const sensor_msgs::JointState& joint_state)
{
	boost::posix_time::ptime curTime = boost::posix_time::microsec_clock::local_time(); // Cebit 2016 code: the time stuff is tested and works (apparently)
	boost::posix_time::time_duration dur = (curTime - theTime);
	if (dur.total_milliseconds() > 100)
	{
		for (size_t k = 0; k < joint_state.name.size(); k++)
		{
			std::cout << joint_state.name[k] << ";";
			if (m_arbitraryController != NULL)
				m_arbitraryController->setRotValueRadByName(joint_state.position[k], joint_state.name[k]);

			// <!-- untested Cebit 2016 code
			//if (joint_state->name[k] == "pg70_finger_left_joint")
			//{
			//    /*double currentOpening = joint_state->position[k];*/
			//    double openingValue = (1.0-(joint_state->position[k]))*(maxGripperClosing - minGripperClosing);

			//    sofa::component::mapping::RigidMapping<Rigid3dTypes, Vec3dTypes>::VecCoord& currentGripper1Opening = const_cast<sofa::component::mapping::RigidMapping<Rigid3dTypes, Vec3dTypes>::VecCoord&>(gripperMap1->getPoints());

			//    for (unsigned int t = 0; t < currentGripper1Opening.size(); t++)
			//    {
			//        currentGripper1Opening[t][1] = gripperInitialPos1[t][1] - openingValue;
			//        //std::cout << "currentGripper1Opening["<<t<<"][1]" << currentGripper1Opening[t][1] << std::endl;
			//    }

			//    sofa::component::mapping::RigidMapping<Rigid3dTypes, Vec3dTypes>::VecCoord& currentGripper2Opening = const_cast<sofa::component::mapping::RigidMapping<Rigid3dTypes, Vec3dTypes>::VecCoord&>(gripperMap2->getPoints());

			//    for (unsigned int t = 0; t < currentGripper2Opening.size(); t++)
			//    {
			//        currentGripper2Opening[t][1] = gripperInitialPos2[t][1] + openingValue;
			//        //std::cout << "currentGripper2Opening[" << t << "][1]" << currentGripper2Opening[t][1] << std::endl;
			//    }
			//    /*sofa::component::mapping::RigidMapping<Rigid3dTypes, Vec3dTypes>::VecCoord* currentGripper2Opening = (gripperMap2->getPoints());*/
			//}
			// -->
		}

		theTime = boost::posix_time::microsec_clock::local_time();
	}
}
