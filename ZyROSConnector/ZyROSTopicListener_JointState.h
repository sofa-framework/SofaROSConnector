#ifndef TRUROSCONNECTOR_TOPIC_LISTENER_JOINTSTATE_H
#define TRUROSCONNECTOR_TOPIC_LISTENER_JOINTSTATE_H

#include "ZyROSConnectorTopicSubscriber.h"
#include "ZyROSConnector.h"

#include <sensor_msgs/JointState.h>

#include <ArbitraryController.h>
#include <sofa/core/objectmodel/BaseContext.h>

using namespace sofa::core::objectmodel;
using namespace sofa::component::controller;

namespace Zyklio
{
	namespace ROSConnector
	{
        class TruRosJointStateListenerPrivate;
        class SOFA_ZY_ROS_CONNECTOR_API ZyROSJointStateListener : public ZyROSListener
		{
			public:
                ZyROSJointStateListener(ros::NodeHandlePtr, const std::string&);
                ~ZyROSJointStateListener();

                ZyROSJointStateListener(const ZyROSJointStateListener&);
                ZyROSJointStateListener& operator=(const ZyROSJointStateListener&);

				void cleanup();

                void setContext(BaseContext* bscon, bool searchForObjectHandler = true);
                void setArbitraryController(ArbitraryController* arbcon) { m_arbitraryController = arbcon; }
		
			protected:
				void handleJointUpdateMessage();
                
		    private:
                TruRosJointStateListenerPrivate* privateData;

		    protected:
                //void updateJointState(const sensor_msgs::JointState::ConstPtr& joint_state);
                void updateJointState(const sensor_msgs::JointState& joint_state);

				boost::signals2::connection m_topicConnection;

                ZyROSConnectorTopicSubscriber<sensor_msgs::JointState>* m_topicSubscriber;

                BaseContext* sceneGraphContext;
				ArbitraryController* m_arbitraryController;
				boost::posix_time::ptime theTime;
		};
	}
}

#endif // TRUROSCONNECTOR_TOPIC_LISTENER_JOINTSTATE_H
