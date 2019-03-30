#ifndef TRUROSCONNECTOR_TOPIC_LISTENER_BOOL_H
#define TRUROSCONNECTOR_TOPIC_LISTENER_BOOL_H

#include "ZyROSConnectorTopicSubscriber.h"
#include "ZyROSConnector.h"

#include <std_msgs/Bool.h>

#include <sofa/core/objectmodel/BaseContext.h>

using namespace sofa::core::objectmodel;

namespace Zyklio
{
	namespace ROSConnector
    {
        class SOFA_ZY_ROS_CONNECTOR_API ZyROSBoolListener : public ZyROSListener
		{
			public:
                ZyROSBoolListener(ros::NodeHandlePtr, const std::string&);
                ~ZyROSBoolListener();

                ZyROSBoolListener(const ZyROSBoolListener&);
                ZyROSBoolListener& operator=(const ZyROSBoolListener&);

				void cleanup();

                void setContext(BaseContext* bscon);

                //boost::signals2::connection* getConnectionSignal() { return &m_topicConnection; }

                boost::signals2::signal<void()>& getSignal();
                std::string getTopic() { return m_topicSubscriber->getTopic();}
		
			protected:
				void handleBoolUpdateMessage();
                
		    /*private:
                TruRosBoolListenerPrivate* privateData;*/

		    protected:
				//boost::signals2::connection m_topicConnection;

                ZyROSConnectorTopicSubscriber<std_msgs::Bool>* m_topicSubscriber;

                BaseContext* sceneGraphContext;
				boost::posix_time::ptime theTime;

                //void handleGrippers(const std_msgs::Bool& joint_state);
                //bool currentGripperOpenSignal; // open gripper if true, do nothing if false
                //bool currentGripperCloseSignal; // close gripper if true, do nothing if false
                //// if both are true, do nothing
		};
	}
}

#endif // TRUROSCONNECTOR_TOPIC_LISTENER_BOOL_H
