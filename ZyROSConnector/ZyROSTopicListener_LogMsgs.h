#ifndef TRUROSCONNECTOR_TOPIC_LISTENER_LOG_H
#define TRUROSCONNECTOR_TOPIC_LISTENER_LOG_H

#include "ZyROSConnectorTopicSubscriber.h"
#include "ZyROSConnector.h"

// Include order and the #undef are important; ROS defines an enum named "ERROR".
#undef ERROR
#include <rosgraph_msgs/Log.h>

using namespace sofa::core::objectmodel;

namespace Zyklio
{
	namespace ROSConnector
	{
		// This listener is currently used for the ROS connector test case only
        class SOFA_ZY_ROS_CONNECTOR_API ZyROSLogListener : public ZyROSListener
		{
		public:
            ZyROSLogListener(ros::NodeHandlePtr, const std::string&);
            ~ZyROSLogListener();

            ZyROSLogListener(const ZyROSLogListener&);
            ZyROSLogListener& operator=(const ZyROSLogListener&);

			void cleanup();

		protected:
			void handleLogMessage();
			boost::signals2::connection m_topicConnection;
            ZyROSConnectorTopicSubscriber<rosgraph_msgs::Log>* m_topicSubscriber;
		};
	}
}

#endif // TRUROSCONNECTOR_TOPIC_LISTENER_LOG_H
