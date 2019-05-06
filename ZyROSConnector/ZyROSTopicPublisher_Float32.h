#ifndef ROSCONNECTOR_TOPIC_PUBLISHER_FLOAT32_H
#define ROSCONNECTOR_TOPIC_PUBLISHER_FLOAT32_H

#include "ZyROSConnectorTopicPublisher.h"
#include "ZyROSConnector.h"

#include <std_msgs/Float32.h>

using namespace sofa::core::objectmodel;

namespace Zyklio
{
	namespace ROSConnector
	{
        class SOFA_ZY_ROS_CONNECTOR_API ZyROSFloat32Publisher : public ZyROSPublisher
		{
			public:
                ZyROSFloat32Publisher(ros::NodeHandlePtr, const std::string&);
                ~ZyROSFloat32Publisher();

                ZyROSFloat32Publisher(const ZyROSFloat32Publisher&);
                ZyROSFloat32Publisher& operator=(const ZyROSFloat32Publisher&);

                void cleanup();
                void publishMessage(std_msgs::Float32 msg);
                void publishMessageQueue();
		
			protected:

				//boost::signals2::connection m_topicConnection;

                ZyROSConnectorTopicPublisher<std_msgs::Float32>* m_topicPublisher;
		};
	}
}

#endif // ROSCONNECTOR_TOPIC_PUBLISHER_FLOAT32_H
