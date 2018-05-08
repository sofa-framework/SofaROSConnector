#ifndef TRUROSCONNECTOR_TOPIC_PUBLISHER_FLOAT32MULTIARRAY_H
#define TRUROSCONNECTOR_TOPIC_PUBLISHER_FLOAT32MULTIARRAY_H

#include "ZyROSConnectorTopicPublisher.h"
#include "ZyROSConnector.h"

#include <std_msgs/Float32MultiArray.h>

using namespace sofa::core::objectmodel;

namespace Zyklio
{
	namespace ROSConnector
	{
        class SOFA_ZY_ROS_CONNECTOR_API ZyROSFloat32MultiArrayPublisher : public ZyROSPublisher
		{
			public:
                ZyROSFloat32MultiArrayPublisher(ros::NodeHandlePtr, const std::string&);
                ~ZyROSFloat32MultiArrayPublisher();

                ZyROSFloat32MultiArrayPublisher(const ZyROSFloat32MultiArrayPublisher&);
                ZyROSFloat32MultiArrayPublisher& operator=(const ZyROSFloat32MultiArrayPublisher&);

                void cleanup();
                void publishMessage(std_msgs::Float32MultiArray msg);
                void publishMessageQueue();
		
			protected:

				//boost::signals2::connection m_topicConnection;

                ZyROSConnectorTopicPublisher<std_msgs::Float32MultiArray>* m_topicPublisher;
		};
	}
}

#endif // TRUROSCONNECTOR_TOPIC_PUBLISHER_FLOAT32MULTIARRAY_H
