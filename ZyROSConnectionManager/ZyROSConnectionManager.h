#ifndef ZYKLIO_ROSCONNECTIONMANAGER_H
#define ZYKLIO_ROSCONNECTIONMANAGER_H

#ifdef _WIN32
// Ensure that Winsock2.h is included before Windows.h, which can get
// pulled in by anybody (e.g., Boost).
#ifndef WINSOCK2_H
#define WINSOCK2_H
#include <Winsock2.h>
#endif
#endif

#include "initZyROSConnectionManager.h"

#include <ZyROSConnector.h>
#include <sofa/core/objectmodel/BaseObject.h>


using namespace sofa::core::objectmodel;
using namespace Zyklio::ROSConnector;

namespace Zyklio
{
    namespace ROSConnectionManager
    {
        class SOFA_ZY_ROS_CONNECTION_MANAGER_API ZyROSConnectionManager : public sofa::core::objectmodel::BaseObject
        {
        public:
            SOFA_CLASS(ZyROSConnectionManager, sofa::core::objectmodel::BaseObject);

            ZyROSConnectionManager();
            virtual ~ZyROSConnectionManager();

            void init();
            void bwdInit();
            void reset();
            void cleanup();

            void addPublisher(boost::shared_ptr<ZyROSPublisher>& pub);

            void addSubscriber(boost::shared_ptr<ZyROSListener> &sub);

            template <class MessageType>
            void addSubscriber(boost::shared_ptr<ZyROSConnectorTopicSubscriber<MessageType>>& sub)
            {
                boost::shared_ptr<ZyROSListener> subscriber_ptr = boost::dynamic_pointer_cast<ZyROSListener>(sub);
                addSubscriber(subscriber_ptr);
            }

            template <class MessageType>
            boost::shared_ptr<ZyROSConnectorTopicSubscriber<MessageType>> subscribeToTopic(std::string topic)
            {
                boost::shared_ptr<ZyROSConnectorTopicSubscriber<MessageType>> sub(new ZyROSConnectorTopicSubscriber<MessageType>(m_ros_connector->getROSNode(), topic));
                addSubscriber<MessageType>(sub);
                return sub;
            }

            // I am not a huge fan of this, but I don't see any other way, if other classes are supposed to be able to create publishers
            ros::NodeHandlePtr getRosNodeHandle() { return m_ros_connector->getROSNode(); }

            std::vector< boost::shared_ptr<ZyROSListener> >* getTopicListeners() { return &topicListeners; }

            // return all subscribers with a message type that matches the given template parameter
            template <class MessageType>
            std::vector< ZyROSConnectorTopicSubscriber<MessageType>* > getSubscribers();

            /**
             * @brief getTopics - retrieve a list of all known ROS topics
             * @return List of all ROS topic URLs
             */
            const std::vector<std::string> getTopics() const;

            /**
             * @brief getServices - retrieve a list of all known ROS services
             * @return List of all ROS service URLs
             */
            const std::vector<std::string> getServices() const;

            /**
             * @brief getNodes - retrieve a list of all known ROS node names
             * @return List of all node names
             */
            const std::vector<std::string> getNodes() const;

        protected:

            ZyROSConnector* m_ros_connector;

            Data<std::string> m_rosMasterURI;
            Data<std::string> m_rosTopics;

            std::vector< boost::shared_ptr<ZyROSListener> > topicListeners;
            std::vector< boost::shared_ptr<ZyROSPublisher> > topicPublishers;

            boost::mutex m_mutex;
        };
    }
}

#endif //ZYKLIO_ROSCONNECTIONMANAGER_H
