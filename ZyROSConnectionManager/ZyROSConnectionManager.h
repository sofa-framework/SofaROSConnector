#ifndef ZYKLIO_TRUROSCONNECTIONMANAGER_H
#define ZYKLIO_TRUROSCONNECTIONMANAGER_H

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

            //bool setRosMasterURI(const std::string&);

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
            std::vector< ZyROSConnectorTopicSubscriber<MessageType>* > getSubscribers()
            {
                std::vector< ZyROSConnectorTopicSubscriber<MessageType>* > subs;

                for (std::vector< boost::shared_ptr<ZyROSListener> >::iterator it = topicListeners.begin();
                    it != topicListeners.end(); it++)
                {
                    if ((*it)) // TODO: find out why there are sometimes NULL pointers in this vector
                    {
                        if ((*it)->getMessageType().compare(ros::message_traits::DataType<MessageType>::value()) == 0)
                        {
                            subs.push_back(&(*it));
                        }
                    }
                }

                return subs;
            }

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

#endif //ZYKLIO_TRUROSCONNECTIONMANAGER_H
