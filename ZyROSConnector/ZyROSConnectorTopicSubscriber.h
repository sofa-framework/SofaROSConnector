#ifndef TRUROSCONNECTOR_TOPIC_SUBSCRIBER_H
#define TRUROSCONNECTOR_TOPIC_SUBSCRIBER_H

#ifdef _WIN32
// Ensure that Winsock2.h is included before Windows.h, which can get
// pulled in by anybody (e.g., Boost).
#include <Winsock2.h>
#endif

#include "init_ZyROSConnector.h"

#include <string>
#include <ros/node_handle.h>

#include <boost/circular_buffer.hpp>
#include <boost/signals2/signal.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include "sofa/helper/logging/Messaging.h"

namespace Zyklio
{
    namespace ROSConnector
    {
        class SOFA_ZY_ROS_CONNECTOR_API TruRosConnectorTopicSubscriberIface
        {
        public:
            void onMessageReceived();
            boost::signals2::signal<void()>& getSignal() { return m_sig; }

        protected:
            boost::signals2::signal<void()> m_sig;
        };

        class SOFA_ZY_ROS_CONNECTOR_API ZyROSListener : public TruRosConnectorTopicSubscriberIface
        {
        public:
            ZyROSListener();
            virtual ~ZyROSListener() {}

            ZyROSListener(const ZyROSListener&);
            ZyROSListener& operator=(const ZyROSListener&);

            virtual void cleanup() { std::cout << "WARNING:  cleanup() not implemented for a ZyROSListener." << std::endl; }

            const boost::uuids::uuid& getUuid() { return m_uuid; }

            std::string getMessageType() { return messageType; }
            //std::string getTopic() const { return m_rosTopic; };
            virtual std::string getTopic() { std::cout << "WARNING: getTopic() not implemented for a ZyROSListener." << std::endl; return ""; }

        protected:
            boost::uuids::uuid m_uuid;
            std::string m_rosTopic;

            boost::signals2::connection m_topicConnection;
            std::string messageType;
        };

        template <class MessageType>
        class SOFA_ZY_ROS_CONNECTOR_API ZyROSConnectorTopicSubscriber : public ZyROSListener
        {
        public:

            void processMessage(const boost::shared_ptr<MessageType const>& msg);
            void subscribeToTopic();
            void unsubscribeFromTopic();

            const MessageType& getLatestMessage();
            const MessageType& getMessage(size_t);
            unsigned int getMessageCount() const;

            void clearMessages();

            ZyROSConnectorTopicSubscriber();
            ZyROSConnectorTopicSubscriber(ros::NodeHandlePtr rosNode, const std::string& topic, unsigned int messageQueueLength = 50, bool useGenericMessageHandling = false);
            ~ZyROSConnectorTopicSubscriber();

            ZyROSConnectorTopicSubscriber(const ZyROSConnectorTopicSubscriber& other);
            ZyROSConnectorTopicSubscriber& operator=(const ZyROSConnectorTopicSubscriber& other);

            void cleanup();
            void handleGenericMessage();

            std::string getTopic() const { return m_rosTopic; }

        protected:
            std::string m_rosTopic;
            ros::NodeHandlePtr m_rosNodeHandle;

            ros::Subscriber m_subscriber;

            boost::circular_buffer<MessageType> m_messageQueue;
            unsigned int m_messageQueueLength;

            boost::mutex m_mutex;
        };
    }
}

#endif // TRUROSCONNECTOR_TOPIC_SUBSCRIBER_H
