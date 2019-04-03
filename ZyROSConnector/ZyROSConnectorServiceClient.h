#ifndef ZY_ROS_CONNECTOR_SERVICE_CLIENT_H
#define ZY_ROS_CONNECTOR_SERVICE_CLIENT_H

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
        class SOFA_ZY_ROS_CONNECTOR_API ZyROSConnectorServiceClientIface
        {
            public:
                void onResponseReceived();
                boost::signals2::signal<void()>& getSignal() { return m_sig; }

            protected:
                boost::signals2::signal<void()> m_sig;
        };

        class SOFA_ZY_ROS_CONNECTOR_API ZyROSServiceClient : public ZyROSConnectorServiceClientIface
        {
            public:
                ZyROSServiceClient();
                virtual ~ZyROSServiceClient() {}

                ZyROSServiceClient(const ZyROSServiceClient&);
                ZyROSServiceClient& operator=(const ZyROSServiceClient&);

                virtual void cleanup() { msg_info("ZyROSServiceClient") << "cleanup() not implemented for a ZyROSServiceClient!"; }

                const boost::uuids::uuid& getUuid() { return m_uuid; }

                virtual std::string getServiceURI() { msg_warning("ZyROSServiceClient") << "getServiceURI() not implemented for a ZyROSServiceClient!"; return ""; }

            protected:
                boost::uuids::uuid m_uuid;
                std::string m_rosServiceURI;

                boost::signals2::connection m_topicConnection;
        };

        template <class ServiceType, class RequestType, class ResponseType>
        class SOFA_ZY_ROS_CONNECTOR_API ZyROSConnectorServiceClient : public ZyROSServiceClient
        {
            public:
                ZyROSConnectorServiceClient(ros::NodeHandlePtr nodeHandle, const std::string& serviceURI, unsigned int queueLength = 10);

                bool enqueueRequest(const RequestType&);
                void clearRequests();

                unsigned int getNumResponses() const;
                const ResponseType& getLatestResponse() const;
                const ResponseType& getResponse(size_t);

            protected:
                void dispatchRequests();

                ros::NodeHandlePtr m_rosNodeHandle;
                std::string m_serviceURI;

                ros::ServiceClientPtr m_client;

                boost::circular_buffer<RequestType> m_requestQueue;
                boost::circular_buffer<ResponseType> m_responseQueue;

                unsigned int m_messageQueueLength;

                boost::mutex m_mutex;
        };
    }

}

#endif // ZY_ROS_CONNECTOR_SERVICE_CLIENT_H
