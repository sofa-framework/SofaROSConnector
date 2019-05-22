#ifndef ZY_ROS_CONNECTOR_SERVICE_SERVER_H
#define ZY_ROS_CONNECTOR_SERVICE_SERVER_H


#ifdef _WIN32
// Ensure that Winsock2.h is included before Windows.h, which can get
// pulled in by anybody (e.g., Boost).
#include <Winsock2.h>
#endif

#include "init_ZyROSConnector.h"
#include <ZyMultiThreading/WorkerThread_SingleTask.h>

#include <string>
#include <ros/node_handle.h>

#include <boost/circular_buffer.hpp>
#include <boost/signals2/signal.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>

#include <boost/thread/condition.hpp>

#include "sofa/helper/logging/Messaging.h"

namespace Zyklio
{
    namespace ROSConnector
    {
        using namespace Zyklio::MultiThreading;

        // Data members in private implementation
        class ZyROSConnectorServiceServerPrivate;
        class SOFA_ZY_ROS_CONNECTOR_API ZyROSConnectorServiceServer
        {
            public:
                ZyROSConnectorServiceServer(ros::NodeHandlePtr rosNode, const std::string&);
                ZyROSConnectorServiceServer(const ZyROSConnectorServiceServer&);
                ~ZyROSConnectorServiceServer();

                virtual bool advertiseService();
                virtual bool stopAdvertisingService();

                virtual void shutdownServer();

            protected:
                friend class ZyROSConnectorServiceServerWorkerThread;
                void serverLoop();

                ZyROSConnectorServiceServerPrivate* m_d;

                bool m_shutdownRequested;
                bool m_serverThreadActive;
                boost::mutex m_mutex;
        };

        struct ZyROSConnectorServerRequestHandlerBase
        {
            virtual bool handleRequest() = 0;
        };

        template <class Request, class Response>
        struct ZyROSConnectorServerRequestHandler : public ZyROSConnectorServerRequestHandlerBase
        {
            public:
                ZyROSConnectorServerRequestHandler(const Request& request, Response& response)
                {
                    req = request;
                    resp = response;
                }

                Request req;
                Response resp;

           bool handleRequest()
           {
                msg_info("ZyROSConnectorServerRequestHandler") << "handleRequest()";
                return false;
           }
        };

        /*class SOFA_ZY_ROS_CONNECTOR_API ZyROSServerRequestHandler
        {
            public:
                template <class RequestType, class ResponseType> bool handleRequest(const RequestType&, ResponseType&)
                {
                    msg_warning("ZyROSServerRequestHandler") << "handleRequest needs to be overridden in derived class for custom logic.";
                    return false;
                }
        };*/

        template <class RequestType, class ResponseType, class RequestHandler>
        class SOFA_ZY_ROS_CONNECTOR_API ZyROSConnectorServiceServerImpl: public ZyROSConnectorServiceServer
        {
            public:
                ZyROSConnectorServiceServerImpl(ros::NodeHandlePtr, const std::string&);
                ~ZyROSConnectorServiceServerImpl();

                virtual bool advertiseService();
                virtual bool stopAdvertisingService();

                bool handleRequest(const RequestType& req, ResponseType& resp);
        };

        class SOFA_ZY_ROS_CONNECTOR_API ZyROSConnectorServiceServerWorkerThread: public WorkerThread_SingleTask
        {
            public:
                ZyROSConnectorServiceServerWorkerThread(boost::shared_ptr<ZyROSConnectorServiceServer>&);

                bool advertiseService();
                bool stopAdvertisingService();

            protected:
                    void main();

            private:
                friend class ZyROSConnectorServiceServer;

                void (ZyROSConnectorServiceServer::*m_func)(void);
                boost::shared_ptr<ZyROSConnectorServiceServer> m_serviceServer;
        };
    }
}

#endif // ZY_ROS_CONNECTOR_SERVICE_SERVER_H
