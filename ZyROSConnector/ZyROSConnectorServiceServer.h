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
                ZyROSConnectorServiceServer(/*ros::NodeHandlePtr rosNode,*/ const std::string&);
                ZyROSConnectorServiceServer(const ZyROSConnectorServiceServer&);
                ~ZyROSConnectorServiceServer();

                virtual bool advertiseService();
                virtual bool stopAdvertisingService();
                virtual void shutdownServer();

                template <class Request, class Response, class RequestHandler>
                bool handleRequest(const Request& req, Response& resp);

            protected:
                // friend class ZyROSConnectorServiceServerWorkerThread;
                void serverLoop();

                ZyROSConnectorServiceServerPrivate* m_d;

                bool m_shutdownRequested;
                bool m_serverThreadActive;
                boost::mutex m_mutex;
        };

        /*struct ZyROSConnectorServerRequestHandlerBase
        {
            virtual bool handleRequest() = 0;
        };*/

        template <class Request, class Response>
        struct ZyROSConnectorServerRequestHandler //: public ZyROSConnectorServerRequestHandlerBase
        {
            public:
                // ZyROSConnectorServerRequestHandler() {}

                ZyROSConnectorServerRequestHandler(/*const Request& request, Response& response, */void* handler_param = nullptr)
                {
                    /*req = request;
                    resp = response;*/
                    param = handler_param;
                }

                // Request req;
                // Response resp;
                void* param;

           bool handleRequest(const Request& req, Response& resp)
           {
                msg_info("ZyROSConnectorServerRequestHandler") << "handleRequest()";
                return false;
           }
        };

        template <class RequestType, class ResponseType, class RequestHandler = ZyROSConnectorServerRequestHandler<RequestType, ResponseType> >
        class SOFA_ZY_ROS_CONNECTOR_API ZyROSConnectorServiceServerImpl: public ZyROSConnectorServiceServer
        {
            public:
                ZyROSConnectorServiceServerImpl(/*ros::NodeHandlePtr,*/ const std::string&);
                ~ZyROSConnectorServiceServerImpl();

                virtual bool advertiseService();

                virtual bool handleRequest(const RequestType& req, ResponseType& resp) /*= 0*/;

            protected:
                RequestHandler m_requestHandler;
        };

        template <class RequestType, class ResponseType, class RequestHandler>
        class SOFA_ZY_ROS_CONNECTOR_API ZyROSConnectorServiceServerWorkerThread: public WorkerThread_SingleTask
        {
            public:
                ZyROSConnectorServiceServerWorkerThread(boost::shared_ptr<ZyROSConnectorServiceServerImpl<RequestType, ResponseType, RequestHandler>>);

            protected:
                    void main();

            private:
                friend class ZyROSConnectorServiceServer;

                void (ZyROSConnectorServiceServer::*m_func)(void);
                boost::shared_ptr<ZyROSConnectorServiceServerImpl<RequestType, ResponseType, RequestHandler> > m_serviceServer;
        };
    }
}

#endif // ZY_ROS_CONNECTOR_SERVICE_SERVER_H
