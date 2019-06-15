#include "ZyROSConnectorServiceServer.h"

namespace Zyklio
{
    namespace ROSConnector
    {
        class ZyROSConnectorServiceServerPrivate
        {
            public:
                ros::NodeHandlePtr m_rosNodeHandle;
                ros::ServiceServer m_rosServer;

                std::string m_serviceURI;
        };
    }
}

using namespace Zyklio::ROSConnector;

bool ZyROSConnectorServiceServer::advertiseService()
{
    return false;
}

template <class RequestType, class ResponseType, class RequestHandler>
bool ZyROSConnectorServiceServerImpl<RequestType, ResponseType, RequestHandler>::advertiseService()
{
    ros::AdvertiseServiceOptions service_options;
    boost::function<bool(RequestType&, ResponseType&)> callback_fn = boost::bind(&ZyROSConnectorServiceServerImpl<RequestType, ResponseType, RequestHandler>::handleRequest, this, _1, _2);
    service_options.init(m_d->m_serviceURI, callback_fn);

    // Create a local/anonymous node handle if the one passed to the constructor is invalid for some reason. Scene reset...
    if (m_d->m_rosNodeHandle == nullptr)
        m_d->m_rosNodeHandle.reset(new ros::NodeHandle());

    m_d->m_rosServer = m_d->m_rosNodeHandle->advertiseService(service_options);
    if (m_d->m_rosServer)
    {
        msg_info("ZyROSConnectorServiceServer") << "Successfully advertised ROS service.";
        return true;
    }
    else
    {
        msg_error("ZyROSConnectorServiceServer") << "Failed to advertise ROS service!";
        return false;
    }
}


bool ZyROSConnectorServiceServer::stopAdvertisingService()
{
    if (m_d->m_rosServer)
    {
        msg_info("ZyROSConnectorServiceServerImpl") << "Shutting down ROS service.";
        m_d->m_rosServer.shutdown();
        return true;
    }
    msg_info("ZyROSConnectorServiceServerImpl") << "ROS service already shut down, or not yet advertised. Doing nothing.";
    return false;
}

template <class Request, class Response, class RequestHandler>
bool ZyROSConnectorServiceServer::handleRequest(const Request& req, Response& resp)
{
    auto obj = dynamic_cast<ZyROSConnectorServiceServerImpl<Request, Response, RequestHandler>*>(this);
    if (obj)
    {
        msg_info("ZyROSConnectorServiceServerImpl") << "Calling handleRequest()";
        return obj->handleRequest(req, resp);
    }
    else
    {
        msg_warning("") << "Could not cast instance to ZyROSConnectorServiceServerImpl, handling service request failed!";
        return false;
    }
}

template <class RequestType, class ResponseType, class RequestHandler>
ZyROSConnectorServiceServerImpl<RequestType, ResponseType, RequestHandler>::ZyROSConnectorServiceServerImpl(const std::string& serviceURI):
    ZyROSConnectorServiceServer(serviceURI)
{

}

template <class RequestType, class ResponseType, class RequestHandler>
ZyROSConnectorServiceServerImpl<RequestType, ResponseType, RequestHandler>::~ZyROSConnectorServiceServerImpl()
{

}

template <class RequestType, class ResponseType, class RequestHandler>
bool ZyROSConnectorServiceServerImpl<RequestType, ResponseType, RequestHandler>::handleRequest(const RequestType& req, ResponseType& resp)
{
    return m_requestHandler.handleRequest(req, resp);
}

/*template <class RequestType, class ResponseType, class RequestHandler>
bool ZyROSConnectorServiceServerImpl<RequestType, ResponseType, RequestHandler>::stopAdvertisingService()
{

}*/

// Placeholder callback method for service requests.
// There is no generic way to provide default handling for service requests.
// Custom logic for providing response data has to be implemented in derived classes, if more sophisticated logic for handling requests ist required.
/* template <class RequestType, class ResponseType, class RequestHandler>
bool ZyROSConnectorServiceServerImpl<RequestType, ResponseType, RequestHandler>::handleRequest(const RequestType& req, ResponseType& resp)
{
    msg_info("ZyROSConnectorServiceServerImpl") << "handleRequest called.";
    RequestHandler rq_handler(req, resp, nullptr);

    bool handler_status = rq_handler.handleRequest();

    msg_info("ZyROSConnectorServiceServerImpl") << "Response from handler: " << rq_handler.resp;
    resp = rq_handler.resp;

    return handler_status;
}*/

template <class RequestType, class ResponseType, class RequestHandler>
ZyROSConnectorServiceServerWorkerThread<RequestType, ResponseType, RequestHandler>::ZyROSConnectorServiceServerWorkerThread(boost::shared_ptr<ZyROSConnectorServiceServerImpl<RequestType, ResponseType, RequestHandler>> service_server): WorkerThread_SingleTask("ROSServiceServerWorker")
{
    // Run directly after start call, no initial pause necessary
    m_start_paused = false;
    this->m_serviceServer = service_server;
    m_func = &ZyROSConnectorServiceServer::serverLoop;
}

template <class RequestType, class ResponseType, class RequestHandler>
void ZyROSConnectorServiceServerWorkerThread<RequestType, ResponseType, RequestHandler>::main()
{
    msg_info("ZyROSConnectorServiceServerWorkerThread") << "Entering main function of ZyROSConnectorServiceServerWorkerThread";
    // read and store current thread id
    m_id = get_current_thread_id();

    // signal that the thread is running
    signal_state(running);

    // perform on-start custom action
    on_start();

    if (m_start_paused)
    {
        m_request = rq_idle;
        signal_state(paused);
    }

    // can throw const boost::thread_interrupted
    // if interrupt() was call in any interrupt
    // point
    try
    {
        if (m_serviceServer->advertiseService())
        {
            (m_serviceServer.get()->*m_func)();
            msg_info("ZyROSConnectorServiceServerWorkerThread") << "Service server function exited, stop advertising ROS service.";
            m_serviceServer->stopAdvertisingService();
        }
    }
    catch (const boost::thread_interrupted& ex)
    {
        SOFA_UNUSED(ex);
        msg_warning("ZyROSConnectorServiceServerWorkerThread") << "Thread " << m_name << ": Caught boost::thread_interrupted";

        m_serviceServer->stopAdvertisingService();
    }

    // update state
    signal_state(completed);

    // perform on-exit custom action
    // after the state was updated
    on_exit();
    // clear id
    m_id = INVALID_THREAD_ID;
}
