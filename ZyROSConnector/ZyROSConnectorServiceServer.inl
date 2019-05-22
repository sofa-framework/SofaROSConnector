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

bool ZyROSConnectorServiceServer::stopAdvertisingService()
{
    return false;
}

template <class RequestType, class ResponseType, class RequestHandler>
ZyROSConnectorServiceServerImpl<RequestType, ResponseType, RequestHandler>::ZyROSConnectorServiceServerImpl(ros::NodeHandlePtr rosNode, const std::string& serviceURI):
    ZyROSConnectorServiceServer(rosNode, serviceURI)
{

}

template <class RequestType, class ResponseType, class RequestHandler>
ZyROSConnectorServiceServerImpl<RequestType, ResponseType, RequestHandler>::~ZyROSConnectorServiceServerImpl()
{

}

template <class RequestType, class ResponseType, class RequestHandler>
bool ZyROSConnectorServiceServerImpl<RequestType, ResponseType, RequestHandler>::advertiseService()
{
    ros::AdvertiseServiceOptions service_options;
    boost::function<bool(RequestType&, ResponseType&)> callback_fn = boost::bind(&ZyROSConnectorServiceServerImpl<RequestType, ResponseType, RequestHandler>::handleRequest, *this, _1, _2);
    service_options.init(m_d->m_serviceURI, callback_fn);
    m_d->m_rosServer = m_d->m_rosNodeHandle->advertiseService(service_options);
    if (m_d->m_rosServer)
    {
        msg_info("ZyROSConnectorServiceServerImpl") << "Successfully advertised ROS service.";
        return true;
    }
    else
    {
        msg_error("ZyROSConnectorServiceServerImpl") << "Failed to advertise ROS service!";
        return false;
    }
}

template <class RequestType, class ResponseType, class RequestHandler>
bool ZyROSConnectorServiceServerImpl<RequestType, ResponseType, RequestHandler>::stopAdvertisingService()
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

// Placeholder callback method for service requests.
// There is no generic way to provide default handling for service requests.
// Custom logic for providing response data has to be implemented in derived classes.
template <class RequestType, class ResponseType, class RequestHandler>
bool ZyROSConnectorServiceServerImpl<RequestType, ResponseType, RequestHandler>::handleRequest(const RequestType& req, ResponseType& resp)
{
    msg_info("ZyROSConnectorServiceServerImpl") << "handleRequest called.";
    RequestHandler rq_handler(req, resp);

    bool handler_status = rq_handler.handleRequest();

    msg_info("ZyROSConnectorServiceServerImpl") << "Response from handler: " << rq_handler.resp;
    resp = rq_handler.resp;

    return handler_status;
}
