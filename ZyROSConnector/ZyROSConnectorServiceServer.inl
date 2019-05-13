#include "ZyROSConnectorServiceServer.h"

namespace Zyklio
{
    namespace ROSConnector
    {
        class ZyROSConnectorServiceServerPrivate
        {
            public:
                ros::NodeHandlePtr m_rosNodeHandle;
                boost::shared_ptr<ros::ServiceServer> m_rosServer;

                std::string m_serviceURI;
                ros::AdvertiseServiceOptions m_serviceOptions;
        };
    }
}

using namespace Zyklio::ROSConnector;

template <class RequestType, class ResponseType>
ZyROSConnectorServiceServerImpl<RequestType, ResponseType>::ZyROSConnectorServiceServerImpl(ros::NodeHandlePtr rosNode, const std::string& serviceURI):
    ZyROSConnectorServiceServer(rosNode, serviceURI)
{

}

template <class RequestType, class ResponseType>
ZyROSConnectorServiceServerImpl<RequestType, ResponseType>::~ZyROSConnectorServiceServerImpl()
{

}

template <class RequestType, class ResponseType>
bool ZyROSConnectorServiceServerImpl<RequestType, ResponseType>::advertiseService()
{
    return false;
}

template <class RequestType, class ResponseType>
bool ZyROSConnectorServiceServerImpl<RequestType, ResponseType>::stopAdvertisingService()
{
    return false;
}

// Placeholder callback method for service requests.
// There is no generic way to provide default handling for service requests.
// Custom logic for providing response data has to be implemented in derived classes.
template <class RequestType, class ResponseType>
bool ZyROSConnectorServiceServerImpl<RequestType, ResponseType>::handleRequest(const RequestType &, ResponseType &)
{
    return false;
}
