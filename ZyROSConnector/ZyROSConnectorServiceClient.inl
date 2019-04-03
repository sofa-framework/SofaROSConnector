#include "ZyROSConnectorServiceClient.h"

#include <sofa/helper/logging/Messaging.h>

using namespace Zyklio::ROSConnector;

template <class ServiceType, class RequestType, class ResponseType>
ZyROSConnectorServiceClient<ServiceType, RequestType, ResponseType>::ZyROSConnectorServiceClient(ros::NodeHandlePtr nodeHandle, const std::string& serviceURI, unsigned int queueLength):
    m_rosNodeHandle(nodeHandle), m_serviceURI(serviceURI), m_messageQueueLength(queueLength)
{

}

template <class ServiceType, class RequestType, class ResponseType>
bool ZyROSConnectorServiceClient<ServiceType, RequestType, ResponseType>::enqueueRequest(const RequestType& request)
{
    m_requestQueue.push_front(request);
    return true;
}

template <class ServiceType, class RequestType, class ResponseType>
void ZyROSConnectorServiceClient<ServiceType, RequestType, ResponseType>::clearRequests()
{
    m_requestQueue.clear();
}

template <class ServiceType, class RequestType, class ResponseType>
unsigned int ZyROSConnectorServiceClient<ServiceType, RequestType, ResponseType>::getNumResponses() const
{
    return m_responseQueue.size();
}

template <class ServiceType, class RequestType, class ResponseType>
const ResponseType& ZyROSConnectorServiceClient<ServiceType, RequestType, ResponseType>::getLatestResponse() const
{
    return m_responseQueue.back();
}

template <class ServiceType, class RequestType, class ResponseType>
const ResponseType& ZyROSConnectorServiceClient<ServiceType, RequestType, ResponseType>::getResponse(size_t idx)
{
    if (idx < m_responseQueue.size())
        return m_responseQueue.at(idx);

    return ResponseType();
}

template <class ServiceType, class RequestType, class ResponseType>
void ZyROSConnectorServiceClient<ServiceType, RequestType, ResponseType>::dispatchRequests()
{
    m_responseQueue.clear();
    m_responseQueue.resize(m_requestQueue.size());
    for (size_t k = 0; k < m_requestQueue.size(); k++)
    {
        m_client->call(m_requestQueue.at(k), m_responseQueue.at(k));
    }
}
