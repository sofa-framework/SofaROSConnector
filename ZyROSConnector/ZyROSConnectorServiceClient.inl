#include "ZyROSConnectorServiceClient.h"

#include <sofa/helper/logging/Messaging.h>

using namespace Zyklio::ROSConnector;

template <class ServiceType, class RequestType, class ResponseType>
ZyROSConnectorServiceClient<ServiceType, RequestType, ResponseType>::ZyROSConnectorServiceClient(ros::NodeHandlePtr nodeHandle, const std::string& serviceURI, unsigned int queueLength):
    m_rosNodeHandle(nodeHandle), m_serviceURI(serviceURI), m_messageQueueLength(queueLength),
    m_lastValidRequest(0), m_lastValidResponse(0)
{
    m_requestQueue.resize(m_messageQueueLength);
    m_responseQueue.resize(m_messageQueueLength);
    m_clientCallStates.resize(m_messageQueueLength);
}

template <class ServiceType, class RequestType, class ResponseType>
bool ZyROSConnectorServiceClient<ServiceType, RequestType, ResponseType>::setupClient()
{
    try
    {
        /*ros::ServiceClientOptions clientOptions;
        clientOptions.persistent = true;
        clientOptions.service = m_serviceURI;*/
        m_client = m_rosNodeHandle->serviceClient<ServiceType>(m_serviceURI, true);
        return true;
    }
    catch (ros::Exception& ex)
    {
        msg_error("ZyROSConnectorServiceClient") << "Failed to instantiate service client instance: " << ex.what();
        return false;
    }
}

template <class ServiceType, class RequestType, class ResponseType>
bool ZyROSConnectorServiceClient<ServiceType, RequestType, ResponseType>::shutdownClient()
{
    if (!m_client.isValid())
        return false;

    m_client.shutdown();
    return true;
}

template <class ServiceType, class RequestType, class ResponseType>
bool ZyROSConnectorServiceClient<ServiceType, RequestType, ResponseType>::enqueueRequest(const RequestType& request)
{
    m_requestQueue.push_front(request);

    m_lastValidRequest++;
    if (m_lastValidRequest >= m_messageQueueLength)
        m_lastValidRequest = 0;

    return true;
}

template <class ServiceType, class RequestType, class ResponseType>
void ZyROSConnectorServiceClient<ServiceType, RequestType, ResponseType>::clearRequests()
{
    m_requestQueue.clear();
    m_requestQueue.resize(m_messageQueueLength);
    m_responseQueue.clear();
    m_responseQueue.resize(m_messageQueueLength);
    m_clientCallStates.clear();
    m_clientCallStates.resize(m_messageQueueLength);
    m_lastValidRequest = 0;
    m_lastValidResponse = 0;
}

template <class ServiceType, class RequestType, class ResponseType>
unsigned int ZyROSConnectorServiceClient<ServiceType, RequestType, ResponseType>::getNumResponses() const
{
    return m_lastValidResponse;
}

template <class ServiceType, class RequestType, class ResponseType>
const ResponseType& ZyROSConnectorServiceClient<ServiceType, RequestType, ResponseType>::getLatestResponse()
{
    /*ResponseType& latestResponse = m_responseQueue.back();
    ResponseType returnValue(latestResponse);
    m_responseQueue.pop_back();
    return returnValue;*/
    msg_info("ZyROSConnectorServiceClient") << "Returning latest response: " << m_responseQueue.back();
    return m_responseQueue.back();
}

template <class ServiceType, class RequestType, class ResponseType>
const ResponseType& ZyROSConnectorServiceClient<ServiceType, RequestType, ResponseType>::getResponse(size_t idx)
{
    /*ResponseType response;
    if (idx < m_responseQueue.size())
    {
        response = m_responseQueue.at(idx);
        m_requestQueue[idx] = RequestType();
        m_responseQueue[idx] = ResponseType();
        m_clientCallStates[idx] = false;
    }
    return response;*/

    if (idx < m_responseQueue.size())
        return m_responseQueue.at(idx);

    return ResponseType();
}

template <class ServiceType, class RequestType, class ResponseType>
void ZyROSConnectorServiceClient<ServiceType, RequestType, ResponseType>::dispatchRequests()
{   
    if (m_lastValidRequest > 0)
    {
        // msg_info("ZyROSConnectorServiceClient") << "dispatchRequests()";
        // m_responseQueue.clear();
        // m_responseQueue.resize(m_lastValidRequest);
        // m_lastValidResponse = 0;
        for (size_t k = 0; k < m_lastValidRequest; k++)
        {
            if (m_client.exists())
            {
                if (m_clientCallStates[k])
                {
                    // msg_info("ZyROSConnectorServiceClient") << "Service call " << k << " already dispatched successfully.";
                    continue;
                }
                bool call_result = m_client.call(m_requestQueue.at(k), m_responseQueue.at(k));
                m_clientCallStates[k] = call_result;
                if (call_result)
                {
                    msg_info("ZyROSConnectorServiceClient") << "Service call " << k << " dispatched successfully.";
                    msg_info("ZyROSConnectorServiceClient") << "Message received for call " << k << ": " << m_responseQueue.at(k);
                    m_lastValidResponse++;
                }
                else
                {
                    msg_warning("ZyROSConnectorServiceClient") << "Service call " << k << " failed!";
                }
            }
            else
            {
                msg_warning("ZyROSConnectorServiceClient") << "Service call " << k << " failed: ROS service " << m_client.getService() << " does not exist!";
                m_clientCallStates[k] = false;
            }
        }
    }
}
