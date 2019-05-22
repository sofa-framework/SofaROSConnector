#include "ZyROSConnectorServiceClient.h"

#include <sofa/helper/logging/Messaging.h>

using namespace Zyklio::ROSConnector;

template <class ServiceType, class RequestType, class ResponseType>
ZyROSConnectorServiceClient<ServiceType, RequestType, ResponseType>::ZyROSConnectorServiceClient(ros::NodeHandlePtr nodeHandle, const std::string& serviceURI, unsigned int queueLength):
    m_rosNodeHandle(nodeHandle), m_serviceURI(serviceURI), m_messageQueueLength(queueLength)
{
    m_requestQueue.resize(m_messageQueueLength);
    m_responseQueue.resize(m_messageQueueLength);
    m_clientCallStates.resize(m_messageQueueLength);
    m_clientSlotsUsed.resize(m_messageQueueLength);
    m_clientCallFailedStates.resize(m_messageQueueLength);
}

template <class ServiceType, class RequestType, class ResponseType>
bool ZyROSConnectorServiceClient<ServiceType, RequestType, ResponseType>::setupClient()
{
    try
    {
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
    boost::mutex::scoped_lock lock(m_mutex);

    size_t request_idx = -1;
    for (size_t k = 0; k < m_clientSlotsUsed.size(); k++)
    {
        if (!m_clientSlotsUsed[k])
        {
            request_idx = k;
            break;
        }
    }

    if (request_idx >= 0)
    {
        m_clientSlotsUsed[request_idx] = true;
        m_requestQueue[request_idx] = request;
    }

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
    m_clientCallFailedStates.clear();
    m_clientCallFailedStates.resize(m_messageQueueLength);
}

template <class ServiceType, class RequestType, class ResponseType>
unsigned int ZyROSConnectorServiceClient<ServiceType, RequestType, ResponseType>::getNumResponses() const
{
    return std::count_if(m_clientSlotsUsed.begin(), m_clientSlotsUsed.end(), [](bool i){return i == true;});
}

template <class ServiceType, class RequestType, class ResponseType>
bool ZyROSConnectorServiceClient<ServiceType, RequestType, ResponseType>::removeRequest(const size_t& idx)
{
    boost::mutex::scoped_lock lock(m_mutex);

    if (idx > m_clientSlotsUsed.size())
        return false;

    if (!m_clientSlotsUsed[idx])
    {
        return false;
    }
    else
    {
        m_clientSlotsUsed[idx] = false;
        m_clientCallStates[idx] = false;
        m_clientCallFailedStates[idx] = false;
        m_requestQueue[idx] = RequestType();
        m_responseQueue[idx] = ResponseType();
    }

    return true;
}

template <class ServiceType, class RequestType, class ResponseType>
const ResponseType& ZyROSConnectorServiceClient<ServiceType, RequestType, ResponseType>::getResponse(size_t idx)
{
    static ResponseType emptyResponse;
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

    return emptyResponse;
}

template <class ServiceType, class RequestType, class ResponseType>
void ZyROSConnectorServiceClient<ServiceType, RequestType, ResponseType>::dispatchRequests()
{   

    for (size_t k = 0; k < m_clientSlotsUsed.size(); k++)
    {
        if (m_clientSlotsUsed[k] == true && m_clientCallStates[k] == false)
        {
            if (m_client.exists())
            {
                if (!m_clientCallFailedStates[k])
                {
                    msg_info("ZyROSConnectorServiceClient") << "Dispatching service request: " << m_requestQueue.at(k);
                    bool call_result = m_client.call(m_requestQueue.at(k), m_responseQueue.at(k));
                    if (call_result)
                    {
                        m_clientCallStates[k] = true;
                        msg_info("ZyROSConnectorServiceClient") << "Service call " << k << " dispatched successfully.";
                        msg_info("ZyROSConnectorServiceClient") << "Message received for call " << k << ": " << m_responseQueue.at(k);

                        for (size_t p = 0; p < m_clientCallStates.size(); p++)
                        {
                            if (m_clientCallStates[p])
                                msg_info("ROSConnectorServiceClient") << "Call " << p << ": " << m_responseQueue[p];
                        }
                    }
                    else
                    {
                        msg_warning("ZyROSConnectorServiceClient") << "Service call " << k << " failed!";
                        m_clientCallFailedStates[k] = true;
                    }
                }
                /*else
                {
                    msg_warning("ZyROSConnectorServiceClient") << "Service call " << k << " failed: ROS service " << m_client.getService() << " does not exist!";
                    m_clientCallFailedStates[k] = true;
                }*/
            }
        }
    }
}
