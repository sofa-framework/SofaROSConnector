#include "ZyROSConnectorServiceServer.inl"

using namespace Zyklio::ROSConnector;

ZyROSConnectorServiceServer::ZyROSConnectorServiceServer(ros::NodeHandlePtr rosNode, const std::string& serviceURI): m_d(NULL)
{
    m_d = new ZyROSConnectorServiceServerPrivate();
    m_d->m_serviceURI = serviceURI;
    m_d->m_rosNodeHandle = rosNode;
}

ZyROSConnectorServiceServer::~ZyROSConnectorServiceServer()
{
    if (m_d)
    {
        delete m_d;
        m_d = NULL;
    }
}

