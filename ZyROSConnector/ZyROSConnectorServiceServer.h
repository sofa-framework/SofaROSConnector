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
#include "sofa/helper/logging/Messaging.h"

namespace Zyklio
{
    namespace ROSConnector
    {
        using namespace Zyklio::MultiThreading;

        class SOFA_ZY_ROS_CONNECTOR_API ZyROSConnectorServiceServerWorkerThread: public WorkerThread_SingleTask
        {

        };

        // Data members in private implementation
        class ZyROSConnectorServiceServerPrivate;
        class SOFA_ZY_ROS_CONNECTOR_API ZyROSConnectorServiceServer
        {
            public:
                ZyROSConnectorServiceServer();

            protected:
                ZyROSConnectorServiceServerPrivate* m_d;

        };
    }
}

#endif // ZY_ROS_CONNECTOR_SERVICE_SERVER_H
