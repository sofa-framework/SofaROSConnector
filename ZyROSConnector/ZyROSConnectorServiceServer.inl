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
        };
    }
}

using namespace Zyklio::ROSConnector;
