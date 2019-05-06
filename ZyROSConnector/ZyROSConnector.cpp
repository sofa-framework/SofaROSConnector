#include "ZyROSConnector.h"

#include <sofa/core/ObjectFactory.h>
#include <sofa/core/objectmodel/ClassInfo.h>

#include <sofa/defaulttype/RigidTypes.h>

#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>

#include <boost/regex.hpp>
#include <boost/algorithm/string.hpp>

#include "ZyROSConnectorWorkerThread.h"

namespace Zyklio
{
	namespace ROSConnector
	{
		class ZyklioRosConnectorPrivate
		{
			public:
                boost::shared_ptr<ZyROSConnectorWorkerThread> m_connectorThread;
				boost::shared_ptr<ros::AsyncSpinner> m_async_ros_spin;
		};
	}
}

using namespace Zyklio::ROSConnector;
using namespace sofa::core::objectmodel;
using namespace sofa::defaulttype;

ZyROSConnector::ZyROSConnector()
    : m_rosNode(NULL),
    m_connectorThreadActive(false),
    m_rosMasterURI(std::string("http://localhost:11311"))
{
    m_d = new ZyklioRosConnectorPrivate;
    msg_info("ZyROSConnector") << "Constructor";
}

ZyROSConnector::~ZyROSConnector()
{
    msg_info("ZyROSConnector") << "Destructor";

	if (m_d)
	{
		delete m_d;
		m_d = NULL;
	}
}

bool ZyROSConnector::addTopicListener(const boost::shared_ptr<ZyROSListener>& subscriber)
{
    msg_info("ZyROSConnector") << "addTopicListener for topic " << subscriber->getTopic();
    return m_d->m_connectorThread->addTopicListener(subscriber);
}

bool ZyROSConnector::removeTopicListener(boost::shared_ptr<ZyROSListener>& subscriber)
{
    msg_info("ZyROSConnector") << "removeTopicListener for topic " << subscriber->getTopic();
    return m_d->m_connectorThread->removeTopicListener(subscriber);
}

size_t ZyROSConnector::getNumTopicListeners() const
{
    return m_d->m_connectorThread->getNumTopicListeners();
}

bool ZyROSConnector::addTopicPublisher(boost::shared_ptr<ZyROSPublisher>& publisher)
{
    msg_info("ZyROSConnector") << "addTopicPublisher";
    return m_d->m_connectorThread->addTopicPublisher(publisher);
}

bool ZyROSConnector::removeTopicPublisher(boost::shared_ptr<ZyROSPublisher>& publisher)
{
    msg_info("ZyROSConnector") << "removeTopicPublisher";
    return m_d->m_connectorThread->removeTopicPublisher(publisher);
}

size_t ZyROSConnector::getNumTopicPublishers() const
{
    return m_d->m_connectorThread->getNumTopicPublishers();
}

bool ZyROSConnector::setRosMasterURI(const std::string& masterUri)
{
	if (!ros::isStarted())
	{
		m_rosMasterURI = masterUri;
		return true;
	}

	return false;
}

bool ZyROSConnector::isConnected() const
{
	return (ros::isStarted() && !ros::isShuttingDown());
}

bool ZyROSConnector::isThreadRunning() const
{
	return m_connectorThreadActive;
}

boost::condition& ZyROSConnector::connectorCondition()
{
	return m_runCondition;
}

ros::NodeHandlePtr ZyROSConnector::getROSNode()
{
	return m_rosNode;
}

void ZyROSConnector::rosLoop()
{
	m_connectorThreadActive = true;
	m_runCondition.notify_all();
	while (ros::ok())
    {
        // Clean up ROS subscribers that are no longer active
        for (size_t k = 0; k < m_d->m_connectorThread->m_activeSubscribers.size(); ++k)
        {
            if (m_d->m_connectorThread->m_activeSubscribers[k] == false)
            {
                m_d->m_connectorThread->m_topicSubscribers[k]->cleanup();
            }
        }

        // Publish messages from ROS publishers that are active, if any
        for (size_t k = 0; k < m_d->m_connectorThread->m_activePublishers.size(); ++k)
        {
            if (m_d->m_connectorThread->m_activePublishers[k] == true)
            {
                m_d->m_connectorThread->m_topicPublishers[k]->publishMessageQueue();
            }
        }

		ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0.01));
	}

	m_connectorThreadActive = false;
}

void ZyROSConnector::startComponent()
{
    msg_info("ZyROSConnector") << "startComponent";
	startConnector();
    m_d->m_connectorThread.reset(new ZyROSConnectorWorkerThread(this));
	m_d->m_async_ros_spin.reset(new ros::AsyncSpinner(2));
}

void ZyROSConnector::stopComponent()
{
    msg_info("ZyROSConnector") << "stopComponent";
	stopConnector();
	m_d->m_connectorThread->join();
}

void ZyROSConnector::pauseComponent()
{
    msg_info("ZyROSConnector") << "pauseComponent";
	m_d->m_async_ros_spin->stop();
}

void ZyROSConnector::resumeComponent()
{
    msg_info("ZyROSConnector") << "resumeComponent";
	m_d->m_connectorThread->start();
	m_d->m_async_ros_spin->start();
}

void ZyROSConnector::startConnector()
{
    msg_info("ZyROSConnector") << "startConnector()";

	char *master_uri_env = NULL;
	#ifdef _MSC_VER
	_dupenv_s(&master_uri_env, NULL, "ROS_MASTER_URI");
	#else
	master_uri_env = getenv("ROS_MASTER_URI");
	#endif

	bool setRosMasterURIValue = true;
	std::string rosMasterUri;
	if (master_uri_env != NULL)
	{
        msg_info("ZyROSConnector") << "Connecting to ROS master (env. variable): " << master_uri_env;
		rosMasterUri = master_uri_env;
		setRosMasterURIValue = false;
	}
	if (this->m_rosMasterURI.compare(rosMasterUri) != 0)
	{
        msg_info("ZyROSConnector") << "Connecting to ROS master (given from config): " << this->m_rosMasterURI;
		rosMasterUri = this->m_rosMasterURI;
	}



#ifdef _MSC_VER
	_putenv_s("ROS_MASTER_URI", rosMasterUri.c_str());
#else
    if (setRosMasterURIValue == true)
    {
        std::stringstream envVarStream;
        envVarStream << "ROS_MASTER_URI=" << rosMasterUri;
        msg_info("ZyROSConnector") << "Setting ROS_MASTER_URI: " << envVarStream.str();
        int putenvRet = putenv((char*) (envVarStream.str().c_str()));
        msg_info("ZyROSConnector") << "putenv return value: " << putenvRet;
        if (putenvRet != 0)
        {
            msg_error("ZyROSConnector") << "Error on putenv call: (" << putenvRet << "): " << strerror(errno);
        }
    }
    else
    {
        msg_info("ZyROSConnector") << "ROS_MASTER_URI already set in existing environment variable; no resetting necessary.";
    }
#endif

#ifdef _MSC_VER
	_dupenv_s(&master_uri_env, NULL, "ROS_MASTER_URI");
#else
	master_uri_env = getenv("ROS_MASTER_URI");
#endif

	if (master_uri_env != NULL)
      msg_warning() << "After _putenv -- Connecting to ROS master: " << master_uri_env;
	else
      msg_error() << "No ROS_MASTER_URI set! Did putenv fail?";

	if (!ros::isInitialized())
	{
		try
		{
            msg_info("ZyROSConnector") << "ROS not initialized, do init (Ctrl+C to abort ROS init and start without ROS)";
            msg_info("ZyROSConnector") << "Allocating remappings placeholder.";
            ros::VP_string remappings;
            msg_info("ZyROSConnector") << "Allocated remappings placeholder.";
			
            msg_info("ZyROSConnector") << "Calling ros::init()";
            ros::init(remappings, std::string("ZyRosConnector"));
            msg_info("ZyROSConnector") << "ros::init() finished";

			ros::master::setRetryTimeout(ros::WallDuration(2.0));

            msg_info("ZyROSConnector") << "Creating ROS node handle...";
			m_rosNode.reset(new ros::NodeHandle());
            msg_info("ZyROSConnector") << "ROS initialization done";

		}
		catch (std::exception& ex)
		{
            msg_error("ZyROSConnector") << "Caught std::exception during ROS init: " << ex.what();
		}													 
		catch (...)
		{
            msg_error("ZyROSConnector") << "Caught unspecified exception during ROS init!";
		}
	}
	else
	{
        msg_info("ZyROSConnector") << "ROS is already initialized, no init necessary.";
	}
}

void ZyROSConnector::stopConnector()
{
    msg_info("ZyROSConnector") << "ZyROSConnector::stopConnector()";
	if (ros::isInitialized())
	{	
        msg_info("ZyROSConnector") << "ROS running, shutdown";
		
		for (std::vector<ros::Subscriber>::iterator it = m_activeSubscribers.begin(); it != m_activeSubscribers.end(); it++)
		{
			ros::Subscriber& subscriber = (*it);
			subscriber.shutdown();
		}
		
		if (m_rosNode != NULL)
		{
			m_rosNode->shutdown();
			m_rosNode = NULL;
		}

		ros::shutdown();
        msg_info("ZyROSConnector") << "ROS shutdown done";
	}
	else
	{
        msg_info("ZyROSConnector") << "ROS not running, no shutdown necessary";
	}
}
