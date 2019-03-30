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
    std::cout << "ZyROSConnector constructor" << std::endl;
}

ZyROSConnector::~ZyROSConnector()
{
    std::cout << "ZyROSConnector destructor" << std::endl;
	std::cout << "..." << std::endl;

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
        std::cout << "Setting ROS_MASTER_URI: " << envVarStream.str() << std::endl;
        int putenvRet = putenv((char*) (envVarStream.str().c_str()));
        std::cout << "putenv return value: " << putenvRet << std::endl;
        if (putenvRet != 0)
        {
            std::cout << "Error on putenv call: (" << putenvRet << "): " << strerror(errno) << std::endl;
        }
    }
    else
    {
        std::cout << "ROS_MASTER_URI already set in existing environment variable; no resetting necessary." << std::endl;
    }
#endif

#ifdef _MSC_VER
	_dupenv_s(&master_uri_env, NULL, "ROS_MASTER_URI");
#else
	master_uri_env = getenv("ROS_MASTER_URI");
#endif

	if (master_uri_env != NULL)
	  std::cout << "After _putenv -- Connecting to ROS master: " << master_uri_env << std::endl;
	else
	  std::cerr << "No ROS_MASTER_URI set! Did putenv fail?" << std::endl;

	if (!ros::isInitialized())
	{
		try
		{
			std::cout << "ROS not initialized, do init (Ctrl+C to abort ROS init and start without ROS)" << std::endl;
			ros::VP_string remappings;
			
			ros::init(remappings, std::string("TruRosConnector"));

			ros::master::setRetryTimeout(ros::WallDuration(2.0));

			msg_info() << "Creating ROS node handle..." << sendl;
			m_rosNode.reset(new ros::NodeHandle());
			msg_info() << "ROS initialization done" << sendl;

		}
		catch (std::exception& ex)
		{
			std::cerr << "Caught std::exception during ROS init: " << ex.what() << std::endl;
		}													 
		catch (...)
		{
			std::cerr << "Caught unspecified exception during ROS init!" << std::endl;
		}
	}
	else
	{
		std::cout << "ROS is already initialized, no init necessary." << std::endl;
	}
}

void ZyROSConnector::stopConnector()
{
    std::cout << "ZyROSConnector::stopConnector()" << std::endl;
	if (ros::isInitialized())
	{	
		std::cout << "ROS running, shutdown" << std::endl;
		
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
		std::cout << "ROS shutdown done" << std::endl;
	}
	else
	{
		std::cout << "ROS not running, no shutdown necessary" << std::endl;
	}
}
