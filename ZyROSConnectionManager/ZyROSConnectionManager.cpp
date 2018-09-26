#include "ZyROSConnectionManager.h"

#include <sofa/core/ObjectFactory.h>
#include <sofa/core/objectmodel/ClassInfo.h>

#include <boost/regex.hpp>

#include <sensor_msgs/JointState.h>
#include <std_msgs/Bool.h>

#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/String.h>

using namespace Zyklio::ROSConnector;
using namespace Zyklio::ROSConnectionManager;
using namespace sofa::core::objectmodel;
using namespace sofa::defaulttype;

SOFA_DECL_CLASS(ZyROSConnectionManager)

int ZyRosConnectionManagerClass = sofa::core::RegisterObject("Zyklio ROS connection manager.")
.add< ZyROSConnectionManager >()
;

ZyROSConnectionManager::ZyROSConnectionManager()
    : m_rosTopics(initData(&m_rosTopics, "rosTopics", "", "List of ROS topics to subscribe to (semicolon separated, format per entry: <ros topic>:::<message type>)"))
    , m_rosMasterURI(initData(&m_rosMasterURI, std::string("http://localhost:11311"), "rosMasterURI", "ROS master URI to connect to", true, false))
    , m_ros_connector(NULL)
{
}

ZyROSConnectionManager::~ZyROSConnectionManager()
{
    std::cout << "ZyROSConnectionManager destructor" << std::endl;
    std::cout << "..." << std::endl;
}

void ZyROSConnectionManager::cleanup()
{
    std::cout << "ZyROSConnectionManager cleanup" << std::endl;

    if (m_ros_connector)
    {
        msg_info("ZyROSConnectionManager") << "Removing topic listeners:";
        while (!topicListeners.empty())
        {
            boost::shared_ptr<ZyROSListener>& current = topicListeners.back();
            /*msg_info("ZyROSConnectionManager") << "      - calling cleanup";
                  current->cleanup(); // this crashes for some reason
                  msg_info("ZyROSConnectionManager") << "      - trying to remove a listener";*/
            if (m_ros_connector->removeTopicListener(current))
            {
                msg_info("ZyROSConnectionManager") << "-> removed a topic listener";
            }
            else
            {
                msg_info("ZyROSConnectionManager") << "-> could not remove a topic listener";
            }
            topicListeners.pop_back();
        }

        msg_info("ZyROSConnectionManager") << "pauseComponent";
        m_ros_connector->pauseComponent();
        msg_info("ZyROSConnectionManager") << "stopComponent";
        m_ros_connector->stopComponent();
        delete m_ros_connector;
    }

    std::cout << "ZyROSConnectionManager cleanup done" << std::endl;
}

void ZyROSConnectionManager::addPublisher(boost::shared_ptr<ZyROSPublisher>& pub)
{
    m_ros_connector->addTopicPublisher(pub);
    topicPublishers.push_back(pub);
}

#ifndef _WIN32
void ZyROSConnectionManager::addSubscriber(boost::shared_ptr<ZyROSListener>& sub)
#else
void ZyROSConnectionManager::addSubscriber(boost::shared_ptr<ZyROSListener>& sub)
#endif
{
    m_ros_connector->addTopicListener(sub);
    topicListeners.push_back(sub);
}

//void ZyROSConnectionManager::addSubscriber(boost::shared_ptr<ZyROSListener>& sub)
//{
//    m_ros_connector->addTopicListener(sub);
//    topicListeners.push_back(sub);
//}

//bool ZyROSConnectionManager::setRosMasterURI(const std::string& masterUri)
//{
//	if (!ros::isStarted())
//	{
//		m_rosMasterURI.setValue(masterUri);
//		return true;
//	}
//
//	return false;
//}

void ZyROSConnectionManager::init()
{
    std::cout << "ZyROSConnectionManager::init()" << std::endl;

    m_ros_connector = new ZyROSConnector();
    m_ros_connector->setRosMasterURI(m_rosMasterURI.getValue());
    m_ros_connector->startComponent();
    m_ros_connector->resumeComponent();
    msg_info("ZyROSConnectionManager") << "connectToROSMaster: resumeComponent";

    boost::mutex::scoped_lock lock(m_mutex);
    while (!m_ros_connector->isThreadRunning())
    {
        msg_info("ZyROSConnectionManager") << "Waiting for connector thread to start...";
        m_ros_connector->connectorCondition().wait(lock);
    }
    msg_info("ZyROSConnectionManager") << "Connector thread started.";
    if (m_ros_connector->isConnected())
    {
        msg_info("ZyROSConnectionManager") << "Connection to roscore established.";
    }
    else
    {
        msg_info("ZyROSConnectionManager") << "Failed to connect to roscore.";
    }
}

void ZyROSConnectionManager::bwdInit()
{
    std::cout << "ZyROSConnectionManager::bwdInit() begin" << std::endl;

    if (m_ros_connector)
    {
        std::string rosTopics_Sequence = this->m_rosTopics.getValue();
        if (!rosTopics_Sequence.empty())
        {
            boost::regex entrySplit(";");
            boost::regex topicTypeSplit(":::");

            boost::sregex_token_iterator i(rosTopics_Sequence.begin(), rosTopics_Sequence.end(), entrySplit, -1);
            boost::sregex_token_iterator j;
            std::cout << "Detected ROS topics: " << std::endl;
            while (i != j)
            {
                std::string topicEntryStr = i->str();
                std::cout << "   - " << *i++ << ": ";
                boost::sregex_token_iterator e_i(topicEntryStr.begin(), topicEntryStr.end(), topicTypeSplit, -1);
                boost::sregex_token_iterator e_j;
                unsigned int numTokens = 0;

                std::string rosTopic;
                std::string messageType;
                while (e_i != e_j)
                {
                    if (numTokens == 0)
                    {
                        std::cout << " ROS topic: " << *e_i << " ";
                        rosTopic = *e_i;
                    }
                    else if (numTokens == 1)
                    {
                        std::cout << " Message type: " << *e_i << " ";
                        messageType = *e_i;
                    }
                    *e_i++;
                    numTokens++;

                    std::cout << std::endl;
                    if (!rosTopic.empty() && !messageType.empty())
                    {
                        //this->m_subscribedRosTopics.push_back(std::make_pair(rosTopic, messageType));

                        msg_info("ZyROSConnectionManager") << "Trying to start listener for topic " << rosTopic << " of type " << messageType;

                        bool supported = false;

                        if (messageType.compare("std_msgs::Bool") == 0)
                        {
                            supported = true;

                            //boost::shared_ptr<ZyROSConnectorTopicSubscriber<std_msgs::Bool>> tmp(new ZyROSConnectorTopicSubscriber<std_msgs::Bool>(m_ros_connector->getROSNode(), rosTopic));
                            //boost::shared_ptr<ZyROSConnectorTopicSubscriber<std_msgs::Bool>> tmp(new ZyROSConnectorTopicSubscriber<std_msgs::Bool>(m_ros_connector->getROSNode(), rosTopic, 50 , true));

                            // "manually":
                            /*m_ros_connector->addTopicListener(boost::dynamic_pointer_cast<ZyROSListener>(tmp));
                            topicListeners.push_back(boost::dynamic_pointer_cast<ZyROSListener>(tmp)); */

                            // variant 1:
                            //addSubscriber(boost::dynamic_pointer_cast<ZyROSListener>(tmp));
                            
                            // variant 2:
                            //addSubscriber<std_msgs::Bool>(tmp);

                            //////
                            // variant 3 (doesn't need the boost::shared_ptr<ZyROSConnectorTopicSubscriber<std_msgs::Bool>> at all):
                            subscribeToTopic<std_msgs::Bool>(rosTopic);
                        }
                        else if (messageType.compare("std_msgs::Float32") == 0)
                        {
                            supported = true;

                            const boost::shared_ptr<ZyROSConnectorTopicSubscriber<std_msgs::Float32>> tmp(new ZyROSConnectorTopicSubscriber<std_msgs::Float32>(m_ros_connector->getROSNode(), rosTopic, 50, true));

                            m_ros_connector->addTopicListener(boost::dynamic_pointer_cast<ZyROSListener>(tmp));
                            topicListeners.push_back(boost::dynamic_pointer_cast<ZyROSListener>(tmp));
                        }
                        else if (messageType.compare("std_msgs::Float64MultiArray") == 0)
                        {
                            supported = true;

                            const boost::shared_ptr<ZyROSConnectorTopicSubscriber<std_msgs::Float64MultiArray>> tmp(new ZyROSConnectorTopicSubscriber<std_msgs::Float64MultiArray>(m_ros_connector->getROSNode(), rosTopic, 50, true));

                            m_ros_connector->addTopicListener(boost::dynamic_pointer_cast<ZyROSListener>(tmp));
                            topicListeners.push_back(boost::dynamic_pointer_cast<ZyROSListener>(tmp));
                        }
                        else if (messageType.compare("std_msgs::String") == 0)
                        {
                            supported = true;

                            const boost::shared_ptr<ZyROSConnectorTopicSubscriber<std_msgs::String>> tmp(new ZyROSConnectorTopicSubscriber<std_msgs::String>(m_ros_connector->getROSNode(), rosTopic, 50, true));

                            m_ros_connector->addTopicListener(boost::dynamic_pointer_cast<ZyROSListener>(tmp));
                            topicListeners.push_back(boost::dynamic_pointer_cast<ZyROSListener>(tmp));
                        }
                        else if (messageType.compare("sensor_msgs::JointState") == 0)
                        {
                            supported = true;

                            const boost::shared_ptr<ZyROSConnectorTopicSubscriber<sensor_msgs::JointState>> tmp(new ZyROSConnectorTopicSubscriber<sensor_msgs::JointState>(m_ros_connector->getROSNode(), rosTopic, 50, true));
                            m_ros_connector->addTopicListener(boost::dynamic_pointer_cast<ZyROSListener>(tmp));
                            topicListeners.push_back(boost::dynamic_pointer_cast<ZyROSListener>(tmp));
                        }

                        if (!supported)
                        {
                            msg_info("ZyROSConnectionManager") << "Unsupported message type: " << messageType;
                        }
                    }
                }
                std::cout << std::endl;
            }
        }
    }

    ros::master::V_TopicInfo topinf;
    ros::V_string nodeStr;
    std::stringstream tmpmsg;

    if (ros::master::check())
    {
        tmpmsg << "      available? : yes\n";
        tmpmsg << "        hostname : " << ros::master::getHost() << "\n";
        tmpmsg << "            port : " << ros::master::getPort() << "\n";
        tmpmsg << "             uri : " << ros::master::getURI() << "\n";

        ros::master::getNodes(nodeStr);
        tmpmsg << "           Nodes :" << "\n";
        for (unsigned int asd = 0; asd < nodeStr.size(); asd++)
        {
            tmpmsg << "               " << nodeStr.at(asd) << " " << "\n";
        }
        tmpmsg << "\n";

        ros::master::getTopics(topinf);
        tmpmsg << "          Topics :" << "\n";
        tmpmsg << "  (datatype, name)" << "\n";
        for (unsigned int asd = 0; asd < topinf.size(); asd++)
        {
            tmpmsg << "                       " << "(" << topinf.at(asd).datatype << ", " << topinf.at(asd).name << ") " << "\n";
        }
        tmpmsg << "\n";
    }
    else
    {
        tmpmsg << "      available? : no (this should not happen at this point and something is probably wrong)\n";
    }
    msg_info("ZyROSConnectionManager") << "Current ROS master status:\n" << tmpmsg.str();

    std::cout << "ZyROSConnectionManager::bwdInit() done" << std::endl;
}

void ZyROSConnectionManager::reset()
{
    std::cout << "ZyROSConnectionManager::reset()" << std::endl;
    /*if (m_ros_connector)
      {
          if (ros::isStarted())
          {
              m_ros_connector->pauseComponent();
          }
          m_ros_connector->setRosMasterURI(m_rosMasterURI.getValue());
          m_ros_connector->resumeComponent();
      }*/
}

