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
    : m_ros_connector(NULL),
      m_rosMasterURI(initData(&m_rosMasterURI, std::string("http://localhost:11311"), "rosMasterURI", "ROS master URI to connect to", true, false)),
      m_rosTopics(initData(&m_rosTopics, "rosTopics", "", "List of ROS topics to subscribe to (semicolon separated, format per entry: <ros topic>:::<message type>)"))

{

}

ZyROSConnectionManager::~ZyROSConnectionManager()
{
    msg_info("ZyROSConnectionManager") << "ZyROSConnectionManager destructor";
}

void ZyROSConnectionManager::cleanup()
{
    msg_info("ZyROSConnectionManager") << "cleanup()";

    if (m_ros_connector)
    {
        msg_info("ZyROSConnectionManager") << "Removing topic listeners.";
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

    msg_info("ZyROSConnectionManager") << "cleanup() done";
}

void ZyROSConnectionManager::addPublisher(boost::shared_ptr<ZyROSPublisher>& pub)
{
    m_ros_connector->addTopicPublisher(pub);
    topicPublishers.push_back(pub);
}

void ZyROSConnectionManager::addSubscriber(boost::shared_ptr<ZyROSListener>& sub)
{
    m_ros_connector->addTopicListener(sub);
    topicListeners.push_back(sub);
}

void ZyROSConnectionManager::init()
{
    msg_info("ZyROSConnectionManager") << "ZyROSConnectionManager::init()";

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
        msg_error("ZyROSConnectionManager") << "Failed to connect to roscore.";
    }
}

void ZyROSConnectionManager::bwdInit()
{
    msg_info("ZyROSConnectionManager") << "ZyROSConnectionManager::bwdInit() begin";

    if (m_ros_connector)
    {
        std::string rosTopics_Sequence = this->m_rosTopics.getValue();
        if (!rosTopics_Sequence.empty())
        {
            boost::regex entrySplit(";");
            boost::regex topicTypeSplit(":::");

            boost::sregex_token_iterator i(rosTopics_Sequence.begin(), rosTopics_Sequence.end(), entrySplit, -1);
            boost::sregex_token_iterator j;
            msg_info("ZyROSConnectionManager") << "Detected ROS topics: ";
            while (i != j)
            {
                std::string topicEntryStr = i->str();
                msg_info("ZyROSConnectionManager") << "   - " << *i++ << ": ";
                boost::sregex_token_iterator e_i(topicEntryStr.begin(), topicEntryStr.end(), topicTypeSplit, -1);
                boost::sregex_token_iterator e_j;
                unsigned int numTokens = 0;

                std::string rosTopic;
                std::string messageType;
                while (e_i != e_j)
                {
                    if (numTokens == 0)
                    {
                        msg_info("ZyROSConnectionManager") << " ROS topic: " << *e_i << " ";
                        rosTopic = *e_i;
                    }
                    else if (numTokens == 1)
                    {
                        msg_info("ZyROSConnectionManager") << " Message type: " << *e_i << " ";
                        messageType = *e_i;
                    }
                    *e_i++;
                    numTokens++;

                    msg_info("ZyROSConnectionManager");
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
                msg_info("ZyROSConnectionManager");
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

    msg_info("ZyROSConnectionManager") << "ZyROSConnectionManager::bwdInit() done";
}

void ZyROSConnectionManager::reset()
{
    msg_info("ZyROSConnectionManager") << "ZyROSConnectionManager::reset()";
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

template <class MessageType>
std::vector< ZyROSConnectorTopicSubscriber<MessageType>* > ZyROSConnectionManager::getSubscribers()
{
    std::vector< ZyROSConnectorTopicSubscriber<MessageType>* > subs;

    for (std::vector< boost::shared_ptr<ZyROSListener> >::iterator it = topicListeners.begin();
        it != topicListeners.end(); it++)
    {
        if ((*it)) // TODO: find out why there are sometimes NULL pointers in this vector
        {
            if ((*it)->getMessageType().compare(ros::message_traits::DataType<MessageType>::value()) == 0)
            {
                subs.push_back(&(*it));
            }
        }
    }

    return subs;
}

const std::vector<std::string> ZyROSConnectionManager::getTopics() const
{
    std::vector<std::string> allTopics;
    ros::master::V_TopicInfo topinf;

    if (ros::master::check())
    {
        ros::master::getTopics(topinf);
        for (unsigned int k = 0; k < topinf.size(); k++)
        {
            allTopics.push_back(topinf.at(k).name);
        }
    }

    return allTopics;
}

const std::vector<std::string> ZyROSConnectionManager::getServices() const
{
    std::vector<std::string> allServices;
    XmlRpc::XmlRpcValue req = "/node";
    XmlRpc::XmlRpcValue res;
    XmlRpc::XmlRpcValue pay;

    if (ros::master::check())
    {
        ros::master::execute("getSystemState", req, res, pay, true);
        std::string state[res.size()];
        for(int x = 0; x < res[2][2].size(); x++)
        {
            std::string gh = res[2][2][x][0].toXml().c_str();
            gh.erase(gh.begin(), gh.begin()+7);
            gh.erase(gh.end()-8, gh.end());
            state[x] = gh;

            msg_info("ZyROSConnectionManager") << "Active ROS service: " << gh;
        }
    }

    return allServices;
}

const std::vector<std::string> ZyROSConnectionManager::getNodes() const
{
    std::vector<std::string> allNodes;
    if (ros::master::check())
    {
        ros::V_string nodeNames;
        if (ros::master::getNodes(nodeNames))
        {
            for (size_t k = 0; k < nodeNames.size(); k++)
            {
                allNodes.push_back(nodeNames.at(k));
            }
        }
    }
    return allNodes;
}
