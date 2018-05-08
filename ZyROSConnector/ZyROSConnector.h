#ifndef TRUPHYSICS_ROSCONNECTOR_H
#define TRUPHYSICS_ROSCONNECTOR_H

#ifdef _WIN32
// Ensure that Winsock2.h is included before Windows.h, which can get
// pulled in by anybody (e.g., Boost).
#ifndef WINSOCK2_H
#define WINSOCK2_H
#include <Winsock2.h>
#endif
#endif

#include <boost/shared_ptr.hpp>

#include <ros/master.h>
#include <ros/node_handle.h>
#include <ros/topic.h>

#include <ros/callback_queue.h>

#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Float32.h>

#include <sofa/core/objectmodel/BaseObject.h>
#include <sofa/core/objectmodel/Data.h>

#include "init_ZyROSConnector.h"
#include "ZyROSConnectorTopicSubscriber.h"
#include "ZyROSConnectorTopicPublisher.h"

#include <boost/thread/condition.hpp>

//#include <sofa/simulation/SimulationConcurrentComponent.h>

using namespace sofa::core::objectmodel;

namespace Zyklio
{
namespace ROSConnector
{
// Data members in private implementation; keep ROS headers out of the header
class TruPhysicsRosConnectorPrivate;
class SOFA_ZY_ROS_CONNECTOR_API ZyROSConnector : public sofa::core::objectmodel::BaseObject /*: public sofa::simulation::SimulationConcurrentComponent*/
{
public:
  SOFA_CLASS(ZyROSConnector, sofa::core::objectmodel::BaseObject);

  ZyROSConnector();
  //ZyROSConnector(std::string rosMasterURI);
  virtual ~ZyROSConnector();

  // SimulationConcurrentComponent API
  void startComponent();
  void stopComponent();
  void pauseComponent();
  void resumeComponent();

  bool isConnected() const;
  bool isThreadRunning() const;
  boost::condition& connectorCondition();

  bool setRosMasterURI(const std::string&);

  ros::NodeHandlePtr getROSNode();

  bool addTopicListener(const boost::shared_ptr<ZyROSListener> &);
  bool removeTopicListener(boost::shared_ptr<ZyROSListener>&);

  bool addTopicPublisher(boost::shared_ptr<ZyROSPublisher>&);
  bool removeTopicPublisher(boost::shared_ptr<ZyROSPublisher>&);

  size_t getNumTopicListeners() const;
  size_t getNumTopicPublishers() const;

  boost::posix_time::ptime theTime;

  std::string m_rosMasterURI;

protected:
  friend class ZyROSConnectorWorkerThread;

  void startConnector();
  void stopConnector();

  void rosLoop();

  TruPhysicsRosConnectorPrivate* m_d;

  ros::NodeHandlePtr m_rosNode;
  ros::CallbackQueue cb_queue;

  bool m_connectorThreadActive;
  boost::condition m_runCondition;

  //Data<std::string> m_rosTopics;

  //std::vector<std::pair<std::string, std::string> > m_subscribedRosTopics;
  std::vector<ros::Subscriber> m_activeSubscribers;
};
}
}

#endif //TRUPHYSICS_ROSCONNECTOR_H
