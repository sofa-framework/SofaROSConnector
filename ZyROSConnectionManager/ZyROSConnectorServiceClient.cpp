/***********************************************************************
ROS service definition headers and ROS connector template instantiations.
This file is AUTO-GENERATED during the CMake run.
Please do not modify it by hand.
The contents will be overwritten and re-generated.
************************************************************************/


#include <ZyROS_ServiceType_Client_Instantiations.h>


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
        

using namespace control_msgs;
using namespace diagnostic_msgs;
using namespace dynamic_reconfigure;
using namespace nav_msgs;
using namespace nodelet;
using namespace roscpp;
using namespace roscpp_tutorials;
using namespace rospy_tutorials;
using namespace sensor_msgs;
using namespace std_srvs;
using namespace tf;
using namespace tf2_msgs;
using namespace topic_tools;

template class ZyROSConnectorServiceClient<control_msgs::QueryCalibrationState, control_msgs::QueryCalibrationStateRequest, control_msgs::QueryCalibrationStateResponse>;
template class ZyROSConnectorServiceClient<control_msgs::QueryTrajectoryState, control_msgs::QueryTrajectoryStateRequest, control_msgs::QueryTrajectoryStateResponse>;
template class ZyROSConnectorServiceClient<diagnostic_msgs::AddDiagnostics, diagnostic_msgs::AddDiagnosticsRequest, diagnostic_msgs::AddDiagnosticsResponse>;
template class ZyROSConnectorServiceClient<diagnostic_msgs::SelfTest, diagnostic_msgs::SelfTestRequest, diagnostic_msgs::SelfTestResponse>;
template class ZyROSConnectorServiceClient<dynamic_reconfigure::Reconfigure, dynamic_reconfigure::ReconfigureRequest, dynamic_reconfigure::ReconfigureResponse>;
template class ZyROSConnectorServiceClient<nav_msgs::GetMap, nav_msgs::GetMapRequest, nav_msgs::GetMapResponse>;
template class ZyROSConnectorServiceClient<nav_msgs::GetPlan, nav_msgs::GetPlanRequest, nav_msgs::GetPlanResponse>;
template class ZyROSConnectorServiceClient<nav_msgs::SetMap, nav_msgs::SetMapRequest, nav_msgs::SetMapResponse>;
template class ZyROSConnectorServiceClient<nodelet::NodeletList, nodelet::NodeletListRequest, nodelet::NodeletListResponse>;
template class ZyROSConnectorServiceClient<nodelet::NodeletLoad, nodelet::NodeletLoadRequest, nodelet::NodeletLoadResponse>;
template class ZyROSConnectorServiceClient<nodelet::NodeletUnload, nodelet::NodeletUnloadRequest, nodelet::NodeletUnloadResponse>;
template class ZyROSConnectorServiceClient<roscpp::Empty, roscpp::EmptyRequest, roscpp::EmptyResponse>;
template class ZyROSConnectorServiceClient<roscpp::GetLoggers, roscpp::GetLoggersRequest, roscpp::GetLoggersResponse>;
template class ZyROSConnectorServiceClient<roscpp::SetLoggerLevel, roscpp::SetLoggerLevelRequest, roscpp::SetLoggerLevelResponse>;
template class ZyROSConnectorServiceClient<roscpp_tutorials::TwoInts, roscpp_tutorials::TwoIntsRequest, roscpp_tutorials::TwoIntsResponse>;
template class ZyROSConnectorServiceClient<rospy_tutorials::AddTwoInts, rospy_tutorials::AddTwoIntsRequest, rospy_tutorials::AddTwoIntsResponse>;
template class ZyROSConnectorServiceClient<rospy_tutorials::BadTwoInts, rospy_tutorials::BadTwoIntsRequest, rospy_tutorials::BadTwoIntsResponse>;
template class ZyROSConnectorServiceClient<sensor_msgs::SetCameraInfo, sensor_msgs::SetCameraInfoRequest, sensor_msgs::SetCameraInfoResponse>;
template class ZyROSConnectorServiceClient<std_srvs::Empty, std_srvs::EmptyRequest, std_srvs::EmptyResponse>;
template class ZyROSConnectorServiceClient<std_srvs::SetBool, std_srvs::SetBoolRequest, std_srvs::SetBoolResponse>;
template class ZyROSConnectorServiceClient<std_srvs::Trigger, std_srvs::TriggerRequest, std_srvs::TriggerResponse>;
template class ZyROSConnectorServiceClient<tf::FrameGraph, tf::FrameGraphRequest, tf::FrameGraphResponse>;
template class ZyROSConnectorServiceClient<tf2_msgs::FrameGraph, tf2_msgs::FrameGraphRequest, tf2_msgs::FrameGraphResponse>;
template class ZyROSConnectorServiceClient<topic_tools::DemuxAdd, topic_tools::DemuxAddRequest, topic_tools::DemuxAddResponse>;
template class ZyROSConnectorServiceClient<topic_tools::DemuxDelete, topic_tools::DemuxDeleteRequest, topic_tools::DemuxDeleteResponse>;
template class ZyROSConnectorServiceClient<topic_tools::DemuxList, topic_tools::DemuxListRequest, topic_tools::DemuxListResponse>;
template class ZyROSConnectorServiceClient<topic_tools::DemuxSelect, topic_tools::DemuxSelectRequest, topic_tools::DemuxSelectResponse>;
template class ZyROSConnectorServiceClient<topic_tools::MuxAdd, topic_tools::MuxAddRequest, topic_tools::MuxAddResponse>;
template class ZyROSConnectorServiceClient<topic_tools::MuxDelete, topic_tools::MuxDeleteRequest, topic_tools::MuxDeleteResponse>;
template class ZyROSConnectorServiceClient<topic_tools::MuxList, topic_tools::MuxListRequest, topic_tools::MuxListResponse>;
template class ZyROSConnectorServiceClient<topic_tools::MuxSelect, topic_tools::MuxSelectRequest, topic_tools::MuxSelectResponse>;
boost::shared_ptr<ZyROSServiceClient> ZyROSConnectorServiceClientFactory::createServiceClient(ros::NodeHandlePtr rosNode, const std::string& serviceURI, const std::string& serviceType)
{
	bool supported = false;
	boost::shared_ptr<ZyROSServiceClient> serviceClient;
	// Service client instance for ROS service type: control_msgs/QueryCalibrationState
	if (serviceType == "control_msgs::QueryCalibrationState")
	{
		supported = true;
		serviceClient.reset(new ZyROSConnectorServiceClient<control_msgs::QueryCalibrationState, control_msgs::QueryCalibrationStateRequest, control_msgs::QueryCalibrationStateResponse>(rosNode, serviceURI, 10));
	}
	// Service client instance for ROS service type: control_msgs/QueryTrajectoryState
	if (serviceType == "control_msgs::QueryTrajectoryState")
	{
		supported = true;
		serviceClient.reset(new ZyROSConnectorServiceClient<control_msgs::QueryTrajectoryState, control_msgs::QueryTrajectoryStateRequest, control_msgs::QueryTrajectoryStateResponse>(rosNode, serviceURI, 10));
	}
	// Service client instance for ROS service type: diagnostic_msgs/AddDiagnostics
	if (serviceType == "diagnostic_msgs::AddDiagnostics")
	{
		supported = true;
		serviceClient.reset(new ZyROSConnectorServiceClient<diagnostic_msgs::AddDiagnostics, diagnostic_msgs::AddDiagnosticsRequest, diagnostic_msgs::AddDiagnosticsResponse>(rosNode, serviceURI, 10));
	}
	// Service client instance for ROS service type: diagnostic_msgs/SelfTest
	if (serviceType == "diagnostic_msgs::SelfTest")
	{
		supported = true;
		serviceClient.reset(new ZyROSConnectorServiceClient<diagnostic_msgs::SelfTest, diagnostic_msgs::SelfTestRequest, diagnostic_msgs::SelfTestResponse>(rosNode, serviceURI, 10));
	}
	// Service client instance for ROS service type: dynamic_reconfigure/Reconfigure
	if (serviceType == "dynamic_reconfigure::Reconfigure")
	{
		supported = true;
		serviceClient.reset(new ZyROSConnectorServiceClient<dynamic_reconfigure::Reconfigure, dynamic_reconfigure::ReconfigureRequest, dynamic_reconfigure::ReconfigureResponse>(rosNode, serviceURI, 10));
	}
	// Service client instance for ROS service type: nav_msgs/GetMap
	if (serviceType == "nav_msgs::GetMap")
	{
		supported = true;
		serviceClient.reset(new ZyROSConnectorServiceClient<nav_msgs::GetMap, nav_msgs::GetMapRequest, nav_msgs::GetMapResponse>(rosNode, serviceURI, 10));
	}
	// Service client instance for ROS service type: nav_msgs/GetPlan
	if (serviceType == "nav_msgs::GetPlan")
	{
		supported = true;
		serviceClient.reset(new ZyROSConnectorServiceClient<nav_msgs::GetPlan, nav_msgs::GetPlanRequest, nav_msgs::GetPlanResponse>(rosNode, serviceURI, 10));
	}
	// Service client instance for ROS service type: nav_msgs/SetMap
	if (serviceType == "nav_msgs::SetMap")
	{
		supported = true;
		serviceClient.reset(new ZyROSConnectorServiceClient<nav_msgs::SetMap, nav_msgs::SetMapRequest, nav_msgs::SetMapResponse>(rosNode, serviceURI, 10));
	}
	// Service client instance for ROS service type: nodelet/NodeletList
	if (serviceType == "nodelet::NodeletList")
	{
		supported = true;
		serviceClient.reset(new ZyROSConnectorServiceClient<nodelet::NodeletList, nodelet::NodeletListRequest, nodelet::NodeletListResponse>(rosNode, serviceURI, 10));
	}
	// Service client instance for ROS service type: nodelet/NodeletLoad
	if (serviceType == "nodelet::NodeletLoad")
	{
		supported = true;
		serviceClient.reset(new ZyROSConnectorServiceClient<nodelet::NodeletLoad, nodelet::NodeletLoadRequest, nodelet::NodeletLoadResponse>(rosNode, serviceURI, 10));
	}
	// Service client instance for ROS service type: nodelet/NodeletUnload
	if (serviceType == "nodelet::NodeletUnload")
	{
		supported = true;
		serviceClient.reset(new ZyROSConnectorServiceClient<nodelet::NodeletUnload, nodelet::NodeletUnloadRequest, nodelet::NodeletUnloadResponse>(rosNode, serviceURI, 10));
	}
	// Service client instance for ROS service type: roscpp/Empty
	if (serviceType == "roscpp::Empty")
	{
		supported = true;
		serviceClient.reset(new ZyROSConnectorServiceClient<roscpp::Empty, roscpp::EmptyRequest, roscpp::EmptyResponse>(rosNode, serviceURI, 10));
	}
	// Service client instance for ROS service type: roscpp/GetLoggers
	if (serviceType == "roscpp::GetLoggers")
	{
		supported = true;
		serviceClient.reset(new ZyROSConnectorServiceClient<roscpp::GetLoggers, roscpp::GetLoggersRequest, roscpp::GetLoggersResponse>(rosNode, serviceURI, 10));
	}
	// Service client instance for ROS service type: roscpp/SetLoggerLevel
	if (serviceType == "roscpp::SetLoggerLevel")
	{
		supported = true;
		serviceClient.reset(new ZyROSConnectorServiceClient<roscpp::SetLoggerLevel, roscpp::SetLoggerLevelRequest, roscpp::SetLoggerLevelResponse>(rosNode, serviceURI, 10));
	}
	// Service client instance for ROS service type: roscpp_tutorials/TwoInts
	if (serviceType == "roscpp_tutorials::TwoInts")
	{
		supported = true;
		serviceClient.reset(new ZyROSConnectorServiceClient<roscpp_tutorials::TwoInts, roscpp_tutorials::TwoIntsRequest, roscpp_tutorials::TwoIntsResponse>(rosNode, serviceURI, 10));
	}
	// Service client instance for ROS service type: rospy_tutorials/AddTwoInts
	if (serviceType == "rospy_tutorials::AddTwoInts")
	{
		supported = true;
		serviceClient.reset(new ZyROSConnectorServiceClient<rospy_tutorials::AddTwoInts, rospy_tutorials::AddTwoIntsRequest, rospy_tutorials::AddTwoIntsResponse>(rosNode, serviceURI, 10));
	}
	// Service client instance for ROS service type: rospy_tutorials/BadTwoInts
	if (serviceType == "rospy_tutorials::BadTwoInts")
	{
		supported = true;
		serviceClient.reset(new ZyROSConnectorServiceClient<rospy_tutorials::BadTwoInts, rospy_tutorials::BadTwoIntsRequest, rospy_tutorials::BadTwoIntsResponse>(rosNode, serviceURI, 10));
	}
	// Service client instance for ROS service type: sensor_msgs/SetCameraInfo
	if (serviceType == "sensor_msgs::SetCameraInfo")
	{
		supported = true;
		serviceClient.reset(new ZyROSConnectorServiceClient<sensor_msgs::SetCameraInfo, sensor_msgs::SetCameraInfoRequest, sensor_msgs::SetCameraInfoResponse>(rosNode, serviceURI, 10));
	}
	// Service client instance for ROS service type: std_srvs/Empty
	if (serviceType == "std_srvs::Empty")
	{
		supported = true;
		serviceClient.reset(new ZyROSConnectorServiceClient<std_srvs::Empty, std_srvs::EmptyRequest, std_srvs::EmptyResponse>(rosNode, serviceURI, 10));
	}
	// Service client instance for ROS service type: std_srvs/SetBool
	if (serviceType == "std_srvs::SetBool")
	{
		supported = true;
		serviceClient.reset(new ZyROSConnectorServiceClient<std_srvs::SetBool, std_srvs::SetBoolRequest, std_srvs::SetBoolResponse>(rosNode, serviceURI, 10));
	}
	// Service client instance for ROS service type: std_srvs/Trigger
	if (serviceType == "std_srvs::Trigger")
	{
		supported = true;
		serviceClient.reset(new ZyROSConnectorServiceClient<std_srvs::Trigger, std_srvs::TriggerRequest, std_srvs::TriggerResponse>(rosNode, serviceURI, 10));
	}
	// Service client instance for ROS service type: tf/FrameGraph
	if (serviceType == "tf::FrameGraph")
	{
		supported = true;
		serviceClient.reset(new ZyROSConnectorServiceClient<tf::FrameGraph, tf::FrameGraphRequest, tf::FrameGraphResponse>(rosNode, serviceURI, 10));
	}
	// Service client instance for ROS service type: tf2_msgs/FrameGraph
	if (serviceType == "tf2_msgs::FrameGraph")
	{
		supported = true;
		serviceClient.reset(new ZyROSConnectorServiceClient<tf2_msgs::FrameGraph, tf2_msgs::FrameGraphRequest, tf2_msgs::FrameGraphResponse>(rosNode, serviceURI, 10));
	}
	// Service client instance for ROS service type: topic_tools/DemuxAdd
	if (serviceType == "topic_tools::DemuxAdd")
	{
		supported = true;
		serviceClient.reset(new ZyROSConnectorServiceClient<topic_tools::DemuxAdd, topic_tools::DemuxAddRequest, topic_tools::DemuxAddResponse>(rosNode, serviceURI, 10));
	}
	// Service client instance for ROS service type: topic_tools/DemuxDelete
	if (serviceType == "topic_tools::DemuxDelete")
	{
		supported = true;
		serviceClient.reset(new ZyROSConnectorServiceClient<topic_tools::DemuxDelete, topic_tools::DemuxDeleteRequest, topic_tools::DemuxDeleteResponse>(rosNode, serviceURI, 10));
	}
	// Service client instance for ROS service type: topic_tools/DemuxList
	if (serviceType == "topic_tools::DemuxList")
	{
		supported = true;
		serviceClient.reset(new ZyROSConnectorServiceClient<topic_tools::DemuxList, topic_tools::DemuxListRequest, topic_tools::DemuxListResponse>(rosNode, serviceURI, 10));
	}
	// Service client instance for ROS service type: topic_tools/DemuxSelect
	if (serviceType == "topic_tools::DemuxSelect")
	{
		supported = true;
		serviceClient.reset(new ZyROSConnectorServiceClient<topic_tools::DemuxSelect, topic_tools::DemuxSelectRequest, topic_tools::DemuxSelectResponse>(rosNode, serviceURI, 10));
	}
	// Service client instance for ROS service type: topic_tools/MuxAdd
	if (serviceType == "topic_tools::MuxAdd")
	{
		supported = true;
		serviceClient.reset(new ZyROSConnectorServiceClient<topic_tools::MuxAdd, topic_tools::MuxAddRequest, topic_tools::MuxAddResponse>(rosNode, serviceURI, 10));
	}
	// Service client instance for ROS service type: topic_tools/MuxDelete
	if (serviceType == "topic_tools::MuxDelete")
	{
		supported = true;
		serviceClient.reset(new ZyROSConnectorServiceClient<topic_tools::MuxDelete, topic_tools::MuxDeleteRequest, topic_tools::MuxDeleteResponse>(rosNode, serviceURI, 10));
	}
	// Service client instance for ROS service type: topic_tools/MuxList
	if (serviceType == "topic_tools::MuxList")
	{
		supported = true;
		serviceClient.reset(new ZyROSConnectorServiceClient<topic_tools::MuxList, topic_tools::MuxListRequest, topic_tools::MuxListResponse>(rosNode, serviceURI, 10));
	}
	// Service client instance for ROS service type: topic_tools/MuxSelect
	if (serviceType == "topic_tools::MuxSelect")
	{
		supported = true;
		serviceClient.reset(new ZyROSConnectorServiceClient<topic_tools::MuxSelect, topic_tools::MuxSelectRequest, topic_tools::MuxSelectResponse>(rosNode, serviceURI, 10));
	}

	if (supported)
	{
		msg_info("ZyROSConnectorServiceClientFactory") << "ROS service type supported: " << serviceType;
	}
	else
	{
		msg_warning("ZyROSConnectorServiceClientFactory") << "ROS service type NOT supported: " << serviceType;
	}
	return serviceClient;
}
