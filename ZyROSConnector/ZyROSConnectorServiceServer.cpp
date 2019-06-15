/***********************************************************************
ROS service definition headers and ROS connector template instantiations.
This file is AUTO-GENERATED during the CMake run.
Please do not modify it by hand.
The contents will be overwritten and re-generated.
************************************************************************/




#include "ZyROSConnectorServiceServer.inl"

using namespace Zyklio::ROSConnector;

ZyROSConnectorServiceServer::ZyROSConnectorServiceServer(/*ros::NodeHandlePtr rosNode,*/ const std::string& serviceURI): m_d(NULL)
{
    m_d = new ZyROSConnectorServiceServerPrivate();
    m_d->m_serviceURI = serviceURI;
    m_d->m_rosNodeHandle.reset(new ros::NodeHandle());
    m_shutdownRequested = false;
}

ZyROSConnectorServiceServer::ZyROSConnectorServiceServer(const ZyROSConnectorServiceServer& other): m_d(NULL)
{
    m_d = new ZyROSConnectorServiceServerPrivate();
    m_d->m_serviceURI = other.m_d->m_serviceURI;
    m_d->m_rosNodeHandle.reset(new ros::NodeHandle());
    m_shutdownRequested = other.m_shutdownRequested;
}

ZyROSConnectorServiceServer::~ZyROSConnectorServiceServer()
{
    if (m_d)
    {
        if (m_d->m_rosNodeHandle)
            m_d->m_rosNodeHandle->shutdown();

        delete m_d;
        m_d = NULL;
    }
}

void ZyROSConnectorServiceServer::shutdownServer()
{
    boost::mutex::scoped_lock lock(m_mutex);
    m_shutdownRequested = true;
}

void ZyROSConnectorServiceServer::serverLoop()
{
    msg_info("ZyROSConnectorServiceServer") << "Entering serverLoop.";

    ros::Rate service_server_rate(25);
    m_serverThreadActive = true;
    while (ros::ok())
    {
        if (m_shutdownRequested)
            break;

        service_server_rate.sleep();
    }

    m_serverThreadActive = false;

    if (m_d->m_rosServer)
    {
        msg_info("ZyROSConnectorServiceServer") << "Shutting down ROS service server.";
        m_d->m_rosServer.shutdown();
    }

    if (m_d->m_rosNodeHandle)
    {
        msg_info("ZyROSConnectorServiceServer") << "Shutting down service server ROS node.";
        m_d->m_rosNodeHandle->shutdown();
    }
}

        
#include <control_msgs/QueryCalibrationStateRequest.h>
#include <control_msgs/QueryCalibrationStateResponse.h>
#include <control_msgs/QueryCalibrationState.h>
#include <control_msgs/QueryTrajectoryStateResponse.h>
#include <control_msgs/QueryTrajectoryStateRequest.h>
#include <control_msgs/QueryTrajectoryState.h>
#include <diagnostic_msgs/AddDiagnostics.h>
#include <diagnostic_msgs/AddDiagnosticsRequest.h>
#include <diagnostic_msgs/AddDiagnosticsResponse.h>
#include <diagnostic_msgs/SelfTestResponse.h>
#include <diagnostic_msgs/SelfTest.h>
#include <diagnostic_msgs/SelfTestRequest.h>
#include <dynamic_reconfigure/ReconfigureResponse.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/ReconfigureRequest.h>
#include <nav_msgs/GetMapActionGoal.h>
#include <nav_msgs/GetMap.h>
#include <nav_msgs/GetMapActionResult.h>
#include <nav_msgs/GetMapResult.h>
#include <nav_msgs/GetMapAction.h>
#include <nav_msgs/GetMapFeedback.h>
#include <nav_msgs/GetMapRequest.h>
#include <nav_msgs/GetMapActionFeedback.h>
#include <nav_msgs/GetMapResponse.h>
#include <nav_msgs/GetMapGoal.h>
#include <nav_msgs/GetPlan.h>
#include <nav_msgs/GetPlanResponse.h>
#include <nav_msgs/GetPlanRequest.h>
#include <nav_msgs/SetMapRequest.h>
#include <nav_msgs/SetMap.h>
#include <nav_msgs/SetMapResponse.h>
#include <nodelet/NodeletListRequest.h>
#include <nodelet/NodeletListResponse.h>
#include <nodelet/NodeletList.h>
#include <nodelet/NodeletLoad.h>
#include <nodelet/NodeletLoadResponse.h>
#include <nodelet/NodeletLoadRequest.h>
#include <nodelet/NodeletUnloadRequest.h>
#include <nodelet/NodeletUnload.h>
#include <nodelet/NodeletUnloadResponse.h>
#include <roscpp/EmptyRequest.h>
#include <roscpp/Empty.h>
#include <roscpp/EmptyResponse.h>
#include <roscpp/GetLoggers.h>
#include <roscpp/GetLoggersResponse.h>
#include <roscpp/GetLoggersRequest.h>
#include <roscpp/SetLoggerLevel.h>
#include <roscpp/SetLoggerLevelRequest.h>
#include <roscpp/SetLoggerLevelResponse.h>
#include <roscpp_tutorials/TwoIntsRequest.h>
#include <roscpp_tutorials/TwoInts.h>
#include <roscpp_tutorials/TwoIntsResponse.h>
#include <rospy_tutorials/AddTwoInts.h>
#include <rospy_tutorials/AddTwoIntsResponse.h>
#include <rospy_tutorials/AddTwoIntsRequest.h>
#include <rospy_tutorials/BadTwoInts.h>
#include <rospy_tutorials/BadTwoIntsRequest.h>
#include <rospy_tutorials/BadTwoIntsResponse.h>
#include <sensor_msgs/SetCameraInfo.h>
#include <sensor_msgs/SetCameraInfoResponse.h>
#include <sensor_msgs/SetCameraInfoRequest.h>
#include <sofa_softrobots_msgs/SoftRobotActuators.h>
#include <sofa_softrobots_msgs/SoftRobotActuatorsRequest.h>
#include <sofa_softrobots_msgs/SoftRobotActuatorsResponse.h>
#include <sofa_softrobots_msgs/SoftRobotCableActuatorsRequest.h>
#include <sofa_softrobots_msgs/SoftRobotCableActuatorsResponse.h>
#include <sofa_softrobots_msgs/SoftRobotCableActuators.h>
#include <sofa_softrobots_msgs/SoftRobotSurfacePressureActuatorsResponse.h>
#include <sofa_softrobots_msgs/SoftRobotSurfacePressureActuatorsRequest.h>
#include <sofa_softrobots_msgs/SoftRobotSurfacePressureActuators.h>
#include <std_srvs/EmptyRequest.h>
#include <std_srvs/Empty.h>
#include <std_srvs/EmptyResponse.h>
#include <std_srvs/SetBoolResponse.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/SetBoolRequest.h>
#include <std_srvs/Trigger.h>
#include <std_srvs/TriggerResponse.h>
#include <std_srvs/TriggerRequest.h>
#include <tf/FrameGraphRequest.h>
#include <tf/FrameGraphResponse.h>
#include <tf/FrameGraph.h>
#include <tf2_msgs/FrameGraphRequest.h>
#include <tf2_msgs/FrameGraphResponse.h>
#include <tf2_msgs/FrameGraph.h>
#include <topic_tools/DemuxAddRequest.h>
#include <topic_tools/DemuxAdd.h>
#include <topic_tools/DemuxAddResponse.h>
#include <topic_tools/DemuxDelete.h>
#include <topic_tools/DemuxDeleteRequest.h>
#include <topic_tools/DemuxDeleteResponse.h>
#include <topic_tools/DemuxListResponse.h>
#include <topic_tools/DemuxList.h>
#include <topic_tools/DemuxListRequest.h>
#include <topic_tools/DemuxSelectRequest.h>
#include <topic_tools/DemuxSelect.h>
#include <topic_tools/DemuxSelectResponse.h>
#include <topic_tools/MuxAddResponse.h>
#include <topic_tools/MuxAdd.h>
#include <topic_tools/MuxAddRequest.h>
#include <topic_tools/MuxDeleteResponse.h>
#include <topic_tools/MuxDelete.h>
#include <topic_tools/MuxDeleteRequest.h>
#include <topic_tools/MuxListResponse.h>
#include <topic_tools/MuxListRequest.h>
#include <topic_tools/MuxList.h>
#include <topic_tools/MuxSelect.h>
#include <topic_tools/MuxSelectRequest.h>
#include <topic_tools/MuxSelectResponse.h>
#include <zyrosconnector_test/ArrayOfFloats.h>
#include <zyrosconnector_test/ArrayOfFloatsResponse.h>
#include <zyrosconnector_test/ArrayOfFloatsRequest.h>
// ROS service server template type instantiations
template class ZyROSConnectorServiceServerImpl<control_msgs::QueryCalibrationStateRequest, control_msgs::QueryCalibrationStateResponse, ZyROSConnectorServerRequestHandler<control_msgs::QueryCalibrationStateRequest, control_msgs::QueryCalibrationStateResponse>>;
template class ZyROSConnectorServiceServerImpl<control_msgs::QueryTrajectoryStateRequest, control_msgs::QueryTrajectoryStateResponse, ZyROSConnectorServerRequestHandler<control_msgs::QueryTrajectoryStateRequest, control_msgs::QueryTrajectoryStateResponse>>;
template class ZyROSConnectorServiceServerImpl<diagnostic_msgs::AddDiagnosticsRequest, diagnostic_msgs::AddDiagnosticsResponse, ZyROSConnectorServerRequestHandler<diagnostic_msgs::AddDiagnosticsRequest, diagnostic_msgs::AddDiagnosticsResponse>>;
template class ZyROSConnectorServiceServerImpl<diagnostic_msgs::SelfTestRequest, diagnostic_msgs::SelfTestResponse, ZyROSConnectorServerRequestHandler<diagnostic_msgs::SelfTestRequest, diagnostic_msgs::SelfTestResponse>>;
template class ZyROSConnectorServiceServerImpl<dynamic_reconfigure::ReconfigureRequest, dynamic_reconfigure::ReconfigureResponse, ZyROSConnectorServerRequestHandler<dynamic_reconfigure::ReconfigureRequest, dynamic_reconfigure::ReconfigureResponse>>;
template class ZyROSConnectorServiceServerImpl<nav_msgs::GetMapRequest, nav_msgs::GetMapResponse, ZyROSConnectorServerRequestHandler<nav_msgs::GetMapRequest, nav_msgs::GetMapResponse>>;
template class ZyROSConnectorServiceServerImpl<nav_msgs::GetPlanRequest, nav_msgs::GetPlanResponse, ZyROSConnectorServerRequestHandler<nav_msgs::GetPlanRequest, nav_msgs::GetPlanResponse>>;
template class ZyROSConnectorServiceServerImpl<nav_msgs::SetMapRequest, nav_msgs::SetMapResponse, ZyROSConnectorServerRequestHandler<nav_msgs::SetMapRequest, nav_msgs::SetMapResponse>>;
template class ZyROSConnectorServiceServerImpl<nodelet::NodeletListRequest, nodelet::NodeletListResponse, ZyROSConnectorServerRequestHandler<nodelet::NodeletListRequest, nodelet::NodeletListResponse>>;
template class ZyROSConnectorServiceServerImpl<nodelet::NodeletLoadRequest, nodelet::NodeletLoadResponse, ZyROSConnectorServerRequestHandler<nodelet::NodeletLoadRequest, nodelet::NodeletLoadResponse>>;
template class ZyROSConnectorServiceServerImpl<nodelet::NodeletUnloadRequest, nodelet::NodeletUnloadResponse, ZyROSConnectorServerRequestHandler<nodelet::NodeletUnloadRequest, nodelet::NodeletUnloadResponse>>;
template class ZyROSConnectorServiceServerImpl<roscpp::EmptyRequest, roscpp::EmptyResponse, ZyROSConnectorServerRequestHandler<roscpp::EmptyRequest, roscpp::EmptyResponse>>;
template class ZyROSConnectorServiceServerImpl<roscpp::GetLoggersRequest, roscpp::GetLoggersResponse, ZyROSConnectorServerRequestHandler<roscpp::GetLoggersRequest, roscpp::GetLoggersResponse>>;
template class ZyROSConnectorServiceServerImpl<roscpp::SetLoggerLevelRequest, roscpp::SetLoggerLevelResponse, ZyROSConnectorServerRequestHandler<roscpp::SetLoggerLevelRequest, roscpp::SetLoggerLevelResponse>>;
template class ZyROSConnectorServiceServerImpl<roscpp_tutorials::TwoIntsRequest, roscpp_tutorials::TwoIntsResponse, ZyROSConnectorServerRequestHandler<roscpp_tutorials::TwoIntsRequest, roscpp_tutorials::TwoIntsResponse>>;
template class ZyROSConnectorServiceServerImpl<rospy_tutorials::AddTwoIntsRequest, rospy_tutorials::AddTwoIntsResponse, ZyROSConnectorServerRequestHandler<rospy_tutorials::AddTwoIntsRequest, rospy_tutorials::AddTwoIntsResponse>>;
template class ZyROSConnectorServiceServerImpl<rospy_tutorials::BadTwoIntsRequest, rospy_tutorials::BadTwoIntsResponse, ZyROSConnectorServerRequestHandler<rospy_tutorials::BadTwoIntsRequest, rospy_tutorials::BadTwoIntsResponse>>;
template class ZyROSConnectorServiceServerImpl<sensor_msgs::SetCameraInfoRequest, sensor_msgs::SetCameraInfoResponse, ZyROSConnectorServerRequestHandler<sensor_msgs::SetCameraInfoRequest, sensor_msgs::SetCameraInfoResponse>>;
template class ZyROSConnectorServiceServerImpl<sofa_softrobots_msgs::SoftRobotActuatorsRequest, sofa_softrobots_msgs::SoftRobotActuatorsResponse, ZyROSConnectorServerRequestHandler<sofa_softrobots_msgs::SoftRobotActuatorsRequest, sofa_softrobots_msgs::SoftRobotActuatorsResponse>>;
template class ZyROSConnectorServiceServerImpl<sofa_softrobots_msgs::SoftRobotCableActuatorsRequest, sofa_softrobots_msgs::SoftRobotCableActuatorsResponse, ZyROSConnectorServerRequestHandler<sofa_softrobots_msgs::SoftRobotCableActuatorsRequest, sofa_softrobots_msgs::SoftRobotCableActuatorsResponse>>;
template class ZyROSConnectorServiceServerImpl<sofa_softrobots_msgs::SoftRobotSurfacePressureActuatorsRequest, sofa_softrobots_msgs::SoftRobotSurfacePressureActuatorsResponse, ZyROSConnectorServerRequestHandler<sofa_softrobots_msgs::SoftRobotSurfacePressureActuatorsRequest, sofa_softrobots_msgs::SoftRobotSurfacePressureActuatorsResponse>>;
template class ZyROSConnectorServiceServerImpl<std_srvs::EmptyRequest, std_srvs::EmptyResponse, ZyROSConnectorServerRequestHandler<std_srvs::EmptyRequest, std_srvs::EmptyResponse>>;
template class ZyROSConnectorServiceServerImpl<std_srvs::SetBoolRequest, std_srvs::SetBoolResponse, ZyROSConnectorServerRequestHandler<std_srvs::SetBoolRequest, std_srvs::SetBoolResponse>>;
template class ZyROSConnectorServiceServerImpl<std_srvs::TriggerRequest, std_srvs::TriggerResponse, ZyROSConnectorServerRequestHandler<std_srvs::TriggerRequest, std_srvs::TriggerResponse>>;
template class ZyROSConnectorServiceServerImpl<tf::FrameGraphRequest, tf::FrameGraphResponse, ZyROSConnectorServerRequestHandler<tf::FrameGraphRequest, tf::FrameGraphResponse>>;
template class ZyROSConnectorServiceServerImpl<tf2_msgs::FrameGraphRequest, tf2_msgs::FrameGraphResponse, ZyROSConnectorServerRequestHandler<tf2_msgs::FrameGraphRequest, tf2_msgs::FrameGraphResponse>>;
template class ZyROSConnectorServiceServerImpl<topic_tools::DemuxAddRequest, topic_tools::DemuxAddResponse, ZyROSConnectorServerRequestHandler<topic_tools::DemuxAddRequest, topic_tools::DemuxAddResponse>>;
template class ZyROSConnectorServiceServerImpl<topic_tools::DemuxDeleteRequest, topic_tools::DemuxDeleteResponse, ZyROSConnectorServerRequestHandler<topic_tools::DemuxDeleteRequest, topic_tools::DemuxDeleteResponse>>;
template class ZyROSConnectorServiceServerImpl<topic_tools::DemuxListRequest, topic_tools::DemuxListResponse, ZyROSConnectorServerRequestHandler<topic_tools::DemuxListRequest, topic_tools::DemuxListResponse>>;
template class ZyROSConnectorServiceServerImpl<topic_tools::DemuxSelectRequest, topic_tools::DemuxSelectResponse, ZyROSConnectorServerRequestHandler<topic_tools::DemuxSelectRequest, topic_tools::DemuxSelectResponse>>;
template class ZyROSConnectorServiceServerImpl<topic_tools::MuxAddRequest, topic_tools::MuxAddResponse, ZyROSConnectorServerRequestHandler<topic_tools::MuxAddRequest, topic_tools::MuxAddResponse>>;
template class ZyROSConnectorServiceServerImpl<topic_tools::MuxDeleteRequest, topic_tools::MuxDeleteResponse, ZyROSConnectorServerRequestHandler<topic_tools::MuxDeleteRequest, topic_tools::MuxDeleteResponse>>;
template class ZyROSConnectorServiceServerImpl<topic_tools::MuxListRequest, topic_tools::MuxListResponse, ZyROSConnectorServerRequestHandler<topic_tools::MuxListRequest, topic_tools::MuxListResponse>>;
template class ZyROSConnectorServiceServerImpl<topic_tools::MuxSelectRequest, topic_tools::MuxSelectResponse, ZyROSConnectorServerRequestHandler<topic_tools::MuxSelectRequest, topic_tools::MuxSelectResponse>>;
template class ZyROSConnectorServiceServerImpl<zyrosconnector_test::ArrayOfFloatsRequest, zyrosconnector_test::ArrayOfFloatsResponse, ZyROSConnectorServerRequestHandler<zyrosconnector_test::ArrayOfFloatsRequest, zyrosconnector_test::ArrayOfFloatsResponse>>;


// ROS service server worker thread template type instantiations
template class ZyROSConnectorServiceServerWorkerThread<control_msgs::QueryCalibrationStateRequest, control_msgs::QueryCalibrationStateResponse, ZyROSConnectorServerRequestHandler<control_msgs::QueryCalibrationStateRequest, control_msgs::QueryCalibrationStateResponse>>;
template class ZyROSConnectorServiceServerWorkerThread<control_msgs::QueryTrajectoryStateRequest, control_msgs::QueryTrajectoryStateResponse, ZyROSConnectorServerRequestHandler<control_msgs::QueryTrajectoryStateRequest, control_msgs::QueryTrajectoryStateResponse>>;
template class ZyROSConnectorServiceServerWorkerThread<diagnostic_msgs::AddDiagnosticsRequest, diagnostic_msgs::AddDiagnosticsResponse, ZyROSConnectorServerRequestHandler<diagnostic_msgs::AddDiagnosticsRequest, diagnostic_msgs::AddDiagnosticsResponse>>;
template class ZyROSConnectorServiceServerWorkerThread<diagnostic_msgs::SelfTestRequest, diagnostic_msgs::SelfTestResponse, ZyROSConnectorServerRequestHandler<diagnostic_msgs::SelfTestRequest, diagnostic_msgs::SelfTestResponse>>;
template class ZyROSConnectorServiceServerWorkerThread<dynamic_reconfigure::ReconfigureRequest, dynamic_reconfigure::ReconfigureResponse, ZyROSConnectorServerRequestHandler<dynamic_reconfigure::ReconfigureRequest, dynamic_reconfigure::ReconfigureResponse>>;
template class ZyROSConnectorServiceServerWorkerThread<nav_msgs::GetMapRequest, nav_msgs::GetMapResponse, ZyROSConnectorServerRequestHandler<nav_msgs::GetMapRequest, nav_msgs::GetMapResponse>>;
template class ZyROSConnectorServiceServerWorkerThread<nav_msgs::GetPlanRequest, nav_msgs::GetPlanResponse, ZyROSConnectorServerRequestHandler<nav_msgs::GetPlanRequest, nav_msgs::GetPlanResponse>>;
template class ZyROSConnectorServiceServerWorkerThread<nav_msgs::SetMapRequest, nav_msgs::SetMapResponse, ZyROSConnectorServerRequestHandler<nav_msgs::SetMapRequest, nav_msgs::SetMapResponse>>;
template class ZyROSConnectorServiceServerWorkerThread<nodelet::NodeletListRequest, nodelet::NodeletListResponse, ZyROSConnectorServerRequestHandler<nodelet::NodeletListRequest, nodelet::NodeletListResponse>>;
template class ZyROSConnectorServiceServerWorkerThread<nodelet::NodeletLoadRequest, nodelet::NodeletLoadResponse, ZyROSConnectorServerRequestHandler<nodelet::NodeletLoadRequest, nodelet::NodeletLoadResponse>>;
template class ZyROSConnectorServiceServerWorkerThread<nodelet::NodeletUnloadRequest, nodelet::NodeletUnloadResponse, ZyROSConnectorServerRequestHandler<nodelet::NodeletUnloadRequest, nodelet::NodeletUnloadResponse>>;
template class ZyROSConnectorServiceServerWorkerThread<roscpp::EmptyRequest, roscpp::EmptyResponse, ZyROSConnectorServerRequestHandler<roscpp::EmptyRequest, roscpp::EmptyResponse>>;
template class ZyROSConnectorServiceServerWorkerThread<roscpp::GetLoggersRequest, roscpp::GetLoggersResponse, ZyROSConnectorServerRequestHandler<roscpp::GetLoggersRequest, roscpp::GetLoggersResponse>>;
template class ZyROSConnectorServiceServerWorkerThread<roscpp::SetLoggerLevelRequest, roscpp::SetLoggerLevelResponse, ZyROSConnectorServerRequestHandler<roscpp::SetLoggerLevelRequest, roscpp::SetLoggerLevelResponse>>;
template class ZyROSConnectorServiceServerWorkerThread<roscpp_tutorials::TwoIntsRequest, roscpp_tutorials::TwoIntsResponse, ZyROSConnectorServerRequestHandler<roscpp_tutorials::TwoIntsRequest, roscpp_tutorials::TwoIntsResponse>>;
template class ZyROSConnectorServiceServerWorkerThread<rospy_tutorials::AddTwoIntsRequest, rospy_tutorials::AddTwoIntsResponse, ZyROSConnectorServerRequestHandler<rospy_tutorials::AddTwoIntsRequest, rospy_tutorials::AddTwoIntsResponse>>;
template class ZyROSConnectorServiceServerWorkerThread<rospy_tutorials::BadTwoIntsRequest, rospy_tutorials::BadTwoIntsResponse, ZyROSConnectorServerRequestHandler<rospy_tutorials::BadTwoIntsRequest, rospy_tutorials::BadTwoIntsResponse>>;
template class ZyROSConnectorServiceServerWorkerThread<sensor_msgs::SetCameraInfoRequest, sensor_msgs::SetCameraInfoResponse, ZyROSConnectorServerRequestHandler<sensor_msgs::SetCameraInfoRequest, sensor_msgs::SetCameraInfoResponse>>;
template class ZyROSConnectorServiceServerWorkerThread<sofa_softrobots_msgs::SoftRobotActuatorsRequest, sofa_softrobots_msgs::SoftRobotActuatorsResponse, ZyROSConnectorServerRequestHandler<sofa_softrobots_msgs::SoftRobotActuatorsRequest, sofa_softrobots_msgs::SoftRobotActuatorsResponse>>;
template class ZyROSConnectorServiceServerWorkerThread<sofa_softrobots_msgs::SoftRobotCableActuatorsRequest, sofa_softrobots_msgs::SoftRobotCableActuatorsResponse, ZyROSConnectorServerRequestHandler<sofa_softrobots_msgs::SoftRobotCableActuatorsRequest, sofa_softrobots_msgs::SoftRobotCableActuatorsResponse>>;
template class ZyROSConnectorServiceServerWorkerThread<sofa_softrobots_msgs::SoftRobotSurfacePressureActuatorsRequest, sofa_softrobots_msgs::SoftRobotSurfacePressureActuatorsResponse, ZyROSConnectorServerRequestHandler<sofa_softrobots_msgs::SoftRobotSurfacePressureActuatorsRequest, sofa_softrobots_msgs::SoftRobotSurfacePressureActuatorsResponse>>;
template class ZyROSConnectorServiceServerWorkerThread<std_srvs::EmptyRequest, std_srvs::EmptyResponse, ZyROSConnectorServerRequestHandler<std_srvs::EmptyRequest, std_srvs::EmptyResponse>>;
template class ZyROSConnectorServiceServerWorkerThread<std_srvs::SetBoolRequest, std_srvs::SetBoolResponse, ZyROSConnectorServerRequestHandler<std_srvs::SetBoolRequest, std_srvs::SetBoolResponse>>;
template class ZyROSConnectorServiceServerWorkerThread<std_srvs::TriggerRequest, std_srvs::TriggerResponse, ZyROSConnectorServerRequestHandler<std_srvs::TriggerRequest, std_srvs::TriggerResponse>>;
template class ZyROSConnectorServiceServerWorkerThread<tf::FrameGraphRequest, tf::FrameGraphResponse, ZyROSConnectorServerRequestHandler<tf::FrameGraphRequest, tf::FrameGraphResponse>>;
template class ZyROSConnectorServiceServerWorkerThread<tf2_msgs::FrameGraphRequest, tf2_msgs::FrameGraphResponse, ZyROSConnectorServerRequestHandler<tf2_msgs::FrameGraphRequest, tf2_msgs::FrameGraphResponse>>;
template class ZyROSConnectorServiceServerWorkerThread<topic_tools::DemuxAddRequest, topic_tools::DemuxAddResponse, ZyROSConnectorServerRequestHandler<topic_tools::DemuxAddRequest, topic_tools::DemuxAddResponse>>;
template class ZyROSConnectorServiceServerWorkerThread<topic_tools::DemuxDeleteRequest, topic_tools::DemuxDeleteResponse, ZyROSConnectorServerRequestHandler<topic_tools::DemuxDeleteRequest, topic_tools::DemuxDeleteResponse>>;
template class ZyROSConnectorServiceServerWorkerThread<topic_tools::DemuxListRequest, topic_tools::DemuxListResponse, ZyROSConnectorServerRequestHandler<topic_tools::DemuxListRequest, topic_tools::DemuxListResponse>>;
template class ZyROSConnectorServiceServerWorkerThread<topic_tools::DemuxSelectRequest, topic_tools::DemuxSelectResponse, ZyROSConnectorServerRequestHandler<topic_tools::DemuxSelectRequest, topic_tools::DemuxSelectResponse>>;
template class ZyROSConnectorServiceServerWorkerThread<topic_tools::MuxAddRequest, topic_tools::MuxAddResponse, ZyROSConnectorServerRequestHandler<topic_tools::MuxAddRequest, topic_tools::MuxAddResponse>>;
template class ZyROSConnectorServiceServerWorkerThread<topic_tools::MuxDeleteRequest, topic_tools::MuxDeleteResponse, ZyROSConnectorServerRequestHandler<topic_tools::MuxDeleteRequest, topic_tools::MuxDeleteResponse>>;
template class ZyROSConnectorServiceServerWorkerThread<topic_tools::MuxListRequest, topic_tools::MuxListResponse, ZyROSConnectorServerRequestHandler<topic_tools::MuxListRequest, topic_tools::MuxListResponse>>;
template class ZyROSConnectorServiceServerWorkerThread<topic_tools::MuxSelectRequest, topic_tools::MuxSelectResponse, ZyROSConnectorServerRequestHandler<topic_tools::MuxSelectRequest, topic_tools::MuxSelectResponse>>;
template class ZyROSConnectorServiceServerWorkerThread<zyrosconnector_test::ArrayOfFloatsRequest, zyrosconnector_test::ArrayOfFloatsResponse, ZyROSConnectorServerRequestHandler<zyrosconnector_test::ArrayOfFloatsRequest, zyrosconnector_test::ArrayOfFloatsResponse>>;
