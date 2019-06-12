#ifdef _WIN32
// Ensure that Winsock2.h is included before Windows.h, which can get
// pulled in by anybody (e.g., Boost).
#ifndef WINSOCK2_H
#define WINSOCK2_H
#include <Winsock2.h>
#endif
#endif

#ifndef _WIN32
#include <dirent.h>
#include <codecvt>
#endif

#include <SofaTest/Sofa_test.h>

#include <init_ZyROSConnector.h>

#include <ZyROSConnector.h>
#include <ZyROSConnectionManager.h>

#include <ZyROSConnectorTopicPublisher.h>
#include <ZyROSConnectorTopicSubscriber.h>

#include <ZyROSConnectorServiceClient.h>
#include <ZyROSConnectorServiceServer.h>
#include <ZyROSConnectorServiceServer.inl>

#include <ZyROS_MessageType_Instantiations_Publishers.h>
#include <ZyROS_MessageType_Instantiations_Subscribers.h>

#include <ZyROS_ServiceType_Client_Instantiations.h>

#include <rosgraph_msgs/Log.h>
#include <ros/console.h>

#include "ArrayOfFloats.h"
#include "ArrayOfFloatsRequest.h"
#include "ArrayOfFloatsResponse.h"

#include <boost/filesystem.hpp>
#include <boost/thread/mutex.hpp>

namespace Zyklio
{
    using namespace Zyklio::ROSConnector;
    using namespace Zyklio::ROSConnectionManager;

	struct RosConnectorTest : public ::testing::Test
	{
		public:
			RosConnectorTest();
			~RosConnectorTest();

			bool connectToROSMaster(const std::string& masterUri);
			bool disconnectFromROSMaster();

            bool ProcessRunning(
#if _WIN32
                    const std::wstring&
#else
                    const std::string&
                    , pid_t* processPid
#endif
                                );
            int CloseProcesses(
#if _WIN32
                    const std::wstring& processName
#else
                    const std::string& processName
#endif
                    );

			void TestBody();

            ZyROSConnectionManager* m_rosConnector;

			boost::mutex m_mutex;
	};

	RosConnectorTest::RosConnectorTest()
	{
        m_rosConnector = new ZyROSConnectionManager();
	}

	RosConnectorTest::~RosConnectorTest()
	{
		if (m_rosConnector)
		{
			delete m_rosConnector;
			m_rosConnector = NULL;
		}
	}

    bool RosConnectorTest::ProcessRunning(
#if _WIN32
            const std::wstring& processName
#else
            const std::string&processName,
            pid_t* processPid
#endif
                                          )
	{
        bool processOpen = false;
#if _WIN32
		HANDLE hProcessSnap;
		PROCESSENTRY32 pe32;

		// Take a snapshot of all processes in the system.
		hProcessSnap = CreateToolhelp32Snapshot(TH32CS_SNAPPROCESS, 0);
		if (hProcessSnap == INVALID_HANDLE_VALUE)
		{
			return false;
		}

		// Set the size of the structure before using it.
		pe32.dwSize = sizeof(PROCESSENTRY32);

		// Retrieve information about the first process,
		// and exit if unsuccessful
		if (!Process32First(hProcessSnap, &pe32))
		{
			CloseHandle(hProcessSnap);          // clean the snapshot object
			return false;
		}

		// Now walk the snapshot of processes, and
		// display information about each process in turn
		do
		{
			std::wstring processExe(pe32.szExeFile);
			
			if (processExe.compare(processName) == 0)
			{
				processOpen = true;
				break;
			}
		} while (Process32Next(hProcessSnap, &pe32));

		CloseHandle(hProcessSnap);
		return processOpen;
#else
        DIR* dir;
        struct dirent* ent;
        char buf[512];

        long  pid;
        char pname[100] = {0,};
        char state;
        FILE *fp=NULL;

        if (!(dir = opendir("/proc")))
        {
            msg_error("ZyRosConnector_test") << "can't open /proc";
            return false;
        }

        while((ent = readdir(dir)) != NULL)
        {
            long lpid = atol(ent->d_name);
            if(lpid < 0)
                continue;

            snprintf(buf, sizeof(buf), "/proc/%ld/stat", lpid);
            fp = fopen(buf, "r");

            if (fp) {
                if ( (fscanf(fp, "%ld (%[^)]) %c", &pid, pname, &state)) != 3 )
                {
                    msg_error("ZyRosConnetor_test") << "fscanf failed";
                    fclose(fp);
                    closedir(dir);
                    return false;
                }

                std::string currentProcessName(pname);

                // msg_info("ZyROSConnector_test") << "Comparing process names: " << processName << " -- " << currentProcessName;

                if (currentProcessName.compare(processName) == 0)
                {
                    fclose(fp);
                    closedir(dir);
#ifndef _WIN32
                    if (processPid != NULL)
                        *processPid = (pid_t)lpid;
#endif
                    processOpen = true;
                }
                fclose(fp);

                if (processOpen)
                    break;
            }
        }
        if (!processOpen)
            closedir(dir);
#endif
        return processOpen;
	}

    int RosConnectorTest::CloseProcesses(
#if _WIN32
            const std::wstring& processName
#else
            const std::string& processName
#endif
            )
	{
        int closed = 0;
#if _WIN32
		HANDLE hProcessSnap;
		PROCESSENTRY32 pe32;

		// Take a snapshot of all processes in the system.
		hProcessSnap = CreateToolhelp32Snapshot(TH32CS_SNAPPROCESS, 0);
		if (hProcessSnap == INVALID_HANDLE_VALUE)
		{
			return 0;
		}

		// Set the size of the structure before using it.
		pe32.dwSize = sizeof(PROCESSENTRY32);

		// Retrieve information about the first process,
		// and exit if unsuccessful
		if (!Process32First(hProcessSnap, &pe32))
		{
			CloseHandle(hProcessSnap);          // clean the snapshot object
			return 0;
		}

		// Now walk the snapshot of processes, and
		// display information about each process in turn
		do
		{
			std::wstring processExe(pe32.szExeFile);

			if (processExe.compare(processName) == 0)
			{
				closed++;

				HANDLE hProcess = OpenProcess(PROCESS_ALL_ACCESS, false, pe32.th32ProcessID);
				DWORD ExitCode = 0;

				GetExitCodeProcess(hProcess, &ExitCode);
				TerminateProcess(hProcess, ExitCode);
				CloseHandle(hProcess);
			}
		} while (Process32Next(hProcessSnap, &pe32));

		CloseHandle(hProcessSnap);
#else
        DIR* dir;
        struct dirent* ent;
        char buf[512];

        long  pid;
        char pname[100] = {0,};
        char state;
        FILE *fp=NULL;

        if (!(dir = opendir("/proc")))
        {
            msg_error("ZyRosConnector_test") << "can't open /proc";
            return closed;
        }

        while((ent = readdir(dir)) != NULL)
        {
            long lpid = atol(ent->d_name);
            if(lpid < 0)
                continue;

            snprintf(buf, sizeof(buf), "/proc/%ld/stat", lpid);
            fp = fopen(buf, "r");

            if (fp) {
                if ((fscanf(fp, "%ld (%[^)]) %c", &pid, pname, &state)) != 3)
                {
                    msg_error("ZyRosConnector_test") << "fscanf failed";
                    fclose(fp);
                    closedir(dir);
                    return closed;
                }
                std::string currentProcessName(pname);
                if (currentProcessName.compare(processName) == 0)
                {
                    int kill_ret = kill(pid, SIGTERM);
                    if (kill_ret == 0)
                    {
                        msg_info("ZyROSConnector_test") << "Closed running instance of: " << pname;
                        closed++;
                    }
                }
                fclose(fp);
            }
        }

        closedir(dir);
#endif
		return closed;
	}

	bool RosConnectorTest::connectToROSMaster(const std::string& masterUri)
    {
        m_rosConnector->init();
        m_rosConnector->bwdInit();
        msg_info("ZyROSConnector_Test") << "connectToROSMaster: " << masterUri;
        m_rosConnector->getROSConnector()->setRosMasterURI(masterUri);

		boost::mutex::scoped_lock lock(m_mutex);
        while (!m_rosConnector->getROSConnector()->isThreadRunning())
		{
            msg_info("ZyROSConnector_Test") << "Waiting for connector thread to start...";
            m_rosConnector->getROSConnector()->connectorCondition().wait(lock);
		}
        msg_info("ZyROSConnector_Test") << "Connector thread started.";
        if (m_rosConnector->getROSConnector()->isConnected())
		{
            msg_info("ZyROSConnector_Test") << "Connection to roscore established.";
			return true;
		}

        msg_info("ZyROSConnector_Test") << "Failed to connect to roscore.";
		return false;
	}

	bool RosConnectorTest::disconnectFromROSMaster()
	{
        msg_info("ZyROSConnector_test") << "Cleaning up ROS connector.";
        m_rosConnector->cleanup();

		return true;
	}

	void RosConnectorTest::TestBody()
	{
        msg_info("ZyROSConnector_test") << "Executing Test body.";
	}
}

template <class Request, class Response>
struct AddTwoIntsRequestHandler: public ZyROSConnectorServerRequestHandler<Request, Response>
{
    public:
        AddTwoIntsRequestHandler(/*const Request& req, Response& resp, */void* param = nullptr): ZyROSConnectorServerRequestHandler<Request, Response>(param)
        {

        }

        bool handleRequest(const Request& req, Response& resp)
        {
            msg_info("AddTwoIntsRequestHandler") << "Received a request to add two integers: " << req.a << " + " << req.b;
            resp.sum = req.a + req.b;
            msg_info("AddTwoIntsRequestHandler") << "Result: " << resp.sum;
            return true;
        }
};

template <class Request, class Response>
struct ArrayOfFloatsRequestHandler: public ZyROSConnectorServerRequestHandler<Request, Response>
{
    public:
        ArrayOfFloatsRequestHandler(/*const Request& req, Response& resp,*/ void* param = nullptr): ZyROSConnectorServerRequestHandler<Request, Response>(param)
        {

        }

        bool handleRequest(const Request& req, Response& resp)
        {
            msg_info("ArrayOfFloatsRequestHandler") << "Received a request to return an array of floats of size: " << req.FloatsToReturn.data;
            resp.return_values.data.resize(req.FloatsToReturn.data);
            for (int k = 0; k < req.FloatsToReturn.data; k++)
            {
                resp.return_values.data[k] = 1.0f * k;
            }
            return true;
        }
};

template class ZyROSConnectorServiceServerImpl<rospy_tutorials::AddTwoIntsRequest, rospy_tutorials::AddTwoIntsResponse, AddTwoIntsRequestHandler<rospy_tutorials::AddTwoIntsRequest, rospy_tutorials::AddTwoIntsResponse>>;

class AddTwoIntsServiceServer: public ZyROSConnectorServiceServerImpl<rospy_tutorials::AddTwoIntsRequest, rospy_tutorials::AddTwoIntsResponse, AddTwoIntsRequestHandler<rospy_tutorials::AddTwoIntsRequest, rospy_tutorials::AddTwoIntsResponse>>
{
    public:
    AddTwoIntsServiceServer(const std::string& service_uri):
        ZyROSConnectorServiceServerImpl<rospy_tutorials::AddTwoIntsRequest, rospy_tutorials::AddTwoIntsResponse, AddTwoIntsRequestHandler<rospy_tutorials::AddTwoIntsRequest, rospy_tutorials::AddTwoIntsResponse>>(service_uri)
    {

    }

    ~AddTwoIntsServiceServer()
    {}
};

template <class RequestType, class ResponseType, class RequestHandler>
class ArrayOfFloatsServiceServer: public ZyROSConnectorServiceServerImpl<RequestType, ResponseType, RequestHandler>
{
    public:
    ArrayOfFloatsServiceServer(const std::string& service_uri):
        ZyROSConnectorServiceServerImpl<RequestType, ResponseType, RequestHandler>(service_uri)
    {

    }

    ~ArrayOfFloatsServiceServer()
    {}

    bool handleRequest(const RequestType&, ResponseType&);
};

template <class RequestType, class ResponseType, class RequestHandler>
bool ArrayOfFloatsServiceServer<RequestType, ResponseType, RequestHandler>::handleRequest(const RequestType& req, ResponseType& resp)
{
    msg_info("ArrayOfFloatsServiceServer") << "handleRequest called.";
    return this->m_requestHandler.handleRequest(req, resp);
}

template class ArrayOfFloatsServiceServer<zyrosconnector_test::ArrayOfFloatsRequest, zyrosconnector_test::ArrayOfFloatsResponse, ArrayOfFloatsRequestHandler<zyrosconnector_test::ArrayOfFloatsRequest, zyrosconnector_test::ArrayOfFloatsResponse>>;

int main(int argc, char** argv)
{
    using namespace Zyklio::ROSConnector;
    using namespace Zyklio::ROSConnectionManager;

	char* rosRoot_env = getenv("ROS_ROOT");
	std::string rosCoreExecutable;
	std::string rosCoreExecCommand;
	if (rosRoot_env != NULL)
	{
        std::stringstream w_str;
#if _WIN32
		w_str << "/C " << rosRoot_env << L"\\setup.bat & " << rosRoot_env << L"\\bin\\roscore.exe";
		rosCoreExecCommand = w_str.str();
		w_str.str("");
		w_str << rosRoot_env << "\\bin\\roscore.exe";
		rosCoreExecutable = w_str.str();
#else
        w_str << "source " << rosRoot_env << "/../../setup.bash && " << rosRoot_env << "/../../bin/roscore &";
        rosCoreExecCommand = w_str.str();
        w_str.str("");
        w_str << rosRoot_env << "/../../bin/roscore";
        rosCoreExecutable = w_str.str();
#endif
	}
	else
	{
#if _WIN32
		rosCoreExecutable = "C:\\opt\\ros\\hydro\\x86\\bin\\roscore.exe";
		rosCoreExecCommand = "C:\\opt\\ros\\hydro\\x86\\setup.bat & C:\\opt\\ros\\hydro\\x86\\bin\\roscore.exe";
#else
        rosCoreExecutable = "/opt/ros/kinetic/bin/roscore";
        rosCoreExecCommand = "source /opt/ros/kinetic/setup.bash && /opt/ros/kinetic/bin/roscore &";
#endif
	}
		
	std::string rosMasterURI("http://localhost:11311");
	boost::filesystem::path rosCoreExecutablePath(rosCoreExecutable);

#ifndef _WIN32
    pid_t rosCorePID;
#endif

	if (boost::filesystem::exists(rosCoreExecutablePath))
	{
#if _WIN32
		SHELLEXECUTEINFO rSEI = { 0 };
		rSEI.cbSize = sizeof(rSEI);
		rSEI.lpVerb = "open";
		rSEI.lpFile = "cmd.exe";
		rSEI.lpParameters = rosCoreExecCommand.c_str();
		rSEI.nShow = SW_NORMAL;
		rSEI.fMask = SEE_MASK_NOCLOSEPROCESS;
#else
        bool rosCoreStartSucceeded = false;
        std::string rosCoreStartCommand("/bin/bash -c \"" + rosCoreExecCommand + "\"");
        unsigned int rosCoreStartTries = 0;

        msg_info("ZyROSConnector_test") << "Starting roscore using command line: " << rosCoreStartCommand;
        while (rosCoreStartTries < 5 || rosCoreStartSucceeded)
        {
           int ret = system(rosCoreStartCommand.c_str());
           msg_info("ZyROSConnector_test") << "Attempt " << rosCoreStartTries << " 'system' return value: " << ret;
           rosCoreStartTries++;

           sleep(2);

           if (ret == 0)
           {
               msg_info("ZyROSConnector_test") << "Start of roscore process succeeded.";
               rosCoreStartSucceeded = true;
               break;
           }

           if (WIFSIGNALED(ret) &&
               (WTERMSIG(ret) == SIGINT || WTERMSIG(ret) == SIGQUIT))
           {
               msg_info("ZyROSConnector_test") << "Start of roscore process succeeded.";
               rosCoreStartSucceeded = true;
               break;
           }
        }

#endif
			
        Zyklio::RosConnectorTest test_fixture;
#if _WIN32
        if (ShellExecuteEx(&rSEI))
#else
        if (!test_fixture.ProcessRunning("roscore", &rosCorePID))
        {
            msg_error("ZyROSConnector_test") << "Failed to start roscore process!";
            rosCoreStartSucceeded = false;
        }
        if (rosCoreStartSucceeded)
#endif
		{
			int waitAttempts = 0;
			// Wait for rosout.exe, since it's the last child process started by roscore
#if _WIN32
            while (!test_fixture.ProcessRunning(L"rosout.exe") && waitAttempts <= 20)
#else
            pid_t processPid;
            while (!test_fixture.ProcessRunning("rosout", &processPid) && waitAttempts <= 20)
#endif
			{
                msg_info("ZyROSConnector_test") << "Waiting for roscore components to start.";
				waitAttempts++;
#if _WIN32
				Sleep(2000);
#else
                sleep(2);
#endif
			}

			if (waitAttempts >= 20)
			{
                msg_error("ZyROSConnector_test") << "Waiting for roscore startup failed, abort test.";
				return 1;
			}

            msg_info("ZyROSConnector_test") << "Connecting to ROS master...";
			if (test_fixture.connectToROSMaster(rosMasterURI))
			{
                // ros::NodeHandle local_nh;
                // ros::Publisher log_pub = local_nh.advertise<rosgraph_msgs::Log>("ZyROSConnector_test", 1000);

                msg_info("ZyROSConnector_test") << "Connected successfully.";

                const std::vector<std::string> topicNames = test_fixture.m_rosConnector->getTopics();
                const std::vector<std::string> serviceNames = test_fixture.m_rosConnector->getServices();

                msg_info("ZyROSConnector_test") << "Number of topics available: " << topicNames.size();
                for (size_t k = 0; k < topicNames.size(); k++)
                {
                    msg_info("ZyROSConnector_test") << "Topic " << k << ": " << topicNames[k];
                }

                msg_info("ZyROSConnector_test") << "Number of services available: " << serviceNames.size();
                bool getLoggersServiceExists = false;
                bool setLoggerLevelServiceExists = false;
                for (size_t k = 0; k < serviceNames.size(); k++)
                {
                    msg_info("ZyROSConnector_test") << "Service " << k << ": " << serviceNames[k];
                    if (serviceNames[k] == "/rosout/get_loggers")
                        getLoggersServiceExists = true;
                    if (serviceNames[k] == "/rosout/set_logger_level")
                        setLoggerLevelServiceExists = true;
                }

                ZyROSConnectorTopicSubscriber<rosgraph_msgs::Log>* logListener = new ZyROSConnectorTopicSubscriber<rosgraph_msgs::Log>(test_fixture.m_rosConnector->getROSConnector()->getROSNode(), "/ZyROSConnector_test");
                ZyROSConnectorTopicPublisher<rosgraph_msgs::Log>* logPublisher = new ZyROSConnectorTopicPublisher<rosgraph_msgs::Log>(test_fixture.m_rosConnector->getROSConnector()->getROSNode(), "/ZyROSConnector_test");

                boost::shared_ptr<ZyROSListener> logClient(logListener);
                boost::shared_ptr<ZyROSPublisher> logSource(logPublisher);

                test_fixture.m_rosConnector->getROSConnector()->addTopicPublisher(logSource);
                test_fixture.m_rosConnector->getROSConnector()->addTopicListener(logClient);

                // Test publisher and subscriber wrappers using rosgraph_msgs::Log
                for (unsigned int k = 0; k <= 10; ++k)
				{
					std::stringstream msg_stream;
					rosgraph_msgs::Log msg;
					msg_stream << "Log message " << k;

                    msg.name = "ZyROSConnector_Test";
					msg.level = rosgraph_msgs::Log::INFO;
					msg.file = __FILE__;
					msg.line = __LINE__;
					msg.msg = msg_stream.str();

                    msg_info("ZyROSConnector_test") << "Publish message " << k;
                    usleep(50000);
                    logPublisher->publishMessage(msg);
				}

                msg_info("ZyROSConnector_test") << "Messages received by listener: " << logListener->getMessageCount();
                for (size_t k = 0; k < logListener->getMessageCount(); ++k)
                {
                    const rosgraph_msgs::Log& log_msg = logListener->getMessage(k);
                    msg_info("ZyROSConnector_test") << "Log message in received message " << k << ": " << log_msg.msg;
                }

                logListener->clearMessages();
                msg_info("ZyROSConnector_test") << "Message count after cleaning: " << logListener->getMessageCount();

                msg_info("ZyROSConnector_test") << "Removing topic publisher.";
                test_fixture.m_rosConnector->getROSConnector()->removeTopicPublisher(logSource);
                msg_info("ZyROSConnector_test") << "Removing topic subscriber.";
                test_fixture.m_rosConnector->getROSConnector()->removeTopicListener(logClient);

                // Test instantiation via factory methods
                boost::shared_ptr<ZyROSListener> log_sink_2 = ZyROSConnectorMessageSubscriberFactory::createTopicSubscriber(test_fixture.m_rosConnector->getRosNodeHandle(), "/ZyROSConnector_test", "rosgraph_msgs::Log");
                boost::shared_ptr<ZyROSPublisher> log_source_2 = ZyROSConnectorMessagePublisherFactory::createTopicPublisher(test_fixture.m_rosConnector->getRosNodeHandle(), "/ZyROSConnector_test", "rosgraph_msgs::Log");

                ZyROSConnectorTopicPublisher<rosgraph_msgs::Log>* log_publisher_2 = (ZyROSConnectorTopicPublisher<rosgraph_msgs::Log>*)(log_source_2.get());

                test_fixture.m_rosConnector->getROSConnector()->addTopicPublisher(log_source_2);
                test_fixture.m_rosConnector->getROSConnector()->addTopicListener(log_sink_2);

                // Test publisher and subscriber wrappers using rosgraph_msgs::Log
                for (unsigned int k = 0; k <= 10; ++k)
                {
                    std::stringstream msg_stream;
                    rosgraph_msgs::Log msg;
                    msg_stream << "Log message " << k;

                    msg.name = "ZyROSConnector_Test_FactoryMethods";
                    msg.level = rosgraph_msgs::Log::INFO;
                    msg.file = __FILE__;
                    msg.line = __LINE__;
                    msg.msg = msg_stream.str();

                    msg_info("ZyROSConnector_test") << "Publish message " << k;
                    usleep(50000);
                    log_publisher_2->publishMessage(msg);
                }

                msg_info("ZyROSConnector_test") << "Removing topic publisher.";
                test_fixture.m_rosConnector->getROSConnector()->removeTopicPublisher(log_source_2);
                msg_info("ZyROSConnector_test") << "Removing topic subscriber.";
                test_fixture.m_rosConnector->getROSConnector()->removeTopicListener(log_sink_2);

                // Test service client: Get and set logging parameters of the running roscore
                if (getLoggersServiceExists)
                {
                    msg_info("ZyROSConnector_test") << "Getting list of active logger instances in running roscore.";
                    boost::shared_ptr<ZyROSServiceClient> getLoggersClient = ZyROSConnectorServiceClientFactory::createServiceClient(test_fixture.m_rosConnector->getRosNodeHandle(), "/rosout/get_loggers", "roscpp::GetLoggers");
                    if (getLoggersClient)
                    {
                        ZyROSConnectorServiceClient<roscpp::GetLoggers, roscpp::GetLoggersRequest, roscpp::GetLoggersResponse>* loggers_service_client = (ZyROSConnectorServiceClient<roscpp::GetLoggers, roscpp::GetLoggersRequest, roscpp::GetLoggersResponse>*)(getLoggersClient.get());
                        loggers_service_client->setupClient();
                        test_fixture.m_rosConnector->getROSConnector()->addServiceClient(getLoggersClient);

                        roscpp::GetLoggersRequest loggers_request;
                        loggers_service_client->enqueueRequest(loggers_request);

                        usleep(50000);

                        msg_info("ZyROSConnector_test") << "Replies received by service client: " << loggers_service_client->getNumResponses();

                        if (loggers_service_client->getNumResponses() > 0)
                        {
                            for (size_t t = 0; t < loggers_service_client->getNumResponses(); t++)
                            {
                                const roscpp::GetLoggersResponse& loggers_list = loggers_service_client->getResponse(t);
                                msg_info("ZyROSConnector_test") << "Loggers known in roscore: " << loggers_list.loggers.size();
                                for (size_t m = 0; m < loggers_list.loggers.size(); ++m)
                                    msg_info("ZyROSConnector_test") << "Logger: " << loggers_list.loggers[m] << "\n";
                            }
                        }

                        loggers_service_client->shutdownClient();

                        msg_info("ZyROSConnector_test") << "Removing service client from ZyROSConnector";
                        test_fixture.m_rosConnector->getROSConnector()->removeServiceClient(getLoggersClient);
                    }
                }

                if (setLoggerLevelServiceExists)
                {
                    msg_info("ZyROSConnector_test") << "Setting logger level in running roscore.";
                    boost::shared_ptr<ZyROSServiceClient> setLogLevelClient = ZyROSConnectorServiceClientFactory::createServiceClient(test_fixture.m_rosConnector->getRosNodeHandle(), "/rosout/set_logger_level", "roscpp::SetLoggerLevel");
                    if (setLogLevelClient)
                    {
                        ZyROSConnectorServiceClient<roscpp::SetLoggerLevel, roscpp::SetLoggerLevelRequest, roscpp::SetLoggerLevelResponse>* log_level_client = (ZyROSConnectorServiceClient<roscpp::SetLoggerLevel, roscpp::SetLoggerLevelRequest, roscpp::SetLoggerLevelResponse>*)(setLogLevelClient.get());
                        log_level_client->setupClient();
                        test_fixture.m_rosConnector->getROSConnector()->addServiceClient(setLogLevelClient);

                        roscpp::SetLoggerLevelRequest log_level_request;
                        log_level_request.level = ros::console::levels::Debug;
                        log_level_request.logger = "ros.roscpp";

                        log_level_client->enqueueRequest(log_level_request);
                        usleep(50000);

                        log_level_client->shutdownClient();

                        msg_info("ZyROSConnector_test") << "Removing service client from ZyROSConnector";
                        test_fixture.m_rosConnector->getROSConnector()->removeServiceClient(setLogLevelClient);
                    }

                }

                // Test ROS service server, part 1
                boost::shared_ptr< ZyROSConnectorServiceServerImpl<zyrosconnector_test::ArrayOfFloatsRequest, zyrosconnector_test::ArrayOfFloatsResponse, ArrayOfFloatsRequestHandler<zyrosconnector_test::ArrayOfFloatsRequest, zyrosconnector_test::ArrayOfFloatsResponse>> > array_of_floats_server;
                array_of_floats_server.reset(new ArrayOfFloatsServiceServer<zyrosconnector_test::ArrayOfFloatsRequest, zyrosconnector_test::ArrayOfFloatsResponse, ArrayOfFloatsRequestHandler<zyrosconnector_test::ArrayOfFloatsRequest, zyrosconnector_test::ArrayOfFloatsResponse> >("/ZyROSConnector/array_of_floats"));
                ZyROSConnectorServiceServerWorkerThread<zyrosconnector_test::ArrayOfFloatsRequest, zyrosconnector_test::ArrayOfFloatsResponse, ArrayOfFloatsRequestHandler<zyrosconnector_test::ArrayOfFloatsRequest, zyrosconnector_test::ArrayOfFloatsResponse> >* service_worker_aof = new ZyROSConnectorServiceServerWorkerThread<zyrosconnector_test::ArrayOfFloatsRequest, zyrosconnector_test::ArrayOfFloatsResponse, ArrayOfFloatsRequestHandler<zyrosconnector_test::ArrayOfFloatsRequest, zyrosconnector_test::ArrayOfFloatsResponse> >(array_of_floats_server);

                if (service_worker_aof->start())
                {
                    msg_info("ZyROSConnector_test") << "Started service worker thread for ArrayOfFloats ServiceServer.";
                    usleep(100000);
                    boost::shared_ptr<ZyROSServiceClient> array_of_floats_client;
                    array_of_floats_client.reset(new ZyROSConnectorServiceClient<zyrosconnector_test::ArrayOfFloats, zyrosconnector_test::ArrayOfFloatsRequest, zyrosconnector_test::ArrayOfFloatsResponse>(test_fixture.m_rosConnector->getRosNodeHandle(), "/ZyROSConnector/array_of_floats"));

                    ZyROSConnectorServiceClient<zyrosconnector_test::ArrayOfFloats, zyrosconnector_test::ArrayOfFloatsRequest, zyrosconnector_test::ArrayOfFloatsResponse>* array_of_floats_templated_client = (ZyROSConnectorServiceClient<zyrosconnector_test::ArrayOfFloats, zyrosconnector_test::ArrayOfFloatsRequest, zyrosconnector_test::ArrayOfFloatsResponse>*) (array_of_floats_client.get());
                    if (array_of_floats_templated_client)
                    {
                        array_of_floats_templated_client->setupClient();
                        test_fixture.m_rosConnector->getROSConnector()->addServiceClient(array_of_floats_client);

                        msg_info("ZyROSConnector_test") << "Sending requests to ArrayOfFloats ServiceServer.";
                        for (unsigned int k = 0; k < 10; k++)
                        {
                            zyrosconnector_test::ArrayOfFloatsRequest req;
                            req.FloatsToReturn.data = (k + 1) * 2;
                            array_of_floats_templated_client->enqueueRequest(req);

                            // Wait a little between requests
                            usleep(25000);
                        }

                        for (size_t l = 0; l < array_of_floats_templated_client->getNumResponses(); l++)
                        {
                            const zyrosconnector_test::ArrayOfFloatsResponse& resp = array_of_floats_templated_client->getResponse(l);
                            msg_info("ZyROSConnector_test") << "Response " << l << " from array_of_floats_server:\n" << resp.return_values.data.size() << " values.";

                            array_of_floats_templated_client->removeRequest(l);
                        }

                        array_of_floats_templated_client->shutdownClient();
                        test_fixture.m_rosConnector->getROSConnector()->removeServiceClient(array_of_floats_client);

                        msg_info("ZyROSConnector_test") << "Stopping ROS service server";
                        array_of_floats_server->shutdownServer();

                        msg_info("ZyROSConnector_test") << "Shutting down service worker thread";
                        service_worker_aof->stop();
                    }
                }

                // Shutdown the ROS connector and the roscore instance
                msg_info("ZyROSConnector_test") << "Disconnecting from ROS master...";
                test_fixture.disconnectFromROSMaster();
                msg_info("ZyROSConnector_test") << "Disconnected from ROS master.";

                msg_info("ZyROSConnector_test") << "Shutting down roscore instance.";
				int closingAttempts = 0;
				bool rosClosed = false;
				do
                {
#if _WIN32
					LRESULT wmCloseRs = SendMessage(rSEI.hwnd, WM_CLOSE, 0, 0);

					PostMessage(rSEI.hwnd, WM_CLOSE, 0, 0);

					Sleep(2000);

					if (test_fixture.ProcessRunning(L"roscore.exe"))
					{
                        msg_info("ZyROSConnector_test") << "roscore.exe process still running.";
					}
					else
					{
                        msg_info("ZyROSConnector_test") << "roscore.exe process finished.";
						rosClosed = true;
						break;
					}
#else
                    int kill_ret = kill(rosCorePID, SIGTERM);

                    if (kill_ret == 0)
                    {
                        pid_t rosCorePid_tmp;
                        if (test_fixture.ProcessRunning("roscore", &rosCorePid_tmp))
                        {
                            msg_info("ZyROSConnector_test") << "roscore process still running.";
                        }
                        else
                        {
                            msg_info("ZyROSConnector_test") << "roscore process finished.";
                            rosClosed = true;
                            break;
                        }
                    }
#endif
					closingAttempts++;
				} while (closingAttempts < 3);

				if (!rosClosed)
				{
#ifdef _WIN32
					test_fixture.CloseProcesses(L"python.exe");
					test_fixture.CloseProcesses(L"roscore.exe");
					test_fixture.CloseProcesses(L"rosmaster.exe");
					test_fixture.CloseProcesses(L"rosout.exe");
#else
                    test_fixture.CloseProcesses("rosmaster");
                    test_fixture.CloseProcesses("rosout");
                    test_fixture.CloseProcesses("roscore");
#endif
				}
			}
		}
	}
	else
	{
        msg_error("ZyROSConnector_test") << "Given roscore executable does not exist: " << rosCoreExecutablePath;
		return 1;
	}
	return 0;
}

