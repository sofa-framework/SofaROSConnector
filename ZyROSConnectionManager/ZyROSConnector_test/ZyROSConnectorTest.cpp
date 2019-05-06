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
#include <ZyROSConnectorTopicSubscriber.h>

#include <rosgraph_msgs/Log.h>

#include <boost/filesystem.hpp>
#include <boost/thread/mutex.hpp>

namespace Zyklio
{
    using namespace Zyklio::ROSConnector;

	struct RosConnectorTest : public ::testing::Test
	{
		public:
			RosConnectorTest();
			~RosConnectorTest();

			bool connectToROSMaster(const std::string& masterUri);
			bool disconnectFromROSMaster();

            bool ProcessRunning(const std::wstring&
#ifndef _WIN32
                                , pid_t* processPid
#endif
                                );
			int CloseProcesses(const std::wstring& processName);

			void TestBody();

            ZyROSConnector* m_rosConnector;

			boost::mutex m_mutex;
	};

	RosConnectorTest::RosConnectorTest()
	{
        m_rosConnector = new ZyROSConnector();
	}

	RosConnectorTest::~RosConnectorTest()
	{
		if (m_rosConnector)
		{
			delete m_rosConnector;
			m_rosConnector = NULL;
		}
	}

    bool RosConnectorTest::ProcessRunning(const std::wstring& processName
#ifndef _WIN32
                                          , pid_t* processPid
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

                std::wstring_convert<std::codecvt_utf8_utf16<wchar_t>> converter;
                std::wstring currentProcessName = converter.from_bytes(pname);
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

	int RosConnectorTest::CloseProcesses(const std::wstring& processName)
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
                std::wstring_convert<std::codecvt_utf8_utf16<wchar_t>> converter;
                std::wstring currentProcessName = converter.from_bytes(pname);
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
        msg_info("ZyROSConnector_Test") << "connectToROSMaster: " << masterUri;
		m_rosConnector->setRosMasterURI(masterUri);
		m_rosConnector->startComponent();
        msg_info("ZyROSConnector_Test") << "connectToROSMaster: startComponent";
		m_rosConnector->resumeComponent();
        msg_info("ZyROSConnector_Test") << "connectToROSMaster: resumeComponent";
		
		boost::mutex::scoped_lock lock(m_mutex);
		while (!m_rosConnector->isThreadRunning())
		{
            msg_info("ZyROSConnector_Test") << "Waiting for connector thread to start...";
			m_rosConnector->connectorCondition().wait(lock);
		}
        msg_info("ZyROSConnector_Test") << "Connector thread started.";
		if (m_rosConnector->isConnected())
		{
            msg_info("ZyROSConnector_Test") << "Connection to roscore established.";
			return true;
		}

        msg_info("ZyROSConnector_Test") << "Failed to connect to roscore.";
		return false;
	}

	bool RosConnectorTest::disconnectFromROSMaster()
	{
        msg_info("ZyROSConnector_Test") << "connectToROSMaster: pauseComponent";
		m_rosConnector->pauseComponent();
        msg_info("ZyROSConnector_Test") << "connectToROSMaster: stopComponent";
		m_rosConnector->stopComponent();
		
		return true;
	}

	void RosConnectorTest::TestBody()
	{

	}
}

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
        if (!test_fixture.ProcessRunning(L"roscore", &rosCorePID))
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
            while (!test_fixture.ProcessRunning(L"rosout", &processPid) && waitAttempts <= 20)
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

			if (test_fixture.connectToROSMaster(rosMasterURI))
			{
				ros::NodeHandle local_nh;
                ros::Publisher log_pub = local_nh.advertise<rosgraph_msgs::Log>("ZyROSConnector_test", 1000);

                boost::shared_ptr<ZyROSListener> logListener;
                logListener.reset(new ZyROSConnectorTopicSubscriber<rosgraph_msgs::Log>(test_fixture.m_rosConnector->getROSNode(), "/ZyROSConnector_test"));
				test_fixture.m_rosConnector->addTopicListener(logListener);

				for (unsigned int k = 0; k < 10; ++k)
				{
					std::stringstream msg_stream;
					rosgraph_msgs::Log msg;
					msg_stream << "Log message " << k;

                    msg.name = "ZyROSConnector_Test";
					msg.level = rosgraph_msgs::Log::INFO;
					msg.file = __FILE__;
					msg.line = __LINE__;
					msg.msg = msg_stream.str();

                    msg_info("ZyROSConnector_test") << "Publish message " << k << ": " << msg;

					log_pub.publish(msg);
				}

				test_fixture.m_rosConnector->removeTopicListener(logListener);
				test_fixture.disconnectFromROSMaster();

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
                        if (test_fixture.ProcessRunning(L"roscore", &rosCorePid_tmp))
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
                    test_fixture.CloseProcesses(L"roscore");
                    test_fixture.CloseProcesses(L"rosmaster");
                    test_fixture.CloseProcesses(L"rosout");
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
