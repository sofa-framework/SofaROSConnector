#ifdef _WIN32
// Ensure that Winsock2.h is included before Windows.h, which can get
// pulled in by anybody (e.g., Boost).
#ifndef WINSOCK2_H
#define WINSOCK2_H
#include <Winsock2.h>
#endif
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

			bool ProcessRunning(const std::wstring&);
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

	bool RosConnectorTest::ProcessRunning(const std::wstring& processName)
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
		w_str << "/C " << rosRoot_env << L"\\setup.bat & " << rosRoot_env << L"\\bin\\roscore.exe";
		rosCoreExecCommand = w_str.str();
		w_str.str("");
		w_str << rosRoot_env << "\\bin\\roscore.exe";
		rosCoreExecutable = w_str.str();
	}
	else
	{
		rosCoreExecutable = "C:\\opt\\ros\\hydro\\x86\\bin\\roscore.exe";
		rosCoreExecCommand = "C:\\opt\\ros\\hydro\\x86\\setup.bat & C:\\opt\\ros\\hydro\\x86\\bin\\roscore.exe";
	}
		
	std::string rosMasterURI("http://localhost:11311");
	boost::filesystem::path rosCoreExecutablePath(rosCoreExecutable);
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
#endif
			
        Zyklio::RosConnectorTest test_fixture;
#if _WIN32
        if (ShellExecuteEx(&rSEI))
#endif
		{
			int waitAttempts = 0;
			// Wait for rosout.exe, since it's the last child process started by roscore
			while (!test_fixture.ProcessRunning(L"rosout.exe") && waitAttempts <= 20)
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
                        msg_info("ZyROSConnector_test") << "roscore.exe process finished, exiting.";
						rosClosed = true;
						break;
					}
#else

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

#endif
				}
			}
		}
	}
	else
	{
		return 1;
	}
	return 0;
}
