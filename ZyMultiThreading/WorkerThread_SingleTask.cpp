#include "WorkerThread_SingleTask.h"

using namespace Zyklio::MultiThreading;

WorkerThread_SingleTask::WorkerThread_SingleTask(const std::string& name, priority_type p, size_t stack_size) : WorkerThreadIface(name, p, stack_size) 
{

}

WorkerThread_SingleTask::~WorkerThread_SingleTask()
{

}

void WorkerThread_SingleTask::main() 
{

}

/*bool WorkerThread_SingleTask::action() 
{ 
	return false; 
}*/