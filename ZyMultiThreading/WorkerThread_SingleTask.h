#ifndef TRU_MULTITHREADING_WORKERTHREAD_SINGLE_TASK_H
#define TRU_MULTITHREADING_WORKERTHREAD_SINGLE_TASK_H

#include "WorkerThreadIface.h"

#include <string>
#include <sofa/core/objectmodel/BaseObject.h>

namespace Zyklio
{
	namespace MultiThreading
	{
		/**
		Thread class for running a single asynchronous task
		Used for long-running background loops.
		Runs as long it is not paused or stopped.
		*/
		class TRU_MULTITHREADING_API WorkerThread_SingleTask : public WorkerThreadIface
		{
			public:
				WorkerThread_SingleTask(const std::string& name = std::string(),
					priority_type p = priority_type(),
					size_t stack_size = DEFAULT_STACK_VALUE);

				~WorkerThread_SingleTask();

			protected:
				virtual void main();
				/*virtual bool action();*/
		};
	}
}

#endif // TRU_MULTITHREADING_WORKERTHREAD_SINGLE_TASK_H