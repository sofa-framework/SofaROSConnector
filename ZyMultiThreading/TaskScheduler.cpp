#include "TaskScheduler.h"

#include <sofa/helper/system/atomic.h>

using namespace sofa;
using namespace Zyklio::MultiThreading;

unsigned TaskScheduler::GetHardwareThreadsCount()
{
    return boost::thread::hardware_concurrency();
}

unsigned TaskScheduler::size() const volatile
{
    return mWorkerCount;
}
