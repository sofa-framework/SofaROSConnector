#ifndef MultiThreadingTasks_inl__
#define MultiThreadingTasks_inl__

namespace Zyklio
{
	namespace MultiThreading
    {
        inline bool TaskStatus::IsBusy() const
		{
			return mBusy.operator long() != 0L;
		}

        inline void TaskStatus::MarkBusy(bool bBusy)
		{
			if (bBusy)
			{
				++mBusy;
			}
			else
			{
				--mBusy;
			}
		}
	} // namespace MultiThreading
} // namespace Zyklio


#endif
