#ifndef TRU_SOFA_SUBSCRIPTIONHELPER_TMP_H
#define TRU_SOFA_SUBSCRIPTIONHELPER_TMP_H

#ifdef _WIN32
// Ensure that Winsock2.h is included before Windows.h, which can get
// pulled in by anybody (e.g., Boost).
#include <Winsock2.h>
#endif

#include "init_ZyROSSubscriptionHelper.h"

#include <sofa/core/objectmodel/BaseContext.h>
#include <ros/forwards.h>

namespace Zyklio
{
    namespace ROSConnector
    {
        class ZyROSSimulationAnalyzerPrivate;

        // TODO replace this with something better (by cleaning up the Zyklio plugin)
        /**
         * \brief This is a helper class that enables a ZyROSPhysicsSubscriber class to access a class in the Zyklio plugin.
         * This stops being necessary once the Zyklio plugin has been split into separate projects, like the newer plugins.
         */
        class SOFA_ZY_ROS_SUBSCRIPTIONHELPER_API ZyROSSubscriptionHelper
		{
            public:
                ZyROSSubscriptionHelper();
                ~ZyROSSubscriptionHelper();

                /**
                * \brief 
                * 
                * (CAREFUL! When this is called, the ZyROSSimulationAnalyzer and the ZyROSConnectionManager in the scene graph must already
                * be instantiated, so a constructor or an init() method are bad places to call setROSConnectionManagerByContext)
                * 
                * \param cntxt 
                */
                bool setROSConnectionManagerByContext(sofa::core::objectmodel::BaseContext* cntxt);

                void setLinValueInAnalyzer(float lin);
                void setRotValueInAnalyzer(float rot);

                ros::NodeHandlePtr getROSNodeHandle();

            private:
                ZyROSSimulationAnalyzerPrivate* simulationAnalyzerPrivate;
		};
	}
}

#endif // TRU_SOFA_SUBSCRIPTIONHELPER_TMP_H
