#ifndef TRU_SOFA_VELOCITYAPPROXIMATIONHANDLER_H
#define TRU_SOFA_VELOCITYAPPROXIMATIONHANDLER_H

#include "init_ZyVelocityApproximationHandler.h"

#include <sofa/core/objectmodel/BaseContext.h>
#include <sofa/simulation/MechanicalOperations.h>
#include <sofa/simulation/VectorOperations.h>

namespace Zyklio
{
    namespace VelocityApproximation
    {
        class ZyROSVelocityApproximatorPrivate;

        /**
         * \brief This is a helper class to help with a ZyVelocityApproximator.
         */
        class SOFA_ZY_VELOCITY_APPROXIMATION_HANDLER_API ZyROSVelocityApproximationHandler
		{
            public:
                ZyROSVelocityApproximationHandler();
                ~ZyROSVelocityApproximationHandler();

                /**
                * \brief 
                * 
                * (CAREFUL! When this is called, the ZyVelocityApproximator in the scene graph must already
                * be instantiated, so a constructor or an init() method are bad places to call setVelocityApproximatorByContext)
                * 
                * \param cntxt 
                */
                bool setVelocityApproximatorByContext(sofa::core::objectmodel::BaseContext* cntxt);
                void approximateVelocity(sofa::simulation::common::MechanicalOperations& mop, sofa::core::behavior::MultiVecCoord& pos);
                void pushJointMsg(/*jointMsg*/ std::pair<std::pair<unsigned int, double>, sofa::helper::vector< std::pair <double, std::string> > > theMsg);
                
            private:
                ZyROSVelocityApproximatorPrivate* velocityApproximatorPrivate;
		};
	}
}

#endif // TRU_SOFA_VELOCITYAPPROXIMATIONHANDLER_H
