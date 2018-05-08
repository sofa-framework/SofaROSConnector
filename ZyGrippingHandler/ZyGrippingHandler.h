#ifndef TRU_SOFA_GRIPPING_H
#define TRU_SOFA_GRIPPING_H

#include "init_ZyGrippingHandler.h"

#include <sofa/core/objectmodel/BaseContext.h>
#include <sofa/simulation/MechanicalOperations.h>
#include <sofa/simulation/VectorOperations.h>

namespace Zyklio
{
    namespace GripperHandling
    {
        class ZyGrippingPrivate;

        /**
         * \brief This is a helper class to help with ZyGripping.
         */
        class TRU_SOFA_GRIPPINGHANDLER_API ZyGrippingHandler
		{
            public:
                ZyGrippingHandler();
                ~ZyGrippingHandler();

                /**
                * \brief 
                * 
                * (CAREFUL! When this is called, the ZyGripping in the scene graph must already
                * be instantiated, so a constructor or an init() method are bad places to call setVelocityApproximatorByContext)
                * 
                * \param cntxt 
                */
                bool setZyGrippingByContext(sofa::core::objectmodel::BaseContext* cntxt);
                void handleGrippers(sofa::simulation::common::MechanicalOperations& mop, sofa::core::behavior::MultiVecCoord& pos);
                
            private:
                ZyGrippingPrivate* zyGrippingPrivate;
		};
	}
}

#endif // TRU_SOFA_GRIPPING_H
