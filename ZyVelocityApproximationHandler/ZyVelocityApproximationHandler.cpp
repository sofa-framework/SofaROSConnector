#include "ZyVelocityApproximationHandler.h"

#include <ZyVelocityApproximator.h>

using namespace Zyklio::VelocityApproximation;

class Zyklio::VelocityApproximation::ZyROSVelocityApproximatorPrivate
{
    public:
        ZyVelocityApproximator* velocityApproximator;
};

ZyROSVelocityApproximationHandler::ZyROSVelocityApproximationHandler()
    : velocityApproximatorPrivate(NULL)
{
    velocityApproximatorPrivate = new ZyROSVelocityApproximatorPrivate();
}

ZyROSVelocityApproximationHandler::~ZyROSVelocityApproximationHandler()
{
    delete velocityApproximatorPrivate;
}

bool ZyROSVelocityApproximationHandler::setVelocityApproximatorByContext(sofa::core::objectmodel::BaseContext* cntxt)
{
    velocityApproximatorPrivate->velocityApproximator = cntxt->getRootContext()->get< ZyVelocityApproximator>();

    if (velocityApproximatorPrivate->velocityApproximator)
    {
        std::cout << "(ZyROSVelocityApproximationHandler::setROSConnectionManagerByContext) Found the ZyVelocityApproximator " << velocityApproximatorPrivate->velocityApproximator->getName() << std::endl;
        return true;
    }
    else
    {
        std::cout << "(ZyROSVelocityApproximationHandler::setROSConnectionManagerByContext) WARNING: Could not find a ZyVelocityApproximator, cannot approximate velocity." << std::endl;
        return false;
    }    
}

void ZyROSVelocityApproximationHandler::approximateVelocity(sofa::simulation::common::MechanicalOperations& mop, sofa::core::behavior::MultiVecCoord& pos)
{
    velocityApproximatorPrivate->velocityApproximator->approximateVelocity(mop, pos);
}

void ZyROSVelocityApproximationHandler::pushJointMsg(std::pair<std::pair<unsigned, double>, sofa::helper::vector<std::pair<double, std::string>>> theMsg)
{
    velocityApproximatorPrivate->velocityApproximator->pushJointMsg(theMsg);
}
