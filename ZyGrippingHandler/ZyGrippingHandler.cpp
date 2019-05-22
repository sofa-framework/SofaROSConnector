#include "ZyGrippingHandler.h"
#include <ZyGripping.h>

using namespace Zyklio::GripperHandling;

class Zyklio::GripperHandling::ZyGrippingPrivate
{
    public:
        std::vector< ZyGripping *> zyGrippings;
};

ZyGrippingHandler::ZyGrippingHandler()
    : zyGrippingPrivate(NULL)
{
    zyGrippingPrivate = new ZyGrippingPrivate();
}

ZyGrippingHandler::~ZyGrippingHandler()
{
    delete zyGrippingPrivate;
}

bool ZyGrippingHandler::setZyGrippingByContext(sofa::core::objectmodel::BaseContext* cntxt)
{
    simulation::Node* root = dynamic_cast<simulation::Node*>(cntxt->getRootContext());
    if (root == NULL) { return false; };
    root->getTreeObjects< ZyGripping >(&zyGrippingPrivate->zyGrippings);

    if (zyGrippingPrivate->zyGrippings.size() > 0)
    {
        for (unsigned int ah = 0; ah < zyGrippingPrivate->zyGrippings.size(); ah++)
        {
            msg_info("ZyGrippingHandler") << "(ZyGrippingHandler::setROSConnectionManagerByContext) Found ZyGrippingHandler " << zyGrippingPrivate->zyGrippings.at(ah)->getName();
        }
        return true;
    }
    else
    {
        msg_warning("ZyGrippingHandler") << "(ZyGrippingHandler::setROSConnectionManagerByContext) WARNING: Could not find a ZyGrippingHandler instance, cannot handle grippers.";
        return false;
    }    
}

void ZyGrippingHandler::handleGrippers(sofa::simulation::common::MechanicalOperations& mop, sofa::core::behavior::MultiVecCoord& pos)
{
    for (unsigned int ah = 0; ah < zyGrippingPrivate->zyGrippings.size(); ah++)
    {
        zyGrippingPrivate->zyGrippings.at(ah)->moveGrippers(mop, pos);
    }
}
