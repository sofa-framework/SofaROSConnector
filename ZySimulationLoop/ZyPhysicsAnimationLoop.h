/******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, version 1.0 RC 1        *
*                (c) 2006-2011 MGH, INRIA, USTL, UJF, CNRS                    *
*                                                                             *
* This library is free software; you can redistribute it and/or modify it     *
* under the terms of the GNU Lesser General Public License as published by    *
* the Free Software Foundation; either version 2.1 of the License, or (at     *
* your option) any later version.                                             *
*                                                                             *
* This library is distributed in the hope that it will be useful, but WITHOUT *
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or       *
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License *
* for more details.                                                           *
*                                                                             *
* You should have received a copy of the GNU Lesser General Public License    *
* along with this library; if not, write to the Free Software Foundation,     *
* Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA.          *
*******************************************************************************
*                               SOFA :: Modules                               *
*                                                                             *
* Authors: The SOFA Team and external contributors (see Authors.txt)          *
*                                                                             *
* Contact information: contact@sofa-framework.org                             *
******************************************************************************/
#ifndef SOFA_COMPONENT_ANIMATIONLOOP_TRUPHYSICSANIMATIONLOOP_H
#define SOFA_COMPONENT_ANIMATIONLOOP_TRUPHYSICSANIMATIONLOOP_H

#ifdef _WIN32
// Ensure that Winsock2.h is included before Windows.h, which can get
// pulled in by anybody (e.g., Boost).
#include <Winsock2.h>
#endif

#include "initTruPhysicsPlugin.h"

#include <SofaConstraint/FreeMotionAnimationLoop.h>
#include <sofa/simulation/MechanicalOperations.h>
#include <sofa/simulation/VectorOperations.h>

#include "TruPhysicsColladaToolHandler.h"
//#include "TruPhysicsObjectManager.h"

#ifdef TRUPHYSICS_USE_ROS_SUPPORT
#include <TruPhysicsROSConnector.h>
//#include <TruRosTopicPublisher_Float32.h>
//#include <TruRosTopicPublisher_Float32MultiArray.h>
#include <TruRosPublishingHandler.h>
#include <std_msgs/Bool.h>
#endif

#include <TruGrippingHandler.h>
#include <TruVelocityApproximationHandler.h>
#include "TruPhysicsSimulationAnalyzer.h"


// ArrayFire test 1 of 2 begin
//#include <arrayfire.h>
// ArrayFire test 1 of 2 end
namespace sofa
{

namespace component
{

namespace animationloop
{

class SOFA_TRUPHYSICS_API TruPhysicsAnimationLoop : public sofa::component::animationloop::FreeMotionAnimationLoop
{
public:
    typedef sofa::component::animationloop::FreeMotionAnimationLoop Inherit;

    SOFA_CLASS(TruPhysicsAnimationLoop, sofa::component::animationloop::FreeMotionAnimationLoop);

    void reset();
    void init();
    void bwdInit();

protected:
    TruPhysicsAnimationLoop(simulation::Node* gnode);
    void UpdateObbTreeGpuPositions();
    virtual ~TruPhysicsAnimationLoop();

public:
    //virtual void step (const sofa::core::ExecParams* params /* PARAMS FIRST */, double dt);

    //----

    // TODO: Delete this, if it's not needed for anything anymore
    /*bool getIntersectionOccurred() const;
    void setIntersectionOccurred(bool value);*/

    //sofa::component::controller::TruPhysicsObjectManager* getObjectHandler() { return objectHandler; };

private:
    //Data <bool> intersectionReset;
    //bool intersectionOccurred;

	sofa::core::behavior::MultiVecCoord tmpResetPos, tmpResetPosNew;
	sofa::core::behavior::MultiVecDeriv tmpResetVel, tmpResetVelNew;
	sofa::core::behavior::MultiVecCoord tmpResetFreePos, tmpResetFreePosNew;
	sofa::core::behavior::MultiVecDeriv tmpResetFreeVel, tmpResetFreeVelNew;
    bool resetVecsInitialized;
    bool resetPointSet;
    bool oneResetPointSet;
    bool resetOccurred;
    int resetCount;

	void initTmpResetVecs(sofa::simulation::common::VectorOperations& vop, const sofa::core::ExecParams* params);
	void setTmpResetPoint(sofa::simulation::common::VectorOperations& vop, sofa::core::behavior::MultiVecCoord& pos, sofa::core::behavior::MultiVecDeriv& vel, sofa::core::behavior::MultiVecCoord& freePos, sofa::core::behavior::MultiVecDeriv& freeVel);
	void resetToTmpResetPoint(sofa::simulation::common::VectorOperations& vop, simulation::common::MechanicalOperations& mop, sofa::core::behavior::MultiVecCoord& pos, sofa::core::behavior::MultiVecDeriv& vel, sofa::core::behavior::MultiVecCoord& freePos, sofa::core::behavior::MultiVecDeriv& freeVel);
    
// virtual functions called in FreeMotionAnimationLoop:
private:
    clock_t timeStart;
    void tpRealTimeStart();
    void tpRealTimeEnd(double dt);
    void tpHandleTool(simulation::common::MechanicalOperations&, sofa::core::behavior::MultiVecCoord&/*, MultiVecCoord&*/);
    //void tpHandleObject(simulation::common::MechanicalOperations&, sofa::core::behavior::MultiVecCoord&/*, MultiVecCoord&*/);

    void tpApprox(simulation::common::MechanicalOperations& mop, sofa::core::behavior::MultiVecCoord& pos);
    void tpGrip(simulation::common::MechanicalOperations& mop, sofa::core::behavior::MultiVecCoord& pos);

    void tpSimAnalyze();

#ifdef TRUPHYSICS_USE_ROS_SUPPORT
    // supposed to be used to publish ROS messages
    void tpHandleROSMessagesStepBegin(); // called at the begin of the animation step
    void tpHandleROSMessagesStepEnd(); // called at the end of the animation step

    TruPhysics::ROSPublishing::TruRosPublishingHandler publishingHandler;
    TruPhysics::VelocityApproximation::TruVelocityApproximationHandler velocityApproximationHandler;
    TruPhysics::GripperHandling::TruGrippingHandler grippingHandler;

    //TruPhysics::ROSConnector::TruRosFloat32Publisher* timePublisher;
    //TruPhysics::ROSConnector::TruRosFloat32MultiArrayPublisher* rigidPosRotPublisher;
    TruPhysics::ROSConnector::TruRosConnectorTopicPublisher<std_msgs::Float32>* timePublisher;
    TruPhysics::ROSConnector::TruRosConnectorTopicPublisher<std_msgs::Float32MultiArray>* rigidPosRotPublisher;

    bool connectionManagerFound, velocityApproximatorFound, grippingFound;

    TruPhysics::SimulationAnalysis::TruSimulationAnalyzer* simAnalyzer;

    //std::vector< boost::shared_ptr< TruPhysics::ROSConnector::TruRosConnectorTopicSubscriber<std_msgs::Bool> > > boolSubscribers; // if for some reason all Bool subscribers are needed
    boost::shared_ptr< TruPhysics::ROSConnector::TruRosConnectorTopicSubscriber<std_msgs::Bool> > testBoolSubscriber;
    std::vector < boost::signals2::connection > connectionVec;
    void handleBool();
#endif


#ifndef SOFA_FLOAT
    sofa::component::controller::TruPhysicsColladaToolHandler<sofa::defaulttype::Rigid3dTypes>* toolHandler;
#else
	sofa::component::controller::TruPhysicsColladaToolHandler<sofa::defaulttype::Rigid3fTypes>* toolHandler;
#endif
    //sofa::component::controller::TruPhysicsObjectManager* objectHandler;

    // ArrayFire test 1 of 2 begin
    //af::array bananaAr;
    // ArrayFire test 1 of 2 end
};

} // namespace animationloop

} // namespace component

} // namespace sofa

#endif /* SOFA_COMPONENT_ANIMATIONLOOP_FREEMOTIONANIMATIONLOOP_H */
