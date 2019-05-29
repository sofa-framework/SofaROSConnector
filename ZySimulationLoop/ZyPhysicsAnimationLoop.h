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
#ifndef SOFA_COMPONENT_ANIMATIONLOOP_ZyPhysicsAnimationLoop_H
#define SOFA_COMPONENT_ANIMATIONLOOP_ZyPhysicsAnimationLoop_H

#ifdef _WIN32
// Ensure that Winsock2.h is included before Windows.h, which can get
// pulled in by anybody (e.g., Boost).
#include <Winsock2.h>
#endif

#include "initZySimulationLoop.h".

#include <SofaConstraint/FreeMotionAnimationLoop.h>
#include <sofa/simulation/MechanicalOperations.h>
#include <sofa/simulation/VectorOperations.h>

#include "ZyColladaToolHandler.h"

#ifdef ZY_USE_ROS_SUPPORT
#include <ZyklioROSConnector.h>
//#include <TruRosTopicPublisher_Float32.h>
//#include <TruRosTopicPublisher_Float32MultiArray.h>
#include <ZyRosPublishingHandler.h>
#include <std_msgs/Bool.h>
#endif

#include <ZyGrippingHandler.h>
#include <ZyVelocityApproximationHandler.h>
#include <ZyROSSimulationAnalyzer.h>

namespace sofa
{

namespace component
{

namespace animationloop
{

class SOFA_ZYSOFA_SIMULATION_LOOP_API ZyPhysicsAnimationLoop : public sofa::component::animationloop::FreeMotionAnimationLoop
{
public:
    typedef sofa::component::animationloop::FreeMotionAnimationLoop Inherit;

    SOFA_CLASS(ZyPhysicsAnimationLoop, sofa::component::animationloop::FreeMotionAnimationLoop);

    void reset();
    void init();
    void bwdInit();

protected:
    ZyPhysicsAnimationLoop(simulation::Node* gnode);
    void UpdateObbTreeGpuPositions();
    virtual ~ZyPhysicsAnimationLoop();

private:
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
    void zyRealTimeStart();
    void zyRealTimeEnd(double dt);
    void zyHandleTool(simulation::common::MechanicalOperations&, sofa::core::behavior::MultiVecCoord&/*, MultiVecCoord&*/);
    //void tpHandleObject(simulation::common::MechanicalOperations&, sofa::core::behavior::MultiVecCoord&/*, MultiVecCoord&*/);

    void zyApprox(simulation::common::MechanicalOperations& mop, sofa::core::behavior::MultiVecCoord& pos);
    void zyGrip(simulation::common::MechanicalOperations& mop, sofa::core::behavior::MultiVecCoord& pos);

    void zySimAnalyze();

    bool connectionManagerFound, velocityApproximatorFound, grippingFound;
    Zyklio::SimulationAnalysis::ZyROSSimulationAnalyzer* simAnalyzer;
    Zyklio::VelocityApproximation::ZyROSVelocityApproximationHandler velocityApproximationHandler;
    Zyklio::GripperHandling::ZyGrippingHandler grippingHandler;

#ifdef ZY_USE_ROS_SUPPORT
    // supposed to be used to publish ROS messages
    void tpHandleROSMessagesStepBegin(); // called at the begin of the animation step
    void tpHandleROSMessagesStepEnd(); // called at the end of the animation step

    Zyklio::ROSPublishing::ZyRosPublishingHandler publishingHandler;

    Zyklio::ROSConnector::ZyRosConnectorTopicPublisher<std_msgs::Float32>* timePublisher;
    Zyklio::ROSConnector::ZyRosConnectorTopicPublisher<std_msgs::Float32MultiArray>* rigidPosRotPublisher;
#endif

    sofa::component::controller::ZyColladaToolHandler<sofa::defaulttype::Rigid3Types>* toolHandler;
};

} // namespace animationloop

} // namespace component

} // namespace sofa

#endif /* SOFA_COMPONENT_ANIMATIONLOOP_FREEMOTIONANIMATIONLOOP_H */
