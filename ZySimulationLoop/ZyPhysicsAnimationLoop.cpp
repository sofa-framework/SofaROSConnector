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
#include "TruPhysicsAnimationLoop.h"
#include <sofa/core/visual/VisualParams.h>

#include <SofaConstraint/LCPConstraintSolver.h>

#include <sofa/core/ObjectFactory.h>
#include <sofa/core/VecId.h>

#include <sofa/helper/AdvancedTimer.h>

#include <sofa/simulation/BehaviorUpdatePositionVisitor.h>
#include <sofa/simulation/MechanicalOperations.h>
#include <sofa/simulation/SolveVisitor.h>
#include <sofa/simulation/VectorOperations.h>

#include <sofa/simulation/InitVisitor.h>

#include <SofaBaseMechanics/MechanicalObject.h>

#include <sofa/simulation/Node.h>

#ifdef TRUPHYSICS_USE_ROS_SUPPORT
#include <std_msgs/Bool.h>
#endif

namespace sofa
{

namespace component
{

namespace animationloop
{

using namespace core::behavior;
using namespace sofa::component::container;

SOFA_DECL_CLASS(TruPhysicsAnimationLoop)

int TruPhysicsAnimationLoopClass = core::RegisterObject("truPhysics animationloop")
        .add< TruPhysicsAnimationLoop >()
        .addAlias("TruPhysicsFreeMotionMasterSolver")
        ;


TruPhysicsAnimationLoop::TruPhysicsAnimationLoop(simulation::Node* gnode)
    : FreeMotionAnimationLoop(gnode)
    //, intersectionReset(initData(&intersectionReset, false, "intersectionReset", "TruPhysics: set to true to reset a simulation step in which a contact with intersecting triangles occurs."))
    , toolHandler(NULL)
    //, objectHandler(NULL)
#ifdef TRUPHYSICS_USE_ROS_SUPPORT
    , publishingHandler()
    , connectionManagerFound(false)
    , timePublisher(NULL)
    , rigidPosRotPublisher(NULL)
    , simAnalyzer(NULL)
#endif
    , velocityApproximatorFound(false)
    , velocityApproximationHandler()
    , grippingFound(false)
    , grippingHandler()
    //, intersectionOccurred(false)
    //, tmpResetPos()
    //, tmpResetVel()
    //, tmpResetFreePos()
    //, tmpResetFreeVel()
    //, resetVecsInitialized(false)
    //, resetPointSet(false)
    //, oneResetPointSet(false)
    //, resetOccurred(false)
    //, resetCount(0)
{
}

void TruPhysicsAnimationLoop::init()
{
    Inherit::init();

#ifndef SOFA_FLOAT
    toolHandler = getContext()->getRootContext()->get<sofa::component::controller::TruPhysicsColladaToolHandler<Rigid3dTypes> >();
#else
    toolHandler = getContext()->getRootContext()->get<sofa::component::controller::TruPhysicsColladaToolHandler<Rigid3fTypes>>();
#endif

    simAnalyzer = getContext()->getRootContext()->get< TruPhysics::SimulationAnalysis::TruSimulationAnalyzer >();

    //objectHandler = getContext()->getRootContext()->get< sofa::component::controller::TruPhysicsObjectManager >();

    if (toolHandler)
    {
        std::cout << "(TruPhysicsAnimationLoop::init) Found TruPhysicsColladaToolHandler " << toolHandler->getName() << std::endl;
    }

    if (simAnalyzer)
    {
        std::cout << "(TruPhysicsAnimationLoop::init) Found TruSimulationAnalyzer " << simAnalyzer->getName() << std::endl;
    }

    /*if (objectHandler)
    {
        std::cout << "(TruPhysicsAnimationLoop::init) Found TruPhysicsObjectManager " << objectHandler->getName() << std::endl;
    }*/

}

void TruPhysicsAnimationLoop::bwdInit()
{
    Inherit::bwdInit();

#ifdef TRUPHYSICS_USE_ROS_SUPPORT
    connectionManagerFound = publishingHandler.setROSConnectionManagerByContext(getContext());
    
    if (connectionManagerFound)
    {
        timePublisher = new TruPhysics::ROSConnector::TruRosConnectorTopicPublisher<std_msgs::Float32>(publishingHandler.getROSNodeHandle(), "sofaSimTime");
        publishingHandler.registerPublisher(timePublisher);

        rigidPosRotPublisher = new TruPhysics::ROSConnector::TruRosConnectorTopicPublisher<std_msgs::Float32MultiArray>(publishingHandler.getROSNodeHandle(), "sofaRigidPositionRotation");
        publishingHandler.registerPublisher(rigidPosRotPublisher);

        /*
        // Test code for dealing with all subscribers to a specific topic (not sure if this is needed)
        std::cout << "(TruPhysicsAnimationLoop::bwdInit) Looking for std_msgs/Bool listeners:" << std::endl;
        boolSubscribers = publishingHandler.getSubscribers<std_msgs::Bool>();

        for (std::vector< boost::shared_ptr< TruPhysics::ROSConnector::TruRosConnectorTopicSubscriber<std_msgs::Bool> > >::iterator 
            it = boolSubscribers.begin();
            it != boolSubscribers.end();
            it++)
        {
            std::cout << "(TruPhysicsAnimationLoop::bwdInit) Found ROS listener of type " << (*it)->getMessageType() << ", for topic " << (*it)->getTopic() << std::endl;
            connectionVec.push_back
            (
                (*it)->getSignal().connect(boost::bind(&TruPhysicsAnimationLoop::handleBool, this))
            );
        }
        */

        std::cout << "(TruPhysicsAnimationLoop::bwdInit) Looking for std_msgs/Bool listener:" << std::endl;
        testBoolSubscriber = publishingHandler.getSubscriber<std_msgs::Bool>("/testBool");
        if (testBoolSubscriber)
        {
            std::cout << "(TruPhysicsAnimationLoop::bwdInit) Found ROS listener of type " << testBoolSubscriber->getMessageType() << ", for topic " << testBoolSubscriber->getTopic() << std::endl;
            connectionVec.push_back
            (
                testBoolSubscriber->getSignal().connect(boost::bind(&TruPhysicsAnimationLoop::handleBool, this))
            );
        }
        std::cout << "(TruPhysicsAnimationLoop::bwdInit) Stopped looking for std_msgs/Bool listener" << std::endl;

    }
#endif

    velocityApproximatorFound = velocityApproximationHandler.setVelocityApproximatorByContext(getContext());
    grippingFound = grippingHandler.setTruGrippingByContext(getContext());
    //simAnalyzerFound = simAnalyzerHandler.setSimAnalyzerByContext(getContext());
}

void TruPhysicsAnimationLoop::reset()
{
    UpdateObbTreeGpuPositions();
}

void TruPhysicsAnimationLoop::UpdateObbTreeGpuPositions()
{
	simulation::Node* root = dynamic_cast<simulation::Node*>(getContext());
	if (root == NULL) return;

	std::vector<MechanicalObject<RigidTypes>*> mechanicalObjects;
	root->getTreeObjects<MechanicalObject<RigidTypes> >(&mechanicalObjects);

#ifdef TRUPHYSICSANIMATIONLOOP_STEP_DEBUG
    std::cout << "TruPhysicsAnimationLoop::step(): MechObj Positions at end of step" << std::endl;
#endif
    for (std::vector<MechanicalObject<RigidTypes>*>::const_iterator it = mechanicalObjects.begin(); it != mechanicalObjects.end(); it++)
    {
        defaulttype::Vec3d objPosition(((*it)->getPosition())[0][0], ((*it)->getPosition())[0][1], ((*it)->getPosition())[0][2]);
        defaulttype::Quaternion objOrientation(((*it)->getPosition())[0][3], ((*it)->getPosition())[0][4], ((*it)->getPosition())[0][5], ((*it)->getPosition())[0][6]);

#ifdef TRUPHYSICSANIMATIONLOOP_STEP_DEBUG
        std::cout << " * " << (*it)->getName() << ": " << objPosition << ", orientation = " << objOrientation << std::endl;
#endif
        BaseContext* mechObjContext = (*it)->getContext();

        std::vector<sofa::core::CollisionModel* > collisionModels;
		sofa::core::objectmodel::BaseContext::GetObjectsCallBackT<sofa::core::CollisionModel, std::vector<sofa::core::CollisionModel* > > cb(&collisionModels);

		mechObjContext->getObjects(sofa::core::objectmodel::TClassInfo<sofa::core::CollisionModel>::get(), cb, sofa::core::objectmodel::TagSet(), sofa::core::objectmodel::BaseContext::SearchDown);
        if (collisionModels.size() > 0)
        {
			for (std::vector<sofa::core::CollisionModel*>::iterator cit = collisionModels.begin(); cit != collisionModels.end(); cit++)
            {
                //std::cout << "objPosition: " << objPosition << " objOrientation: " << objOrientation << std::endl;
                (*cit)->setCachedPosition(objPosition);
                (*cit)->setCachedOrientation(objOrientation);
               // (*cit)->updateInternalGeometry(); // moved into pipeline, behind broadphase
            }
        }
    }
}

TruPhysicsAnimationLoop::~TruPhysicsAnimationLoop()
{
    if (defaultSolver != NULL)
        defaultSolver.reset();

    for (std::vector < boost::signals2::connection >::iterator it = connectionVec.begin(); it != connectionVec.end(); it++)
    {
        if ((*it).connected())
        {
            (*it).disconnect();
        }
    }

    /*if (timePublisher)
    {
        delete timePublisher;
    }

    if (rigidPosRotPublisher)
    {
        delete rigidPosRotPublisher;
    }*/
}

//bool TruPhysicsAnimationLoop::getIntersectionOccurred() const
//{
//    return intersectionOccurred;
//}
//
//void TruPhysicsAnimationLoop::setIntersectionOccurred(bool value)
//{
//    intersectionOccurred = value;
//}

void TruPhysicsAnimationLoop::initTmpResetVecs(simulation::common::VectorOperations& vop, const sofa::core::ExecParams* params)
{
    if (!resetVecsInitialized) {
        // allocate and initialize only once
        resetVecsInitialized=true;

        vop.v_alloc(tmpResetPos.id());
        vop.v_alloc(tmpResetVel.id());
        vop.v_alloc(tmpResetFreePos.id());
        vop.v_alloc(tmpResetFreeVel.id());

        simulation::MechanicalVInitVisitor< core::V_COORD >(params, tmpResetPos.id(), core::ConstVecCoordId::position(), true).execute(this->gnode);
        simulation::MechanicalVInitVisitor< core::V_DERIV >(params, tmpResetVel.id(), core::ConstVecDerivId::velocity(), true).execute(this->gnode);
        simulation::MechanicalVInitVisitor< core::V_COORD >(params, tmpResetFreePos.id(), core::ConstVecCoordId::freePosition(), true).execute(this->gnode);
        simulation::MechanicalVInitVisitor< core::V_DERIV >(params, tmpResetFreeVel.id(), core::ConstVecDerivId::freeVelocity(), true).execute(this->gnode);

        vop.v_alloc(tmpResetPosNew.id());
        vop.v_alloc(tmpResetVelNew.id());
        vop.v_alloc(tmpResetFreePosNew.id());
        vop.v_alloc(tmpResetFreeVelNew.id());

        simulation::MechanicalVInitVisitor< core::V_COORD >(params, tmpResetPosNew.id(), core::ConstVecCoordId::position(), true).execute(this->gnode);
        simulation::MechanicalVInitVisitor< core::V_DERIV >(params, tmpResetVelNew.id(), core::ConstVecDerivId::velocity(), true).execute(this->gnode);
        simulation::MechanicalVInitVisitor< core::V_COORD >(params, tmpResetFreePosNew.id(), core::ConstVecCoordId::freePosition(), true).execute(this->gnode);
        simulation::MechanicalVInitVisitor< core::V_DERIV >(params, tmpResetFreeVelNew.id(), core::ConstVecDerivId::freeVelocity(), true).execute(this->gnode);
    }
}

void TruPhysicsAnimationLoop::setTmpResetPoint(simulation::common::VectorOperations& vop, sofa::core::behavior::MultiVecCoord& pos, sofa::core::behavior::MultiVecDeriv& vel, sofa::core::behavior::MultiVecCoord& freePos, sofa::core::behavior::MultiVecDeriv& freeVel)
{
    if (resetVecsInitialized)
    {
        /*
        if (oneResetPointSet)
        {
            std::cout << "Setting two points" << std::endl;
            vop.v_eq(tmpResetPos.id(),tmpResetPosNew.id());
            vop.v_eq(tmpResetVel.id(),tmpResetVel.id());
            vop.v_eq(tmpResetFreePos.id(),tmpResetFreePos.id());
            vop.v_eq(tmpResetFreeVel.id(),tmpResetFreeVel.id());

            vop.v_eq(tmpResetPosNew.id(),pos.id());
            vop.v_eq(tmpResetVelNew.id(),vel.id());
            vop.v_eq(tmpResetFreePosNew.id(),freePos.id());
            vop.v_eq(tmpResetFreeVelNew.id(),freeVel.id());

            resetPointSet = true;
        }
        else
        {
            vop.v_eq(tmpResetPosNew.id(),pos.id());
            vop.v_eq(tmpResetVelNew.id(),vel.id());
            vop.v_eq(tmpResetFreePosNew.id(),freePos.id());
            vop.v_eq(tmpResetFreeVelNew.id(),freeVel.id());

            oneResetPointSet = true;
        }
        */

        vop.v_eq(tmpResetPos.id(),pos.id());
        vop.v_eq(tmpResetVel.id(),vel.id());
        vop.v_eq(tmpResetFreePos.id(),freePos.id());
        vop.v_eq(tmpResetFreeVel.id(),freeVel.id());
        resetPointSet = true;

        std::cout << "set reset point" << std::endl;
        std::cout << "pos: " << pos << std::endl;
        std::cout << "tmpResetPosNew: " << tmpResetPosNew << std::endl;
        std::cout << "tmpResetPos: " << tmpResetPos << std::endl;

    }
}

void TruPhysicsAnimationLoop::resetToTmpResetPoint(simulation::common::VectorOperations& vop, simulation::common::MechanicalOperations& mop, sofa::core::behavior::MultiVecCoord& pos, sofa::core::behavior::MultiVecDeriv& vel, sofa::core::behavior::MultiVecCoord& freePos, sofa::core::behavior::MultiVecDeriv& freeVel)
{
    if (resetPointSet)
    {
        //std::cout << "Reset to earlier position. WARNING resets to two steps before last intersection." << std::endl;
        std::cout << "Reset to earlier position." << std::endl;

        std::cout << "writing : " << tmpResetPos ;
        std::cout << "into: " << pos << std::endl;

        vop.v_eq(pos.id(),tmpResetPos.id());
        vop.v_eq(vel.id(),tmpResetVel.id());
        vop.v_eq(freePos.id(),tmpResetFreePos.id());
        vop.v_eq(freeVel.id(),tmpResetFreeVel.id());

        //mop.propagateX(pos,true);

        std::cout << "now pos is: " << pos << std::endl;

        mop.propagateXAndV(pos,vel,true);
        mop.propagateXAndV(freePos,freeVel,true);
    }
}

//void TruPhysicsAnimationLoop::step(const sofa::core::ExecParams* params /* PARAMS FIRST */, double dt)
//{
//
//    // RealTime
//    clock_t timeStart = clock();
//
//	if (dt == 0)
//		dt = this->gnode->getDt();
//
//	simulation::Node* root = dynamic_cast<simulation::Node*>(getContext());
//	if (root == NULL) return;
//
//	std::vector<MechanicalObject<RigidTypes>*> mechanicalObjects;
//	root->getTreeObjects<MechanicalObject<RigidTypes> >(&mechanicalObjects);
//
//#ifdef TRUPHYSICSANIMATIONLOOP_STEP_DEBUG
//	std::cout << "TruPhysicsAnimationLoop::step(): MechObj Positions at beginning of step" << std::endl;
//	for (std::vector<MechanicalObject<RigidTypes>*>::const_iterator it = mechanicalObjects.begin(); it != mechanicalObjects.end(); it++)
//	{
//		defaulttype::Vec3d objPosition(((*it)->getPosition())[0][0], ((*it)->getPosition())[0][1], ((*it)->getPosition())[0][2]);
//		defaulttype::Quaternion objOrientation(((*it)->getPosition())[0][3], ((*it)->getPosition())[0][4], ((*it)->getPosition())[0][5], ((*it)->getPosition())[0][6]);
//
//		std::cout << " * " << (*it)->getName() << ": " << objPosition << ", orientation = " << objOrientation << std::endl;
//	}
//#endif
//
//	sofa::helper::AdvancedTimer::begin("Animate");
//
//	sofa::helper::AdvancedTimer::stepBegin("AnimationStep");
//#ifdef SOFA_DUMP_VISITOR_INFO
//	simulation::Visitor::printNode("Step");
//#endif
//
//	{
//		sofa::helper::AdvancedTimer::stepBegin("AnimateBeginEvent");
//		AnimateBeginEvent ev ( dt );
//		PropagateEventVisitor act ( params, &ev );
//		this->gnode->execute ( act );
//		sofa::helper::AdvancedTimer::stepEnd("AnimateBeginEvent");
//	}
//
//	double startTime = this->gnode->getTime();
//
//	simulation::common::VectorOperations vop(params, this->getContext());
//	simulation::common::MechanicalOperations mop(params, this->getContext());
//
//	MultiVecCoord pos(&vop, core::VecCoordId::position());
//	MultiVecDeriv vel(&vop, core::VecDerivId::velocity());
//	MultiVecCoord freePos(&vop, core::VecCoordId::freePosition());
//	MultiVecDeriv freeVel(&vop, core::VecDerivId::freeVelocity());
//
//    // copied from freemotion etc loop
//    {
//        MultiVecDeriv dx(&vop, core::VecDerivId::dx()); dx.realloc(&vop, true, true);
//        MultiVecDeriv df(&vop, core::VecDerivId::dforce()); df.realloc(&vop, true, true);
//    }
//    // copied end
//    /*
//    if (intersectionReset.getValue()) {
//        // TP: Ops need to be set in each iteration, or else the resetVectors don't know their ExecParams, which can lead to problems
//        tmpResetPos.setOps(&vop);
//        tmpResetVel.setOps(&vop);
//        tmpResetFreePos.setOps(&vop);
//        tmpResetFreeVel.setOps(&vop);
//        tmpResetPosNew.setOps(&vop);
//        tmpResetVelNew.setOps(&vop);
//        tmpResetFreePosNew.setOps(&vop);
//        tmpResetFreeVelNew.setOps(&vop);
//
//        if (!resetVecsInitialized)
//        {
//            resetCount = 0;
//        }
//        initTmpResetVecs(vop,params); // there is probably some better way to do this, but this works for now
//        //resetTimer = 0;
//    }
//    */
//
//    //setTmpResetPoint(vop,pos,vel,freePos,freeVel); // simulation reset test
//
//	// This solver will work in freePosition and freeVelocity vectors.
//	// We need to initialize them if it's not already done.
//	sofa::helper::AdvancedTimer::stepBegin("MechanicalVInitVisitor");
//	simulation::MechanicalVInitVisitor< core::V_COORD >(params, core::VecCoordId::freePosition(), core::ConstVecCoordId::position(), true).execute(this->gnode);
//	simulation::MechanicalVInitVisitor< core::V_DERIV >(params, core::VecDerivId::freeVelocity(), core::ConstVecDerivId::velocity(), true).execute(this->gnode);
//
//	sofa::helper::AdvancedTimer::stepEnd("MechanicalVInitVisitor");
//
//	BehaviorUpdatePositionVisitor beh(params, dt);
//
//	using helper::system::thread::CTime;
//	using sofa::helper::AdvancedTimer;
//
//	double time = 0.0;
//	//double timeTotal = 0.0;
//	double timeScale = 1000.0 / (double)CTime::getTicksPerSec();
//
//	if (displayTime.getValue())
//	{
//		time = (double)CTime::getTime();
//		//timeTotal = (double) CTime::getTime();
//	}
//
//	// Update the BehaviorModels
//	// Required to allow the RayPickInteractor interaction
//	if (f_printLog.getValue())
//		serr << "updatePos called" << sendl;
//
//	AdvancedTimer::stepBegin("UpdatePosition");
//	this->gnode->execute(&beh);
//	AdvancedTimer::stepEnd("UpdatePosition");
//
//	if (f_printLog.getValue())
//		serr << "updatePos performed - beginVisitor called" << sendl;
//
//	simulation::MechanicalBeginIntegrationVisitor beginVisitor(params, dt);
//	this->gnode->execute(&beginVisitor);
//
//	if (f_printLog.getValue())
//		serr << "beginVisitor performed - SolveVisitor for freeMotion is called" << sendl;
//
//	// Free Motion
//	AdvancedTimer::stepBegin("FreeMotion");
//	simulation::SolveVisitor freeMotion(params, dt, true);
//	this->gnode->execute(&freeMotion);
//	AdvancedTimer::stepEnd("FreeMotion");
//
//	mop.propagateXAndV(freePos, freeVel, true); // apply projective constraints
//
//	if (f_printLog.getValue())
//		serr << " SolveVisitor for freeMotion performed" << sendl;
//
//	if (displayTime.getValue())
//	{
//		sout << " >>>>> Begin display TruPhysicsAnimationLoop time" << sendl;
//		sout << " Free Motion " << ((double)CTime::getTime() - time) * timeScale << " ms" << sendl;
//
//		time = (double)CTime::getTime();
//	}
//
//	if (f_printLog.getValue())
//        sout << "TruPhysicsAnimationLoop: computeCollision START" << sendl;
//
//	// Collision detection and response creation
//	AdvancedTimer::stepBegin("Collision");
//	computeCollision(params);
//    AdvancedTimer::stepEnd("Collision");
//
//    if (f_printLog.getValue())
//        sout << "TruPhysicsAnimationLoop: computeCollision END" << sendl;
//
//    // for simulation reset
//
//    /* //working test code
//    if (resetCount == 2)
//    {
//        setTmpResetPoint(vop,pos,vel,freePos,freeVel);
//    }
//    if (resetCount == 6)
//    {
//        resetToTmpResetPoint(vop,mop,pos,vel,freePos,freeVel);
//        resetCount = 3;
//    }
//    resetCount++;
//    if (!intersectionOccurred && intersectionReset.getValue())
//    {
//        setTmpResetPoint(vop,pos,vel,freePos,freeVel);
//    }
//    if (intersectionOccurred && intersectionReset.getValue())
//    {
//        resetToTmpResetPoint(vop,mop,pos,vel,freePos,freeVel);
//    }
//    */
//
//
//    /*
//    std::cout << "intersectionOccurred is currently set to " << intersectionOccurred << "." << std::endl;
//    if (intersectionOccurred && intersectionReset.getValue())
//    {
//        std::cout << "Reset to previous point." << std::endl;
//        std::cout << "pos: " << pos << std::endl;
//        resetToTmpResetPoint(vop,mop,pos,vel,freePos,freeVel);
//        std::cout << "pos: " << pos << std::endl;
//        intersectionOccurred = false;
//
//        resetCount = resetCount + 30;
//        dt = dt*0.1;
//        getContext()->getRootContext()->setDt(dt); // setting only dt, without changing it in the root context, seems to only change the displayed time, not the actual time step used in the simulation.
//        resetOccurred = true;
//    }
//    else
//    {
//        setTmpResetPoint(vop,pos,vel,freePos,freeVel);
//        if (resetOccurred)
//        {
//            if (resetCount == 0)
//            {
//                dt = dt * 10;
//                getContext()->getRootContext()->setDt(dt); // setting only dt, without changing it in the root context, seems to only change the displayed time, not the actual time step used in the simulation.
//                resetOccurred = false;
//            }
//            else
//            {
//                resetCount--;
//            }
//        }
//    }
//    */
//    // for simulation reset end
//
//	mop.propagateX(pos, false); // Why is this done at that point ???
//
//	if (displayTime.getValue())
//	{
//		sout << " computeCollision " << ((double)CTime::getTime() - time) * timeScale << " ms" << sendl;
//		time = (double)CTime::getTime();
//	}
//
//    //if (!intersectionOccurred) // if the simulation is reset, the constraint solver isn't needed.
//    {
//	// Solve constraints
//	if (constraintSolver)
//	{
//		AdvancedTimer::stepBegin("ConstraintSolver");
//
//		if (m_solveVelocityConstraintFirst.getValue())
//		{
//			core::ConstraintParams cparams(*params);
//			cparams.setX(freePos);
//			cparams.setV(freeVel);
//
//			cparams.setOrder(core::ConstraintParams::VEL);
//			constraintSolver->solveConstraint(&cparams, vel);
//
//			MultiVecDeriv dv(&vop, constraintSolver->getDx());
//			mop.projectResponse(dv);
//			mop.propagateDx(dv);
//
//			// xfree += dv * dt
//			freePos.eq(freePos, dv, dt);
//			mop.propagateX(freePos, false); // ignore projective constraints
//
//			cparams.setOrder(core::ConstraintParams::POS);
//			constraintSolver->solveConstraint(&cparams, pos);
//
//			MultiVecDeriv dx(&vop, constraintSolver->getDx());
//
//			mop.propagateV(vel, true); // apply projective constraints
//			mop.projectResponse(dx);
//			mop.propagateDx(dx, true);
//
//			// "mapped" x = xfree + dx
//			simulation::MechanicalVOpVisitor(params, pos, freePos, dx, 1.0).setOnlyMapped(true).execute(this->gnode);
//		}
//		else
//		{
//			core::ConstraintParams cparams(*params);
//			cparams.setX(freePos);
//			cparams.setV(freeVel);
//
//			constraintSolver->solveConstraint(&cparams, pos, vel);
//			mop.propagateV(vel, true); // apply projective constraints
//
//			MultiVecDeriv dx(&vop, constraintSolver->getDx());
//			mop.projectResponse(dx);
//			mop.propagateDx(dx, true);
//
//			// "mapped" x = xfree + dx
//			simulation::MechanicalVOpVisitor(params, pos, freePos, dx, 1.0).setOnlyMapped(true).execute(this->gnode);
//		}
//		AdvancedTimer::stepEnd("ConstraintSolver");
//
//	}
//    }
//
//	if (displayTime.getValue())
//	{
//		sout << " contactCorrections " << ((double)CTime::getTime() - time) * timeScale << " ms" << sendl;
//		sout << "<<<<<< End display TruPhysicsAnimationLoop time." << sendl;
//	}
//
//	simulation::MechanicalEndIntegrationVisitor endVisitor(params /* PARAMS FIRST */, dt);
//	this->gnode->execute(&endVisitor);
//
//	this->gnode->setTime(startTime + dt);
//	this->gnode->execute<UpdateSimulationContextVisitor>(params);  // propagate time
//
//    /*
//    std::string forceOutputFilename = "B:\\tstfile.txt";
//    std::ofstream forceOutput;
//    if (forceOutputFilename != "")
//    {
//#ifndef _WIN32
//        forceOutput.open(forceOutputFilename.c_str(), std::ofstream::out | std::ofstream::app);
//#else
//        forceOutput.open(forceOutputFilename, std::ofstream::out | std::ofstream::app);
//#endif
//
//        std::vector<MechanicalObject<Vec3dTypes>*> mecjects;
//        root->getTreeObjects<MechanicalObject<Vec3dTypes> >(&mecjects);
//
//        for (std::vector<MechanicalObject<Vec3dTypes>*>::const_iterator it = mecjects.begin(); it != mecjects.end(); it++)
//        {
//            const sofa::helper::vector<sofa::defaulttype::Vec<3,double>> &forceVector = (*it)->getForce();
//
//            bool totallyZero = true;
//            for (unsigned int vecI=0; (vecI < forceVector.size()) && totallyZero; vecI++)
//            {
//                totallyZero = (totallyZero && (forceVector[vecI][0] == 0));
//                totallyZero = (totallyZero && (forceVector[vecI][1] == 0));
//                totallyZero = (totallyZero && (forceVector[vecI][2] == 0));
//            }
//
//            if (!totallyZero)
//            {
//                std::stringstream tmpstream;
//
//                if (getContext()->getRootContext()->getTime() <= 0.004)
//                {
//                    tmpstream << std::right << std::fixed << std::setprecision(8) << (*it)->getName() << " " << 0.0 << " " ;
//                    for (unsigned int dimensionIndex = 0; dimensionIndex < forceVector.size(); dimensionIndex++)
//                    {
//                        {
//                            std::stringstream blubb;
//                            blubb.width(21);
//                            std::stringstream gnaaaah;
//                            gnaaaah << "index_" << dimensionIndex << "_x ";
//                            blubb << std::right << gnaaaah.str();
//                            tmpstream << blubb.str();
//                        }
//                        {
//                            std::stringstream blubb;
//                            blubb.width(21);
//                            std::stringstream gnaaaah;
//                            gnaaaah << "index_" << dimensionIndex << "_y ";
//                            blubb << std::right << gnaaaah.str();
//                            tmpstream << blubb.str();
//                        }
//                        {
//                            std::stringstream blubb;
//                            blubb.width(21);
//                            std::stringstream gnaaaah;
//                            gnaaaah << "index_" << dimensionIndex << "_z ";
//                            blubb << std::right << gnaaaah.str();
//                            tmpstream << blubb.str();
//                        }
//                    }
//                    tmpstream << std::endl;
//                }
//
//                tmpstream << std::right << std::fixed << std::setprecision(8) << (*it)->getName() << " " << getContext()->getRootContext()->getTime() << " " ;
//
//                for (unsigned int vertexIndex = 0; vertexIndex < forceVector.size(); vertexIndex++)
//                {
//                    for (unsigned int dimensionIndex = 0; dimensionIndex < forceVector.at(vertexIndex).size(); dimensionIndex++)
//                    {
//                        std::stringstream blubb;
//                        blubb.width(20);
//                        blubb << std::right << std::fixed << std::setprecision(12) << forceVector.at(vertexIndex).at(dimensionIndex) << " ";
//                        tmpstream << blubb.str();
//                    }
//                }
//
//                forceOutput << tmpstream.str() << std::endl;
//
//                //forceOutput << std::internal << std::fixed << std::setprecision(12) << (*it)->getName() << " " << getContext()->getRootContext()->getTime() << " " << forceVector << std::endl;
//            }
//
//        }
//
//        forceOutput.close();
//    }
//    */
//
//    // for simulation reset
//
//    //std::cout << "intersectionOccurred is currently set to " << intersectionOccurred << "." << std::endl;
///*    if (intersectionReset.getValue() && intersectionOccurred)
//    {
//        //std::cout << "Reset to previous point." << std::endl;
//        //std::cout << "pos: " << pos << std::endl;
//        resetToTmpResetPoint(vop,mop,pos,vel,freePos,freeVel);
//        //std::cout << "pos: " << pos << std::endl;
//        intersectionOccurred = false;
//*/
//        /*
//        resetCount = resetCount + 100;
//        dt = dt*0.1;
//        getContext()->getRootContext()->setDt(dt); // setting only dt, without changing it in the root context, seems to only change the displayed time, not the actual time step used in the simulation.
//        resetOccurred = true;
//        */
//    /*
//    }
//    else
//    {
//        setTmpResetPoint(vop,pos,vel,freePos,freeVel);*/
//        /*if (resetOccurred)
//        {
//            if (resetCount == 0)
//            {
//                dt = dt * 10;
//                getContext()->getRootContext()->setDt(dt); // setting only dt, without changing it in the root context, seems to only change the displayed time, not the actual time step used in the simulation.
//                resetOccurred = false;
//            }
//            else
//            {
//                resetCount--;
//            }
//        }
//        */
//    //}
//
//    // for simulation reset end
//
//	{
//		AnimateEndEvent ev(dt);
//		PropagateEventVisitor act(params, &ev);
//		this->gnode->execute(act);
//	}
//
//
//	sofa::helper::AdvancedTimer::stepBegin("UpdateMapping");
//	//Visual Information update: Ray Pick add a MechanicalMapping used as VisualMapping
//	this->gnode->execute<UpdateMappingVisitor>(params);
//	//	sofa::helper::AdvancedTimer::step("UpdateMappingEndEvent");
//	{
//		UpdateMappingEndEvent ev(dt);
//		PropagateEventVisitor act(params, &ev);
//		this->gnode->execute(act);
//	}
//	sofa::helper::AdvancedTimer::stepEnd("UpdateMapping");
//
//
//#ifndef SOFA_NO_UPDATE_BBOX
//	sofa::helper::AdvancedTimer::stepBegin("UpdateBBox");
//	this->gnode->execute<UpdateBoundingBoxVisitor>(params);
//	sofa::helper::AdvancedTimer::stepEnd("UpdateBBox");
//#endif
//
//#ifdef SOFA_DUMP_VISITOR_INFO
//	simulation::Visitor::printCloseNode("Step");
//#endif
//
//#ifdef TRUPHYSICSANIMATIONLOOP_DEBUG
//    std::cout << "===== Timing data =====" << std::endl;
//	sofa::helper::AdvancedTimer::dumpAll();
//    std::cout << "===== Timing data =====" << std::endl;
//#endif
//	sofa::helper::AdvancedTimer::stepEnd("AnimationStep");
//	sofa::helper::AdvancedTimer::end("Animate");
//
//    UpdateObbTreeGpuPositions();
//
//
//    // RealTime
//    if (this->gnode->getRt()) {
//        clock_t timePassed = clock() - timeStart;
//        double wait = dt - ((double)timePassed)/CLOCKS_PER_SEC;
//        std::cout << "RealTime Active. Waiting: " << wait << std::endl;
//        if (wait > 0.0) {
//#ifdef _WIN32
//            Sleep(wait * 1000);
//#else
//            usleep(wait * 1000);
//#endif
//        } else {
//            std::cout << "Computer not fast enough for realtime step." << std::endl;
//        }
//    }
//
//}

void TruPhysicsAnimationLoop::tpHandleTool(simulation::common::MechanicalOperations& mop, MultiVecCoord& pos/*, MultiVecCoord& freePos*/)
{
    if (toolHandler)
    {
        toolHandler->handleTool(mop, pos/*,freePos*/);
    }
}

//void TruPhysicsAnimationLoop::tpHandleObject(simulation::common::MechanicalOperations& mop, MultiVecCoord& pos/*, MultiVecCoord& freePos*/)
//{
//    if (objectHandler)
//    {
//        objectHandler->moveObjectToPosition(mop, pos/*,freePos*/);
//    }
//}

void TruPhysicsAnimationLoop::tpRealTimeStart()
{
    // RealTime
    clock_t timeStart = clock();
    // asd
    //sofa::component::controller::TruPhysicsObjectManager<Rigid3dTypes>* banane = getObjectHandler();

    //if (banane)
    //{
    //    if ((getContext()->getTime() > 0.2) && (getContext()->getTime() < 0.21))
    //    {
    //        Rigid3dTypes::VecCoord bla;
    //        bla.resize(1);
    //        Rigid3dTypes::set(bla.at(0), 10.0, 0.0, 0.0);

    //        banane->setObjectPosition(bla);
    //    }
    //    if ((getContext()->getTime() > 0.35) && (getContext()->getTime() < 0.36))
    //    {
    //        Rigid3dTypes::VecCoord bla;
    //        bla.resize(1);
    //        Rigid3dTypes::set(bla.at(0), 10.0, 10.0, 0.0);

    //        banane->setObjectPosition(bla);
    //    }
    //    if ((getContext()->getTime() > 0.5) && (getContext()->getTime() < 0.51))
    //    {
    //        Rigid3dTypes::VecCoord bla;
    //        bla.resize(1);
    //        Rigid3dTypes::set(bla.at(0), 10.0, 10.0, 10.0);

    //        banane->setObjectPosition(bla);
    //    }
    //}
    // qwe
}

void TruPhysicsAnimationLoop::tpRealTimeEnd(double dt)
{
    // RealTime
    if (this->gnode->getRt()) {
        clock_t timePassed = clock() - timeStart;
        double wait = dt - ((double)timePassed) / CLOCKS_PER_SEC;
        std::cout << "RealTime Active. Waiting: " << wait << std::endl;
        if (wait > 0.0) {
#ifdef _WIN32
			DWORD wait_duration = (unsigned long) wait * 1000;
			Sleep(wait_duration);
#else
            usleep(wait * 1000);
#endif
        }
        else {
            std::cout << "Computer not fast enough for realtime step." << std::endl;
        }
    }
}

#ifdef TRUPHYSICS_USE_ROS_SUPPORT
void TruPhysicsAnimationLoop::tpHandleROSMessagesStepBegin()
{
}
#endif

#ifdef TRUPHYSICS_USE_ROS_SUPPORT
void TruPhysicsAnimationLoop::tpHandleROSMessagesStepEnd()
{
    if (connectionManagerFound)
    {
        // publish simulation time
        std_msgs::Float32 msgFloat32;
        msgFloat32.data = getContext()->getTime();
        if (timePublisher)
        {
            timePublisher->publishMessage(msgFloat32);
        }

        // publish rigid body positions and rotations
        simulation::Node* root = dynamic_cast<simulation::Node*>(getContext()->getRootContext());
        if (root == NULL) return;

        std::vector<MechanicalObject<RigidTypes>*> mechanicalObjects;
        root->getTreeObjects<MechanicalObject<RigidTypes> >(&mechanicalObjects);

        std_msgs::Float32MultiArray msgFloat32MultiArray;
        std::vector<float> dataVec;
        std::vector<std_msgs::MultiArrayDimension> dim;

        unsigned int numberOfValuesPerRigid = 7;

        dataVec.resize(mechanicalObjects.size() * numberOfValuesPerRigid);

        dim.resize(2);
        dim.at(0).label = "rigid bodies";
        dim.at(0).size = mechanicalObjects.size();
        dim.at(0).stride = mechanicalObjects.size() * numberOfValuesPerRigid;

        dim.at(1).label = "position, rotation (quaternion)";
        dim.at(1).size = numberOfValuesPerRigid;
        dim.at(1).stride = numberOfValuesPerRigid;

        unsigned int rigidCount = 0;
        for (std::vector<MechanicalObject<RigidTypes>*>::const_iterator it = mechanicalObjects.begin(); it != mechanicalObjects.end(); it++)
        {
            std::string rigidName = (*it)->getName();
            defaulttype::Vec3d objPosition(((*it)->getPosition())[0][0], ((*it)->getPosition())[0][1], ((*it)->getPosition())[0][2]);
            defaulttype::Quaternion objOrientation(((*it)->getPosition())[0][3], ((*it)->getPosition())[0][4], ((*it)->getPosition())[0][5], ((*it)->getPosition())[0][6]);

            dataVec.at((rigidCount* dim[1].stride) + 0) = objPosition[0];
            dataVec.at((rigidCount* dim[1].stride) + 1) = objPosition[1];
            dataVec.at((rigidCount* dim[1].stride) + 2) = objPosition[2];
            dataVec.at((rigidCount* dim[1].stride) + 3) = objOrientation[0];
            dataVec.at((rigidCount* dim[1].stride) + 4) = objOrientation[1];
            dataVec.at((rigidCount* dim[1].stride) + 5) = objOrientation[2];
            dataVec.at((rigidCount* dim[1].stride) + 6) = objOrientation[3];
            rigidCount++;
        }
    
        msgFloat32MultiArray.data = dataVec;
        msgFloat32MultiArray.layout.data_offset = 0;
        msgFloat32MultiArray.layout.dim = dim;

        rigidPosRotPublisher->publishMessage(msgFloat32MultiArray);
    }
    return;
}
#endif

void TruPhysicsAnimationLoop::tpApprox(simulation::common::MechanicalOperations& mop, sofa::core::behavior::MultiVecCoord& pos)
{
    if (velocityApproximatorFound)
    {
        velocityApproximationHandler.approximateVelocity(mop, pos);
    }
}

void TruPhysicsAnimationLoop::tpGrip(simulation::common::MechanicalOperations& mop, sofa::core::behavior::MultiVecCoord& pos)
{
    if (grippingFound)
    {
        grippingHandler.handleGrippers(mop, pos);
    }
}

void TruPhysicsAnimationLoop::tpSimAnalyze()
{
    if (simAnalyzer)
    {
        simAnalyzer->analyzeSimulationStateAndPublishResult();
    }
}

void TruPhysicsAnimationLoop::handleBool()
{
    std::cout << "TruPhysicsAnimationLoop saw a Bool! It was: " << testBoolSubscriber->getLatestMessage() << std::endl;
}

} // namespace animationloop

} // namespace component

} // namespace sofa
