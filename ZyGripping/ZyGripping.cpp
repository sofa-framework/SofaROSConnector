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
#include "ZyGripping.h"

#include <sofa/simulation/Node.h>
#include <sofa/core/objectmodel/BaseContext.h>
#include <sofa/core/objectmodel/BaseContext.h>
#include <sofa/core/ObjectFactory.h>

#ifdef ZYKLIO_DEMO
#include <sofa/simulation/common/Simulation.h>
#endif //ZYKLIO_DEMO

//#define TP_VELOCITY_APPROXIMATION_DEBUG

namespace Zyklio
{
    namespace GripperHandling
    {

        using namespace sofa;
        using namespace sofa::component;

        using namespace sofa::core::objectmodel;
        using namespace sofa;

        SOFA_DECL_CLASS(ZyGripping);

        int ZyGrippingClass = core::RegisterObject("Description Test test.")
        .add< ZyGripping >()
        ;

        ZyGripping::ZyGripping()
            : direction(initData(&direction, false, "direction", "Set to determine if the gripper moves inward or outward."))
            , gripperMoveStep(initData(&gripperMoveStep, 1.0, "gripperMoveStep", "How far the gripper should move per iteration when opening or closing.")) 
            , movementDuration(initData(&movementDuration, 10u, "movementDuration", "How many iterations the gripper should move when opening or closing.")) 
            , movementAxis(initData(&movementAxis, "movementAxis", "Movement axis for the gripper, either x,y or z.")) 
            , gripperMap(NULL)
            , currentlyClosing(false)
            , currentlyOpening(false)
            , currentlyClosed(false)
            , currentlyOpen(true)
            , iterationsMoved(0)
            , dir(0)
            , movementAxisIndex(0)
        {
        }

        void ZyGripping::init()
        {
#ifdef ZYKLIO_DEMO
            std::string sceneHash = sofa::simulation::getSimulation()->getSceneHash();

            sofa::helper::hashCheckHelper hch;
            if (!(hch.checkHashCorrectness(sceneHash)))
            {
                std::cout << "This plugin can only be used with the Zyklio demo and the correct demo scene." << std::endl;
                exit(1);
            }
#endif //ZYKLIO_DEMO
            Inherit::init();

            gripperMap = getContext()->get< mapping::RigidMapping<defaulttype::Rigid3dTypes, defaulttype::Vec3dTypes> >();

            if (!gripperMap)
            {
                msg_error("ZyGripping") << "No rigid map found in the context of the ZyGripping " << getName();
            }

            if (direction.getValue())
            {
                dir = 1;
            }
            else
            {
                dir = -1;
            }

            switch(movementAxis.getValue())
            {
                case 'x':
                    movementAxisIndex = 0;
                    break;
                case 'y':
                    movementAxisIndex = 1;
                    break;
                case 'z':
                    movementAxisIndex = 2;
                    break;
                default:
                    msg_warning("ZyGripping") << "movementAxis not correctly (either 'x', 'y' or 'z') configured for ZyGripping " << getName() << ", using x-axis as default.";
                    movementAxisIndex = 0;
                    break;
            }
        }

        void ZyGripping::bwdInit()
        {                
 
        }

        void ZyGripping::reinit()
        {

        }

        void ZyGripping::startOpening()
        {
            //if (!currentlyClosing && !currentlyOpening)
            if (currentlyClosed)
            {
                msg_info("ZyGripping") << "Opening grippers ";
                currentlyOpening = true;
                currentlyClosed = false;
                iterationsMoved = 0;
            }
        }

        void ZyGripping::startClosing()
        {
            //if (!currentlyClosing && !currentlyOpening)
            if (currentlyOpen)
            {
                msg_info("ZyGripping") << "Closing grippers ";
                currentlyClosing = true;
                currentlyOpen = false;
                iterationsMoved = 0;
            }
        }

        void ZyGripping::moveGrippers(sofa::simulation::common::MechanicalOperations& mop, sofa::core::behavior::MultiVecCoord& pos)
        {
            if (gripperMap)
            {
                if (currentlyClosing)
                {
                    msg_info("ZyGripping") << "Moving grippers (close)";
                    mapping::RigidMapping<defaulttype::Rigid3dTypes, defaulttype::Vec3dTypes>::VecCoord & mapPoints = const_cast<mapping::RigidMapping<defaulttype::Rigid3dTypes, defaulttype::Vec3dTypes>::VecCoord&>(gripperMap->points.getValue());

                    for (unsigned int t = 0; t < mapPoints.size(); t++)
                    {
                        mapPoints[t][movementAxisIndex] = mapPoints[t][movementAxisIndex] + (dir*gripperMoveStep.getValue());
                    }

                    iterationsMoved++;

                    if (iterationsMoved >= movementDuration.getValue())
                    {
                        currentlyClosing = false;
                        currentlyClosed = true;
                    }
                }

                if (currentlyOpening)
                {
                    msg_info("ZyGripping") << "Moving grippers (open)";
                    mapping::RigidMapping<defaulttype::Rigid3dTypes, defaulttype::Vec3dTypes>::VecCoord & mapPoints = const_cast<mapping::RigidMapping<defaulttype::Rigid3dTypes, defaulttype::Vec3dTypes>::VecCoord&>(gripperMap->points.getValue());

                    for (unsigned int t = 0; t < mapPoints.size(); t++)
                    {
                        mapPoints[t][movementAxisIndex] = mapPoints[t][movementAxisIndex] - (dir*gripperMoveStep.getValue());
                    }

                    iterationsMoved++;

                    if (iterationsMoved >= movementDuration.getValue())
                    {
                        currentlyOpening = false;
                        currentlyOpen = true;
                    }
                }
            }
        }
    } // namespace VelocityApproximation

} // namespace Zyklio
