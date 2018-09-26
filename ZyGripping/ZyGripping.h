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
#ifndef ZYKLIO_ZyGripping_H
#define ZYKLIO_ZyGripping_H

#include "initZyGripping.h"

#include <sofa/core/objectmodel/Base.h>
#include <sofa/core/objectmodel/BaseObject.h>
#include <sofa/simulation/Node.h>

#include <SofaBaseMechanics/MechanicalObject.h>
#include <sofa/simulation/MechanicalOperations.h>

#include "SofaRigid/RigidMapping.h"

namespace Zyklio
{
    namespace GripperHandling
    {
        using namespace sofa;
        using namespace sofa::component;

        /**
         * \brief Implements the opening and closing of a gripper by setting boolean values.
         * 
         * The corresponding scene-graph component needs to be put into the scene-graph node 
         * that contains the rigid body which corresponds to the gripper. The gripper is 
         * then moved by modifying the corresponding RigidMapping in the same node.
         * 
         * The gripper is always in one of four states: open, closing, closed or opening. If
         * it is open or closed, it can be set to closing or opening by using the startClosing()
         * or startOpening() methods accordingly.
         * 
         * If the gripper is closing or opening, it moves each iteration a distance given by 
         * gripperMoveStep. It does that for a number of iterations given by movementDuration.
         * The movement direction of the gripper can be changed by setting direction to true or 
         * false.
         * 
         */
        class SOFA_ZYGRIPPING_API ZyGripping : public /*virtual*/ core::objectmodel::BaseObject
        {
        public:
            typedef core::objectmodel::BaseObject Inherit;

            SOFA_CLASS(ZyGripping, core::objectmodel::BaseObject);

            typedef sofa::core::objectmodel::BaseContext BaseContext;
            typedef sofa::core::objectmodel::BaseObjectDescription BaseObjectDescription;

        public:

            ZyGripping();
            ~ZyGripping() {};

            void init();
            void bwdInit();
            void reinit();

            void startOpening();
            void startClosing();

            void moveGrippers(sofa::simulation::common::MechanicalOperations& mop, sofa::core::behavior::MultiVecCoord& pos);

            Data<bool> direction;
            Data<unsigned int> movementDuration;
            Data<double> gripperMoveStep;
            Data<char> movementAxis;
                    
#ifdef ZYKLIO_DEMO
            template<class T>
            static bool canCreate(T*& obj, core::objectmodel::BaseContext* context, core::objectmodel::BaseObjectDescription* arg)
            {
                std::string sceneHash = sofa::simulation::getSimulation()->getSceneHash();

                sofa::helper::hashCheckHelper hch;
                if (hch.checkHashCorrectness(sceneHash))
                {
                    return Inherit::canCreate(obj, context, arg);
                }

                std::cout << "This plugin can only be used with the Zyklio demo and the correct demo scene." << std::endl;
                return false;
            }
#endif 
        private:
            short int dir; // either 1 or -1
            bool currentlyClosing, currentlyOpening;
            bool currentlyClosed, currentlyOpen;
            unsigned int iterationsMoved;

            unsigned int movementAxisIndex;

            sofa::component::mapping::RigidMapping<defaulttype::Rigid3dTypes, defaulttype::Vec3dTypes>* gripperMap;

        };

    } // namespace VelocityApproximation

} // namespace Zyklio

#endif //ZYKLIO_ZyGripping_H
