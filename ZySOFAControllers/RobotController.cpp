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
//
// C++ Implementation: RobotController
//

#include <RobotController.h>
#include <sofa/core/visual/VisualParams.h>
#include <sofa/helper/system/FileRepository.h>
#include <sofa/core/ObjectFactory.h>

namespace sofa
{

namespace component
{

namespace controller
{

void RobotController::init()
{
    sofa::simulation::Node* curNode = dynamic_cast<sofa::simulation::Node*>(this->getContext());
    if (curNode)
    {
        curNode->getTreeObjects<ArticulationCenter, ArtCenterVec >(&m_artCenterVec);
        curNode->getTreeObject(ahc);
    }

    hasNewRotValues = false;
}

void RobotController::reset()
{
    hasNewRotValues = false;
}

void RobotController::applyController(void)
{
    // Copy Values
    if (hasNewRotValues) {
        Vec6d * axes  = rotValues.beginEdit();
        for (int i = 0; i < 6; i++) {
            (*axes)[i] = newRotValues[i];
        }
        rotValues.endEdit();
        hasNewRotValues = false;
    }

    int array_size = m_artCenterVec.size() + 1;
    for (unsigned int i=0; i<m_artCenterVec.size(); i++)
    {
        ArtCenterVecIt artCenterIt = m_artCenterVec.begin();
        ArtCenterVecIt artCenterItEnd = m_artCenterVec.end();

        while ((artCenterIt != artCenterItEnd))
        {
            ArtVecIt it = (*artCenterIt)->articulations.begin();
            ArtVecIt itEnd = (*artCenterIt)->articulations.end();
            while (it != itEnd)
            {
                std::vector< core::behavior::MechanicalState<sofa::defaulttype::Vec1Types>* > articulatedObjects;

                sofa::simulation::Node* curNode = dynamic_cast<sofa::simulation::Node*>(this->getContext());
                if (curNode)
                    curNode->getTreeObjects<core::behavior::MechanicalState<sofa::defaulttype::Vec1Types>, std::vector< core::behavior::MechanicalState<sofa::defaulttype::Vec1Types>* > >(&articulatedObjects);

                if (!articulatedObjects.empty())
                {
                    // Set new Robot position and rotation values
                    std::vector< core::behavior::MechanicalState<sofa::defaulttype::Vec1Types>* >::iterator articulatedObjIt = articulatedObjects.begin();

                    if ((*it)->translation.getValue())
                    {
                        unsigned int index = (*it)->articulationIndex.getValue();
                        double newPositionValue = 0.0;
                        if (index < 3 && index < posValues.getValue().size()) {
                            assert((*it)->motion.size() > 0);
                            newPositionValue = posValues.getValue()[index] - (*it)->motion[0];
                        }

                        helper::WriteAccessor<Data<sofa::defaulttype::Vec1Types::VecCoord> > x = *(*articulatedObjIt)->write(sofa::core::VecCoordId::position());
                        helper::WriteAccessor<Data<sofa::defaulttype::Vec1Types::VecCoord> > xfree = *(*articulatedObjIt)->write(sofa::core::VecCoordId::freePosition());
                        
                        if (x.size() <= index) x.resize(array_size);
                        if (xfree.size() <= index) xfree.resize(array_size);

						x[index] = newPositionValue;
                        xfree[index] = newPositionValue;
                    }
                    else // Rotation
                    {
                        unsigned int index = (*it)->articulationIndex.getValue();
                        unsigned int rotIdx = index - 3; // The first 3 are Xpos Ypos Zpos of the object.
                        double newRotationValue = 0.0;
                        if (rotIdx < rotValues.getValue().size()) {
                            double dir = rotDirection.getValue()[rotIdx] == 0? -1 : 1;  // Get rotation direction
                            double off = (rotOffsetValues.getValue()[rotIdx]/180.0)*M_PI;            // Get offset
                            newRotationValue =  dir*((rotValues.getValue()[rotIdx]/180.0)*M_PI) + off;
                        }

                        helper::WriteAccessor<Data<sofa::defaulttype::Vec1Types::VecCoord> > x = *(*articulatedObjIt)->write(sofa::core::VecCoordId::position());
                        helper::WriteAccessor<Data<sofa::defaulttype::Vec1Types::VecCoord> > xfree = *(*articulatedObjIt)->write(sofa::core::VecCoordId::freePosition());

                        if (x.size() <= index) x.resize(array_size);
                        if (xfree.size() <= index) xfree.resize(array_size);

                        x[(*it)->articulationIndex.getValue()] = newRotationValue;
                        xfree[(*it)->articulationIndex.getValue()] = newRotationValue;
                    }
                }
                ++it;
            }
            ++artCenterIt;
        }
    }
}

void RobotController::setRotValueRad(int num, double* value)
{
    newRotValues.clear();
    if (num>6) num=6; // We only support 6 axis right now
    for (int i = 0; i < num; i++) {
        newRotValues[i] = value[i] * 180.0 / M_PI;
    }
    hasNewRotValues = true;
}



SOFA_DECL_CLASS(RobotController)

// Register in the Factory
int RobotControllerClass = core::RegisterObject("Implements a handler that controls the values of the articulations of an articulated hierarchy container via a robot control")
        .add< RobotController >()
        ;
} // namespace controller

} // namespace component

} // namespace sofa
