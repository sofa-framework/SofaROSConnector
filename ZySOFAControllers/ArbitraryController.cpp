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
// C++ Implementation: ArbitraryController
//
// Description:
//
//
// Author: The SOFA team </www.sofa-framework.org>, (C) 2008
//
// Copyright: See COPYING file that comes with this distribution
//
//
#include <ArbitraryController.h>
#include <sofa/core/visual/VisualParams.h>
#include <sofa/helper/system/FileRepository.h>
#include <sofa/core/ObjectFactory.h>

namespace sofa
{

namespace component
{

namespace controller
{

void ArbitraryController::init()
{
    sofa::simulation::Node* curNode = dynamic_cast<sofa::simulation::Node*>(this->getContext());
    if (curNode)
    {
        curNode->getTreeObjects<ArticulationCenter, ArtCenterVec >(&m_artCenterVec);
        curNode->getTreeObject(ahc);
        int size = 0;

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
                        int current = (*it)->articulationIndex.getValue();
                        size = current > size ? current : size;
                        if ((*it)->translation.getValue())
                        {

                        }
                        else // Rotation
                        {

                        }
                    }
                    ++it;
                }
                ++artCenterIt;
            }
        }

        doubleVector * c = kinValues.beginEdit();
        c->resize(size+1);
        kinValues.endEdit();
    }

    initialized = true;
}



void ArbitraryController::reset()
{
}

void ArbitraryController::applyController(void)
{
    // following code moved out of the "while (it != itEnd)"-loop
    std::vector< core::behavior::MechanicalState<sofa::defaulttype::Vec1Types>* > articulatedObjects;

    sofa::simulation::Node* curNode = dynamic_cast<sofa::simulation::Node*>(this->getContext());
    if (curNode)
    {
        curNode->getTreeObjects<core::behavior::MechanicalState<sofa::defaulttype::Vec1Types>, std::vector< core::behavior::MechanicalState<sofa::defaulttype::Vec1Types>* > >(&articulatedObjects);
    }
    // moved code end

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
                // moved the following lines of code above, to speed up this method
                //std::vector< core::behavior::MechanicalState<sofa::defaulttype::Vec1Types>* > articulatedObjects;

                //sofa::simulation::Node* curNode = dynamic_cast<sofa::simulation::Node*>(this->getContext());
                //if (curNode)
                //{
                //    curNode->getTreeObjects<core::behavior::MechanicalState<sofa::defaulttype::Vec1Types>, std::vector< core::behavior::MechanicalState<sofa::defaulttype::Vec1Types>* > >(&articulatedObjects); // TP: this line eats a lot of time
                //}
                // moved code end

                if (!articulatedObjects.empty())
                {
                    // Set new Robot position and rotation values
                    std::vector< core::behavior::MechanicalState<sofa::defaulttype::Vec1Types>* >::iterator articulatedObjIt = articulatedObjects.begin();

                    int index = (*it)->articulationIndex.getValue();
					if ((getMinIndex() != -1 && getMaxIndex() != -1)
						&& (index >= getMinIndex() && index <= getMaxIndex())) {

                        if (index < (int) kinValues.getValue().size()) {
                            double newValue = kinValues.getValue()[index];

                            // Lower bound
                            if (index < (int) minValues.getValue().size()) {
                                double min =  minValues.getValue()[index];
                                newValue = newValue < min ? min : newValue;
                            }
                            // Upper bound
                            if (index < (int) maxValues.getValue().size()) {
                                double max =  maxValues.getValue()[index];
                                newValue = newValue > max ? max : newValue;
                            }

                            // Update kinValue
                            doubleVector *d = kinValues.beginEdit();
                            (*d)[index] = newValue;
                            kinValues.endEdit();
  
                            if ((*it)->rotation.getValue())  // Rotation: use degrees (convert deg 2 rad)
                            {
                                //std::cout << "Rotation by " << newValue << " deg" << " (" << newValue * M_PI / 180.0 << " rad)" << std::endl;
                                newValue *= M_PI/180.0;
                            }

                            if (index < invertAxis.getValue().size()) {
                                if (invertAxis.getValue()[index]) {
                                    newValue *=-1;
                                }
                            }

                            // Finally write new Value
                            helper::WriteAccessor<Data<sofa::defaulttype::Vec1Types::VecCoord> > x = *(*articulatedObjIt)->write(sofa::core::VecCoordId::position());
                            helper::WriteAccessor<Data<sofa::defaulttype::Vec1Types::VecCoord> > xfree = *(*articulatedObjIt)->write(sofa::core::VecCoordId::freePosition());
                            
                            if (x.size() <= index) x.resize(array_size);
                            if (xfree.size() <= index) xfree.resize(array_size);

    						x[index] = newValue;											
							xfree[index] = newValue;
						
                        }
                        else {
                            serr << "Index out of range" << sendl;
                        }

                    }

                }
                ++it;
            }
            ++artCenterIt;
        }
    }
}

void ArbitraryController::setJointNames(std::vector<std::string> jointNames)
{
    setValuesFromVector<std::string> (this->jointNames, jointNames);
}

void ArbitraryController::setKinValues(std::vector<double> kinValues)
{
    setValuesFromVector<double>(this->kinValues, kinValues);
}

void ArbitraryController::setMinValues(std::vector<double> minValues)
{
    setValuesFromVector<double>(this->minValues, minValues);
}

void ArbitraryController::setMaxValues(std::vector<double> maxValues)
{
    setValuesFromVector<double>(this->maxValues, maxValues);
}

void ArbitraryController::setInvValues(std::vector<bool> invertAxis)
{
    setValuesFromVector<bool>(this->invertAxis, invertAxis);
}

void ArbitraryController::initJointControlledByROS(unsigned int size)
{
    boolVector *tmp = jointControlledByROS.beginEdit();
    tmp->empty();
    tmp->resize(size,true);
    jointControlledByROS.endEdit();
}

void ArbitraryController::setControlIndex(int min, int max)
{
    Vec2i * val = this->controlIndex.beginEdit();
    (*val).set(min, max);
    this->controlIndex.endEdit();
}


void ArbitraryController::setToolValues(std::vector<double> values)
{
    unsigned int min = getMinIndex() == -1 ? 0 : getMinIndex();
    unsigned int max = getMaxIndex() == -1 ? values.size() : getMaxIndex() + 1;

    sofa::helper::vector<double> * val = kinValues.beginEdit();
    val->resize(max);
    for (unsigned int i = min; i < max; i++) {
        (*val)[i] = values[i];
    }
    kinValues.endEdit();
}

void ArbitraryController::setRotValueRad(double value, int index)
{
    sofa::helper::vector<double> * val = kinValues.beginEdit();
    (*val)[index] = value * 180.0 / M_PI;
    kinValues.endEdit();
    // <!-- untested Cebit 2016 code
    //robotController->setRotValueRad(index, &value);
    // -->
}

void ArbitraryController::setRotValueRadByName(double value, std::string name) {
    //std::cout << "\nSetting joint with name: " << name << std::endl;
    stringVector names = this->jointNames.getValue();
    for (unsigned int i = 0; i < names.size(); i++) {
        std::size_t found = names.at(i).find(name);
        if (found != std::string::npos) {
            if (jointControlledByROS.getValue().at(i))
            {
                //std::cout << "\nSetting joint with name: " << name << " (index " << i << ")" << std::endl;
                setRotValueRad(value, i);
            }
            return;
        }
    }
    serr << "Could not find joint for name: " << name << sendl;
}

void ArbitraryController::setToolValue(double value, int index)
{
    // Don't do anything, if not initialized yet
    if (!initialized) return;

    if (getMinIndex() == -1) {
        std::cerr  << "Called setToolValue with index: " << index << " but minIndex = -1;" << std::endl;
        return;
    }

    int IDX = getMinIndex() + index;
    if (IDX > getMaxIndex()) {
        std::cerr  << "Called setToolValue with index: " << index << " > maxindex: " << getMaxIndex() << std::endl;
        return;
    }

    sofa::helper::vector<double> * val = kinValues.beginEdit();
    if (IDX >= val->size()) {
        std::cerr << "Called setToolValue with index: " << index << " > kinValuesSize: " << val->size() << std::endl;
        kinValues.endEdit();
        return;
    }

    // Finally set Value
    (*val)[IDX] = value;
    kinValues.endEdit();
}



SOFA_DECL_CLASS(ArbitraryController)

// Register in the Factory
int ArbitraryControllerClass = core::RegisterObject("Implements a handler that controls the values of the articulations of an articulated hierarchy container via a robot control")
        .add< ArbitraryController >()
        ;
} // namespace controller

} // namespace component

} // namespace sofa
