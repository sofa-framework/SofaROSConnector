/******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, development version     *
*                (c) 2006-2016 INRIA, USTL, UJF, CNRS, MGH                    *
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
#include "ZyColladaLoader.h"
#include <sofa/simulation/Simulation.h>
#include <sofa/core/ObjectFactory.h>
#include <SofaBaseMechanics/MechanicalObject.h>
#include <SofaBaseMechanics/UniformMass.h>
#include <SofaBaseTopology/MeshTopology.h>
#include <SofaOpenglVisual/OglModel.h>												
#include <SofaMeshCollision/TriangleModel.h>
#include <SofaMeshCollision/LineModel.h>
#include <SofaMeshCollision/PointModel.h>
#include <SofaRigid/RigidMapping.h>
#include <SofaGeneralRigid/SkinningMapping.h>
#include <SofaBaseMechanics/BarycentricMapping.h>
#include <SofaBaseMechanics/IdentityMapping.h>
#include <SofaBoundaryCondition/FixedConstraint.h>
#include <SofaBoundaryCondition/SkeletalMotionConstraint.inl>
#include <sofa/core/visual/VisualParams.h>
#include <sofa/helper/system/SetDirectory.h>
#include <stack>
#include <algorithm>

#include <boost/algorithm/string.hpp>

#include <sofa/gui/GUIManager.h>
#include <sofa/gui/qt/RealGUI.h>

#ifdef _WIN32
#include <gl/glut.h>
#else
#include <GL/glut.h>
#endif

#ifdef SOFA_HAVE_PLUGIN_FLEXIBLE
#include <Flexible/deformationMapping/LinearMapping.h>
#endif

#ifdef SOFA_HAVE_PLUGIN_IMAGE
#include <image/ImageContainer.h>
#include <image/MeshToImageEngine.h>
#include <image/ImageFilter.h>
#include <image/ImageViewer.h>
#endif

#ifdef SOFA_HAVE_QTOGREVIEWER
#include <QtOgreViewer/QtOgreViewer.h>
#include <QtOgreViewer/OgreVisualModel.h>
using namespace sofa::gui::qt;
#endif

#include <iomanip>

// New Includes
#include <assimp/ai_assert.h>
#include <BaseImporter.h>
#include <assimp/Exporter.hpp>
#include <assimp/Importer.hpp>
#include <assimp/importerdesc.h>
#include <assimp/DefaultLogger.hpp>

#include <ColladaParser.h>
#include <ColladaLoader.h>

#include <boost/regex.hpp>
#include <boost/algorithm/string/predicate.hpp>

#include <ZySOFAControllers/ArticulatedHierarchyBVHController.h>
#include <SofaGeneralRigid/ArticulatedSystemMapping.h>
#include <SofaGeneralRigid/ArticulatedHierarchyContainer.h>
#include <SofaRigid/RigidRigidMapping.h>
#include <SofaGraphComponent/Gravity.h>
#include <ZySOFAControllers/ArbitraryController.h>
#include <sofa/core/behavior/ConstraintSolver.h>
#include <SofaConstraint/SlidingConstraint.h>
#include <SofaBoundaryCondition/ConstantForceField.h>
#include <SofaBoundaryCondition/LinearMovementConstraint.h>

using namespace sofa::core::objectmodel;

using sofa::component::container::MechanicalObject;
using sofa::component::mass::UniformMass;
using sofa::component::mapping::ArticulatedSystemMapping;
using sofa::component::container::ArticulatedHierarchyContainer;
using sofa::component::controller::RobotController;
using sofa::component::controller::ArbitraryController;
using sofa::component::mapping::RigidRigidMapping;
using sofa::component::contextobject::Gravity;
using sofa::core::behavior::ConstraintSolver;
using sofa::component::constraintset::SlidingConstraint;
using sofa::component::forcefield::ConstantForceField;
using sofa::component::projectiveconstraintset::LinearMovementConstraint;

#ifdef SOFA_HAVE_PLUGIN_ROBOTCONNECTOR
#include <RobotConnector.h>
#include <ToolManipulator.h>
using sofa::component::controller::RobotConnector;
using sofa::component::controller::ToolManipulator;
#endif

#ifdef SOFA_HAVE_PLUGIN_OBBTREEGPU
#include <ObbTreeGPUCollisionModel.h>
//#include <ObbTreeGPUCollisionDetection.h>
#include <ObbTreeGPUFrictionContact.h>
#include <SofaConstraint/PrecomputedConstraintCorrection.h>
#include <SofaConstraint/UncoupledConstraintCorrection.h>
#include <SofaBaseLinearSolver/CGLinearSolver.h>
#include <SofaImplicitOdeSolver/EulerImplicitSolver.h>

using sofa::component::collision::ObbTreeGPUCollisionModel;
//using sofa::component::collision::ObbTreeGPUCollisionDetection;
//using sofa::component::collision::ObbTreeGPULocalMinDistance;
using sofa::component::constraintset::PrecomputedConstraintCorrection;
using sofa::component::constraintset::UncoupledConstraintCorrection;
using sofa::component::linearsolver::CGLinearSolver;
using sofa::component::linearsolver::GraphScatteredMatrix;
using sofa::component::linearsolver::GraphScatteredVector;
using sofa::component::odesolver::EulerImplicitSolver;
#endif

#include <sofa/core/objectmodel/Base.h>

#ifdef _WIN32
#include <cryptohash.h> // for calculating an MD5 hash
#endif

#ifdef SOFA_HAVE_PLUGIN_FLEXIBLE
#include <deformationMapping/LinearMapping.h>
#endif

#define DEMOSCENE_HASH_STRING "80d4ca0fd58eed84cb448619adeb6d80" 

// Defined in ColladaParser.h
// #define DBG(x)  std::cout << x

namespace sofa
{

namespace component
{

namespace loader
{

using namespace sofa::defaulttype;
using namespace sofa::core::loader;
using namespace sofa::component::container;
using namespace sofa::component::mass;
using namespace sofa::component::topology;
using namespace sofa::component::visualmodel;
using namespace sofa::component::mapping;
using namespace sofa::component::collision;
using namespace sofa::component::projectiveconstraintset;
using namespace sofa::simulation;

SOFA_DECL_CLASS(ZyColladaLoader)

int ZyColladaLoaderClass = core::RegisterObject("Specific scene loader for Collada file format.")
        .add< ZyColladaLoader >()
        ;

class ZyColladaLoaderPrivate
{
public:
	struct KinematicModelStructure
	{
	public:
		KinematicModelStructure() {}

		KinematicModelStructure(const KinematicModelStructure& other)
		{
			if (this != &other)
			{
				modelName = other.modelName;
				rootJoint = other.rootJoint;
				for (std::map<unsigned int, std::vector<std::string> >::const_iterator it = other.kinematicJointsByLevel.begin(); it != other.kinematicJointsByLevel.end(); ++it)
				{
					kinematicJointsByLevel.insert(std::make_pair(it->first, std::vector<std::string>()));
					for (int k = 0; k < it->second.size(); ++k)
						kinematicJointsByLevel[it->first].push_back(it->second[k]);
				}
			}
		}

		KinematicModelStructure& operator=(const KinematicModelStructure& other)
		{
			if (this != &other)
			{
				modelName = other.modelName;
				rootJoint = other.rootJoint;
				for (std::map<unsigned int, std::vector<std::string> >::const_iterator it = other.kinematicJointsByLevel.begin(); it != other.kinematicJointsByLevel.end(); ++it)
				{
					kinematicJointsByLevel.insert(std::make_pair(it->first, std::vector<std::string>()));
					for (int k = 0; k < it->second.size(); ++k)
						kinematicJointsByLevel[it->first].push_back(it->second[k]);
				}
			}
			return *this;
		}

		std::string modelName;
		std::string rootJoint;
		std::map<unsigned int, std::vector<std::string> > kinematicJointsByLevel;
	};

	std::map<std::string, KinematicModelStructure> kinematicsModels;

	// FA: Copy/paste, but who cares anyway?
	void DecomposeAiMatrix(const aiMatrix4x4& trans, Vec3d & translation, Quat &quaternion) {
		Vec3d rotation, scale;
		DecomposeAiMatrix(trans, translation, scale, quaternion, rotation);
	}

	void DecomposeAiMatrix(const aiMatrix4x4& trans, Vec3d & translation, Vec3d& scale, Quat &quaternion, Vec3d & rotation) {
		aiVector3D aiScale, aiTranslation;
		aiQuaternion aiRotation;
		trans.Decompose(aiScale, aiRotation, aiTranslation);
		Quat q(aiRotation.x, aiRotation.y, aiRotation.z, aiRotation.w);

		translation.set(aiTranslation.x, aiTranslation.y, aiTranslation.z);
		scale.set(aiScale.x, aiScale.y, aiScale.z);
		rotation.set(q.toEulerVector() * (180.0 / M_PI));
		quaternion = q;
	}

    void buildJointHierarchyRec(Assimp::ColladaArschFotze* colladaLoader, Assimp::ColladaParser* parser, const Assimp::Collada::KinematicsModel& kinModel, const std::string& rootJoint, const std::map<std::string, std::string>& kinematicModelInstances, ColladaTransformHelper* transformHelper = NULL)
	{
		KinematicModelStructure k_ms;
		k_ms.modelName = kinModel.name;
		k_ms.rootJoint = rootJoint;

		std::vector<std::string> childJoints;

		std::cout << "====== Build joint hierarchy starting from: " << rootJoint << " ======" << std::endl;
		for (std::map<std::string, std::string>::const_iterator lit = kinModel.preLinks.begin(); lit != kinModel.preLinks.end(); ++lit)
		{
			if (lit->second.compare(rootJoint) == 0)
			{
				std::cout << " Level 0 joint added: " << lit->first << std::endl;
				childJoints.push_back(lit->first);
			}
		}

		k_ms.kinematicJointsByLevel.insert(std::make_pair(0, childJoints));

		for (int k = 0; k < childJoints.size(); ++k)
		{
			this->buildJointHierarchyRecHelper(kinModel, childJoints[k], k_ms, 1);
		}

		std::cout << "===========================================================================================" << std::endl;
		std::cout << "====== Model " << k_ms.modelName << ": Match links by level with relative transforms ======" << std::endl;
		std::cout << "===========================================================================================" << std::endl;

		// Testing with origin-centered model for the SVH hand
		std::string root_joint_name;
		if (k_ms.kinematicJointsByLevel.find(0) != k_ms.kinematicJointsByLevel.end())
		{
			// Assign root joint of SVH for name mapping test
			root_joint_name = k_ms.kinematicJointsByLevel[0].front();
			std::cout << " -> Root joint mapping: " << root_joint_name << std::endl;
		}
		transformHelper->createJointParentTransform(k_ms.modelName, Vector3(0, 0, 0), Quaternion(0, 0, 0, 1) /*, root_joint_name*/);

        std::cout << "===================================================" << std::endl;
        std::cout << "KinematicsModels known: " << this->kinematicsModels.size() << std::endl;
        for (auto model_it = this->kinematicsModels.begin(); model_it != this->kinematicsModels.end(); ++model_it)
        {
            std::cout << " * " << model_it->first << std::endl;
        }

        Assimp::ColladaParser::KinematicsModelsRelativeTransforms& relativeTransforms = parser->getKinematicsModelsRelativeTransforms();
        Assimp::ColladaParser::KinematicsModelsRelativeTransforms::const_iterator rtm_it = relativeTransforms.find(k_ms.modelName);

        std::cout << "Searching kinematic model: \"" << k_ms.modelName << "\" in relativeTransforms" << std::endl;
        std::cout << "relativeTransforms size: " << relativeTransforms.size() << std::endl;

        for (auto rt_it = relativeTransforms.begin(); rt_it != relativeTransforms.end(); ++rt_it)
        {
            std::cout << " * " << rt_it->first << std::endl;
        }
        std::cout << "===================================================" << std::endl;

        if (rtm_it != relativeTransforms.end())
		{
			const Assimp::RelativeTransformStack& rt_stack = rtm_it->second;
			for (std::map<unsigned int, std::vector<std::string> >::const_iterator kbl_it = k_ms.kinematicJointsByLevel.begin(); kbl_it != k_ms.kinematicJointsByLevel.end(); ++kbl_it)
			{
				std::cout << " - Links in level " << kbl_it->first << ": " << kbl_it->second.size() << std::endl;
				const std::vector<std::string>& linksInLevel = kbl_it->second;
				for (size_t m = 0; m < linksInLevel.size(); ++m)
				{
					std::cout << "    - link " << linksInLevel[m] << std::endl;
					
					std::map<std::string, std::vector<std::pair<unsigned int, Assimp::Collada::Transform> > > transformsByIndex;

					for (size_t n = 0; n < rt_stack.transformStack.size(); ++n)
					{	
						if (rt_stack.transformStack[n].first.compare(linksInLevel[m]) == 0)
						{
							const Assimp::RelativeTransformEntry& rt_entry = rt_stack.transformStack[n].second;
							const Assimp::Collada::Transform& tf = rt_stack.transformStack[n].second.transform;

							if (transformsByIndex.find(rt_entry.indicesToPostLinks.begin()->second) == transformsByIndex.end())
								transformsByIndex.insert(std::make_pair(rt_entry.indicesToPostLinks.begin()->second, std::vector<std::pair<unsigned int, Assimp::Collada::Transform> >()));

							std::cout << "      -> matching transform: ";
							if (tf.mType == Assimp::Collada::TF_TRANSLATE)
								std::cout << "postLink index " << rt_entry.indicesToPostLinks.begin()->first << ", translation by " << tf.f[0] << ", " << tf.f[1] << ", " << tf.f[2];
							else if (tf.mType == Assimp::Collada::TF_ROTATE)
								std::cout << "postLink index " << rt_entry.indicesToPostLinks.begin()->first << ", rotation    around " << tf.f[0] << "," << tf.f[1] << "," << tf.f[2] << " by " << tf.f[3] << " deg.";

							transformsByIndex[rt_entry.indicesToPostLinks.begin()->second].push_back(std::make_pair(rt_entry.indicesToPostLinks.begin()->first, tf));

							std::cout << " from " << rt_entry.elementId << " to: ";
							if (rt_entry.indicesToPostLinks.size() == 1)
							{
								std::cout << rt_entry.indicesToPostLinks.begin()->second;
							}
							else
							{
								std::cout << rt_entry.indicesToPostLinks.size() << " post-links.";
							}
							std::cout << std::endl;
						}
					}

					std::cout << "      ===== SORTED POST-LINK LIST WITH TRANSFORMS =====" << std::endl;
					for (std::map<std::string, std::vector<std::pair<unsigned int, Assimp::Collada::Transform> > >::const_iterator tr_it = transformsByIndex.begin(); tr_it != transformsByIndex.end(); ++tr_it)
					{
						std::cout << "     -> post-link " << tr_it->first << ": " << std::endl;
						const std::vector<std::pair<unsigned int, Assimp::Collada::Transform > >& pl_transforms = tr_it->second;

						std::vector<Assimp::Collada::Transform> relativeTransforms;
						for (size_t m = 0; m < pl_transforms.size(); ++m)
						{
							if (pl_transforms[m].second.mType == Assimp::Collada::TF_TRANSLATE)
							{
								std::cout << "      postLink index " << pl_transforms[m].first << ", translation by " << pl_transforms[m].second.f[0] << ", " << pl_transforms[m].second.f[1] << ", " << pl_transforms[m].second.f[2] << std::endl;
								relativeTransforms.push_back(pl_transforms[m].second);
							}
							else if (pl_transforms[m].second.mType == Assimp::Collada::TF_ROTATE)
							{
								std::cout << "      postLink index " << pl_transforms[m].first << ", rotation    around " << pl_transforms[m].second.f[0] << "," << pl_transforms[m].second.f[1] << "," << pl_transforms[m].second.f[2] << " by " << pl_transforms[m].second.f[3] << " deg." << std::endl;
								relativeTransforms.push_back(pl_transforms[m].second);
							}
						}

						aiQuaternion rotation;
						aiVector3D position;

						aiMatrix4x4 pl_matrix = parser->CalculateResultTransform(relativeTransforms);
						pl_matrix.DecomposeNoScaling(rotation, position);

						Vector3 pl_translation(position.x, position.y, position.z);
						Quaternion pl_quaternion(rotation.x, rotation.y, rotation.z, rotation.w);

						std::cout << "    --> resulting relative transform: translation = " << pl_translation << ", quaternion = " << pl_quaternion << std::endl;
						transformHelper->addJointTransform(k_ms.modelName, linksInLevel[m], tr_it->first, pl_translation, pl_quaternion);
					}
				}
			}
		}
        else
        {
            std::cerr << "" << std::endl;
        }

        std::cout << "Adding to kinematicsModels: " << k_ms.modelName << std::endl;
		kinematicsModels[k_ms.modelName] = k_ms;
	}

	void buildJointHierarchyRecHelper(const Assimp::Collada::KinematicsModel& kinModel, const std::string& rootJoint, KinematicModelStructure& kms, unsigned int level)
	{
		std::cout << " buildJointHierarchyRecHelper(): level = " << level << ", rootJoint = " << rootJoint << std::endl;
		std::vector<std::string> childJoints;
		for (std::map<std::string, std::string>::const_iterator lit = kinModel.preLinks.begin(); lit != kinModel.preLinks.end(); ++lit)
		{
			if (lit->second.compare(rootJoint) == 0)
			{
				std::cout << " Level " << level << " joint added: " << lit->first << std::endl;
				kms.kinematicJointsByLevel[level].push_back(lit->first);
				childJoints.push_back(lit->first);
			}
		}

		if (childJoints.size() > 0)
		{
			for (int k = 0; k < childJoints.size(); ++k)
			{
				this->buildJointHierarchyRecHelper(kinModel, childJoints[k], kms, level + 1);
			}
		}
	}
};

ZyColladaLoader::ZyColladaLoader() : SceneLoader()
  , subSceneRoot()
  , importer(NULL)
  , animationSpeed(initData(&animationSpeed, 1.0f, "animationSpeed", "animation speed"))
  , generateCollisionModels(initData(&generateCollisionModels, 1, "generateCollisionModels", "1: generate point/line/triangle collision models for imported meshes\n2: Use ObbTreeGPUCollisionDetection"))
  , loadScene(initData(&loadScene, true, "loadScene", "Default: true. Is set automatically to false if scene loades."))
  , explicitWhitelist(initData(&explicitWhitelist, false, "explicitWhitelist", "If set to true, only whitelisted elements will collide."))
  , useContactManifolds(initData(&useContactManifolds, false, "useContactManifolds", "Use Contact Manifolds (reduces number of contact points to 4)"))
  , maxNumberOfLineLineManifolds(initData(&maxNumberOfLineLineManifolds, 1u, "maxNumberOfLineLineManifolds", "Maximum number of Line/Line contact manifolds that should be created. The lower number defined in both models is used. Cannot be smaller than 1."))
  , maxNumberOfFaceVertexManifolds(initData(&maxNumberOfFaceVertexManifolds, 1u, "maxNumberOfFaceVertexManifolds", "Maximum number of Face/Vertex contact manifolds that should be created. The lower number defined in both models is used. Cannot be smaller than 1."))
  , maxNumberOfManifolds(initData(&maxNumberOfManifolds, 1u, "maxNumberOfManifolds", "Maximum number of contact manifolds that should be created. The lower number defined in both models is used. Cannot be smaller than 1."))
{
    d = new ZyColladaLoaderPrivate();
    m_transformHelper = new ColladaTransformHelper();
	importer = new Assimp::Importer();
}

ZyColladaLoader::~ZyColladaLoader()
{
    importer->FreeScene();
	delete importer; 
	importer = NULL;
	delete d;
	d = NULL;
    if (m_transformHelper)
    {
        delete m_transformHelper;
        m_transformHelper = NULL;
    }
}

void ZyColladaLoader::init()
{

    if (0 == subSceneRoot) {
        serr << "SubSceneRoot is NULL" << sendl;
        return;
    }

    // retrieving parent node
    core::objectmodel::BaseContext* currentContext = getContext();
    Node* parentNode = dynamic_cast<Node*>(currentContext);
    if (!parentNode)
    {
        sout << "Error: ZyColladaLoader::init, loader " << name.getValue() << "has no parentNode" << sendl;
        if (currentContext)
            sout << "Context is : " << currentContext->getName() << sendl;

        return;
    }

    // placing root node of the loaded sub scene
    std::string subSceneName(name.getValue());
    if (!subSceneName.empty())
        subSceneName += "_";
    subSceneName += "scene";
    subSceneRoot->setName(subSceneName);
    parentNode->addChild(subSceneRoot);

    // find how many siblings scene loaders there are upward the current one
    int sceneLoaderNum = 0;

    Node::ObjectIterator objectIt;
    for (objectIt = parentNode->object.begin(); objectIt != parentNode->object.end(); ++objectIt)
    {
        if (dynamic_cast<SceneLoader*>(objectIt->get()))
            ++sceneLoaderNum;

        if (this == *objectIt)
            break;
    }

    // place an iterator on the last scene loader generated node
    int sceneLoaderNodeNum = 0;

    Node::ChildIterator childIt = parentNode->child.begin();
    if (1 != sceneLoaderNum)
    {
        for (; childIt != parentNode->child.end() - 1; ++childIt)
        {
            ++sceneLoaderNodeNum;
            if (subSceneRoot == *childIt || sceneLoaderNum == sceneLoaderNodeNum)
                break;
        }
    }

    // swap our generated node position till it is at the right place
    for (Node::ChildIterator it = parentNode->child.end() - 1; it != childIt; --it)
        parentNode->child.swap(it, it - 1);
}

void ZyColladaLoader::bwdInit()
{
    CheckSolverOrder();
}

bool ZyColladaLoader::load()
{
    sout << "Loading Collada (.dae) file: " << m_filename << sendl;

    bool fileRead = false;

    // loading file
    const char* filename = m_filename.getFullPath().c_str();

    std::ifstream file(filename);

    if (!file.good())
    {
        serr << "Error: ZyColladaLoader: Cannot read file '" << m_filename << "'." << sendl;
        return false;
    }

    // reading file
    try {
        Assimp::DefaultLogger::create("ZyColladaLoader", Assimp::Logger::VERBOSE, aiDefaultLogStream_STDERR);
        fileRead = readDAE(file, filename);
    }
    catch (std::exception &ex) {
        std::cerr << "Error while importing dae: " << ex.what() << std::endl;
    }
    catch (...) {
        std::cerr << "unknown error while importing dae." << std::endl;
    }

    file.close();

    return fileRead;
}

void ZyColladaLoader::Display3DText(Vec3d v, std::string str)
{
    int scale = 0.3;
    const char* s = str.c_str();

    glPushMatrix();
    glTranslatef(v.x(), v.y(), v.z());
    glScalef(scale, scale, scale);

    while (*s)
    {
        glutStrokeCharacter(GLUT_STROKE_ROMAN, *s);
        s++;
    }
    glPopMatrix();
}

void ZyColladaLoader::DrawLine(const sofa::core::visual::VisualParams *vparams, Vec3d from, Vec3d to, const Vec<4, float> &color, const std::string & text) {
    std::vector<Vector3> points;
    points.push_back(from);
    points.push_back(to);
    vparams->drawTool()->drawLines(points, 1, color);
    if (!text.empty()) {
        Display3DText(to - from, text);
    }
}

void ZyColladaLoader::draw(const sofa::core::visual::VisualParams *vparams)
{
    if (m_transformHelper != NULL)
        m_transformHelper->draw(vparams);
        
                    /*static Vec<4,float> red  (1.0,0.2,0.2,1);
                    static Vec<4,float> green(0.2,1.0,0.2,1);
                    static Vec<4,float> blue (0.2,0.2,1.0,1);

                    for (unsigned int i = 0; i < meshTransformations.size(); i++) {
                        MeshTransform t = meshTransformations.at(i);

                        if (t.isBasePhysicsModel) {
                            DrawLine(vparams, Vector3(), t.jointTrans, red, t.nodeID);
                            DrawLine(vparams, t.jointTrans,  t.jointTrans+t.obj2Trans, green);
                            DrawLine(vparams, t.jointTrans+t.obj1Quat.rotate(t.obj1Trans), t.jointTrans+t.obj1Quat.rotate(t.obj1Trans)+t.obj2Trans, blue);
                        }
                    }*/
                    
}

int ZyColladaLoader::CalcNumAxes(aiVector3D &vec) {
    int num = 0;
    if (fabs(fabs(vec.x) - 1) < FLT_EPSILON) num++;
    if (fabs(fabs(vec.y) - 1) < FLT_EPSILON) num++;
    if (fabs(fabs(vec.z) - 1) < FLT_EPSILON) num++;
    return num;
}

std::string ZyColladaLoader::GetAxes(aiVector3D &vec, aiJoint::JointType & type) {
    std::string ret;
    if (type == aiJoint::REVOLUTE) {
        if (fabs(fabs(vec.x) - 1) < FLT_EPSILON) ret += " Xrotation";
        if (fabs(fabs(vec.y) - 1) < FLT_EPSILON) ret += " Yrotation";
        if (fabs(fabs(vec.z) - 1) < FLT_EPSILON) ret += " Zrotation";
    }
    else if (type == aiJoint::PRISMATIC) {
        if (fabs(fabs(vec.x) - 1) < FLT_EPSILON) ret += " Xposition";
        if (fabs(fabs(vec.y) - 1) < FLT_EPSILON) ret += " Yposition";
        if (fabs(fabs(vec.z) - 1) < FLT_EPSILON) ret += " Zposition";
    }
    return ret;
}

void ZyColladaLoader::GetJointMinMaxValues(aiJoint* j, const std::map<std::string, aiJoint*> &jointMap, std::vector<double> &minValues, std::vector<double> &maxValues, std::vector<bool> &invertAxis, std::vector<std::pair<int, std::string> > &actuators, std::vector<std::string > &jointNames) {

    if (std::strcmp(j->mPreLinkId.C_Str(), "") == 0) {
        for (int i = 0; i < 3; i++) {
            minValues.push_back(-DBL_MAX);
            maxValues.push_back(DBL_MAX);
            invertAxis.push_back(false);
        }
        jointNames.push_back("__BASE_POSITION_X__");
        jointNames.push_back("__BASE_POSITION_Y__");
        jointNames.push_back("__BASE_POSITION_Z__");
    }
    else if (j->getAxis() != aiJoint::UNDEFINED)  {
        minValues.push_back(j->mMin);
        maxValues.push_back(j->mMax);
        invertAxis.push_back(j->isInverted());
        jointNames.push_back(j->mName.C_Str());
    }

    if (j->isToolJoint) {
        actuators.push_back(std::make_pair<int, std::string>(minValues.size() - 1, j->mName.C_Str()));
        std::cout << "IsToolAttached: constraint: " << j->mName.C_Str() << std::endl;
    }

    for (int p = 0; p < j->mNumPostLinkIds; p++) {
        const char* childLinkName = j->mPostLinkIds[p]->C_Str();

        if (jointMap.find(childLinkName) != jointMap.end()) { // Recurse Down
            aiJoint *child = jointMap.at(childLinkName);
            GetJointMinMaxValues(child, jointMap, minValues, maxValues, invertAxis, actuators, jointNames);
        }
        else {
            serr << "ERROR in getJointMinMaxValues: Did not find joint ID(2): " << childLinkName << sendl;
            return;
        }
    }

}


MechanicalObject<Vec3dTypes>::SPtr ZyColladaLoader::AddSlidingLine(int i, MechanicalObject<Rigid3dTypes>::SPtr& src, Node::SPtr& anchorNode, Vec3d* from, Vec3d* to /*= NULL*/) {

    bool isPoint = (to == NULL);

    std::string name;
    if (to == NULL) {
        name = "Point_";
    }
    else {
        name = "Line_";
    }

    std::stringstream nodeIdStream;
    nodeIdStream << name << i;

    Node::SPtr lineNode = getSimulation()->createNewNode(nodeIdStream.str());
    anchorNode.get()->addChild(lineNode);
    MechanicalObject<Vec3dTypes>::SPtr mech = sofa::core::objectmodel::New<MechanicalObject<Vec3dTypes> >();

    mech.get()->setName("points");
    Data<sofa::helper::vector<Vec3dTypes::Coord> >* d_x = mech->write(core::VecCoordId::position());
    sofa::helper::vector<Vec3dTypes::Coord> &x = *d_x->beginEdit();

    if (isPoint) {
        x.resize(1);
    }
    else {
        x.resize(2);
    }

    x[0] = Vec3dTypes::Coord(*from);
    if (!isPoint) {
        x[1] = Vec3dTypes::Coord(*to);
    }

    d_x->endEdit();

    lineNode.get()->addObject(mech);

    // Add Mapping

    RigidMapping<Rigid3dTypes, Vec3dTypes>::SPtr mapping = sofa::core::objectmodel::New< RigidMapping<Rigid3dTypes, Vec3dTypes> >();
    mapping.get()->setModels(src.get(), mech.get());
    lineNode.get()->addObject(mapping);


    return mech;
}


/** Creates a valid BVH file of the given jointMap, starting with j.
             *  ss will be the bvh output
             *  jointInfo will be filled to enable easy links for the objects which will be connected with rigidrigidmappings
             */
int ZyColladaLoader::CreateArticulatedJointHierarchy(std::stringstream &ss, aiJoint* j, std::map<std::string, aiJoint*> &jointMap,
                                                        std::map<std::string, JointInfo> &jointInfo, sofa::component::container::MechanicalObject<Rigid3Types> *mo, std::map<int, Quat> &globalQuatMap, int i, bool firstcall) {
    if (firstcall) {
        __joint_index = 1;
    }

    JointInfo newJ;
    newJ.link6D = mo;
    newJ.index = __joint_index;
    newJ.mName = j->mObjName.C_Str();
    newJ.jName = j->mLinkId.C_Str();
    int numAxes = CalcNumAxes(j->mAxis);

    // root joint (has no preLinkId)
    if (j->mPreLinkId.length == 0) {
        ss << "HIERARCHY" << std::endl
           << "ROOT " << j->mObjName.C_Str() << std::endl
           << std::setw(i * 4) << "" << "{" << std::endl
           << std::setw(i * 4) << "" << "    OFFSET    " << j->mTranslationRel.x << " " << j->mTranslationRel.y << " " << j->mTranslationRel.z << "" << std::endl
           << std::setw(i * 4) << "" << "    CHANNELS  3  Xposition Yposition Zposition" << std::endl;
        numAxes += 3;

        std::string obj_root_joint(j->mKinematicsId.C_Str());
        this->m_objectRootJoints[obj_root_joint] = j;


#if 0
        if (this->m_transformHelper != NULL)
        {
            Vector3 joint_translation;
            Quaternion joint_orientation;
            DecomposeAiMatrix(newJ.mTransformation, joint_translation, joint_orientation);

            std::cout << " --> add ROOT JOINT: " << obj_root_joint << std::endl;

            this->m_transformHelper->createJointParentTransform(obj_root_joint,
                                                                joint_translation, joint_orientation);
        }
#endif
    }
    else { // joint
        ss << std::setw(i * 4) << "" << "JOINT " << j->mObjName.C_Str() << "" << std::endl
           << std::setw(i * 4) << "" << "{" << std::endl
           << std::setw(i * 4) << "" << "    OFFSET    " << j->mTranslationRel.x << " " << j->mTranslationRel.y << " " << j->mTranslationRel.z << "" << std::endl
           << std::setw(i * 4) << "" << "    CHANNELS  " << numAxes << " " << GetAxes(j->mAxis, j->mType) << "" << std::endl;
											 
#if 0
        if (this->m_transformHelper != NULL)
        {
            std::string predecessor_joint(j->mPreLinkId.C_Str());

            std::string mechanism_name(j->mKinematicsId.C_Str());
            std::string joint_name(j->mKinematicsId.C_Str());
            joint_name.append("/");
            joint_name.append(j->mId.C_Str());

            Vector3 joint_translation(j->mTranslationRel.x, j->mTranslationRel.y, j->mTranslationRel.z);
            Quaternion joint_orientation(j->mRotationQuat.x, j->mRotationQuat.y, j->mRotationQuat.z, j->mRotationQuat.w);

            //DecomposeAiMatrix(, joint_translation, joint_orientation);

            std::cout << " --> add child transform: " << predecessor_joint << " --> " << joint_name << std::endl;
            std::cout << "     translation: " << joint_translation << ", orientation: " << joint_orientation << std::endl;

            this->m_transformHelper->addJointTransform(mechanism_name, predecessor_joint, joint_name,
                                                       joint_translation, joint_orientation);
        }
#endif
    }

    // Append absolute translation for later mesh update
    newJ.transX = j->mTranslation.x;
    newJ.transY = j->mTranslation.y;
    newJ.transZ = j->mTranslation.z;

    std::cout << "===================== newJ '" << j->mName.C_Str() << "' =======================" << std::endl;
    /*std::cout << "Joints registered in jointByJointName map: " << jointByJointName.size() << std::endl;
                std::cout << " registered names: ";
                for (std::map<std::string, aiJoint*>::const_iterator it = jointByJointName.begin(); it != jointByJointName.end(); it++)
                    std::cout << it->first << ";";*/

    std::cout << "Physics models registered in physicsModelsByName map = " << physicsModelsByName.size() << ": " << std::endl;
    for (std::map<std::string, aiPhysicsModel*>::const_iterator it = physicsModelsByName.begin(); it != physicsModelsByName.end(); it++)
    {
        std::cout << " - " << it->first << ": parent = " << it->second->mParentId.C_Str() << ", meshID = " << it->second->mMeshId.C_Str() << ", isTool = " << it->second->isTool << std::endl;
    }

    std::cout << "Joints registered in rbcByName map = " << rbcByName.size() << ": " << std::endl;
    for (std::map<std::string, aiRigidBodyConstraint*>::const_iterator it = rbcByName.begin(); it != rbcByName.end(); it++)
    {
        std::cout << " - " << it->first << ": object1 = " << it->second->mObject1.C_Str() << ", object2 = " << it->second->mObject2.C_Str() << std::endl;
    }

    std::cout << std::endl;

    std::cout << " F# names: mLinkId = " << j->mLinkId.C_Str() << ", mName = " << j->mName.C_Str() << ", mKinematicsID = " << j->mKinematicsId.C_Str() << ", mObjName = " << j->mObjName.C_Str() << std::endl;

    std::string joint_name(j->mId.C_Str());
    std::string joint_kinematicsID(j->mKinematicsId.C_Str());
    joint_kinematicsID.append("/");
    std::string joint_kinematicsID_noSlash(j->mKinematicsId.C_Str());
    std::string joint_prelinkID_noRoot(j->mPreLinkId.C_Str());
    std::string joint_prelinkID(j->mPreLinkId.C_Str());
    boost::algorithm::replace_all(joint_prelinkID_noRoot, joint_kinematicsID, "");

    std::cout << " F# Check in PhysicsModels for matching end-effector to tool link" << std::endl;
    for (std::map<std::string, aiPhysicsModel*>::iterator pm_it = physicsModelsByName.begin(); pm_it != physicsModelsByName.end(); pm_it++)
    {
        std::cout << " - check PhysicsModel " << pm_it->first << std::endl;
        if (pm_it->second->isTool)
        {
            std::string toolID(pm_it->second->mToolId.C_Str());
            std::cout << "   marked as isTool; toolID = " << toolID << std::endl;
            std::cout << "   compared with joint_name = " << joint_name << "; result = " << joint_name.compare(toolID) << std::endl;
            if (joint_name.compare(toolID) == 0)
            {
                std::cout << "   We FOUND a MATCH for END-EFFECTOR to TOOL LINK: " << joint_prelinkID << " --> " << joint_name << std::endl;

                Vector3 joint_translation;
                Quaternion joint_orientation;
                DecomposeAiMatrix(pm_it->second->mToolTransform, joint_translation, joint_orientation);
                this->m_transformHelper->addJointTransform(joint_kinematicsID_noSlash, joint_prelinkID, joint_name, joint_translation, joint_orientation);
            }
        }
    }

    std::cout << " F# chain: mPreLinkId = " << j->mPreLinkId.C_Str() << "; preLinkID with stripped kinematics ID = " << joint_prelinkID_noRoot << std::endl;

    std::cout << " F# Check in PhysicsModels for matching end-effector pre-link" << std::endl;
    for (std::map<std::string, aiPhysicsModel*>::iterator pm_it = physicsModelsByName.begin(); pm_it != physicsModelsByName.end(); pm_it++)
    {
        std::cout << " - check PhysicsModel " << pm_it->first << std::endl;
        if (pm_it->second->isTool)
        {
            std::string toolID(pm_it->second->mToolId.C_Str());
            std::cout << "   marked as isTool; toolID = " << toolID << std::endl;
            std::cout << "   compared with joint_prelinkID = " << joint_prelinkID << "; result = " << joint_prelinkID.compare(toolID) << std::endl;
        }
    }

    std::map<std::string,std::string>::iterator jn_to_id = jointIDToJointName.find(j->mPreLinkId.C_Str());
    if (jn_to_id != jointIDToJointName.end())
    {
        std::cout << " F# prelink ID found: YES; known under name " << jointIDToJointName[j->mPreLinkId.C_Str()] << std::endl;

#if 0
        if (this->m_transformHelper != NULL)
        {
            bool preLink_exists = this->m_transformHelper->hasJointTransform(joint_kinematicsID_noSlash, joint_prelinkID);
            bool thisLink_exists = this->m_transformHelper->hasJointTransform(joint_kinematicsID_noSlash, joint_kinematicsID_noSlash);
            std::cout << "  preLink_exists in hierarchy " << joint_kinematicsID_noSlash << ": " << preLink_exists << std::endl;
            std::cout << "  this joint exists in hierarchy " << joint_kinematicsID_noSlash << ": " << thisLink_exists << std::endl;

            if (preLink_exists && !thisLink_exists)
            {
                Vector3 joint_translation(j->mTranslation.x, j->mTranslation.y, j->mTranslation.z);
                Quaternion joint_orientation(j->mRotationQuat.x, j->mRotationQuat.y, j->mRotationQuat.z, j->mRotationQuat.w);

                std::cout << "  register previously unknown joint " << joint_name << " under parent " << joint_prelinkID << " in hierarchy " << joint_kinematicsID_noSlash << std::endl;
                this->m_transformHelper->addJointTransform(joint_kinematicsID_noSlash, joint_prelinkID, joint_name,
                                                           joint_translation, joint_orientation);
            }
        }
#endif
    }
    else
    {
        std::cout << " F# prelink ID NOT FOUND!" << std::endl;
    }

    std::map<std::string, std::string>::iterator jn_to_id_stripped = jointIDToJointName.find(joint_prelinkID_noRoot);
    if (jn_to_id_stripped != jointIDToJointName.end())
    {
        std::cout << " F# stripped prelink ID found: YES; known under name " << jointIDToJointName[joint_prelinkID_noRoot] << std::endl;
#if 0
        if (this->m_transformHelper != NULL)
        {
            bool preLink_exists = this->m_transformHelper->hasJointTransform(joint_kinematicsID_noSlash, joint_prelinkID);
            bool thisLink_exists = this->m_transformHelper->hasJointTransform(joint_kinematicsID_noSlash, joint_name);
            std::cout << "  preLink_exists in hierarchy " << joint_kinematicsID_noSlash << ": " << preLink_exists << std::endl;
            std::cout << "  this joint exists in hierarchy " << joint_kinematicsID_noSlash << ": " << thisLink_exists << std::endl;

            if (preLink_exists && !thisLink_exists)
            {
                Vector3 joint_translation(j->mTranslation.x, j->mTranslation.y, j->mTranslation.z);
                Quaternion joint_orientation(j->mRotationQuat.x, j->mRotationQuat.y, j->mRotationQuat.z, j->mRotationQuat.w);

                std::cout << "  register previously unknown joint " << joint_name << " under parent " << joint_prelinkID << " in hierarchy " << joint_kinematicsID_noSlash << "???" << std::endl;
                //this->m_transformHelper->addJointTransform(joint_kinematicsID_noSlash, joint_prelinkID, joint_name,
                //	joint_translation, joint_orientation);
            }
        }
#endif
    }
    else
    {
        std::cout << " F# stripped prelink ID NOT FOUND!" << std::endl;
    }

    std::cout << " F# mPostLinkIds = " << j->mNumPostLinkIds << std::endl;
    for (int o = 0; o < j->mNumPostLinkIds; o++)
    {
        std::string joint_postlinkID(j->mPostLinkIds[o]->C_Str());

        std::string joint_postlinkID_noRoot(j->mPostLinkIds[o]->C_Str());
        boost::algorithm::replace_all(joint_postlinkID_noRoot, joint_kinematicsID, "");
        std::cout << "  * " << j->mPostLinkIds[o]->C_Str() << "; without kinematicsID = " << joint_postlinkID_noRoot << std::endl;

        std::cout << " F# Check in PhysicsModels for matching tool postlink" << std::endl;
        for (std::map<std::string, aiPhysicsModel*>::iterator pm_it = physicsModelsByName.begin(); pm_it != physicsModelsByName.end(); pm_it++)
        {
            std::cout << " - check PhysicsModel " << pm_it->first << std::endl;
            if (pm_it->second->isTool)
            {
                std::string toolID(pm_it->second->mToolId.C_Str());
                std::cout << "   marked as isTool; toolID = " << toolID << std::endl;
                std::cout << "   compared with joint_postlinkID = " << joint_postlinkID << "; result = " << joint_postlinkID.compare(toolID) << std::endl;
            }
        }

        std::map<std::string, std::string>::iterator jn_to_id_post = jointIDToJointName.find(joint_postlinkID);
        if (jn_to_id_post != jointIDToJointName.end())
        {
            std::cout << "   found in jointIDToJointName: YES";
        }
        else
        {
            std::cout << "   found in jointIDToJointName: NO";
        }
        std::cout << std::endl;

        std::map<std::string, std::string>::iterator jn_to_id_post_stripped = jointIDToJointName.find(joint_postlinkID_noRoot);
        if (jn_to_id_post_stripped != jointIDToJointName.end())
        {
            std::cout << " F# stripped postlink ID found: YES; known under name " << jointIDToJointName[joint_postlinkID_noRoot] << std::endl;
        }
        else
        {
            std::cout << " F# stripped postlink ID NOT FOUND!" << std::endl;
        }
        std::cout << std::endl;

        std::map<std::string, aiRigidBodyConstraint*>::const_iterator rbc_it = rbcByName.find(joint_postlinkID);
        if (rbc_it != rbcByName.end())
        {
            std::cout << " F# unstripped postlink ID found in rbcByName" << std::endl;
        }
        else
        {
            std::cout << " F# unstripped postlink ID NOT FOUND in rbcByName" << std::endl;
        }

        std::map<std::string, aiRigidBodyConstraint*>::const_iterator rbc_it_stripped = rbcByName.find(joint_postlinkID_noRoot);
        if (rbc_it_stripped != rbcByName.end())
        {
            std::cout << " F# stripped postlink ID found in rbcByName" << std::endl;
        }
        else
        {
            std::cout << " F# stripped postlink ID NOT FOUND in rbcByName" << std::endl;
        }
    }

    std::cout << " F# TRANSLATION " << __joint_index << " - " << j->mObjName.C_Str() << ": " << j->mTranslation.x << "  " << j->mTranslation.y << "  " << j->mTranslation.z << std::endl;
    std::cout << " B# ROTATION    " << __joint_index << " - " << j->mObjName.C_Str() << ": " << j->mRotationQuat.x << "  " << j->mRotationQuat.y << "  " << j->mRotationQuat.z << "  " << j->mRotationQuat.w << std::endl;

    std::cout << "===================== newJ '" << j->mName.C_Str() << "' =======================" << std::endl;

    newJ.quatX = j->mRotationQuat.x;
    newJ.quatY = j->mRotationQuat.y;
    newJ.quatZ = j->mRotationQuat.z;
    newJ.quatW = j->mRotationQuat.w;

    globalQuatMap[__joint_index] = Quat(newJ.quatX, newJ.quatY, newJ.quatZ, newJ.quatW);

    if (j->mNumPostLinkIds == 0)
    {  // joint has no postLinkId
        __joint_index++;
        i++;
        ss << std::setw(i * 4) << "" << "End Site" << std::endl
           << std::setw(i * 4) << "" << "{" << std::endl
           << std::setw(i * 4) << "" << "    OFFSET    0 0 0" << std::endl
           << std::setw(i * 4) << "" << "}" << std::endl;
        i--;
    }
    else
    {
        // Recurse down for all childs (PostLinkIds)
        for (int p = 0; p < j->mNumPostLinkIds; p++) {
            const char* childLinkName = j->mPostLinkIds[p]->C_Str();
            if (jointMap.find(childLinkName) != jointMap.end())
            { // recurse next joint
                __joint_index++;

                numAxes += CreateArticulatedJointHierarchy(ss, jointMap[childLinkName], jointMap, jointInfo, mo, globalQuatMap, i + 1, false);
            }
            else {
                serr << "ERROR in CreateArticulatedJointHierarchy: Did not find Joint ID(1): " << childLinkName << sendl;
                return 0;
            }
        }
    }
    ss << std::setw(i * 4) << "" << "}" << std::endl;

    // Append 2 Frames at the end of the file, when we are back on the root joint
    if (j->mPreLinkId.length == 0) {
        ss << "MOTION" << std::endl
           << "Frames:      3" << std::endl
           << "Frame Time:  1.0" << std::endl;
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < numAxes; j++) { ss << "0 "; }
            ss << "" << std::endl;
        }
    }
    jointInfo[newJ.mName] = newJ;



    return numAxes;
}

/*
            bool ZyColladaLoader::hasObbTreeGPUActive() {
            #ifdef SOFA_HAVE_PLUGIN_OBBTREEGPU

            std::vector<ObbTreeGPUCollisionDetection* > colDec;
            sofa::core::objectmodel::BaseContext::GetObjectsCallBackT<ObbTreeGPUCollisionDetection,std::vector<ObbTreeGPUCollisionDetection* > > cd(&colDec);
            getContext()->getObjects(TClassInfo<ObbTreeGPUCollisionDetection>::get(), cd, TagSet(), BaseContext::SearchRoot);

            std::vector<ObbTreeGPULocalMinDistance* > lmin;
            sofa::core::objectmodel::BaseContext::GetObjectsCallBackT<ObbTreeGPULocalMinDistance,std::vector<ObbTreeGPULocalMinDistance* > > lm(&lmin);
            getContext()->getObjects(TClassInfo<ObbTreeGPUCollisionDetection>::get(), lm, TagSet(), BaseContext::SearchRoot);

            sout << "Searching TruGPU " << colDec.size() << " " << lmin.size() << sendl;

            if (colDec.size() == 1 && lmin.size() == 1) return true;

            #endif
            return false;
            }
            */


void ZyColladaLoader::CheckSolverOrder() {

    std::vector<BaseObject* > colDec;
    sofa::core::objectmodel::BaseContext::GetObjectsCallBackT<BaseObject, std::vector<BaseObject* > > cd(&colDec);
    getContext()->getObjects(TClassInfo<BaseObject>::get(), cd, TagSet(), sofa::core::objectmodel::BaseContext::SearchRoot);
    bool foundAfter = false;
    bool foundMe = false;
    for (int i = 0; i < colDec.size(); i++) {
        if (dynamic_cast<ZyColladaLoader*>(colDec.at(i)) != NULL) {
            foundMe = true;
            continue;
        }
        if (foundMe) {
            if (dynamic_cast<ConstraintSolver*>(colDec.at(i)) != NULL) {
                foundAfter = true;
                break;
            }
        }
    }
    if (foundAfter) {
        // OK
        sout << "Found ConstraintSolver after ZyColladaLoader node. This is good." << sendl;
    }
    else {
        // NOK
        serr << "Did not find a ConstraintSolver AFTER ZyColladaLoader. Make shure it is AFTER the ZyColladaLoader. Else objects will not fall." << sendl;
    }
}


void ZyColladaLoader::fillObbTreeBlacklists2(std::map < std::string, std::vector<std::string> > &blacklist) {

    std::cout << "===================================================" << "BLACKLIST\n" << "===================================================" << std::endl;
    std::vector<sofa::core::CollisionModel* > vec;
    sofa::core::objectmodel::BaseContext::GetObjectsCallBackT<sofa::core::CollisionModel, std::vector<sofa::core::CollisionModel* > > model(&vec);
    subSceneRoot->getObjects(TClassInfo<sofa::core::CollisionModel>::get(), model, TagSet(), BaseContext::SearchDown);
    for (std::vector<sofa::core::CollisionModel* >::iterator it = vec.begin(); it != vec.end(); it++)
    {
        std::string name = (*it)->getName();
        if (blacklist.find(name) != blacklist.end()) {
            std::cout << name << "\t << ";
            std::vector<std::string> models = blacklist.at(name);
            for (std::vector<std::string>::iterator mit = models.begin(); mit != models.end(); mit++) {
                std::cout << *mit << " | ";
                (*it)->addToCollisionModelBlacklist(*mit);
            }
            std::cout << std::endl;
        }
    }
    std::cout << "END BLACKLIST\n" << "===================================================" << std::endl;
}

void ZyColladaLoader::fillObbTreeBlacklists(std::map<std::string, std::vector<std::string> > &connected, std::map<std::string, std::string> &jointForMesh, const  std::map<std::string, aiJoint*>  &jointMap, std::map<std::string, aiJoint*> & joints, std::map < std::string, std::vector<std::string> > &blacklist) {
#ifdef SOFA_HAVE_PLUGIN_OBBTREEGPU

    std::vector<sofa::core::CollisionModel* > vec;
    sofa::core::objectmodel::BaseContext::GetObjectsCallBackT<sofa::core::CollisionModel, std::vector<sofa::core::CollisionModel* > > model(&vec);
    subSceneRoot->getObjects(TClassInfo<sofa::core::CollisionModel>::get(), model, TagSet(), BaseContext::SearchDown);

    for (std::vector<sofa::core::CollisionModel* >::iterator it = vec.begin(); it != vec.end(); it++)
    {
        sofa::core::CollisionModel * c = (*it);
        std::string mesh = c->getName();
        //std::cout << "GPUModel: " << mesh;
        if (jointForMesh.find(mesh) != jointForMesh.end()) {
            std::string joint = jointForMesh.at(mesh);
            //std::cout << " joint: " << joint;
            if (joints.find(joint) != joints.end()) {
                aiJoint * j = joints.at(joint);
                std::string parent = j->mPreLinkId.C_Str();
                //std::cout << " parent: " << parent;
                if (jointMap.find(parent) != jointMap.end()) {
                    aiJoint * p = jointMap.at(parent);
                    std::string object = p->mObjName.C_Str();
                    //std::cout << " parentObj: " << object;
                    if (connected.find(object) != connected.end()) {
                        std::vector<std::string> blacklist1 = connected.at(object);
                        std::cout << "#mesh: " << mesh;
                        //std::cout << " Set Blacklist with " << blacklist.size() << " entries.";
                        for (int i = 0; i < blacklist1.size(); i++) {
                            std::cout << " | " << blacklist1.at(i);
                            //c->addToCollisionModelBlacklist(blacklist1.at(i));
                            blacklist[c->getName()].push_back(blacklist1.at(i));
                        }
                        std::cout << std::endl;
                    }
                }
            }
        }
        //std::cout << std::endl;
    }
#endif
}

void * ZyColladaLoader::getRobotConnector() {
#ifdef SOFA_HAVE_PLUGIN_ROBOTCONNECTOR
    sout << "ZyColladaLoader::AddRobotConnectorLink" << sendl;
    std::vector<RobotConnector* > moV;
    sofa::core::objectmodel::BaseContext::GetObjectsCallBackT<RobotConnector, std::vector<RobotConnector* > > cb(&moV);

    getContext()->getObjects(TClassInfo<RobotConnector>::get(), cb, TagSet(), BaseContext::SearchRoot);
    for (std::vector<RobotConnector* >::iterator it = moV.begin(); it != moV.end(); it++)
    {
        sout << "Found a RobotConnector" << sendl;
        RobotConnector * c = (*it);
        return c;
    }
    serr << "Could not find RobotConnector" << sendl;

#else
    sout << "RobotConnector not avaiable (not built)" << sendl;
#endif
    return NULL;
}


aiVector3D ZyColladaLoader::getAxis(aiRigidBodyConstraint * rbc, double &minN, double &maxN, bool &noDof) {
    Vector3 trans;
    Quat quat;
    DecomposeAiMatrix(rbc->mObject1Transform, trans, quat);

    int i = 0;
    Vector3 res(0, 0, 0);
    if (fabs(rbc->mMin.x) > FLT_EPSILON || fabs(rbc->mMax.x) > FLT_EPSILON) {
        res.set(1, 0, 0);
        minN = rbc->mMin.x;
        maxN = rbc->mMax.x;
        i++;
    }
    if (fabs(rbc->mMin.y) > FLT_EPSILON || fabs(rbc->mMax.y) > FLT_EPSILON) {
        res.set(0, 1, 0);
        minN = rbc->mMin.y;
        maxN = rbc->mMax.y;
        i++;
    }
    if (fabs(rbc->mMin.z) > FLT_EPSILON || fabs(rbc->mMax.z) > FLT_EPSILON) {
        res.set(0, 0, 1);
        minN = rbc->mMin.z;
        maxN = rbc->mMax.z;
        i++;
    }

    std::cout << "res1: " << res;
    res = quat.rotate(res);
    std::cout << "  Res2: " << res << std::endl;

    if (i > 1) {
        serr << "Currently only one free Axis and Rotations are supported for rigid_body_constraints! "
             << rbc->mObject1.C_Str() << " - " << rbc->mObject2.C_Str() << sendl;
    }

    noDof = (i == 0);

    return aiVector3D(res.x(), res.y(), res.z());
}

void ZyColladaLoader::DecomposeAiMatrix(const aiMatrix4x4& trans, Vec3d & translation, Quat &quaternion) {
    Vec3d rotation, scale;
    DecomposeAiMatrix(trans, translation, scale, quaternion, rotation);
}

void ZyColladaLoader::DecomposeAiMatrix(const aiMatrix4x4& trans, Vec3d & translation, Vec3d& scale, Quat &quaternion, Vec3d & rotation) {
    aiVector3D aiScale, aiTranslation;
    aiQuaternion aiRotation;
    trans.Decompose(aiScale, aiRotation, aiTranslation);
    Quat q(aiRotation.x, aiRotation.y, aiRotation.z, aiRotation.w);

    translation.set(aiTranslation.x, aiTranslation.y, aiTranslation.z);
    scale.set(aiScale.x, aiScale.y, aiScale.z);
    rotation.set(q.toEulerVector() * (180.0 / M_PI));
    quaternion = q;
}


bool ZyColladaLoader::readDAE(std::ifstream &file, const char* filename)
{
    sout << "ZyColladaLoader::readDAE" << sendl;

    if (!loadScene.getValue()) {
        sout << "Scene already loaded. Break here." << sendl;
        return false;
    }

    // if a scene is already loaded with this importer, free it
    importer->FreeScene();

    // importing scene
    const aiScene* currentAiScene = importer->ReadFile(m_filename.getValue(), 0);

	if (!currentAiScene)
	{
		serr << "Collada import failed: " << importer->GetErrorString() << sendl;
		return false;
	}

    //tst Anfang
    //std::map<std::string, Assimp::Collada::Mesh*> meshesInFile;
    //tst Ende

	//std::string fileExtension(importer.GetImporterInfo(0)->mFileExtensions);
	std::cout << "=== COLLADA parser data evaluation ===" << std::endl;
	//std::cout << " fileExtension(s) = " << fileExtension << std::endl;
	//if (fileExtension.compare("dae") == 0)
	{
		std::map<std::string, std::string> kinematicModelInstances;
		std::map<std::string, std::string> kinematicModelInstancesParams;
		std::map<std::string, unsigned int> kinematicModelInstanceReferences;

		Assimp::BaseImporter* baseImporter = importer->GetImporter("dae");
		if (baseImporter != NULL)
		{
			std::cout << " Our ColladaLoader instance was retrieved successfully." << std::endl;
            Assimp::ColladaArschFotze* colladaLoader = dynamic_cast<Assimp::ColladaArschFotze*>(baseImporter);

			if (colladaLoader != NULL)
			{
                std::vector<Assimp::ColladaArschFotze::ParserStruct>& colladaParsers = colladaLoader->getParsers();

				std::cout << " Parser instances: " << colladaParsers.size() << std::endl;
				unsigned int parserCount = 0;
                for (std::vector<Assimp::ColladaArschFotze::ParserStruct>::iterator pit = colladaParsers.begin(); pit != colladaParsers.end(); ++pit)
				{
					//Assimp::ColladaLoader::ParserStruct& ps = *pit;
					Assimp::ColladaParser* parser = pit->parser;
					std::cout << " Parser " << parserCount << std::endl;
					Assimp::ColladaParser::InstanceKinematicsModelLibrary& instKinModels = parser->getInstanceKinematicsModelLibrary();
					Assimp::ColladaParser::KinematicsModelLibrary& kinModels = parser->getKinematicsModelLibrary();
					Assimp::ColladaParser::InstanceJointLibrary& instJoints = parser->getInstanceJointLibrary();
					Assimp::ColladaParser::JointLibrary& joints = parser->getJointLibrary();
					Assimp::ColladaParser::LinkLibrary& links = parser->getLinkLibrary();

                    Assimp::ColladaParser::KinematicsModelLibrary& kinModelContainers = parser->getKinematicsModelLibraryContainers();
                    // tst Anfang
                    /*Assimp::ColladaParser::MeshLibrary& meshesInFile = parser->getMeshLibrary();
                    for (Assimp::ColladaParser::MeshLibrary::iterator it = meshesInFile.begin(); it != meshesInFile.end(); ++it)
                    {
                        std::cout << "fffff: " << (*it).first << ", " << (*it).second->mId << std::endl;
                    }*/
                    // tst Ende

					
					std::cout << "  * InstanceKinematicsModel count: " << instKinModels.size() << std::endl;
					for (Assimp::ColladaParser::InstanceKinematicsModelLibrary::iterator it = instKinModels.begin(); it != instKinModels.end(); ++it)
					{
						std::cout << "   - " << it->first << ": id = " << it->second.id << " -- sid = " << it->second.sid << " -- url = " << it->second.url << std::endl;
						if (kinematicModelInstances.find(it->second.id) == kinematicModelInstances.end())
						{
							kinematicModelInstances.insert(std::make_pair(it->second.id, it->second.url));
							kinematicModelInstancesParams.insert(std::make_pair(it->second.id, it->second.param));
						}
						if (kinematicModelInstanceReferences.find(it->second.id) == kinematicModelInstanceReferences.end())
						{
							kinematicModelInstanceReferences.insert(std::make_pair(it->second.id, 1));
						}
						else
						{
							kinematicModelInstanceReferences[it->second.id] += 1;
						}
					}

					std::cout << "    ==> Model instances identified: " << kinematicModelInstances.size() << std::endl;
					for (std::map<std::string, std::string>::const_iterator nit = kinematicModelInstances.begin(); nit != kinematicModelInstances.end(); ++nit)
						std::cout << "     - " << nit->first << ": " << nit->second << ", referenced " << kinematicModelInstanceReferences[nit->first] << " times." << std::endl;

					std::cout << "  * KinematicsModel count: " << kinModels.size() << std::endl;
					for (Assimp::ColladaParser::KinematicsModelLibrary::iterator it = kinModels.begin(); it != kinModels.end(); ++it)
					{
						std::string kin_model_instance_id;
						std::cout << "   - " << it->first << ": id = " << it->second.id << " -- name = " << it->second.name << std::endl;
						/*for (std::map<std::string, std::string>::const_iterator nit = kinematicModelInstances.begin(); nit != kinematicModelInstances.end(); ++nit)
						{
							if (nit->second.compare(it->second.id) == 0)
							{
								std::cout << "    --> found matching instance_kinematics_model: " << nit->first << std::endl;
								kin_model_instance_id = nit->first;
							}
						}*/

						std::cout << "      search for identifier '" << it->second.id << "' in KinematicModels container..." << std::endl;
						std::cout << "      identifiers in KinematicModels container: ";
						for (std::map<std::string, Assimp::Collada::KinematicsModel>::const_iterator kmit = kinModelContainers.begin(); kmit != kinModelContainers.end(); ++kmit)
						{
							std::cout << it->first << ";";
						}
						std::cout << std::endl;

						if (kinModelContainers.find(it->second.id) != kinModelContainers.end())
						{
							Assimp::Collada::KinematicsModel& kin_model_container = kinModelContainers[it->second.id];
							Assimp::Collada::KinematicsModel& kin_model = kinModels[it->second.id];

							std::cout << "     --> found matching KinematicModel container; instance_joints count = " << kin_model_container.jointInstances.size() << std::endl;
							/*for (int k = 0; k < kin_model_container.jointInstances.size(); ++k)
							{
								std::string jointInstanceName = kin_model_container.jointInstances[k];
								if (jointInstanceName.at(0) == '#')
									jointInstanceName = jointInstanceName.substr(1);

								std::cout << "         * looking for joint_instance '" << jointInstanceName << "'" << std::endl;
							}*/

							std::cout << "      rootLinkName = " << kin_model.rootLinkName << std::endl;
							std::cout << "      number of post-link entries = " <<  kin_model.links.size() << ", number of pre-link entries = " << kin_model.preLinks.size() << std::endl;
							std::cout << "      ====== Parent -> child relations ======" << std::endl;

							std::string rootJoint;

							for (std::multimap<std::string, std::string>::const_iterator lit = kin_model.links.begin(); lit != kin_model.links.end(); ++lit)
							{
								std::cout << "      -> " << lit->first << " is parent of " << lit->second << std::endl;
							}
							std::cout << "      ====== Child -> parent relations ======" << std::endl;
							for (std::map<std::string, std::string>::const_iterator lit = kin_model.preLinks.begin(); lit != kin_model.preLinks.end(); ++lit)
							{
								std::cout << "      -> " << lit->first << " is child of " << lit->second << std::endl;
								if (lit->second.empty())
								{
									rootJoint = lit->first;
									std::cout << "      found rootJoint: " << rootJoint << std::endl;
								}
							}

							if (!rootJoint.empty())
							{
								d->buildJointHierarchyRec(colladaLoader, parser, kin_model, rootJoint, kinematicModelInstancesParams, m_transformHelper);

								std::cout << "================================================" << std::endl;
								std::cout << "-------- TRANSFORM HIERARCHY DUMP BEGIN --------" << std::endl;
								std::cout << "================================================" << std::endl;
								m_transformHelper->dumpJointHierarchies();
								std::cout << "================================================" << std::endl;
								std::cout << "-------- TRANSFORM HIERARCHY DUMP END   --------" << std::endl;
								std::cout << "================================================" << std::endl;
							}
						}
						else
						{
							std::cout << "    WARNING: No KinematicModel container found for " << it->second.id << std::endl;
						}
					}

					/*std::cout << "  * InstanceJoint count: " << instJoints.size() << std::endl;
					for (Assimp::ColladaParser::InstanceJointLibrary::iterator it = instJoints.begin(); it != instJoints.end(); ++it)
					{
						std::cout << "   - " << it->first << ": kinmodId = " << it->second.kinmodId << " -- kinmodName = " << it->second.kinmodName << " -- sid = " << it->second.sid << " -- url = " << it->second.url << std::endl;
					}

					std::cout << "  * Joints count: " << joints.size() << std::endl;
					for (Assimp::ColladaParser::JointLibrary::iterator it = joints.begin(); it != joints.end(); ++it)
					{
						std::cout << "   - " << it->first << ": id = " << it->second.id << " -- name = " << it->second.name << std::endl;
					}

					std::cout << "  * Links count: " << links.size() << std::endl;
					for (Assimp::ColladaParser::LinkLibrary::iterator it = links.begin(); it != links.end(); ++it)
					{
						std::cout << "   - " << it->first << ": id = " << it->second.id << " -- kinmodId = " << it->second.kinmodId << " -- kinmodName = " << it->second.kinmodName << " -- joint = " << it->second.joint << std::endl;
					}*/

					parserCount++;
				}
			}
			else
			{
				std::cout << "dynamic_cast to ColladaLoader failed! Should not happen..." << std::endl;
			}
		}
		else
		{
			std::cout << "assimp reports missing .dae support! Should not happen..." << std::endl;
		}
	}

    // asd anfang
    /*for (unsigned int fff = 0; fff < currentAiScene->mNumMeshes; ++fff)
    {
    aiMesh* currentAiMesh = currentAiScene->mMeshes[fff];
    std::cout << "all mesh names, before conv: " << currentAiMesh->mName.C_Str() << ", " << currentAiMesh->isVisualMesh << std::endl;
    }*/
    // asd ende

    Assimp::Converter converter;
    converter.collectMeshesByNameInplace(currentAiScene);

    // asd anfang
    /*for (unsigned int fff = 0; fff < currentAiScene->mNumMeshes; ++fff)
    {
    aiMesh* currentAiMesh = currentAiScene->mMeshes[fff];
    std::cout << "all mesh names, after conv: " << currentAiMesh->mName.C_Str() << ", " << currentAiMesh->isVisualMesh << std::endl;
    }*/
    // asd ende

    //Assimp::Exporter exporter;
    //exporter.Export(currentAiScene, "obj", m_filename.getValue()+".obj", 0);

    // Set Gravity according to the section <COLLADA><asset><up_axis>
    Vec3d grav;
    std::string gravName = "undefined";
    switch (currentAiScene->mUpDirection) {
    case aiScene::UP_X:
        gravName = "UP_X";
        grav.x() = -9.81;
        break;
    case aiScene::UP_Y:
        gravName = "UP_Y";
        grav.y() = -9.81;
        break;
    case aiScene::UP_Z:
        gravName = "UP_Z";
        grav.z() = -9.81;
        break;
    default:
        gravName = "Default_Y";
        grav.y() = -9.81;
        break;
    }
    Gravity::SPtr gravityObject = sofa::core::objectmodel::New<Gravity>();
    gravityObject->setName(gravName.c_str());
    gravityObject->f_gravity.setValue(grav);


    std::map<std::string, std::string> jointNameForMesh;

    // put Physics stuff in a nicer structure
    std::map<std::string, aiPhysicsModel*> physicsModelByMeshId;
    std::map<std::string, aiPhysicsModel*> physicsModelByToolId;
    std::map<std::string, aiPhysicsModel*> physicsModelBySid;
    std::map<std::string, std::vector<std::string> > meshesForPhysicsModel;
    for (unsigned int i = 0; i < currentAiScene->mNumPhysicsModel; i++) {
        aiPhysicsModel* p = currentAiScene->mPhysicsModel[i];
        std::cout << "Physic model " << i << ": " << p->mSid.C_Str() << std::endl;
        if (p->isTool) { physicsModelByToolId[p->mToolId.C_Str()] = p; } // Add to toolModelMap for faster access
        physicsModelBySid[p->mSid.C_Str()] = p;
        physicsModelByMeshId[std::string(p->mMeshId.C_Str())] = p;
        meshesForPhysicsModel[p->mSid.C_Str()].push_back(p->mMeshId.C_Str());

        for (unsigned int j = 0; j < p->mNumSubModels; j++) {
            aiPhysicsModel* sp = p->mSubModels[j];
            physicsModelByMeshId[std::string(sp->mMeshId.C_Str())] = sp;
            meshesForPhysicsModel[p->mSid.C_Str()].push_back(sp->mMeshId.C_Str());
        }
    }

    // Save links to the rigid Base Objects (root objects of physical objects)
    std::map<std::string, MechanicalObject<Rigid3dTypes>::SPtr> rigidBaseObject;
    std::map<std::string, Node::SPtr> rigidBaseNode;
    // Save the names of the above objects for later linking;
    std::vector<std::pair< std::string, RigidRigidMapping<Rigid3dTypes, Rigid3dTypes>::SPtr> > rigidBaseLinks;

    // Assign each mech name a corresponding node name
    std::map<std::string, std::string> nodeIdByMeshId;

    // Joints By Object ID
    std::map<std::string, std::vector<std::string> > meshNamesConnectedToJoint;

    // Blacklist: for
    std::map<std::string, std::vector<std::string> > blacklist;
    std::map<std::string, std::vector<std::string> > subMeshesOfMeshId;
    std::map<std::string, std::string > meshIdByPhysicsModel;



    std::cout << "First Traversion of the nodes" << std::endl;
    //first traversion for gathering info
    if (currentAiScene->mRootNode)
    {
        std::stack<aiNode*> nodes;
        nodes.push(currentAiScene->mRootNode);

        while (!nodes.empty())
        {
            aiNode* currentAiNode = nodes.top();
            nodes.pop();
            if (currentAiNode->mNumMeshes > 0) {
                // Only use first Mesh; as we make sure with collectMeshesByNameInplace that each node has only one mesh
                unsigned int meshIdx = currentAiNode->mMeshes[0];
                std::string meshName = currentAiScene->mMeshes[meshIdx]->mName.C_Str();
                std::string nodeName = currentAiNode->mId.C_Str();
                nodeIdByMeshId[meshName] = nodeName;
            }

            if (currentAiNode->mChildren > 0) {
                for (unsigned int i = 0; i < currentAiNode->mNumChildren; i++) {

                    nodes.push(currentAiNode->mChildren[i]);
                }
            }
        }
    }

    // Creating Constraint Accessors
    std::map<std::string, aiRigidBodyConstraint*> rigidBodyConstraintByObj2;
    std::map<std::string, aiRigidBodyConstraint*> rigidBodyConstraintByName;

    for (unsigned int rr = 0; rr < currentAiScene->mNumRigidBodyConstraint; rr++) {
        aiRigidBodyConstraint * rbc = currentAiScene->mRigidBodyConstraint[rr];
        rigidBodyConstraintByObj2[rbc->mObject2.C_Str()] = rbc;
        rigidBodyConstraintByName[rbc->mName.C_Str()] = rbc;
    }

    // use a map to find joints for a given object name
    std::map<std::string, JointInfo> jointInfo;
    std::map<std::string, aiJoint*> jointMap;

#ifdef SOFA_HAVE_PLUGIN_ROBOTCONNECTOR
    RobotConnector * robCon = (RobotConnector*)getRobotConnector();
#endif

    std::cout << "Traversing scene graph" << std::endl;
    // traversing the scene graph
    if (currentAiScene->mRootNode)
    {
        // use a stack to process the nodes of the scene graph in the right order, we link an assimp node with a Node with a NodeInfo
        std::stack<NodeInfo> nodes;



        subSceneRoot = getSimulation()->createNewNode("subroot");
        nodes.push(NodeInfo(currentAiScene->mRootNode, subSceneRoot));
        subSceneRoot->addObject(gravityObject);

        if (currentAiScene->HasJoints()) {
            DBG("*******************************************************\n"
                << "Found Joints. Building Joint hierarchies\n");
            Node::SPtr kinRootNode = getSimulation()->createNewNode("KinematicsModels");
            subSceneRoot->addChild(kinRootNode);


            //std::map<std::string, Node::SPtr> jointNodes;
            std::stack<aiJoint*> rootJointNodes;

            for (unsigned int i = 0; i < currentAiScene->mNumJoints; i++)
            {
                aiJoint *j = currentAiScene->mJoints[i];

                if (j)
                {
                    if (j->mPreLinkId.length == 0)
                    {
                        rootJointNodes.push(j);
                    }
                    jointByObjectName[j->mObjName.C_Str()] = j;
                    jointByJointName[j->mName.C_Str()] = j;
                    jointNameToJointID[j->mName.C_Str()] = std::string(j->mId.C_Str());
                    jointIDToJointName[j->mId.C_Str()] = std::string(j->mName.C_Str());
                }
            }

            for (int i = 0; i < currentAiScene->mNumPhysicsModel; i++)
            {
                aiPhysicsModel* pm = currentAiScene->mPhysicsModel[i];
                if (pm != NULL)
                {
                    std::string pmName(pm->mSid.C_Str());
                    physicsModelsByName[pmName] = pm;
                }
            }

            for (int i = 0; i < currentAiScene->mNumRigidBodyConstraint; i++)
            {
                aiRigidBodyConstraint* rbc = currentAiScene->mRigidBodyConstraint[i];
                if (rbc != NULL)
                {
                    std::string rbcName(rbc->mName.C_Str());
                    rbcByName[rbcName] = rbc;
                }
            }

            // Add Additional tool joints
            for (std::map<std::string, aiJoint*>::iterator it = jointByObjectName.begin(); it != jointByObjectName.end(); it++) {
                aiJoint* j = it->second;
                if (j->isToolAttached) {

                    std::string toolId = j->mToolId.C_Str();

                    if (physicsModelByToolId.find(toolId) == physicsModelByToolId.end()) {
                        std::cerr << "Could not find PhysicsModel for ToolId: '" << toolId << "'" << std::endl;
                        serr << "Could not find PhysicsModel for ToolId: '" << toolId << "'" << sendl;
                        continue;
                    }
                    aiPhysicsModel * p = physicsModelByToolId.at(toolId);
                    if (nodeIdByMeshId.find(p->mMeshId.C_Str()) == nodeIdByMeshId.end()) {
                        std::cerr << "Could not find Node for MeshName: " << p->mMeshId.C_Str() << std::endl;
                        serr << "Could not find Node for MeshName: " << p->mMeshId.C_Str() << sendl;
                        continue;
                    }


                    std::string objectName = nodeIdByMeshId.at(p->mMeshId.C_Str());

                    std::cout << "Found Tool: " << p->mToolId.C_Str() << std::endl;
                    std::cout << "  Sid: " << p->mSid.C_Str() << "    MeshId: " << p->mMeshId.C_Str() << "  ObjectName: " << objectName << std::endl;


                    std::string jointId = std::string(p->mToolId.C_Str());
                    aiJoint *nj = new aiJoint();

                    Vec3d toolTranslation;
                    sofa::defaulttype::Quat toolRotation;
                    DecomposeAiMatrix(j->mToolTransform, toolTranslation, toolRotation);
                    aiVector3D transrel = aiVector3D(toolTranslation.x(), toolTranslation.y(), toolTranslation.z());

                    nj->mId = jointId.c_str();
                    nj->mLinkId = jointId.c_str();
                    nj->mObjName = objectName;
                    nj->mNumPostLinkIds = 0;
                    nj->mType = aiJoint::REVOLUTE;
                    nj->mAxis = aiVector3D(0, 0, 0);
                    nj->mTranslation = j->mTranslation + transrel;
                    nj->mTranslationRel = transrel;
                    jointByObjectName[objectName] = nj;

                    // Find joint with fitting tool endpoint
                    for (std::map<std::string, aiJoint*>::iterator it = jointByObjectName.begin(); it != jointByObjectName.end(); it++) {
                        aiJoint *cJ = it->second;
                        if (cJ->isToolAttached && std::string(cJ->mToolId.C_Str()).compare(p->mToolId.C_Str()) == 0) {
                            //Found Endpoint ... add new joint to it
                            //TODO: This is a hack that kills all other children on this joint!
                            nj->mPreLinkId = cJ->mLinkId;
                            cJ->mNumPostLinkIds = 1;
                            cJ->mPostLinkIds = new aiString*[1];
                            cJ->mPostLinkIds[0] = new aiString(nj->mId);
                            break;
                        }
                    }

                    std::vector<std::string> subobjects;
                    // Create Tool Joints From RigidBodyContraints
                    for (unsigned int rr = 0; rr < currentAiScene->mNumRigidBodyConstraint; rr++) {
                        aiRigidBodyConstraint * rbc = currentAiScene->mRigidBodyConstraint[rr];
                        if (std::string(p->mSid.C_Str()).compare(rbc->mObject1.C_Str()) == 0) {
                            std::cout << "Found Constraint: " << rbc->mObject1.C_Str() << " -> " << rbc->mObject2.C_Str() << std::endl;
                            // Get According PhysicsModel
                            aiPhysicsModel * model = physicsModelBySid.at(rbc->mObject2.C_Str());
                            std::cout << "  Corresponding mesh: " << model->mMeshId.C_Str() << std::endl;
                            std::string sObjectName = nodeIdByMeshId.at(model->mMeshId.C_Str());
                            aiJoint *sJ = new aiJoint();
                            aiVector3D sTransRel;

                            Vec3d obj1Trans, obj2Trans;
                            Quat obj1Quat, obj2Quat;

                            DecomposeAiMatrix(rbc->mObject1Transform, obj1Trans, obj1Quat);
                            DecomposeAiMatrix(rbc->mObject2Transform, obj2Trans, obj2Quat);

                            Vec3d vs = obj1Quat.rotate(obj1Trans);
                            sTransRel.Set(vs.x(), vs.y(), vs.z());
                            bool noDof;

                            sJ->isToolJoint = true;
                            sJ->mName = rbc->mName;
                            sJ->mPreLinkId = nj->mLinkId;
                            sJ->mId = jointId + "." + std::string(rbc->mObject2.C_Str());
                            sJ->mLinkId = sJ->mId;
                            sJ->mType = rbc->mType == aiRigidBodyConstraint::ROTATION ? aiJoint::REVOLUTE : aiJoint::PRISMATIC;
                            sJ->mAxis = getAxis(rbc, sJ->mMin, sJ->mMax, noDof);
                            //if (noDof) sJ->mType = aiJoint::UNDEFINED;
                            sJ->mConstraint = rbc;

                            sJ->mObjName = sObjectName;
                            sJ->mTranslation = nj->mTranslation + sTransRel;
                            sJ->mTranslationRel = sTransRel;
                            jointByObjectName[sObjectName] = sJ;
                            subobjects.push_back(sJ->mLinkId.C_Str());
                        }
                    }

                    nj->mNumPostLinkIds = subobjects.size();
                    nj->mPostLinkIds = new aiString*[nj->mNumPostLinkIds];
                    for (unsigned int i = 0; i < subobjects.size(); i++) {
                        nj->mPostLinkIds[i] = new aiString(subobjects[i].c_str());
                    }

                }
            }

            // Print all joint names
            for (std::map<std::string, aiJoint*>::iterator it = jointByObjectName.begin(); it != jointByObjectName.end(); it++) {
                aiJoint* j = it->second;
                std::cout << "# JOINT: " << j->mLinkId.C_Str() << std::endl;
            }

            for (std::map<std::string, aiJoint*>::iterator it = jointByObjectName.begin(); it != jointByObjectName.end(); it++) {
                aiJoint* j = it->second;
                jointMap[j->mLinkId.C_Str()] = j;


                DBG("J " << j->mLinkId.C_Str() << " (" << j->mId.C_Str() << "):     " << j->mPreLinkId.C_Str() << " -> '" << j->mLinkId.C_Str() << "' -> ");
                for (int k = 0; k < j->mNumPostLinkIds; k++) DBG(j->mPostLinkIds[k]->C_Str() << ", ");
                DBG("\n     Trans: " << j->mTranslation.x << "/" << j->mTranslation.y << "/" << j->mTranslation.z << "  \tTransRel: " << j->mTranslationRel.x << "/" << j->mTranslationRel.y << "/" << j->mTranslationRel.z
                    << "      Axis: " << j->mAxis.x << j->mAxis.y << j->mAxis.z << "\n"
                    << "   ObjName: " << j->mObjName.C_Str() << "     KinematicsName:" << j->mKinematicsId.C_Str() << "\n");
            }


            while (!rootJointNodes.empty()) {
                aiJoint* root = rootJointNodes.top();
                rootJointNodes.pop();
                std::stringstream ss;
                ss << root->mKinematicsName.C_Str() << "." << root->mKinematicsId.C_Str() << "." << root->mId.C_Str();
                Node::SPtr rNode = getSimulation()->createNewNode(ss.str());
                kinRootNode->addChild(rNode);

                /* Create something like this:
                            <MechanicalObject template="Vec1d" name="ArticulatedObject"  position="0 0 0 0 0 0 0 0 0 0 0 0"  velocity="0 0 0 0 0 0 0 0 0 0 0 0"  force="0 0 0 0 0 0 0 0 0 0 0 0"  externalForce="0 0 0 0 0 0 0 0 0 0 0 0"  derivX="0 0 0 0 0 0 0 0 0 0 0 0"  free_position="0 0 0 0 0 0 0 0 0 0 0 0"  free_velocity="0 0 0 0 0 0 0 0 0 0 0 0"  rest_position="0 0 0 0 0 0 0 0 0 0 0 0"  reset_position="0 0 0 0 0 0 0 0 0 0 0 0"  restScale="1" />
                            <ArticulatedHierarchyContainer name="articulatedHierarchyContainer1"  filename="kr161.bvh" />
                            <ArticulatedHierarchyBVHController name="articulatedHierarchyBVHController1"  listening="1" />
                            <Node 	 name="6D_DOFs1"  gravity="0 -9.81 0"  dt="0.01"  time="0"  animate="0"  >
                            <MechanicalObject template="Rigid" name="6D_Dof"  position="0 0 0 0 0 0 1 0 0 0 0 0 0 1 0 0 396.923 0 0 0 1 260 -83 674.923 0 0 0 1 260 -117 1354.92 0 0 0 1 662 0 1319.92 0 0 0 1 930 70 1319.92 0 0 0 1 1076 0 1319.92 0 0 0 1 0 0 0 0 0 0 1"  velocity="0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0"  force="0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0"  externalForce="0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0"  derivX="0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0"  free_position="0 0 0 0 0 0 1 0 0 0 0 0 0 1 0 0 396.923 0 0 0 1 260 -83 674.923 0 0 0 1 260 -117 1354.92 0 0 0 1 662 0 1319.92 0 0 0 1 930 70 1319.92 0 0 0 1 1076 0 1319.92 0 0 0 1 0 0 0 0 0 0 1"  free_velocity="0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0"  rest_position="0 0 0 0 0 0 1 0 0 0 0 0 0 1 0 0 0 0 0 0 1 0 0 0 0 0 0 1 0 0 0 0 0 0 1 0 0 0 0 0 0 1 0 0 0 0 0 0 1 0 0 0 0 0 0 1 0 0 0 0 0 0 1"  reset_position="0 0 0 0 0 0 1 0 0 0 0 0 0 1 0 0 396.923 0 0 0 1 260 -83 674.923 0 0 0 1 260 -117 1354.92 0 0 0 1 662 0 1319.92 0 0 0 1 930 70 1319.92 0 0 0 1 1076 0 1319.92 0 0 0 1 0 0 0 0 0 0 1"  restScale="1" />
                            <UniformMass template="Rigid" name="uniformMass1"  mass="0.5 1 [1 0 0,0 1 0,0 0 1]"  totalmass="4.5" />
                            <ArticulatedSystemMapping template="Multi2Mapping&lt;[Vec1d,Rigid],Rigid&gt;" name="articulatedSystemMapping1"  input1="@../ArticulatedObject"  output="@6D_Dof" />
                            </Node>
                            */

                MechanicalObject<Vec1dTypes>::SPtr articulatedObject = sofa::core::objectmodel::New<MechanicalObject<Vec1dTypes> >();
                articulatedObject->setName("ArticulatedObject");
                rNode->addObject(articulatedObject);

                ArticulatedHierarchyContainer::SPtr ahc = sofa::core::objectmodel::New<ArticulatedHierarchyContainer>();
                ahc->setName("AHC");
                rNode->addObject(ahc);

                if (root->isRobot) {
                    RobotController::SPtr robotControl = sofa::core::objectmodel::New<RobotController>();
                    robotControl->setName("RobotControl");
                    robotControl->f_listening.setValue(true);
                    rNode->addObject(robotControl);

                    ArbitraryController::SPtr arbitraryControl = sofa::core::objectmodel::New<ArbitraryController>();
                    arbitraryControl->setName("ToolControl");
                    arbitraryControl->f_listening.setValue(true);
                    rNode->addObject(arbitraryControl);


                    //Add RobotController link to RobotConnector
#ifdef SOFA_HAVE_PLUGIN_ROBOTCONNECTOR
                    if (robCon) {
                        robCon->setRobotController(robotControl.get());
                        robCon->setToolController(arbitraryControl.get());
                    }
#endif
                    
                    // get min and max values and write to ArbitraryControl
                    std::vector<double> minValues;
                    std::vector<double> maxValues;
                    std::vector<bool> invertAxis;
                    std::vector<std::pair<int, std::string> > actuators;
                    std::vector<std::string> jointNames;
                    GetJointMinMaxValues(root, jointMap, minValues, maxValues, invertAxis, actuators, jointNames);
                    arbitraryControl->setMinValues(minValues);
                    arbitraryControl->setMaxValues(maxValues);
                    arbitraryControl->setInvValues(invertAxis);
                    arbitraryControl->setKinValues(minValues); // initialize on Minimum Values
                    arbitraryControl->setJointNames(jointNames);
                    arbitraryControl->initJointControlledByROS(jointNames.size());

                    // Adding Actuators
                    int minPos = -1;
                    int maxPos = -1;
#ifdef SOFA_HAVE_PLUGIN_ROBOTCONNECTOR
                    if (robCon) {

                        for (std::vector<std::pair<int, std::string>>::iterator it = actuators.begin(); it != actuators.end(); it++) {
                            int kinID = (*it).first;
                            std::string constraintName = (*it).second;
                            aiRigidBodyConstraint* c = rigidBodyConstraintByName.at(constraintName);

                            if (minPos == -1) minPos = kinID;
                            if (maxPos < kinID) maxPos = kinID;

                            robCon->addActuator(c->mObject2.C_Str(), c->mBinaryIO, kinID - minPos, c->mPos0, c->mPos1, c->mDelta_t);
                        }
                    }
#endif
                    // Sett min/max indices for tool
                    /*arbitraryControl->setControlIndex(minPos, maxPos);*/
                    arbitraryControl->setControlIndex(3, 8); // TODO: put sensible values here; this is hard-coded for Cebit Demo (Cebit 2016, in case you read this in 2017 or later)
                    //arbitraryControl->setControlIndex(3, 40); // now it's hard-coded for the SCHUNK-Demo / Automatica (2016)
                    // TODO there is a corresponding "controlIndex.setValue(Vec2i(3,8));" in ArbitraryController.h

                }
                else {

                    ArbitraryController::SPtr arbitraryControl = sofa::core::objectmodel::New<ArbitraryController>();
                    arbitraryControl->setName("ToolControl");
                    arbitraryControl->f_listening.setValue(true);
                    rNode->addObject(arbitraryControl);


                    // get min and max values and write to ArbitraryControl
                    std::vector<double> minValues;
                    std::vector<double> maxValues;
                    std::vector<bool> invertAxis;
                    std::vector<std::pair<int, std::string> > actuators;
                    std::vector < std::string > jointNames;
                    GetJointMinMaxValues(root, jointMap, minValues, maxValues, invertAxis, actuators, jointNames);
                    arbitraryControl->setMinValues(minValues);
                    arbitraryControl->setMaxValues(maxValues);
                    arbitraryControl->setInvValues(invertAxis);
                    arbitraryControl->setJointNames(jointNames);

                    for (std::size_t r = 0; r < jointNames.size(); r++)
                        std::cout << "Joint name " << r << ": "  << jointNames[r] << std::endl;

                    arbitraryControl->initJointControlledByROS(jointNames.size());
                }

                Node::SPtr node6d = getSimulation()->createNewNode("DOF");
                rNode->addChild(node6d);

                MechanicalObject<Rigid3Types>::SPtr mechanicalobject6D = sofa::core::objectmodel::New<MechanicalObject<Rigid3Types> >();
                mechanicalobject6D->setName("6D");
                node6d->addObject(mechanicalobject6D);

                /*
                            UniformMass<Rigid3dTypes, Rigid3dMass>::SPtr um = sofa::core::objectmodel::New<UniformMass<Rigid3dTypes, Rigid3dMass> >();
                            node6d->addObject(um);
                            */
                ArticulatedSystemMapping<Vec1dTypes, Rigid3dTypes, Rigid3dTypes >::SPtr as = sofa::core::objectmodel::New<ArticulatedSystemMapping<Vec1dTypes, Rigid3dTypes, Rigid3dTypes> >();
                as->setName("SystemMapping");
                as->addInputModel1(articulatedObject.get());
                as->addOutputModel(mechanicalobject6D.get());

                node6d->addObject(as);
                std::map<int, Quat> globalRot;
                std::stringstream bvh_string;

                int numAxes = CreateArticulatedJointHierarchy(bvh_string, root, jointMap, jointInfo, mechanicalobject6D.get(), globalRot);

                std::cout << "B# ArticulatedJointHierarchy" << std::endl << bvh_string.str() << std::endl;

                if (numAxes > 0) {
                    // Write BVH to file
                    std::string tmpPath = sofa::helper::system::DataRepository.getTempPath();
                    std::string tmpBVHfile = tmpPath + root->mKinematicsId.C_Str() + ".bvh";

                    DBG("Temp BVH file: " << tmpBVHfile << "\n");

                    // std::cout << bvh_string.str() << std::endl;

                    std::ofstream filePtr;
#ifndef _WIN32
                    filePtr.open(tmpBVHfile.c_str(), std::ios::trunc);
#else
                    filePtr.open(tmpBVHfile, std::ios::trunc);
#endif


                    filePtr << bvh_string.str().c_str();
                    filePtr.close();


                    // use that file for the ArticulatedHierarchyController
                    ahc->setFilename(tmpBVHfile);
                    ahc->setGlobalRotations(globalRot);
                }
                else {
                    serr << "Error generating joint hierarchy." << sendl;
                }
            }

        }
        else {
            std::cout << "No joints found.\n";
        }

        // asd anfang
        /*for (unsigned int fff = 0; fff < currentAiScene->mNumMeshes; ++fff)
        {
            aiMesh* currentAiMesh = currentAiScene->mMeshes[fff];
            std::cout << "all mesh names: " << currentAiMesh->mName.C_Str() << ", " << currentAiMesh->isCollisionMesh << std::endl;
        }*/
        // asd ende

        int meshId = 0;

        // processing each node of the scene graph
        while (!nodes.empty())
        {
            // fast access node parent pointer
            NodeInfo& currentNodeInfo = nodes.top();
            NodeInfo* parentNodeInfo = currentNodeInfo.mParentNode;
            aiNode* currentAiNode = currentNodeInfo.mAiNode;
            Node::SPtr currentNode = currentNodeInfo.mNode;
            std::size_t& childIndex = currentNodeInfo.mChildIndex;
            aiMatrix4x4& currentTransformation = currentNodeInfo.mTransformation;

            // process the node just one time
            if (0 == childIndex)
            {
                {
                    // if the aiNode contains a name do not change it because we will need it to retrieve the node when processing bones
                    std::stringstream nameStream(std::string(currentAiNode->mId.C_Str()));
                    if (nameStream.str().empty())
                        nameStream << childIndex++;
                    currentNode->setName(nameStream.str());

                    DBG("\nProcessing node: " << currentNode->getName() << std::endl);
                }

                // find Joint for current or parent node
                BaseNode* jointSearch = (BaseNode*)currentNode.get();
                std::string baseJointName = jointSearch->getName();
                bool baseJointNameFound = true;
                while (jointInfo.find(baseJointName) == jointInfo.end()) {
                    Node::Parents parents = jointSearch->getParents();
                    if (parents.empty()) {
                        baseJointNameFound = false;
                        break;
                    }
                    else {
                        DBG("Found parent joint: " << baseJointName << std::endl);
                        jointSearch = parents[0];
                        baseJointName = jointSearch->getName();
                    }
                }

                bool isBasePhysicsModel = false;
                bool isForceSensitiveTool = false;
                aiJoint* toolJoint;

                if (baseJointNameFound) {
                    DBG("  Found parent joint: " << baseJointName << std::endl);
                    if (jointByObjectName.find(currentNode->getName()) != jointByObjectName.end()) {
                        toolJoint = jointByObjectName.at(currentNode->getName());
                        if (toolJoint->isToolJoint) {
                            isForceSensitiveTool = true;
                            sout << "isForceSensitiveTool: " << baseJointName << " at " << currentNode->getName() << sendl;
                            DBG("  isForceSensitiveTool" << std::endl);
                        }
                    }
                }
                else {
                    DBG("  No connected joint found.\n");
                }

                // useful to generate a unique index for each component of a node
                int componentIndex = 0;

                // for each mesh in the node
                for (unsigned int j = 0; j < currentAiNode->mNumMeshes; ++j, ++meshId)
                {
                    std::stringstream meshNameStream;
                    meshNameStream << "rigid_" << (int)meshId;

                    Node::SPtr meshNode = getSimulation()->createNewNode(meshNameStream.str());

                    aiMesh* currentAiMesh = currentAiScene->mMeshes[currentAiNode->mMeshes[j]];

                    // Zykl.io begin
                    std::cout << "current node name: " << currentAiNode->mName.C_Str() << ", " << currentAiNode->mId.C_Str() << std::endl;
                    std::cout << "current mesh name: " << currentAiMesh->mName.C_Str() << std::endl;
                    std::cout << "mesh is visualMEsh: " << currentAiMesh->isVisualMesh << std::endl;

                    if (currentAiMesh->isVisualMesh) { continue; }
                    // Zykl.io end

                    // generating a name
                    std::string meshName(currentAiMesh->mName.data, currentAiMesh->mName.length);

                    // the node representing a part of the current mesh construction (skinning, collision, visualization ...)
                    Node::SPtr currentSubNode = meshNode;

                    rigidBaseNode[meshName] = meshNode;

                    // isForceSensitiveTool = false, if force_sensitive is set false on the corresponding RigidBodyConstraint
                    {
                        if (physicsModelByMeshId.find(meshName) != physicsModelByMeshId.end()) {
                            aiPhysicsModel* p = physicsModelByMeshId[meshName];
                            if (rigidBodyConstraintByObj2.find(p->mSid.C_Str()) != rigidBodyConstraintByObj2.end()) {
                                if (rigidBodyConstraintByObj2[p->mSid.C_Str()]->mForceSensitive == false) {
                                    isForceSensitiveTool = false;
                                }
                            }
                        }
                    }


                    MeshTransform currentMeshTransformation;
                    if (this->name == "DAE_blendfix") {
                        currentMeshTransformation.fixBlenderExport = true;
                    }
                    if (isForceSensitiveTool) { currentMeshTransformation.isForceSensitiveTool = true; }

                    bool isAttatchedToJoint = false;
                    // Create MeshTransform
                    {
                        currentMeshTransformation.meshName = meshName;
                        currentMeshTransformation.nodeID = currentNode->getName();
                        DecomposeAiMatrix(currentTransformation, currentMeshTransformation.mainTrans, currentMeshTransformation.mainScale, currentMeshTransformation.mainQuat, currentMeshTransformation.mainRot);


#if 0
                        if (this->m_transformHelper)
                        {
                            this->m_transformHelper->addObjectTransform(meshName + " -- " + currentNode->getName(),
                                                                        Vector3(currentMeshTransformation.mainTrans.x(),
                                                                                currentMeshTransformation.mainTrans.y(),
                                                                                currentMeshTransformation.mainTrans.z()),
                                                                        currentMeshTransformation.mainQuat
                                                                        );
                        }
#endif
                        if (baseJointNameFound) {
                            std::cout << "  Joint found" << std::endl;
                            isAttatchedToJoint = true;
                            JointInfo j = jointInfo.at(baseJointName);
                            currentMeshTransformation.hasJoint = true;
                            currentMeshTransformation.jointTrans.set(j.transX, j.transY, j.transZ);
                            std::cout << " B# joint Quat: " << j.quatX << " " << j.quatY << " " << j.quatZ << " " << j.quatW << std::endl;
                            currentMeshTransformation.jointQuat.set(j.quatX, j.quatY, j.quatZ, j.quatW);
                        }
                        else {
                            std::cout << "  No Joint found" << std::endl;
                        }
                        if (physicsModelByMeshId.find(meshName) != physicsModelByMeshId.end()) {
                            aiPhysicsModel* p = physicsModelByMeshId[meshName];
                            currentMeshTransformation.hasModelTranspose = true;
                            DecomposeAiMatrix(p->mTransform, currentMeshTransformation.physTrans, currentMeshTransformation.physScale, currentMeshTransformation.physQuat, currentMeshTransformation.physRot);
                            if (p->isTool) {
                                std::cout << "  Is Tool" << std::endl;
                                currentMeshTransformation.isTool = true;
                                DecomposeAiMatrix(p->mToolTransform, currentMeshTransformation.toolTrans, currentMeshTransformation.toolScale, currentMeshTransformation.toolQuat, currentMeshTransformation.toolRot);
                            }
                            else if (rigidBodyConstraintByObj2.find(p->mSid.C_Str()) != rigidBodyConstraintByObj2.end()) {



                                aiRigidBodyConstraint *rbc = rigidBodyConstraintByObj2.at(p->mSid.C_Str());
                                currentMeshTransformation.isToolAttached = true;

                                std::string physicsModelName = rbc->mObject1.C_Str();
                                std::string pMeshId = meshIdByPhysicsModel[physicsModelName];

                                std::cout << "  Attached to tool: " << physicsModelName << "mesh: " << pMeshId << std::endl;

                                // Add root mesh of Tool Attachment point to blacklist
                                blacklist[meshName].push_back(pMeshId);

                                // Add all submeshed of Tool Attachment point to blacklist
                                for (std::vector<std::string>::iterator it = subMeshesOfMeshId[pMeshId].begin(); it != subMeshesOfMeshId[pMeshId].end(); it++) {
                                    blacklist[meshName].push_back(*it);
                                }


                                DecomposeAiMatrix(rbc->mObject1Transform, currentMeshTransformation.obj1Trans, currentMeshTransformation.obj1Scale, currentMeshTransformation.obj1Quat, currentMeshTransformation.obj1Rot);
                                DecomposeAiMatrix(rbc->mObject2Transform, currentMeshTransformation.obj2Trans, currentMeshTransformation.obj2Scale, currentMeshTransformation.obj2Quat, currentMeshTransformation.obj2Rot);

                            }
                            else {
                                std::cout << "  No Tool." << std::endl;
                            }

                            if (p->mParentId.length == 0) {
                                std::cout << "  Base Physics Model." << std::endl;
                                currentMeshTransformation.isBasePhysicsModel = true;
                                isBasePhysicsModel = true;
                                currentMeshTransformation.massFrame.set(p->mMassFrame.x, p->mMassFrame.y, p->mMassFrame.z);
                            }
                            else {
                                currentMeshTransformation.massFrame.set(p->mMassFrame.x, p->mMassFrame.y, p->mMassFrame.z);
                            }

                        }

                        meshTransformations.push_back(currentMeshTransformation);
                    }



                    std::string parentMeshId;
                    bool isSubMesh = false;
                    if (physicsModelByMeshId.find(meshName) != physicsModelByMeshId.end()) {
                        aiPhysicsModel* p = physicsModelByMeshId[meshName];
                        std::cout << "  Physics Model-Transform mesh: " << p->mMeshId.C_Str() << std::endl;

                        while (p->mParentId.length > 0) {
                            parentMeshId = std::string(p->mParentId.C_Str());
                            std::cout << "    mesh: " << p->mMeshId.C_Str() << "  parent: " << parentMeshId << std::endl;
                            p = physicsModelByMeshId.at(parentMeshId);
                            isSubMesh = true;
                        }
                    }

                    if (!isSubMesh) {
                        currentNode->addChild(meshNode);
                    }

                    Quaternion firstRot;
                    bool hasAnimation = false;
                    if (currentAiScene->HasAnimations())
                    {
                        // (BE) Animation Test
                        aiNodeAnim *animation = NULL;

                        std::cout << "animation num : " << currentAiScene->mNumAnimations << std::endl;
                        for (int a = 0; a < currentAiScene->mNumAnimations; a++) {

                            aiAnimation* anim = currentAiScene->mAnimations[a];
                            std::cout << "ANIMATION Name: '" << anim->mName.C_Str() << "'" << std::endl;
                            std::cout << "animation duration : " << anim->mDuration << std::endl;
                            std::cout << "animation ticks per second : " << anim->mTicksPerSecond << std::endl;
                            std::cout << "animation channel num : " << anim->mNumChannels << std::endl;

                            for (int c = 0; c < anim->mNumChannels; c++) {
                                std::cout << "Channel:" << anim->mChannels[c]->mNodeName.C_Str() << std::endl;
                                if (currentNode->getName() == std::string(anim->mChannels[c]->mNodeName.C_Str())) {
                                    animation = anim->mChannels[c];
                                }

                                std::cout << "Translations: " << anim->mChannels[c]->mNumPositionKeys << std::endl;
                                for (int i = 0; i < anim->mChannels[c]->mNumPositionKeys; i++) {
                                    aiVectorKey key = anim->mChannels[c]->mPositionKeys[i];
                                    std::cout << "KEY " << i << ": " << key.mTime << " -> " << key.mValue.x << " " << key.mValue.y << " " << key.mValue.z << " " << std::endl;
                                }
                                std::cout << "Rotations: " << anim->mChannels[c]->mNumRotationKeys << std::endl;
                                for (int i = 0; i < anim->mChannels[c]->mNumRotationKeys; i++) {
                                    aiQuatKey key = anim->mChannels[c]->mRotationKeys[i];
                                    std::cout << "KEY " << i << ": " << key.mTime << " -> " << key.mValue.w << " " << key.mValue.x << " " << key.mValue.y << " " << key.mValue.z << " " << std::endl;
                                }

                                std::cout << "Scaling: " << anim->mChannels[c]->mNumScalingKeys << std::endl;
                                for (int i = 0; i < anim->mChannels[c]->mNumScalingKeys; i++) {
                                    aiVectorKey key = anim->mChannels[c]->mScalingKeys[i];
                                    std::cout << "KEY " << i << ": " << key.mTime << " -> " << key.mValue.x << " " << key.mValue.y << " " << key.mValue.z << " " << std::endl;
                                }
                            }
                        }

                        if (animation) {
                            std::cout << "Processing animation: " << animation->mNodeName.C_Str() << std::endl;

                            LinearMovementConstraint<Rigid3dTypes>::SPtr animationConstraint = sofa::core::objectmodel::New<LinearMovementConstraint<Rigid3dTypes> >();
                            animationConstraint->setName("AnimationDAE");
                            animationConstraint->showMovement.setValue(true);
                            Vec3d firstPos;

                            for (int i = 0; i < animation->mNumPositionKeys; i++) {
                                aiVectorKey key = animation->mPositionKeys[i];
                                Vec3d pos = Vec3d(key.mValue.x, key.mValue.y, key.mValue.z);
                                if (i == 0) {
                                    firstPos = pos;
                                }
                                Vec6d newPose(pos.x() - firstPos.x(),
                                              pos.y() - firstPos.y(), pos.z() - firstPos.z(), 0, 0, 0);

                                if (i < animation->mNumRotationKeys) {
                                    aiQuatKey rotkey = animation->mRotationKeys[i];
                                    Quaternion rotQuat(rotkey.mValue.x, rotkey.mValue.y, rotkey.mValue.z, rotkey.mValue.w);
                                    if (i == 0) {
                                        firstRot = rotQuat;
                                    }
                                    //rotQuat = firstRot.inverse() * rotQuat;

                                    std::cout << "rotQUAT:" << rotQuat << " Euler: " << rotQuat.toEulerVector() * 180 / M_PI
                                              << "  TruEuler: " << rotQuat.toTruEulerVector() * 180 / M_PI
                                              << "  NewEuler: " << rotQuat.toEulerAngles() * 180 / M_PI
                                              << std::endl;

                                    Vec3d euler = rotQuat.toTruEulerVector();
                                    newPose[3] = euler.x();
                                    newPose[4] = euler.y();
                                    newPose[5] = euler.z();
                                }


                                animationConstraint->addKeyMovement(key.mTime, Rigid3dTypes::Deriv(newPose));
                                if (animation->mNumPositionKeys == 1) { // Hackfix: LinearConstraintCorrection does not work good with just one Keyframe
                                    animationConstraint->addKeyMovement(key.mTime + 10.0, Rigid3dTypes::Deriv(newPose));
                                }
                                //std::cout << "KEY " << i << ": " << key.mTime << " -> " << key.mValue.x << " " << key.mValue.y << " " << key.mValue.z << " " << std::endl;
                            }

                            currentSubNode->addObject(animationConstraint);

                            hasAnimation = true;

                            currentMeshTransformation.isAnimatedObject = true;
                            currentMeshTransformation.animQuatInverse = firstRot.inverse();
                        }

                        std::cout << "++" << std::endl;
                    }



                    // generating a MechanicalObject and a SkinningMapping if the mesh contains bones and filling up theirs properties
                    Node::SPtr anchorNode;
                    MechanicalObject<Rigid3dTypes>::SPtr currentAnchorMechanicalObject;
                    MechanicalObject<Rigid3dTypes>::SPtr currentBaseMechanicalObject;
                    if (currentAiMesh->HasBones())
                    {
                        if (currentAiScene->mNumAnimations == 0) {
                            sout << "HasBones, bot no animations." << sendl;
                        }
                        else {
                            std::cout << "animation num : " << currentAiScene->mNumAnimations << std::endl;
                            std::cout << "animation duration : " << currentAiScene->mAnimations[0]->mDuration << std::endl;
                            std::cout << "animation ticks per second : " << currentAiScene->mAnimations[0]->mTicksPerSecond << std::endl;
                            std::cout << "animation channel num : " << currentAiScene->mAnimations[0]->mNumChannels << std::endl;
                        }
                        currentBaseMechanicalObject = sofa::core::objectmodel::New<MechanicalObject<Rigid3dTypes> >();
                        {
                            // adding the generated MechanicalObject to its parent Node
                            currentSubNode->addObject(currentBaseMechanicalObject);

                            std::stringstream nameStream(meshName);
                            if (meshName.empty())
                                nameStream << componentIndex++;
                            currentBaseMechanicalObject->setName(nameStream.str());

                            // filling up position coordinate array
                            currentBaseMechanicalObject->resize(currentAiMesh->mNumBones);

                            {
                                Data<sofa::helper::vector<Rigid3dTypes::Coord> >* d_x = currentBaseMechanicalObject->write(core::VecCoordId::position());
                                sofa::helper::vector<Rigid3dTypes::Coord> &x = *d_x->beginEdit();
                                for (unsigned int k = 0; k < currentAiMesh->mNumBones; ++k)
                                {
                                    aiMatrix4x4 offsetMatrix = currentAiMesh->mBones[k]->mOffsetMatrix;
                                    offsetMatrix.Inverse();

                                    // mesh space to world space
                                    offsetMatrix = currentTransformation * offsetMatrix;

                                    // extract the bone transformation
                                    aiVector3D aiBoneScale, aiBoneTranslation;
                                    aiQuaternion aiBoneRotation;
                                    offsetMatrix.Decompose(aiBoneScale, aiBoneRotation, aiBoneTranslation);

                                    Vec3d boneTranslation(aiBoneTranslation.x, aiBoneTranslation.y, aiBoneTranslation.z);
                                    Quaternion boneQuat(aiBoneRotation.x, aiBoneRotation.y, aiBoneRotation.z, aiBoneRotation.w);

                                    x[k] = Rigid3dTypes::Coord(boneTranslation, boneQuat);
                                }
                                d_x->endEdit();
                            }
                        }

                        if (generateCollisionModels.getValue())
                        {
                            UniformMass<Rigid3dTypes, Rigid3dMass>::SPtr currentUniformMass = sofa::core::objectmodel::New<UniformMass<Rigid3dTypes, Rigid3dMass> >();
                            {
                                // adding the generated UniformMass to its parent Node
                                currentSubNode->addObject(currentUniformMass);

                                std::stringstream nameStream(meshName);
                                if (meshName.empty())
                                    nameStream << componentIndex++;
                                currentUniformMass->setName(nameStream.str());

                                currentUniformMass->setTotalMass(80.0);
                            }
                        }


                        FixedConstraint<Rigid3dTypes>::SPtr currentFixedConstraint = sofa::core::objectmodel::New<FixedConstraint<Rigid3dTypes> >();
                        msg_info("ZyColladaLoader") << "Instantiated FixedConstraint: 1.";
                        // adding the generated FixedConstraint to its parent Node
                        currentSubNode->addObject(currentFixedConstraint);

                        std::stringstream nameStream(meshName);

                        if (meshName.empty())
                            nameStream << componentIndex++;

                        currentFixedConstraint->setName(nameStream.str());
                        currentFixedConstraint->d_fixAll.setValue(true);

                        // generating a SkeletalMotionConstraint and filling up its properties
                        SkeletalMotionConstraint<Rigid3dTypes>::SPtr currentSkeletalMotionConstraint = sofa::core::objectmodel::New<SkeletalMotionConstraint<Rigid3dTypes> >();
                        {
                            // adding the generated SkeletalMotionConstraint to its parent Node
                            currentSubNode->addObject(currentSkeletalMotionConstraint);

                            std::stringstream nameStream(meshName);
                            if (meshName.empty())
                                nameStream << componentIndex++;
                            currentSkeletalMotionConstraint->setName(nameStream.str());

                            currentSkeletalMotionConstraint->setAnimationSpeed(animationSpeed.getValue());

                            aiNode* parentAiNode = NULL;
                            if (parentNodeInfo)
                                parentAiNode = parentNodeInfo->mAiNode;

                            helper::vector<SkeletonJoint<Rigid3dTypes> > skeletonJoints;
                            helper::vector<SkeletonBone> skeletonBones;
                            fillSkeletalInfo(currentAiScene, parentAiNode, currentAiNode, currentTransformation, currentAiMesh, skeletonJoints, skeletonBones);
                            currentSkeletalMotionConstraint->setSkeletalMotion(skeletonJoints, skeletonBones);
                        }
                    }
                    else
                    {
                        MechanicalObject<Vec3Types>::SPtr p1, p2, p3, p4, l1, l2, l3, l4;
                        Vec3d VecP(0, 0, 0);

                        if (isForceSensitiveTool)
                        {  // Create Anchor Object
                            currentAnchorMechanicalObject = sofa::core::objectmodel::New<MechanicalObject<Rigid3dTypes> >();
                            anchorNode = getSimulation()->createNewNode("Anchor");
                            currentNode->addChild(anchorNode);
                            currentAnchorMechanicalObject->setName("Anchor_" + meshName);

                            // filling up position coordinate array
                            currentAnchorMechanicalObject->resize(1);

                            {  // Fill with same data as currentBaseMechanicalObject
                                Data<sofa::helper::vector<Rigid3dTypes::Coord> >* d_x = currentAnchorMechanicalObject->write(core::VecCoordId::position());
                                sofa::helper::vector<Rigid3dTypes::Coord> &x = *d_x->beginEdit();

                                currentMeshTransformation.isForceSensitiveTool = false; // Hack to get relative offset here

                                Vec3d baseT = currentMeshTransformation.getBaseT();
                                Quat baseRQ = currentMeshTransformation.getBaseRQ();

                                currentMeshTransformation.isForceSensitiveTool = true;

                                x[0] = Rigid3dTypes::Coord(baseT, baseRQ);

                                d_x->endEdit();
                            }

                            anchorNode->addObject(currentAnchorMechanicalObject);

                            // Kill the Gravity (does not work!)
                            Gravity::SPtr noGravity = sofa::core::objectmodel::New<Gravity>();
                            noGravity->setName("noGravity");
                            noGravity->f_gravity.setValue(Vec3d(0, 0, 0));
                            currentNode->addObject(noGravity);

                            // Add Lines for Sliding Constraints
                            if (toolJoint->mType == aiJoint::PRISMATIC) {
                                aiRigidBodyConstraint *rbc = toolJoint->mConstraint;
                                Vec3d from(rbc->mMin.x, rbc->mMin.y, rbc->mMin.z);
                                Vec3d to(rbc->mMax.x, rbc->mMax.y, rbc->mMax.z);

                                // Make sure, p1 is (0,0,0). This helps making the Constraints stable
                                VecP = -from;
                                from += VecP;
                                to += VecP;

                                Vec3d Xt = Vec3d(10, 0, 0);
                                Vec3d Yt = Vec3d(0, 10, 0);
                                Vec3d Zt = Vec3d(0, 0, 10);

                                Vec3d fromX = from + Xt; // X Translation
                                Vec3d toX = to + Xt;
                                Vec3d fromY = from + Yt; // Y Translation
                                Vec3d toY = to + Yt;
                                Vec3d fromZ = from + Zt; // Z Translation
                                Vec3d toZ = to + Zt;

                                l1 = AddSlidingLine(0, currentAnchorMechanicalObject, anchorNode, &from, &to);
                                l2 = AddSlidingLine(1, currentAnchorMechanicalObject, anchorNode, &fromX, &toX);
                                l3 = AddSlidingLine(2, currentAnchorMechanicalObject, anchorNode, &fromY, &toY);
                                l4 = AddSlidingLine(3, currentAnchorMechanicalObject, anchorNode, &fromZ, &toZ);

                            }
                            else if (toolJoint->mType == aiJoint::REVOLUTE)
                            {
                                serr << "aiJoint::REVOLUTE NOT IMPLEMENTED for tools" << sendl;
                            }

                        }

                        if (!isSubMesh)
                        {
                            currentBaseMechanicalObject = sofa::core::objectmodel::New<MechanicalObject<Rigid3dTypes> >();
                            {
                                // adding the generated MechanicalObject to its parent Node
                                currentSubNode->addObject(currentBaseMechanicalObject);

                                std::stringstream nameStream;
                                nameStream << "Base_" << meshName;
                                currentBaseMechanicalObject->setName(nameStream.str());

                                // filling up position coordinate array
                                currentBaseMechanicalObject->resize(1);

                                {
                                    Data<sofa::helper::vector<Rigid3dTypes::Coord> >* d_x = currentBaseMechanicalObject->write(core::VecCoordId::position());
                                    sofa::helper::vector<Rigid3dTypes::Coord> &x = *d_x->beginEdit();

                                    Vec3d baseT = currentMeshTransformation.getBaseT();
                                    Quat baseRQ = currentMeshTransformation.getBaseRQ();

                                    x[0] = Rigid3dTypes::Coord(baseT, baseRQ);

                                    d_x->endEdit();
                                }
                            }
                        }

                        if (isForceSensitiveTool) {  // Create Constraint Points and Sliding Constraints
                            if (toolJoint->mType == aiJoint::PRISMATIC) {
                                aiRigidBodyConstraint *rbc = toolJoint->mConstraint;
                                Vec3d from(rbc->mMin.x, rbc->mMin.y, rbc->mMin.z);
                                Vec3d Xt = Vec3d(10, 0, 0);
                                Vec3d Yt = Vec3d(0, 10, 0);
                                Vec3d Zt = Vec3d(0, 0, 10);

                                from += VecP; // Make sure, p1 is (0,0,0)

                                Vec3d fromX = from + Xt; // X Translation
                                Vec3d fromY = from + Yt; // Y Translation
                                Vec3d fromZ = from + Zt; // Z Translation

                                // Add "Anchor" Points for the Sliding Constraint. p1=(0,0,0)
                                p1 = AddSlidingLine(0, currentBaseMechanicalObject, currentSubNode, &from);
                                p2 = AddSlidingLine(1, currentBaseMechanicalObject, currentSubNode, &fromX);
                                p3 = AddSlidingLine(2, currentBaseMechanicalObject, currentSubNode, &fromY);
                                p4 = AddSlidingLine(3, currentBaseMechanicalObject, currentSubNode, &fromZ);

                                // Add Sliding Constaints for each point, connecting the anchor points and the line where the respective points are on.
                                SlidingConstraint<Vec3dTypes>::SPtr sl1 = sofa::core::objectmodel::New<SlidingConstraint<Vec3dTypes> >(p1.get(), l1.get());
                                sl1.get()->setName("Slider_1");
                                sl1.get()->setM2b(1);
                                anchorNode.get()->addObject(sl1);

                                SlidingConstraint<Vec3dTypes>::SPtr sl2 = sofa::core::objectmodel::New<SlidingConstraint<Vec3dTypes> >(p2.get(), l2.get());
                                sl2.get()->setName("Slider_2");
                                sl2.get()->setM2b(1);
                                anchorNode.get()->addObject(sl2);

                                SlidingConstraint<Vec3dTypes>::SPtr sl3 = sofa::core::objectmodel::New<SlidingConstraint<Vec3dTypes> >(p3.get(), l3.get());
                                sl3.get()->setName("Slider_3");
                                sl3.get()->setM2b(1);
                                anchorNode.get()->addObject(sl3);

                                SlidingConstraint<Vec3dTypes>::SPtr sl4 = sofa::core::objectmodel::New<SlidingConstraint<Vec3dTypes> >(p4.get(), l4.get());
                                sl4.get()->setName("Slider_4");
                                sl4.get()->setM2b(1);
                                anchorNode.get()->addObject(sl4);

                            }
                            else if (toolJoint->mType == aiJoint::REVOLUTE) {
                                serr << "aiJoint::REVOLUTE NOT IMPLEMENTED for tools" << sendl;
                            }
                        }

                        bool hasFixedConstraint = false;

                        if (!isSubMesh)
                        {
                            bool isFixedObject = false;
                            // Add Uniform Mass object, if the PhysicsModel has an entry for this geometry and if dynamic is true.
                            /*for (std::map<std::string, aiPhysicsModel*>::iterator bla = physicsModelByMeshId.begin(); bla == physicsModelByMeshId.end(); bla++)
                            {
                                std::cout << (*bla).first << std::endl;
                            }*/
                            if (physicsModelByMeshId.find(meshName) != physicsModelByMeshId.end())
                            {
                                aiPhysicsModel* p = physicsModelByMeshId[meshName];
                                /*std::cout << "dumdidum " << p->mMeshId.C_Str() << ", " << p->mSid.C_Str() << ", " << p->mParentId.C_Str() << ", " << std::endl;*/
                                if (p->dynamic) {
                                    std::cout << "  Generating physics model for: " << p->mMeshId.C_Str() << std::endl;

                                    UniformMass<Rigid3dTypes, Rigid3dMass>::SPtr currentUniformMass = sofa::core::objectmodel::New<UniformMass<Rigid3dTypes, Rigid3dMass> >();
                                    currentUniformMass->setName(meshName);
                                    currentUniformMass->setTotalMass(p->mass);

                                    // Zyklio TODO: This used to be setRayleighMass!
									// currentUniformMass->setMass(1.0);

                                    if (isAttatchedToJoint && !isForceSensitiveTool)
                                    {
                                        //currentUniformMass->setTotalMass(9999999.99);
                                    }

                                    if (isForceSensitiveTool)
                                    {
                                        //	currentUniformMass->setTotalMass(0.00001); // Minimal mass -> maximum control ... object doesnt lag behind.
                                    }
                                    currentUniformMass->d_showAxisSize.setValue(5);
                                    currentSubNode->addObject(currentUniformMass);

#ifdef SOFA_HAVE_PLUGIN_ROBOTCONNECTOR
                                    if (robCon) {
                                        if (physicsModelByMeshId.find(meshName) != physicsModelByMeshId.end()) {
                                            aiPhysicsModel* p = physicsModelByMeshId[meshName];
                                            std::cout << "  physics model: " << p->mSid.C_Str() << std::endl;
                                            meshIdByPhysicsModel[p->mSid.C_Str()] = meshName;

                                            if (std::string(p->mSid.C_Str()).empty()) {

                                            }
                                            else {
                                                robCon->setMechanicalObject(p->mSid.C_Str(), currentBaseMechanicalObject);
                                            }
                                        }
                                    }
#endif
                                }
                                else
                                {

                                    UniformMass<Rigid3dTypes, Rigid3dMass>::SPtr currentUniformMass = sofa::core::objectmodel::New<UniformMass<Rigid3dTypes, Rigid3dMass> >();
                                    {
                                        // adding the generated UniformMass to its parent Node
                                        currentSubNode->addObject(currentUniformMass);

                                        std::stringstream nameStream(meshName);
                                        if (meshName.empty())
                                            nameStream << componentIndex++;
                                        currentUniformMass->setName(nameStream.str());

                                        currentUniformMass->setTotalMass(9999999.99);
                                        //currentUniformMass->setTotalMass(1);
                                    }


                                    std::cout << "  Not a dynamic physics model: " << p->mMeshId.C_Str() << std::endl;
                                    isFixedObject = true;
                                }
                            }
                            else
                            {
                                std::cout << "  No physics model for: " << meshName.c_str() << std::endl;
                                isFixedObject = true;
                            }

                            if ((!hasAnimation && isFixedObject) || (isAttatchedToJoint && !isForceSensitiveTool))
                            {
                                FixedConstraint<Rigid3dTypes>::SPtr currentFixedConstraint = sofa::core::objectmodel::New<FixedConstraint<Rigid3dTypes> >();
                                msg_info("ZyColladaLoader") << "Instantiated FixedConstraint: 2.";
                                // adding the generated FixedConstraint to its parent Node
                                currentSubNode->addObject(currentFixedConstraint);
                                hasFixedConstraint = true;

                                std::stringstream nameStream(meshName);
                                if (meshName.empty())
                                    nameStream << componentIndex++;

                                currentFixedConstraint->setName(nameStream.str());
                                currentFixedConstraint->d_fixAll.setValue(true);
                            }

                        }

                        if (isForceSensitiveTool)
                        {
                            // Apply RigidRigidMapping to Anchor if it is a Tool Joint
                            JointInfo ji = jointInfo.at(baseJointName);
                            RigidRigidMapping<Rigid3dTypes, Rigid3dTypes>::SPtr rigidRigidMapping = sofa::core::objectmodel::New<RigidRigidMapping<Rigid3dTypes, Rigid3dTypes> >();
                            rigidRigidMapping->setModels(ji.link6D, currentAnchorMechanicalObject.get());
                            rigidRigidMapping->index.setValue(ji.index);
                            rigidRigidMapping->setConstraintsMapped(false);

                            std::stringstream nameStream;
                            nameStream << "Kinematics_" << ji.mName;
                            rigidRigidMapping->setName(nameStream.str());

                            anchorNode->addObject(rigidRigidMapping);
                        }
                        else if (baseJointNameFound)
                        {
                            // Apply RigidRigidMapping if Joint is present for current node
                            JointInfo ji = jointInfo.at(baseJointName);
                            // <RigidRigidMapping template="Rigid,Rigid" name="rigidRigidMap4"  initialPoints="0 0 0 0 0 0 1"  index="4"  globalToLocalCoords="1"  input="@articulatedObject1/6D_DOFs1/6D_Dof"  output="@." />
                            RigidRigidMapping<Rigid3dTypes, Rigid3dTypes>::SPtr rigidRigidMapping = sofa::core::objectmodel::New<RigidRigidMapping<Rigid3dTypes, Rigid3dTypes> >();
                            rigidRigidMapping->setModels(ji.link6D, currentBaseMechanicalObject.get());
                            rigidRigidMapping->index.setValue(ji.index);
                            rigidRigidMapping->setConstraintsMapped(false);

                            std::stringstream nameStream;
                            nameStream << "Kinematics_" << ji.mName;
                            rigidRigidMapping->setName(nameStream.str());

							// FA: multTest for Schunk Demo with SVH hand
							// rigidRigidMapping->globalToLocalCoords.setValue(true);

                            currentSubNode->addObject(rigidRigidMapping);
                        }

                        if (generateCollisionModels.getValue() > 0 && !isSubMesh)
                        {
#ifdef SOFA_HAVE_PLUGIN_OBBTREEGPU
                            // Create something like that:
                            /*
                                            *	<CGLinearSolver template="GraphScattered" name="cGLinearSolver" />
                                            *	<EulerImplicitSolver name="eulerImplicitSolver" />
                                            *	<PrecomputedConstraintCorrection template="Rigid" name="precomputedConstraintCorrection" />
                                            */
                            if (!hasFixedConstraint)
                            {

                                CGLinearSolver<GraphScatteredMatrix, GraphScatteredVector>::SPtr cgLinearSolver = sofa::core::objectmodel::New<CGLinearSolver<GraphScatteredMatrix, GraphScatteredVector> >();
                                {
                                    cgLinearSolver->setName("ls");
                                    currentSubNode->addObject(cgLinearSolver);
                                }


                                EulerImplicitSolver::SPtr eulerImplicit = sofa::core::objectmodel::New<EulerImplicitSolver>();
                                {
                                    eulerImplicit->setName("ei");
                                    currentSubNode->addObject(eulerImplicit);
                                }

                            }
                            //PrecomputedConstraintCorrection<defaulttype::Rigid3dTypes>::SPtr constraintCorrection = sofa::core::objectmodel::New<PrecomputedConstraintCorrection<defaulttype::Rigid3dTypes> >();
                            UncoupledConstraintCorrection<defaulttype::Rigid3dTypes>::SPtr constraintCorrection = sofa::core::objectmodel::New<UncoupledConstraintCorrection<defaulttype::Rigid3dTypes> >();
                            {
                                constraintCorrection->setName("cc_" + meshName);
                                currentSubNode->addObject(constraintCorrection);
                            }


                            if (isForceSensitiveTool) {
                                // Force Field
                                ConstantForceField<Rigid3dTypes>::SPtr forceField = sofa::core::objectmodel::New<ConstantForceField<Rigid3dTypes> >();
                                forceField.get()->setName("toolForce");
                                Vec6d f0(0, 0, 0, 0, 0, 0);
                                forceField.get()->setForce(0, Rigid3dTypes::Deriv(f0));
                                forceField.get()->setUseLocal(true);
                                currentSubNode.get()->addObject(forceField);
                            }
#endif
                        }

                        /*
                        FixedConstraint<Rigid3dTypes>::SPtr currentFixedConstraint = sofa::core::objectmodel::New<FixedConstraint<Rigid3dTypes> >();
                        {
                        // adding the generated FixedConstraint to its parent Node
                        currentSubNode->addObject(currentFixedConstraint);

                        std::stringstream nameStream(meshName);
                        if(meshName.empty())
                        nameStream << componentIndex++;
                        currentFixedConstraint->setName(nameStream.str());

                        currentFixedConstraint->f_fixAll.setValue(true);
                        }
                        */
                    }

                    std::stringstream rigidNameStream;
                    if (currentAiMesh->HasBones())
                        rigidNameStream << "skinning_" << meshName << "_" << (int)meshId;
                    else
                        rigidNameStream << "mesh_" << meshName << "_" << (int) meshId;

                    Node::SPtr rigidNode = getSimulation()->createNewNode(rigidNameStream.str());

                    if (isSubMesh)
                    {
                        std::cout << "Is SubMesh of " << parentMeshId << std::endl;
                        subMeshesOfMeshId[parentMeshId].push_back(meshName);

                        if (rigidBaseNode.find(parentMeshId) == rigidBaseNode.end())
                        {
                            serr << "Missing Mesh node: " << parentMeshId << sendl;
                        }
                        else
                        {
                            rigidBaseNode[parentMeshId]->addChild(rigidNode);
                            blacklist[parentMeshId].push_back(meshName);
                            for (std::vector<std::string>::iterator it = blacklist[parentMeshId].begin(); it != blacklist[parentMeshId].end(); it++) {
                                if ((*it) != meshName) {
                                    blacklist[meshName].push_back(*it);
                                }
                            }

                        }
                    }
                    else
                    {
                        meshNode->addChild(rigidNode);
                    }
                    currentSubNode = rigidNode;


                    // generating a MeshTopology and filling up its properties
                    MeshTopology::SPtr currentMeshTopology = sofa::core::objectmodel::New<MeshTopology>();
                    {
                        // adding the generated MeshTopology to its parent Node
                        currentSubNode->addObject(currentMeshTopology);

                        std::stringstream nameStream(meshName);
                        if (meshName.empty())
                            nameStream << componentIndex++;
                        currentMeshTopology->setName(nameStream.str());

                        // filling up position array
                        //currentMeshTopology->seqPoints.setParent(&currentMechanicalObject->x);
                        // filling up position coordinate array
                        if (0 != currentAiMesh->mNumVertices)
                        {
                            helper::vector<defaulttype::Vec<3, SReal> >& x = *(currentMeshTopology->seqPoints.beginEdit());

                            std::cout << "  New Transformation " << currentNode->getName() << "  " << meshName << std::endl;
                            for (unsigned int k = 0; k < currentAiMesh->mNumVertices; ++k) {
                                Vec3d vertex = Vec3d(currentAiMesh->mVertices[k][0], currentAiMesh->mVertices[k][1], currentAiMesh->mVertices[k][2]);
                                x.push_back(currentMeshTransformation.VertexTransform(vertex));
                            }

                            currentMeshTopology->seqPoints.endEdit();
                        }

                        // filling up triangle array
                        sofa::helper::vector<sofa::core::topology::Triangle> triangles;
                        unsigned int numTriangles = 0;
                        for (unsigned int k = 0; k < currentAiMesh->mNumFaces; ++k)
                            if (3 == currentAiMesh->mFaces[k].mNumIndices)
                                ++numTriangles;

                        if (0 != numTriangles)
                        {
                            triangles.resize(numTriangles);

                            unsigned int triangleOffset = 0;
#ifdef IMAGE_CONTAINER_SUPPORT
                            SReal vsize = this->voxelSize.getValue();
                            // rasterized mesh
                            Node::SPtr labelNode = currentSubNode->createChild("label");
                            engine::MeshToImageEngine<defaulttype::ImageB>::SPtr M2I = sofa::core::objectmodel::New<engine::MeshToImageEngine<defaulttype::ImageB> >();
                            M2I->setName( "rasterizer" );
                            M2I->voxelSize.setValue( vector<SReal>(1,vsize) );
                            M2I->padSize.setValue(2);
                            M2I->rotateImage.setValue(false);
                            M2I->f_nbMeshes.setValue(1);
                            //                            M2I->createInputMeshesData();
                            M2I->backgroundValue.setValue(0);
                            engine::MeshToImageEngine<defaulttype::ImageB>::SeqValues values(1,1);
                            (*M2I->vf_values[0]).setValue(values);
                            (*M2I->vf_positions[0]).setParent( &currentMechanicalObject->x );
                            (*M2I->vf_triangles[0]).setParent( &currentMeshTopology->seqTriangles );
                            labelNode->addObject(M2I);

                            ImageContainer<defaulttype::ImageB>::SPtr IC0 = sofa::core::objectmodel::New<ImageContainer<defaulttype::ImageB> >();
                            IC0->setName( "image" );
                            IC0->image.setParent(&M2I->image);
                            IC0->transform.setParent(&M2I->transform);
                            labelNode->addObject(IC0);

                            //                            misc::ImageViewer<defaulttype::ImageB>::SPtr IV0 = sofa::core::objectmodel::New<misc::ImageViewer<defaulttype::ImageB> >();
                            //                            IV0->setName( "viewer" );
                            //                            IV0->image.setParent( &M2I->image );
                            //                            IV0->transform.setParent( &M2I->transform );
                            //                            labelNode->addObject(IV0);
#endif
                            for (unsigned int k = 0; k < currentAiMesh->mNumFaces; ++k)
                            {
                                if (3 != currentAiMesh->mFaces[k].mNumIndices)
                                    continue;

                                memcpy(&triangles[0] + triangleOffset, currentAiMesh->mFaces[k].mIndices, sizeof(sofa::core::topology::Triangle));
                                ++triangleOffset;
                            }

                            {
                                sofa::helper::vector<sofa::core::topology::Triangle>& seqTriangles = *currentMeshTopology->seqTriangles.beginEdit();
                                seqTriangles.reserve(triangles.size());

                                for (unsigned int k = 0; k < triangles.size(); ++k)
                                    seqTriangles.push_back(triangles[k]);
                            }

                            /*		  // code makes problems, when there are no boness
                            // rasterized weights on surface
                            for(unsigned int k = 0; k < currentAiMesh->mNumFaces; ++k)
                            {
                            for( unsigned int b = 0 ; b < currentAiMesh->mNumBones; ++b )
                            {
                            if(3 != currentAiMesh->mFaces[k].mNumIndices)
                            continue;

                            memcpy(&triangles[0] + triangleOffset, currentAiMesh->mFaces[k].mIndices, sizeof(topology::Triangle));
                            ++triangleOffset;
                            }
                            }

                            {
                            vector<topology::Triangle>& seqTriangles = *currentMeshTopology->seqTriangles.beginEdit();
                            seqTriangles.reserve(triangles.size());

                            for(unsigned int k = 0; k < triangles.size(); ++k)
                            seqTriangles.push_back(triangles[k]);

                            currentMeshTopology->seqPoints.endEdit();
                            }
                            */
                        }

#ifdef IMAGE_CONTAINER_SUPPORT
                        engine::MeshToImageEngine<defaulttype::ImageD>::SPtr M2I = sofa::core::objectmodel::New<engine::MeshToImageEngine<defaulttype::ImageD> >();
                        M2I->setName( "rasterizer" );
                        M2I->voxelSize.setValue( vector<SReal>(1,vsize) );
                        M2I->padSize.setValue(2);
                        M2I->rotateImage.setValue(false);
                        M2I->f_nbMeshes.setValue(1);
                        //M2I->createInputMeshesData();
#endif
                        // filling up quad array
                        sofa::helper::vector<sofa::core::topology::Quad> quads;
                        unsigned int numQuads = 0;
                        for (unsigned int k = 0; k < currentAiMesh->mNumFaces; ++k)
                            if (4 == currentAiMesh->mFaces[k].mNumIndices)
                                ++numQuads;

                        if (0 != numQuads)
                        {
                            quads.resize(numQuads);

                            unsigned int quadOffset = 0;
                            for (unsigned int k = 0; k < currentAiMesh->mNumFaces; ++k)
                            {
                                if (4 != currentAiMesh->mFaces[k].mNumIndices)
                                    continue;

                                memcpy(&quads[0] + quadOffset, currentAiMesh->mFaces[k].mIndices, sizeof(sofa::core::topology::Quad));
                                ++quadOffset;
                            }

                            {
                                sofa::helper::vector<sofa::core::topology::Quad>& seqQuads = *currentMeshTopology->seqQuads.beginEdit();
                                seqQuads.reserve(quads.size());

                                for (unsigned int k = 0; k < quads.size(); ++k)
                                    seqQuads.push_back(quads[k]);
                            }
                        }
                    }                    

                    // generating a second MechanicalObject and filling up its properties
                    MechanicalObject<Vec3dTypes>::SPtr currentTransMechanicalObject = sofa::core::objectmodel::New<MechanicalObject<Vec3dTypes> >();
                    {
                        // adding the generated MechanicalObject to its parent Node
                        currentSubNode->addObject(currentTransMechanicalObject);

                        std::stringstream nameStream(meshName);
                        if (meshName.empty())
                            nameStream << componentIndex++;

                        nameStream << "Trans_" << nameStream.str();
                        currentTransMechanicalObject->setName(nameStream.str());

                        // Create RigidRigid Mappings for subShapes of PhysicalModels

                        if (physicsModelByMeshId.find(meshName) != physicsModelByMeshId.end()) {
                            aiPhysicsModel* p = physicsModelByMeshId[meshName];
                            std::cout << "  Physics Model-Transform mesh: " << p->mMeshId.C_Str() << std::endl;

                            if (p->mParentId.length > 0) {

                                // Get Parent ID and Parent
                                //parentMeshId = std::string(p->mParentId.C_Str());
                                aiPhysicsModel* parent = physicsModelByMeshId[parentMeshId];

                                // RigidRigidMapping to the base physical Object.
                                /*
                                            RigidRigidMapping<Rigid3dTypes, Rigid3dTypes>::SPtr rigidRigidMapping = sofa::core::objectmodel::New<RigidRigidMapping<Rigid3dTypes, Rigid3dTypes> >();
                                            rigidRigidMapping->setModels(NULL, currentBaseMechanicalObject.get());
                                            std::stringstream nameStream;
                                            nameStream << "PhysicsBase_" << parentMeshId;
                                            rigidRigidMapping->setName(nameStream.str());
                                            meshNode->addObject(rigidRigidMapping);
                                            */


                                if (rigidBaseObject.find(parentMeshId) == rigidBaseObject.end()) {
                                    serr << "The node " << nodeIdByMeshId.at(parentMeshId) << " is instanciated after its child object: " << nodeIdByMeshId.at(currentMeshTransformation.meshName) << ". ";
                                    serr << "This will cause a behaviour where one element will update the position one timestep too late. ";
                                    serr << "Please change the order in the section library_visual_scenes of the collada file. (First " << nodeIdByMeshId.at(currentMeshTransformation.meshName) << " then " << nodeIdByMeshId.at(parentMeshId) << "." << sendl;
                                }

                                // rigidBaseLinks.push_back(std::pair<std::string, RigidRigidMapping<Rigid3dTypes, Rigid3dTypes>::SPtr>(parentMeshId, rigidRigidMapping));
                            }
                            else {
                                // Save Mechanical object for later use of siblings
                                rigidBaseObject[std::string(p->mMeshId.C_Str())] = currentBaseMechanicalObject;
                            }
                        }


                        Vec3d baseT = currentMeshTransformation.getBaseT();
                        Quat baseRQ = currentMeshTransformation.getBaseRQ();
                        Vec3d transT = currentMeshTransformation.getTransT();
                        Quat transRQ = currentMeshTransformation.getTransRQ();

                        //  currentBaseMechanicalObject->setTranslation(baseT.x(), baseT.y(), baseT.z());
                        //  currentBaseMechanicalObject->setRotationQuat(baseRQ);

                        currentTransMechanicalObject->setTranslation(transT.x(), transT.y(), transT.z());
                        currentTransMechanicalObject->setRotationQuat(transRQ);

                        std::cout << "  baseT: " << baseT.x() << " " << baseT.y() << " " << baseT.z() << std::endl;
                        std::cout << "  transT: " << transT.x() << " " << transT.y() << " " << transT.z() << std::endl;
                    }


                    //					std::string xnodeName(currentAiNode->mName.data);
                    //					std::string xmeshName(currentAiMesh->mName.data);
                    //					std::cout << "nodeName: " << xnodeName << std::endl;
                    //					std::cout << " - meshName: " << xmeshName << std::endl;
                    //					std::cout << std::endl;
                    if (currentAiMesh->HasBones())
                    {
#ifdef SOFA_HAVE_PLUGIN_FLEXIBLE
                        LinearMapping<Rigid3dTypes, Vec3dTypes>::SPtr currentLinearMapping = sofa::core::objectmodel::New<LinearMapping<Rigid3dTypes, Vec3dTypes> >();
                        {
                            // adding the generated LinearMapping to its parent Node
                            currentSubNode->addObject(currentLinearMapping);

                            std::stringstream nameStream(meshName);
                            if(meshName.empty())
                                nameStream << componentIndex++;
                            currentLinearMapping->setName(nameStream.str());

                            currentLinearMapping->setModels(currentBoneMechanicalObject.get(), currentMechanicalObject.get());

                            vector<LinearMapping<Rigid3dTypes, Vec3dTypes>::VReal> weights;
                            vector<LinearMapping<Rigid3dTypes, Vec3dTypes>::VRef> indices;

                            indices.resize(currentAiMesh->mNumVertices);
                            weights.resize(currentAiMesh->mNumVertices);

                            size_t nbref = currentAiMesh->mNumBones;

                            for(std::size_t i = 0; i < indices.size(); ++i)
                            {
                                indices[i].reserve(nbref);
                                weights[i].reserve(nbref);
                            }

                            for(unsigned int k = 0; k < currentAiMesh->mNumBones; ++k)
                            {
                                aiBone*& bone = currentAiMesh->mBones[k];

                                for(unsigned int l = 0; l < bone->mNumWeights; ++l)
                                {
                                    unsigned int id = bone->mWeights[l].mVertexId;
                                    float weight = bone->mWeights[l].mWeight;

                                    if(id >= currentAiMesh->mNumVertices)
                                    {
                                        sout << "Error: ZyColladaLoader::readDAE, a mesh could not be load : " << nameStream.str() << " - in node : " << currentNode->getName() << sendl;
                                        return false;
                                    }

                                    indices[id].push_back(k);
                                    weights[id].push_back(weight);
                                }
                            }
                            currentLinearMapping->setWeights(weights, indices);
                        }
#else
                        SkinningMapping<Rigid3dTypes, Vec3dTypes>::SPtr currentSkinningMapping = sofa::core::objectmodel::New<SkinningMapping<Rigid3dTypes, Vec3dTypes> >();
                        {
                            // adding the generated SkinningMapping to its parent Node
                            currentSubNode->addObject(currentSkinningMapping);

                            std::stringstream nameStream(meshName);
                            if (meshName.empty())
                                nameStream << componentIndex++;

                            currentSkinningMapping->setName(nameStream.str());

                            currentSkinningMapping->setModels(currentBaseMechanicalObject.get(), currentTransMechanicalObject.get());

                            sofa::helper::vector<sofa::helper::SVector<SkinningMapping<Rigid3dTypes, Vec3dTypes>::InReal> > weights;
                            sofa::helper::vector<sofa::helper::SVector<unsigned int> > indices;
                            sofa::helper::vector<unsigned int> nbref;

                            indices.resize(currentAiMesh->mNumVertices);
                            weights.resize(currentAiMesh->mNumVertices);
                            nbref.resize(currentAiMesh->mNumVertices);
                            for (unsigned int k = 0; k < nbref.size(); ++k)
                                nbref[k] = 0;

                            for (unsigned int k = 0; k < currentAiMesh->mNumBones; ++k)
                            {
                                aiBone*& bone = currentAiMesh->mBones[k];

                                for (unsigned int l = 0; l < bone->mNumWeights; ++l)
                                {
                                    unsigned int id = bone->mWeights[l].mVertexId;
                                    float weight = bone->mWeights[l].mWeight;

                                    if (id >= currentAiMesh->mNumVertices)
                                    {
                                        sout << "Error: ZyColladaLoader::readDAE, a mesh could not be load : " << nameStream.str() << " - in node : " << currentNode->getName() << sendl;
                                        return false;
                                    }

                                    weights[id].push_back(weight);
                                    indices[id].push_back(k);
                                    ++nbref[id];
                                }
                            }

                            currentSkinningMapping->setWeights(weights, indices, nbref);
                        }
#endif
                    }
                    else
                    {
                        RigidMapping<Rigid3dTypes, Vec3dTypes>::SPtr currentRigidMapping = sofa::core::objectmodel::New<RigidMapping<Rigid3dTypes, Vec3dTypes> >();
                        {
                            // adding the generated RigidMapping to its parent Node
                            currentSubNode->addObject(currentRigidMapping);

                            std::stringstream nameStream(meshName);
                            if (meshName.empty())
                                nameStream << componentIndex++;

                            currentRigidMapping->setName(nameStream.str());

							// FA: Test for Schunk Demo with the SVH
							// currentRigidMapping->globalToLocalCoords.setValue(true);

                            if (isSubMesh) {
                                // Directly couple subMesh to the Base Node of the parent
                                currentRigidMapping->setModels(rigidBaseObject.at(parentMeshId).get(), currentTransMechanicalObject.get());

                            }
                            else {
                                currentRigidMapping->setModels(currentBaseMechanicalObject.get(), currentTransMechanicalObject.get());
                            }
                        }
                    }



                    std::vector<std::string> whitelist;
                    if (generateCollisionModels.getValue() > 0) {  // Whitelists
                        if (physicsModelByMeshId.find(meshName) != physicsModelByMeshId.end()) {
                            aiPhysicsModel* p = physicsModelByMeshId.at(meshName);
                            bool cont = true;
                            do {

                                if (meshesForPhysicsModel.find(p->mSid.C_Str()) != meshesForPhysicsModel.end()) {
                                    std::cout << "  Whitelist has " << p->mNumWhitelist << " entries." << std::endl;
                                    for (unsigned int w = 0; w < p->mNumWhitelist; w++) {
                                        if (meshesForPhysicsModel.find(p->mWhitelist[w]->C_Str()) != meshesForPhysicsModel.end()) {
                                            std::vector<std::string> meshes = meshesForPhysicsModel.at(p->mWhitelist[w]->C_Str());
                                            for (unsigned int w2 = 0; w2 < meshes.size(); w2++) {
                                                whitelist.push_back(meshes.at(w2));

                                            }
                                        }
                                        else {
                                            std::cout << "  Whitelist ERROR: could not find meshes for physicsmodel: " << p->mSid.C_Str() << std::endl;
                                        }
                                    }
                                }
                                // loop over all parents. So the subshapes get the same whitelist as the parents
                                if (p->mParentId.length > 0) {
                                    p = physicsModelByMeshId.at(std::string(p->mParentId.C_Str()));
                                }
                                else {
                                    cont = false;
                                }

                            } while (cont);


                        }
                    }

                    if (generateCollisionModels.getValue() == 2)
                    {  // use ObbTreeGPU collision models

#ifdef SOFA_HAVE_PLUGIN_OBBTREEGPU
                        ObbTreeGPUCollisionModel<defaulttype::Vec3dTypes>::SPtr obbTreeGpuModel = sofa::core::objectmodel::New<ObbTreeGPUCollisionModel<defaulttype::Vec3dTypes> >();
                        {
                            std::stringstream nameStream;
                            nameStream << meshName;
                            obbTreeGpuModel->setName(nameStream.str());

                            if (useContactManifolds.getValue()) {
                                obbTreeGpuModel->setUseContactManifolds(true);
                                /*obbTreeGpuModel->setMaxNumberOfLineLineManifolds(maxNumberOfLineLineManifolds.getValue());
                                            obbTreeGpuModel->setMaxNumberOfFaceVertexManifolds(maxNumberOfFaceVertexManifolds.getValue());*/
                                obbTreeGpuModel->setMaxNumberOfManifolds(maxNumberOfManifolds.getValue());
                            }
                            if (explicitWhitelist.getValue()) { //Disable all collissions except whitelist
                                obbTreeGpuModel->addToCollisionModelWhitelist("__DISABLE_COLLISION__");
                            }
                            // Ghost Actors
                            if (physicsModelByMeshId.find(meshName) != physicsModelByMeshId.end()) {
                                aiPhysicsModel* p = physicsModelByMeshId[meshName];
                                if (p->isGhost) {
                                    obbTreeGpuModel->setGhostObject(true);
                                    obbTreeGpuModel->setGhostObjectTolerance(p->tolerance);
                                }
                            }
                            currentSubNode->addObject(obbTreeGpuModel);

                            if (baseJointNameFound) {
                                jointNameForMesh[meshName] = baseJointName;
                                meshNamesConnectedToJoint[baseJointName].push_back(meshName);
                            }

                            // fill whitelist
                            for (unsigned int wl = 0; wl < whitelist.size(); wl++) {
                                obbTreeGpuModel->addToCollisionModelWhitelist(whitelist.at(wl));
                            }

                        }
#else
                        serr << "Using ObbTreeGpuColissionModels, but OBBTreeGPU Plugin is not compiled."  << sendl;
#endif

                    }
                    else if (generateCollisionModels.getValue() == 1) // Use CPU Collision Models
                    {

                        TTriangleModel<defaulttype::Vec3dTypes>::SPtr currentTTriangleModel = sofa::core::objectmodel::New<TTriangleModel<defaulttype::Vec3dTypes> >();
                        {
                            // adding the generated TTriangleModel to its parent Node
                            currentSubNode->addObject(currentTTriangleModel);

                            std::stringstream nameStream(meshName);
                            if (meshName.empty())
                                nameStream << componentIndex++;
                            currentTTriangleModel->setName(nameStream.str());
                        }

                        TLineModel<defaulttype::Vec3dTypes>::SPtr currentTLineModel = sofa::core::objectmodel::New<TLineModel<defaulttype::Vec3dTypes> >();
                        {
                            // adding the generated TLineModel to its parent Node
                            currentSubNode->addObject(currentTLineModel);

                            std::stringstream nameStream(meshName);
                            if (meshName.empty())
                                nameStream << componentIndex++;
                            currentTLineModel->setName(nameStream.str());
                        }

                        TPointModel<defaulttype::Vec3dTypes>::SPtr currentTPointModel = sofa::core::objectmodel::New<TPointModel<defaulttype::Vec3dTypes> >();
                        {
                            // adding the generated TPointModel to its parent Node
                            currentSubNode->addObject(currentTPointModel);

                            std::stringstream nameStream(meshName);
                            if (meshName.empty())
                                nameStream << componentIndex++;
                            currentTPointModel->setName(nameStream.str());
                        }

                        if (explicitWhitelist.getValue()) { //Disable all collissions except whitelist
                            currentTTriangleModel->addToCollisionModelWhitelist("__DISABLE_COLLISION__");
                            currentTLineModel->addToCollisionModelWhitelist("__DISABLE_COLLISION__");
                            currentTPointModel->addToCollisionModelWhitelist("__DISABLE_COLLISION__");
                        }
                        // fill whitelist
                        for (unsigned int wl = 0; wl < whitelist.size(); wl++) {
                            currentTTriangleModel->addToCollisionModelWhitelist(whitelist.at(wl));
                            currentTLineModel->addToCollisionModelWhitelist(whitelist.at(wl));
                            currentTPointModel->addToCollisionModelWhitelist(whitelist.at(wl));
                        }

                    }


                    // node used for visualization
                    std::stringstream visuNameStream;
                    visuNameStream << "visualization " << (int)meshId;

                    Node::SPtr visuNode = getSimulation()->createNewNode(visuNameStream.str());
                    currentSubNode->addChild(visuNode);

                    currentSubNode = visuNode;

                    sofa::gui::qt::RealGUI* gui = dynamic_cast<sofa::gui::qt::RealGUI*>(sofa::gui::GUIManager::getGUI());
                    sofa::component::visualmodel::VisualModelImpl::SPtr currentOglModel;
#ifdef SOFA_HAVE_QTOGREVIEWER
                    viewer::QtOgreViewer* ogreViewer = dynamic_cast<viewer::QtOgreViewer*>(gui->getViewer());
                    if (ogreViewer)	{
                        // Create Ogre models
                        currentOglModel = sofa::core::objectmodel::New<OgreVisualModel>();
                    }
                    else
#endif //SOFA_HAVE_QTOGREVIEWER
                    {
                        currentOglModel = sofa::core::objectmodel::New<OglModel>();
                    }


                    // setting parameters of OglModel or OgreVisualModel and filling up its properties
                    // Zykl.io begin
                    bool switchedModels = false;
                    // Zykl.io end
                    {
                        // Zykl.io begin
                        aiMesh* visualMesh = NULL;
                        if (physicsModelByMeshId.find(meshName) != physicsModelByMeshId.end()) {
                            aiPhysicsModel* p = physicsModelByMeshId.at(meshName);
                            if (p->mVisualModel)
                            {
                                visualMesh = getAiMeshByName(p->mVisualModel->C_Str(), currentAiScene);
                                //collisionMesh = meshesInFile[p->mCollisionModel->C_Str()];
                                
                                if (visualMesh)
                                {
                                    std::cout << "Visual model: " << visualMesh->mName.C_Str() << std::endl;
                                }
                                /*else
                                {
                                    std::cout << "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa" << std::endl;
                                }*/
                            }
                            /*else
                            {
                                std::cout << "bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb" << std::endl;
                            }*/
                        }

                        aiMesh* currentMeshTmp = currentAiMesh;
                        std::string meshNameTmp;
                        if (visualMesh)
                        {
                            currentAiMesh = visualMesh;
                            switchedModels = true;
                            // generating a name
                            meshName = std::string(currentAiMesh->mName.data, currentAiMesh->mName.length);
                            std::cout << "Switching to visual mesh: " << currentAiMesh->mName.C_Str() << std::endl;
                        }
                        // Zykl.io end

                        // Zykl.io begin /////////////////
                        // Fill vertices, triangles and quads of the OglModel with values taken from the given mesh
                        // This is only necessary when the visual mesh and the collision mesh are supposed to be different

                        // filling up position array
                        //currentOglModel->seqPoints.setParent(&currentMechanicalObject->x);
                        // filling up position coordinate array
                        if (0 != currentAiMesh->mNumVertices)
                        {
                            //helper::vector<defaulttype::Vec<3, SReal> >& x = *(currentOglModel->seqPoints.beginEdit());
                            sofa::defaulttype::ResizableExtVector<sofa::defaulttype::ExtVec3dTypes::Coord> positions;

                            std::cout << "  Creating OglModel " << currentNode->getName() << "  " << meshName << std::endl;
                            for (unsigned int k = 0; k < currentAiMesh->mNumVertices; ++k) {
                                sofa::defaulttype::ExtVec3fTypes::Coord vertex = sofa::defaulttype::ExtVec3fTypes::Coord(currentAiMesh->mVertices[k][0], currentAiMesh->mVertices[k][1], currentAiMesh->mVertices[k][2]);
                                positions.push_back(currentMeshTransformation.VertexTransform(vertex));
                            }

                            currentOglModel->setVertices(&positions);
                        }

                        // filling up triangle array
                        // sofa::helper::vector<sofa::core::topology::Triangle> triangles;
                        sofa::defaulttype::ResizableExtVector<sofa::core::topology::BaseMeshTopology::Triangle> triangles;
                        unsigned int numTrianglesTmp = 0;
                        for (unsigned int k = 0; k < currentAiMesh->mNumFaces; ++k)
                            if (3 == currentAiMesh->mFaces[k].mNumIndices)
                                ++numTrianglesTmp;

                        if (0 != numTrianglesTmp)
                        {
                            triangles.resize(numTrianglesTmp);

                            unsigned int triangleOffset = 0;
                            for (unsigned int k = 0; k < currentAiMesh->mNumFaces; ++k)
                            {
                                if (3 != currentAiMesh->mFaces[k].mNumIndices)
                                    continue;

                                //memcpy(&triangles[0] + triangleOffset, currentAiMesh->mFaces[k].mIndices, sizeof(sofa::core::topology::Triangle));
                                memcpy(&triangles[0] + triangleOffset, currentAiMesh->mFaces[k].mIndices, sizeof(sofa::core::topology::BaseMeshTopology::Triangle));
                                ++triangleOffset;
                            }

                            {
                                /*sofa::helper::vector<sofa::core::topology::Triangle>& seqTriangles = *currentOglModel->seqTriangles.beginEdit();
                                seqTriangles.reserve(triangles.size());

                                for (unsigned int k = 0; k < triangles.size(); ++k)
                                    seqTriangles.push_back(triangles[k]);*/
                                currentOglModel->setTriangles(&triangles);
                            }
                        }

                        // filling up quad array
                        //sofa::helper::vector<sofa::core::topology::Quad> quads;
                        sofa::defaulttype::ResizableExtVector<sofa::core::topology::BaseMeshTopology::Quad> quads;
                        unsigned int numQuads = 0;
                        for (unsigned int k = 0; k < currentAiMesh->mNumFaces; ++k)
                            if (4 == currentAiMesh->mFaces[k].mNumIndices)
                                ++numQuads;

                        if (0 != numQuads)
                        {
                            quads.resize(numQuads);

                            unsigned int quadOffset = 0;
                            for (unsigned int k = 0; k < currentAiMesh->mNumFaces; ++k)
                            {
                                if (4 != currentAiMesh->mFaces[k].mNumIndices)
                                    continue;

                                //memcpy(&quads[0] + quadOffset, currentAiMesh->mFaces[k].mIndices, sizeof(sofa::core::topology::Quad));
                                memcpy(&quads[0] + quadOffset, currentAiMesh->mFaces[k].mIndices, sizeof(sofa::core::topology::BaseMeshTopology::Quad));
                                ++quadOffset;
                            }

                            {
                                /*sofa::helper::vector<sofa::core::topology::Quad>& seqQuads = *currentOglModel->seqQuads.beginEdit();
                                seqQuads.reserve(quads.size());

                                for (unsigned int k = 0; k < quads.size(); ++k)
                                    seqQuads.push_back(quads[k]);*/
                                currentOglModel->setQuads(&quads);
                            }
                        }
                        // Zykl.io end /////////////////

                        // Set Backface Culling on, so we can only see the front of a face
                        Data<int> *data = dynamic_cast<Data<int>*>(currentOglModel->findData("cullFace"));
                        if (data) data->setValue(0);

                        // adding the generated OglModel to its parent Node
                        currentSubNode->addObject(currentOglModel);


                        //currentOglModel->m_rotation.setValue(OglModel::Vec3Real(0,0,60));
                        //currentOglModel->m_scale.setValue(OglModel::Vec3Real(0.5,0.5,1));

                        std::stringstream nameStream(meshName);
                        if (meshName.empty())
                            nameStream << componentIndex++;
                        currentOglModel->setName(nameStream.str());


                        unsigned int numTriangles = 0;
                        for (unsigned int k = 0; k < currentAiMesh->mNumFaces; ++k)
                            if (3 == currentAiMesh->mFaces[k].mNumIndices)
                                ++numTriangles;

                        // Used for badass hack. Only use 1st Material for other topopogies than triangle meshes
                        bool hasTriangles = (numTriangles * 3 == currentAiMesh->mNumVertices);

                        {
                            // Groups and their Materials
                            helper::vector<Material> * materials = currentOglModel->materials.beginEdit();
                            helper::vector<OglModel::FaceGroup> * groups = currentOglModel->groups.beginEdit();
                            materials->resize(currentAiMesh->mNumMaterialGroups + 1);
                            groups->resize(currentAiMesh->mNumMaterialGroups);

                            std::map<std::string, unsigned int> materialMap;

                            unsigned int triIndex = 0;
                            unsigned int materialID = 1;

                            for (unsigned int mi = 0; mi < currentAiMesh->mNumMaterialGroups; mi++) {

                                // Create Material
                                Material mat;

                                unsigned int aiMatIndex = currentAiMesh->mGroupsMaterialIndex[mi];
                                unsigned int triCount = currentAiMesh->mMaterialGroups[mi];

                                aiMaterial* aiMat = currentAiScene->mMaterials[aiMatIndex];

                                // Set Material name (so we can find the material in the dae file)
                                aiString name;
                                if (aiGetMaterialString(aiMat, AI_MATKEY_NAME, &name) == AI_SUCCESS) {
                                    mat.name = std::string(name.C_Str());
                                }
                                else {
                                    std::stringstream materialIdStream;
                                    materialIdStream << materialID;
                                    mat.name = materialIdStream.str();
                                }


                                // Only create new material if the current material is not in the map
                                if (materialMap.find(mat.name) == materialMap.end()) {
                                    // Set Ambient, diffuse, speclar, emissive
                                    mat.useAmbient = false;
                                    mat.useDiffuse = false;
                                    mat.useSpecular = false;
                                    mat.useEmissive = false;

                                    aiColor4D color; // using this for all color lookups
                                    if (aiGetMaterialColor(aiMat, AI_MATKEY_COLOR_AMBIENT, &color) == AI_SUCCESS) {
                                        mat.ambient.set(color.r, color.g, color.b, color.a);
                                        mat.useAmbient = true;
                                    }
                                    if (aiGetMaterialColor(aiMat, AI_MATKEY_COLOR_DIFFUSE, &color) == AI_SUCCESS) {
                                        mat.diffuse.set(color.r, color.g, color.b, color.a);
                                        mat.useDiffuse = true;
                                    }
                                    if (aiGetMaterialColor(aiMat, AI_MATKEY_COLOR_SPECULAR, &color) == AI_SUCCESS) {
                                        mat.specular.set(color.r, color.g, color.b, color.a);
                                        mat.useSpecular = true;
                                    }
                                    if (aiGetMaterialColor(aiMat, AI_MATKEY_COLOR_EMISSIVE, &color) == AI_SUCCESS) {
                                        mat.emissive.set(color.r, color.g, color.b, color.a);
                                        mat.useEmissive = true;
                                    }
                                    float val;
                                    if (aiGetMaterialFloat(aiMat, AI_MATKEY_SHININESS, &val) == AI_SUCCESS) {
                                        mat.shininess = val;
                                        mat.useShininess = true;
                                    }
                                    if (aiGetMaterialFloat(aiMat, AI_MATKEY_OPACITY, &val) == AI_SUCCESS) {
                                        if (mat.diffuse[3] == 1.0f) {
                                            mat.diffuse[3] = val;
                                        }
                                    }

                                    if (hasTriangles) {
                                        // Add Material
                                        (*materials)[materialID] = mat;
                                        // Save Index
                                        materialMap[mat.name] = materialID;
                                        // Increment
                                        materialID++;
                                    }
                                    else {
                                        // set primary material, reset groups and materialS and break here
                                        currentOglModel->material.setValue(mat);
                                        (*materials).resize(0);
                                        (*groups).resize(0);
                                        goto noTrianglesEnd;
                                    }
                                }


                                // Create FaceGroup
                                OglModel::FaceGroup face;
                                face.materialId = materialMap[mat.name];  // Get materialID from map
                                face.materialName = mat.name;
                                face.tri0 = triIndex / 3;
                                face.nbt = triCount / 3;
                                triIndex += triCount;
                                //std::cout << "Face: " << face << std::endl;

                                // Add FaceGroup
                                (*groups)[mi] = face;

                            }

                            (*materials).resize(materialMap.size() + 1);

noTrianglesEnd:  // Badass hack goto (for other types than triangles)
                            currentOglModel->materials.endEdit();
                            currentOglModel->groups.endEdit();

                            // Zykl.io begin
                            if (visualMesh)
                            {
                                currentAiMesh = currentMeshTmp;
                                meshName = meshNameTmp;
                                std::cout << "Switching to back regular mesh: " << currentAiMesh->mName.C_Str() << std::endl;
                            }
                            // Zykl.io end
                        }
                    }

                    // Zykl.io begin
                    if (switchedModels) 
                    {
                    // Zykl.io end
                        // Zykl.io begin
                        RigidMapping<sofa::defaulttype::Rigid3Types, sofa::defaulttype::ExtVec3Types>::SPtr currentRigidMapping = sofa::core::objectmodel::New<RigidMapping<sofa::defaulttype::Rigid3Types, sofa::defaulttype::ExtVec3Types> >();
                        {
                            // adding the generated RigidMapping to its parent Node
                            currentSubNode->addObject(currentRigidMapping);

                            std::stringstream nameStream(meshName);
                            if (meshName.empty())
                                nameStream << componentIndex++;
                            currentRigidMapping->setName(nameStream.str());

                            currentRigidMapping->setModels(currentBaseMechanicalObject.get(), currentOglModel.get());
                        }
                        // Zykl.io end
                    }
                    else
                    {
                        IdentityMapping<sofa::defaulttype::Vec3Types, sofa::defaulttype::ExtVec3Types>::SPtr currentIdentityMapping = sofa::core::objectmodel::New<IdentityMapping<sofa::defaulttype::Vec3Types, sofa::defaulttype::ExtVec3Types> >();
                        {
                            // adding the generated IdentityMapping to its parent Node
                            currentSubNode->addObject(currentIdentityMapping);

                            std::stringstream nameStream(meshName);
                            if (meshName.empty())
                                nameStream << componentIndex++;
                            currentIdentityMapping->setName(nameStream.str());

                            currentIdentityMapping->setModels(currentTransMechanicalObject.get(), currentOglModel.get());
                        }
                    }

                    // Remove Nodes of SubMeshes
                    if (isSubMesh) {
                        core::objectmodel::BaseNode::Parents p = currentNode->getParents();
                        if (p.size() > 0) {
                            BaseNode* parent = p.at(0);
                            parent->removeChild(currentNode);
                        }
                    }


                }

            }



            // pop the current node when each one of its children have been processed
            if (childIndex >= currentAiNode->mNumChildren)
            {
                nodes.pop();
            }
            // process next sub node
            else
            {
                // generating sub Node and filling up its properties
                // store it in the stack to process its children later
                NodeInfo subNodeInfo(currentAiNode->mChildren[childIndex], getSimulation()->createNewNode(""), &currentNodeInfo);
                nodes.push(subNodeInfo);

                // adding the generated node to its parent Node
                currentNode->addChild(subNodeInfo.mNode);

                // this child will be processed, go to the next one
                ++childIndex;
            }
        }
    }

    // Apply dangling links of physics models
    for (unsigned int i = 0; i < rigidBaseLinks.size(); i++) {
        MechanicalObject<Rigid3dTypes>::SPtr rigid = rigidBaseObject.at(rigidBaseLinks.at(i).first);
        RigidRigidMapping<Rigid3dTypes, Rigid3dTypes>::SPtr mapping = rigidBaseLinks.at(i).second;
        mapping->setModels(rigid.get(), mapping->getToModel());
    }

    //removeEmptyNodes();

    if (generateCollisionModels.getValue()) {
        fillObbTreeBlacklists(meshNamesConnectedToJoint, jointNameForMesh, jointMap, jointByObjectName, blacklist);
        fillObbTreeBlacklists2(blacklist);
    }

    std::cout << "====================================================================" << std::endl;
    std::cout << "                        JOINT HIERARCHIES                           " << std::endl;
    std::cout << "====================================================================" << std::endl;

    if (m_transformHelper != NULL)
        m_transformHelper->dumpJointHierarchies();

    std::cout << "====================================================================" << std::endl;
    std::cout << "                        JOINT HIERARCHIES                           " << std::endl;
    std::cout << "====================================================================" << std::endl;

    loadScene.setValue(false);
    return true;
}

bool ZyColladaLoader::fillSkeletalInfo(const aiScene* scene, aiNode* meshParentNode, aiNode* meshNode, aiMatrix4x4 meshTransformation, aiMesh* mesh, helper::vector<SkeletonJoint<Rigid3dTypes> >& skeletonJoints, helper::vector<SkeletonBone>& skeletonBones) const
{
    //std::cout << "fillSkeletalInfo : begin" << std::endl;

    // return now if their is no scene, no mesh or no skeletonBones
    if (!scene || !mesh || !mesh->HasBones())
    {
        sout << "no mesh to load !" << sendl;
        return false;
    }

    std::map<aiNode*, std::size_t> aiNodeToSkeletonJointIndex;

    // compute the mesh transformation into a rigid
    Mat4x4d meshWorldTranformation(meshTransformation[0]);
    //	for(int j = 0; j < 4; ++j)
    //		for(int i = 0; i < 4; ++i)
    //			meshWorldTranformation[j][i] = meshTransformation[j][i];

    Rigid3dTypes::Coord meshTransformationRigid;
    meshTransformationRigid.getCenter()[0] = meshWorldTranformation[0][3];
    meshTransformationRigid.getCenter()[1] = meshWorldTranformation[1][3];
    meshTransformationRigid.getCenter()[2] = meshWorldTranformation[2][3];
    Mat3x3d rot; rot = meshWorldTranformation;
    meshTransformationRigid.getOrientation().fromMatrix(rot);

    //std::cout << "ANIMATION" << std::endl;

    // register every SkeletonJoint
    for (unsigned int j = 0; j < scene->mNumAnimations; ++j)
    {
        // for now we just want to handle one animation
        if (1 == j)
            break;

        //std::cout << "num channels : " << scene->mAnimations[j]->mNumChannels << std::endl;

        aiAnimation*& animation = scene->mAnimations[j];
        for (unsigned int k = 0; k < animation->mNumChannels; ++k)
        {
            aiNodeAnim*& channel = animation->mChannels[k];
            aiString& nodeName = channel->mNodeName;
            aiNode* node = scene->mRootNode->FindNode(nodeName);

            // create the corresponding SkeletonJoint if it does not exist
            std::map<aiNode*, std::size_t>::iterator aiNodeToSkeletonJointIndexIterator = aiNodeToSkeletonJointIndex.find(node);
            if (aiNodeToSkeletonJointIndex.end() == aiNodeToSkeletonJointIndexIterator)
            {
                skeletonJoints.push_back(SkeletonJoint<Rigid3dTypes>());
                aiNodeToSkeletonJointIndex[node] = skeletonJoints.size() - 1;
                aiNodeToSkeletonJointIndexIterator = aiNodeToSkeletonJointIndex.find(node);
            }
            else
            {
                return false;
            }
            SkeletonJoint<Rigid3dTypes>& skeletonJoint = skeletonJoints[aiNodeToSkeletonJointIndexIterator->second];

            aiVectorKey positionKey, scaleKey;
            aiQuatKey	rotationKey;

            unsigned int numKey = std::max(channel->mNumPositionKeys, channel->mNumRotationKeys);
            //int numKey = std::max(channel->mNumScalingKeys , std::max(channel->mNumPositionKeys, channel->mNumRotationKeys));

            skeletonJoint.mTimes.resize(numKey);
            skeletonJoint.mChannels.resize(numKey);
            for (unsigned int l = 0; l < numKey; ++l)
            {
                SReal time = 0.0;
                aiMatrix4x4 transformation;

                if (l < channel->mNumPositionKeys)
                {
                    positionKey = channel->mPositionKeys[l];
                    time = positionKey.mTime;
                    aiMatrix4x4 position;
                    aiMatrix4x4::Translation(positionKey.mValue, position);
                    transformation = position;
                }

                if (l < channel->mNumRotationKeys)
                {
                    rotationKey = channel->mRotationKeys[l];
                    time = rotationKey.mTime;
                    aiMatrix4x4 rotation(rotationKey.mValue.GetMatrix());
                    transformation *= rotation;
                }

                // 							if(l < channel->mNumScalingKeys)
                // 							{
                // 								scaleKey = channel->mScalingKeys[l];
                // 								time = scaleKey.mTime;
                // 								aiMatrix4x4 scale;
                // 								aiMatrix4x4::Scaling(scaleKey.mValue, scale);
                // 								transformation *= scale;
                // 							}

                Mat4x4d localTranformation(transformation[0]);

                Rigid3dTypes::Coord localRigid;
                localRigid.getCenter()[0] = localTranformation[0][3];
                localRigid.getCenter()[1] = localTranformation[1][3];
                localRigid.getCenter()[2] = localTranformation[2][3];
                Mat3x3d rot; rot = localTranformation;
                localRigid.getOrientation().fromMatrix(rot);

                skeletonJoint.mTimes[l] = time;
                skeletonJoint.mChannels[l] = localRigid;
            }
        }
    }

    // register every bone and link them to their SkeletonJoint (or create it if it has not been created)
    skeletonBones.resize(mesh->mNumBones);
    for (unsigned int i = 0; i < mesh->mNumBones; ++i)
    {
        aiBone*& bone = mesh->mBones[i];
        const aiString& boneName = bone->mName;

        // register the parents SkeletonJoint for each bone
        aiNode* node = scene->mRootNode->FindNode(boneName);

        // create the corresponding SkeletonJoint if it does not exist
        std::map<aiNode*, std::size_t>::iterator aiNodeToSkeletonJointIndexIterator = aiNodeToSkeletonJointIndex.find(node);
        if (aiNodeToSkeletonJointIndex.end() == aiNodeToSkeletonJointIndexIterator)
        {
            skeletonJoints.push_back(SkeletonJoint<Rigid3dTypes>());
            aiNodeToSkeletonJointIndex[node] = skeletonJoints.size() - 1;
            aiNodeToSkeletonJointIndexIterator = aiNodeToSkeletonJointIndex.find(node);
        }

        skeletonBones[i] = aiNodeToSkeletonJointIndexIterator->second;
    }

    // register every SkeletonJoint and their parents and fill up theirs properties
    for (std::size_t i = 0; i < skeletonJoints.size(); ++i)
    {
        SkeletonJoint<Rigid3dTypes>& skeletonJoint = skeletonJoints[i];

        aiNode*	node = NULL;

        // find the ai node corresponding to the SkeletonJoint
        for (std::map<aiNode*, std::size_t>::iterator aiNodeToSkeletonJointIndexIterator = aiNodeToSkeletonJointIndex.begin(); aiNodeToSkeletonJointIndexIterator != aiNodeToSkeletonJointIndex.end(); ++aiNodeToSkeletonJointIndexIterator)
        {
            if (i == aiNodeToSkeletonJointIndexIterator->second)
            {
                node = aiNodeToSkeletonJointIndexIterator->first;
                break;
            }
        }

        if (NULL == node)
            return false;

        std::size_t previousSkeletonJointIndex;
        bool firstIteration = true;

        // find parents node
        while (NULL != node)
        {
            // stop if we reach the mesh node or its parent
            if (meshNode == node || meshParentNode == node)
                break;

            // create the corresponding SkeletonJoint if it does not exist
            std::map<aiNode*, std::size_t>::iterator aiNodeToSkeletonJointIndexIterator = aiNodeToSkeletonJointIndex.find(node);
            if (aiNodeToSkeletonJointIndex.end() == aiNodeToSkeletonJointIndexIterator)
            {
                skeletonJoints.push_back(SkeletonJoint<Rigid3dTypes>());
                aiNodeToSkeletonJointIndex[node] = skeletonJoints.size() - 1;
                aiNodeToSkeletonJointIndexIterator = aiNodeToSkeletonJointIndex.find(node);
            }
            SkeletonJoint<Rigid3dTypes>& currentSkeletonJoint = skeletonJoints[aiNodeToSkeletonJointIndexIterator->second];

            // register the current node
            aiMatrix4x4 aiLocalTransformation = node->mTransformation;

            // compute the rigid corresponding to the SkeletonJoint
            Mat4x4d localTranformation(aiLocalTransformation[0]);

            Rigid3dTypes::Coord localRigid;
            localRigid.getCenter()[0] = localTranformation[0][3];
            localRigid.getCenter()[1] = localTranformation[1][3];
            localRigid.getCenter()[2] = localTranformation[2][3];
            Mat3x3d rot; rot = localTranformation;
            localRigid.getOrientation().fromMatrix(rot);

            // apply the mesh transformation to the skeleton root joint only
            // we know that this joint is the root if the corresponding aiNode is the mesh node or its parent
            aiNode* parentNode = node->mParent;
            if (meshNode == parentNode || meshParentNode == parentNode)
            {
                // compute the mesh transformation
                localRigid = meshTransformationRigid.mult(localRigid);

                // apply the mesh transformation to each channel if the skeleton root joint contains animation
                for (std::size_t i = 0; i < currentSkeletonJoint.mChannels.size(); ++i)
                    currentSkeletonJoint.mChannels[i] = meshTransformationRigid.mult(currentSkeletonJoint.mChannels[i]);
            }

            currentSkeletonJoint.setRestPosition(localRigid);

            if (!firstIteration)
                skeletonJoints[previousSkeletonJointIndex].mParentIndex = aiNodeToSkeletonJointIndexIterator->second;

            firstIteration = false;
            previousSkeletonJointIndex = aiNodeToSkeletonJointIndexIterator->second;

            node = node->mParent;
        }
    }

    return true;
}

void ZyColladaLoader::removeEmptyNodes()
{
    // remove intermediary or empty nodes
    {
        std::stack<std::pair<Node::SPtr, std::size_t> > nodes;

        nodes.push(std::pair<Node::SPtr, std::size_t>(subSceneRoot, 0));
        while (!nodes.empty())
        {
            Node::SPtr& node = nodes.top().first;
            std::size_t& index = nodes.top().second;

            if (node->getChildren().size() <= index)
            {
                nodes.pop();

                if (nodes.empty())
                    break;

                Node::SPtr& parentNode = nodes.top().first;
                std::size_t& parentIndex = nodes.top().second;

                // remove the node if it has no objects
                if (node->object.empty())
                {
                    if (0 != node->getChildren().size())
                    {
                        // links its child nodes directly to its parent node before remove the current intermediary node
                        while (!node->getChildren().empty())
                        {
                            Node::SPtr childNode = static_cast<Node*>(node->getChildren()[0]);
                            parentNode->moveChild(childNode);
                        }
                    }

                    parentNode->removeChild(node);
                }
                else
                {
                    ++parentIndex;
                }
            }
            else
            {
                Node::SPtr child = static_cast<Node*>(node->getChildren()[index]);
                nodes.push(std::pair<Node::SPtr, std::size_t>(child, 0));
            }
        }
    }
}

// Zykl.io begin
aiMesh* ZyColladaLoader::getAiMeshByName(std::string meshName, const aiScene* colladaScene)
{
    aiMesh* meshPnt = NULL;
    for (unsigned int k = 0; k < colladaScene->mNumMeshes; k++)
    {
        if (meshName.compare(colladaScene->mMeshes[k]->mName.C_Str()) == 0)
        {
            meshPnt = colladaScene->mMeshes[k];
            break;
        }
    }
    return meshPnt;
}
// Zykl.io end

} // namespace loader

} // namespace component

} // namespace sofa

