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
#ifndef SOFA_COMPONENT_LOADER_ZyColladaLoader_H
#define SOFA_COMPONENT_LOADER_ZyColladaLoader_H

#include <ZyColladaLoader/config.h>
#include <sofa/defaulttype/Vec.h>

#include <sofa/core/loader/SceneLoader.h>
#include <sofa/helper/SVector.h>
#include <sofa/simulation/Node.h>

#include <SofaBaseMechanics/MechanicalObject.h>
#include <ZySOFAControllers/RobotController.h>
#include <ZySOFAControllers/ArbitraryController.h>

#include <SofaBoundaryCondition/SkeletalMotionConstraint.h>

#include <assimp/Importer.hpp>      // C++ importer interface
#include <assimp/scene.h>           // Output data structure
#include <assimp/postprocess.h>     // Post processing flags

#include "ColladaTransformHelper.h"

#include "config.h"

#define noop

using namespace sofa::defaulttype;
using namespace sofa::simulation;
using namespace sofa::component::projectiveconstraintset;

namespace sofa
{

namespace component
{

namespace loader
{

/**
//  using namespace sofa::defaulttype;
//  using namespace sofa::helper::io;
using sofa::defaulttype::Vec4f;
using sofa::defaulttype::Mat4x4f;
using namespace sofa::simulation;
using namespace sofa::component::projectiveconstraintset;

current limitation : one animation per scene
*/

class ZyColladaLoaderPrivate;

class ZYCOLLADALOADER_API ZyColladaLoader : public sofa::core::loader::SceneLoader
{
public:
    SOFA_CLASS(ZyColladaLoader,sofa::core::loader::SceneLoader);

    struct NodeInfo;

    // describing a link between Assimp Node and Sofa Node allowing us to build a node hierarchy
    struct NodeInfo
    {
        std::size_t			mChildIndex;		// index of the current child node to process
        aiNode*				mAiNode;		// aiNode being processed
        Node::SPtr			mNode;			// corresponding Node created in the sofa scene graph
        NodeInfo*			mParentNode;		// parent node (useful to retrieve mesh skeleton and to compute world transformation matrix)
        aiMatrix4x4			mTransformation;	// matrix that transforms from node space to world space

        NodeInfo(aiNode* pAiNode, Node::SPtr pNode, NodeInfo* mParentNode = NULL) :
            mChildIndex(0),
            mAiNode(pAiNode),
            mNode(pNode),
            mParentNode(mParentNode),
            mTransformation()
        {
            if(mParentNode)
                mTransformation = mParentNode->mTransformation;

            if (pAiNode) {
                mTransformation *= pAiNode->mTransformation;
            }

            /*if(root)
            {
            	std::cout << pAiNode->mTransformation.a1 << " - " << pAiNode->mTransformation.b1 << " - " << pAiNode->mTransformation.c1 << " - " << pAiNode->mTransformation.d1 << std::endl;
            	std::cout << pAiNode->mTransformation.a2 << " - " << pAiNode->mTransformation.b2 << " - " << pAiNode->mTransformation.c2 << " - " << pAiNode->mTransformation.d2 << std::endl;
            	std::cout << pAiNode->mTransformation.a3 << " - " << pAiNode->mTransformation.b3 << " - " << pAiNode->mTransformation.c3 << " - " << pAiNode->mTransformation.d3 << std::endl;
            	std::cout << pAiNode->mTransformation.a4 << " - " << pAiNode->mTransformation.b4 << " - " << pAiNode->mTransformation.c4 << " - " << pAiNode->mTransformation.d4 << std::endl;
            }*/
        }

        NodeInfo(const NodeInfo& nodeInfo) :
            mChildIndex(nodeInfo.mChildIndex),
            mAiNode(nodeInfo.mAiNode),
            mNode(nodeInfo.mNode),
            mParentNode(nodeInfo.mParentNode),
            mTransformation(nodeInfo.mTransformation)
        {

        }
    };

    // describing a link between a Node and an Assimp Mesh
    struct MeshInfo
    {
        aiMesh*		mAiMesh;	// mesh being processed
        NodeInfo	mNodeInfo;		// its owner node

        MeshInfo(aiMesh* pAiMesh, NodeInfo pNodeInfo) :
            mAiMesh(pAiMesh),
            mNodeInfo(pNodeInfo)
        {

        }
    };

    // describe a joint information to create link by RigidRigidMappings
    struct JointInfo
    {
        std::string mName;
        std::string jName;
        sofa::component::container::MechanicalObject<Rigid3Types> *link6D;
        int index;

        aiMatrix4x4 mTransformation;

        // These have to be ints!
        float transX;
        float transY;
        float transZ;

        // Doubles here!
        double quatX;
        double quatY;
        double quatZ;
        double quatW;

        JointInfo() {
            index = -1;
            transX = 0.;
            transY = 0.;
            transZ = 0.;
            quatX = 0.;
            quatY = 0.;
            quatZ = 0.;
            quatW = 1.;
        }
    };

    struct MeshTransform
    {
        MeshTransform()
            : meshName()
            , nodeID()

            , mainTrans()
            , mainScale()
            , mainQuat()
            , mainRot()

            , hasJoint(false)
            , jointTrans()
            , jointScale()
            , jointQuat()
            , jointRot()

            , hasModelTranspose(false)
            , physTrans()
            , physScale()
            , physQuat()
            , physRot()

            , isTool(false)
            , toolTrans()
            , toolScale()
            , toolQuat()
            , toolRot()

            , isToolAttached(false)
            , obj1Trans()
            , obj1Scale()
            , obj1Quat()
            , obj1Rot()

            , obj2Trans()
            , obj2Scale()
            , obj2Quat()
            , obj2Rot()
            , isBasePhysicsModel(false)
            , isForceSensitiveTool(false)
            , massFrame()

            , isAnimatedObject(false)
            , fixBlenderExport(false)
        {}

        std::string meshName;  // MeshID
        std::string nodeID;  // NodeID

        Vec3d mainTrans;         // Main translation which is set in the node
        Vec3d mainScale;         // Main scale
        Quat  mainQuat;         // Main rotation
        Vec3d mainRot;

        bool hasJoint;
        Vec3d jointTrans;         // Joint translation
        Vec3d jointScale;
        Quat  jointQuat;         // Joint rotation
        Vec3d jointRot;

        bool hasModelTranspose;
        Vec3d physTrans;         // Physics Model translation
        Vec3d physScale;
        Quat  physQuat;         // Physics Model rotation
        Vec3d physRot;

        bool isTool;
        Vec3d toolTrans;         // Tool translation
        Vec3d toolScale;
        Quat  toolQuat;
        Vec3d toolRot;

        bool isToolAttached;
        Vec3d obj1Trans;         // Tool translation
        Vec3d obj1Scale;
        Quat  obj1Quat;
        Vec3d obj1Rot;

        Vec3d obj2Trans;         // Tool translation
        Vec3d obj2Scale;
        Quat  obj2Quat;
        Vec3d obj2Rot;

        bool isAnimatedObject;
        Quat  animQuatInverse;

        bool isBasePhysicsModel;
        bool isForceSensitiveTool;
        Vec3d massFrame;

        bool isSubShape() {
            return hasModelTranspose && !isBasePhysicsModel;
        }

        bool fixBlenderExport;

        Vec3d scale(Vec3d vertex, Vec3d scale) {
            Vec3d ret;
            Mat3x3d mat;
            mat(0,0) = scale.x();
            mat(1,1) = scale.y();
            mat(2,2) = scale.z();
            ret = mat * vertex;
            return ret;
        }


        Vec3d eulerAngles(double q0, double q1, double q2, double q3)
        {
            Vec3d res;
            res.set(asin( 2 * (q0*q2 - q3*q1)),
                    atan2(2 * (q0*q1 + q2*q3), 1 - 2 * (q1*q1 + q2*q2)),
                    atan2(2 * (q0*q3 + q1*q2), 1 - 2 * (q2*q2 + q3*q3)));
            return res;
        }


        Vec3d getBaseT() {

            if (isForceSensitiveTool)
                return mainTrans + mainQuat.rotate(massFrame-toolTrans);
            else if (isToolAttached)
                return obj1Quat.rotate(massFrame-toolTrans-obj2Trans);
            else if (isSubShape())
                return Vec3d(0,0,0);
            else if (isTool)
                return mainQuat.rotate(massFrame-toolTrans);
            else if (hasJoint)  // Bernd Test
                return Vec3d(0, 0, 0);
            else if (isBasePhysicsModel)
                return mainTrans + mainQuat.rotate(massFrame);
            else
                return mainTrans;
        }

        Quat getBaseRQ() {
            if (isAnimatedObject)
                return Quat();
            else if (isToolAttached)
                return obj1Quat;
            else if (isTool)
                return toolQuat;
            else if (isBasePhysicsModel)
                return mainQuat;
            else
                return Quat();
        }


        Vec3d getTransT() {
            if (isBasePhysicsModel)
                return Vec3d(0,0,0); //-massFrame;
            else if (isSubShape())
                return -massFrame;
            else
                return Vec3d(0,0,0);
        }

        Quat getTransRQ() {
            if (isAnimatedObject)
                return animQuatInverse;
            else
                return Quat();
        }

        // Return Transformation for Meshes
        Vec3d VertexTransform(Vec3d vertex) {
            Vec3d ret;

            //Scale
            ret = scale(vertex, mainScale);

            //Rotations
            if (hasJoint && !isTool && !isToolAttached) {
                ret = jointQuat.rotate(ret); // NEW
            }

            if (isSubShape())
                ret = physTrans + physQuat.rotate(ret);
            else if (isBasePhysicsModel) {
                noop; // rotation is done via BaseRQ
            }
            else
                ret = mainQuat.rotate(ret);

            //Translations
            if (hasJoint && !isTool && !isToolAttached) {
                if (!fixBlenderExport) {
                    //ret -= jointTrans; // incopatiple to robot scene
                    ret -= jointQuat.rotate(jointTrans);  //NEW   
                }
            }
                   
            if (isBasePhysicsModel)
                ret -= massFrame; // (use [mq.rotate(massFrame)] when translating A rotation

            return ret;
        }

    };

protected:
    ZyColladaLoader();
    ~ZyColladaLoader();
public:

    virtual void init();
    virtual void bwdInit();
    
    virtual bool load();

    void draw(const core::visual::VisualParams *);

    template <class T>
    static bool canCreate ( T*& obj, core::objectmodel::BaseContext* context, core::objectmodel::BaseObjectDescription* arg )
    {
        return BaseLoader::canCreate (obj, context, arg);
    }

	float getAnimationSpeed() const			{return animationSpeed.getValue();}
	void setAnimationSpeed(float speed)		{animationSpeed.setValue(speed);}

protected:

    bool readDAE (std::ifstream &file, const char* filename);

    int CreateArticulatedJointHierarchy(std::stringstream &ss, aiJoint *joint, std::map<std::string, aiJoint *> &jointMap, std::map<std::string, JointInfo> &jointInfo, sofa::component::container::MechanicalObject<Rigid3Types> *mo, std::map<int, Quat> &globalQuatMap, int indent = 0, bool firstcall = true);
    int CalcNumAxes(aiVector3D &vec);
    std::string GetAxes(aiVector3D &vec, aiJoint::JointType & type);
    void GetJointMinMaxValues(aiJoint *j, const std::map<std::string, aiJoint *> &jointMap, std::vector<double> &minValues, std::vector<double> &maxValues, std::vector<bool> &invertAxis, std::vector<std::pair<int, std::string> > & actuators, std::vector<std::string > & jointNames);
    void Display3DText(Vec3d v, std::string str);
    void DecomposeAiMatrix(const aiMatrix4x4 &trans, sofa::defaulttype::Vec3d &translation, sofa::defaulttype::Vec3d &scale, sofa::defaulttype::Quat &quaternion, sofa::defaulttype::Vec3d & rotation);
    void DecomposeAiMatrix(const aiMatrix4x4 &trans, Vec3d &translation, Quat &quaternion);
    void DrawLine(const core::visual::VisualParams *vparams, Vec3d from, Vec3d to, const Vec<4, float> &color, const std::string &text ="");
    aiVector3D getAxis(aiRigidBodyConstraint *rbc, double &minN, double &maxN, bool &noDof);
    //bool hasObbTreeGPUActive();
    void * getRobotConnector();
	void fillObbTreeBlacklists(std::map<std::string, std::vector<std::string> > &connected, std::map<std::string, std::string> &jointForMesh, const std::map<std::string, aiJoint*>  &jointMap, std::map<std::string, aiJoint*> & joints, std::map < std::string, std::vector<std::string> > &blacklist);
	void fillObbTreeBlacklists2(std::map < std::string, std::vector<std::string> > &blacklist);
    void CheckSolverOrder();
    // Zykl.io begin
    aiMesh* getAiMeshByName(std::string meshName, const aiScene* colladaScene);
    // Zykl.io end

    sofa::component::container::MechanicalObject<Vec3dTypes>::SPtr AddSlidingLine(int i, sofa::component::container::MechanicalObject<Rigid3dTypes>::SPtr& src, Node::SPtr& anchorNode, Vec3d* from, Vec3d* to = NULL);

private:

    // build the joints and bones array used in the SkeletalMotionConstraint
    bool fillSkeletalInfo(const aiScene* scene, aiNode* meshParentNode, aiNode* meshNode, aiMatrix4x4 meshTransformation, aiMesh* mesh, helper::vector<SkeletonJoint<Rigid3dTypes> >& skeletonJoints, helper::vector<SkeletonBone>& skeletonBones) const;

    // clean the scene graph of its empty and useless intermediary nodes
    void removeEmptyNodes();

public:

    virtual std::string type() { return "The format of this scene is Collada (.dae)."; }

private:
    Node::SPtr subSceneRoot;		// the Node containing the whole Collada loaded scene

    Assimp::Importer* importer;		// the Assimp importer used to easily load the Collada scene

	Data<float> animationSpeed;

#ifdef SOFA_HAVE_PLUGIN_FLEXIBLE
	Data<bool> useFlexible;
#endif
#ifdef SOFA_HAVE_PLUGIN_IMAGE
    Data<bool> generateShapeFunction;
    Data<SReal> voxelSize;
#endif
    Data<int> generateCollisionModels;
    Data<bool> loadScene;
    Data<bool> explicitWhitelist;
    Data<bool> useContactManifolds;
    Data<unsigned int> maxNumberOfLineLineManifolds;
    Data<unsigned int> maxNumberOfFaceVertexManifolds;
    Data<unsigned int> maxNumberOfManifolds;

    int __joint_index;

    std::vector<MeshTransform> meshTransformations;

	ColladaTransformHelper* m_transformHelper;
	std::map<std::string, aiJoint*> m_objectRootJoints;

	std::map<std::string, aiJoint*> jointByObjectName;
	std::map<std::string, aiJoint*> jointByJointName;
	std::map<std::string, std::string> jointNameToJointID;
	std::map<std::string, std::string> jointIDToJointName;

	std::map<std::string, aiRigidBodyConstraint*> rbcByName;
	std::map<std::string, aiPhysicsModel*> physicsModelsByName;

    ZyColladaLoaderPrivate* d;
};

} // namespace loader

} // namespace component

} // namespace sofa

#endif
