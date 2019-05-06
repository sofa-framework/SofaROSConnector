
#include "iostream"
#include "iomanip"
#include "list"


#include "code/Converter.h"
#include "assimp/scene.h"


#define DBG(x) // std::cout << x


using namespace Assimp;
using namespace std;

typedef unsigned int uint;
Converter::Converter()
{

}

bool Assimp::Converter::collectMeshesByNameInplace(const aiScene *inScene)
{

    aiScene *outScene = const_cast<aiScene*>(inScene);
    DBG("\n===========================" <<
          "Resorting " << inScene->mNumMeshes << " mMeshes\n");

    list<aiMesh*> outMeshesList;
    list<aiNode*> nodeList;

    nodeList.push_back(inScene->mRootNode);


    while(!nodeList.empty()) {
        aiNode* current = nodeList.front();
        nodeList.pop_front();

        // Push all Children to the nodeList
        for (uint i = 0; i < current->mNumChildren; i++) {
            nodeList.push_back(current->mChildren[i]);
        }
        // Zykl.io begin
        bool isVisMesh = false;
        // Zykl.io end
        if (current->mNumMeshes > 0) {
            DBG("Merging " << current->mNumMeshes << " into one mesh\n");

            // Count Faces/vertices of all meshes
            int facesCount = 0;
            int verticesCount = 0;
            int boneCount = 0;
            for (uint i = 0; i < current->mNumMeshes; i++) {

                aiMesh * cmesh = inScene->mMeshes[current->mMeshes[i]];
                facesCount += cmesh->mNumFaces;
                verticesCount += cmesh->mNumVertices;
                boneCount += cmesh->mNumBones;
                // Zykl.io begin
                isVisMesh = isVisMesh || cmesh->isVisualMesh;
                // Zykl.io end
            }


            DBG("Counting " << facesCount << " faces and " << verticesCount << " vertices\n");

            // Create new Faces and vertex arrays
            aiMesh * bigMesh = new aiMesh();
            outMeshesList.push_back(bigMesh);

            bigMesh->mNumVertices = verticesCount;
            bigMesh->mVertices = new aiVector3D[verticesCount];
            bigMesh->mNormals = new aiVector3D[verticesCount];

            bigMesh->mNumFaces = facesCount;
            bigMesh->mFaces = new aiFace[facesCount];

            bigMesh->mNumBones = boneCount;
            bigMesh->mBones =  new aiBone*[boneCount];

            bigMesh->mNumMaterialGroups = current->mNumMeshes;
            bigMesh->mMaterialGroups = new unsigned int[bigMesh->mNumMaterialGroups];
            bigMesh->mGroupsMaterialIndex = new unsigned int[bigMesh->mNumMaterialGroups];

            // Zykl.io begin
            bigMesh->isVisualMesh = isVisMesh;
            // Zykl.io end

            // Add All vertices and faces to the bigMesh
            int vertexOffset = 0;
            int boneOffset = 0;
            int facePos = 0;


            for (uint i = 0; i < current->mNumMeshes; i++) {

                aiMesh * cmesh = inScene->mMeshes[current->mMeshes[i]];
                bigMesh->mName = cmesh->mName;
               /* if (i == 0) { // Obsolete: use Materiel of first Mesh for bigMesh
                    bigMesh->mMaterialIndex = cmesh->mMaterialIndex;
                }*/
                // Copy Material of current mesh as new group;
                bigMesh->mMaterialGroups[i] = cmesh->mNumVertices;
                bigMesh->mGroupsMaterialIndex[i] = cmesh->mMaterialIndex;

                // Copy the vertex array of the current mesh to the right position (+vertexOffset) in the big mesh
                memcpy(bigMesh->mVertices + vertexOffset, cmesh->mVertices, sizeof(aiVector3D)*cmesh->mNumVertices);
                // copy normals (if there are any)
                if (cmesh->mNormals != NULL) {
                    memcpy(bigMesh->mNormals + vertexOffset, cmesh->mNormals, sizeof(aiVector3D)*cmesh->mNumVertices);
                }

                // Go over all faces and copy the face indices (+vertexOffset) to the big mesh
                for (uint f = 0; f < cmesh->mNumFaces; f++) {
                    aiFace cface = cmesh->mFaces[f];
                    aiFace *bface = &bigMesh->mFaces[facePos++];
                    bface->mNumIndices = cface.mNumIndices;
                    bface->mIndices = new uint[cface.mNumIndices];

                    // Copy the vertex indices of current face
                    for (uint v = 0; v < cface.mNumIndices; v++) {
                        bface->mIndices[v] = cface.mIndices[v] + vertexOffset;
                    }
                }

                // Copy bones
                for (uint cbone = 0; cbone < cmesh->mNumBones; cbone++)
                {
                    aiBone* pc = new aiBone();
                    bigMesh->mBones[boneOffset+cbone] = pc;
                    aiBone* cc = cmesh->mBones[cbone];
                    pc->mName = cc->mName;
                    pc->mOffsetMatrix = cc->mOffsetMatrix;
                    pc->mNumWeights = cc->mNumWeights;
                    pc->mWeights = new aiVertexWeight[cc->mNumWeights];

                    // Copy Vertex Indices and Weights
                    for (unsigned int captainkirk = 0; captainkirk < pc->mNumWeights;++captainkirk)
                    {
                        aiVertexWeight weight = cc->mWeights[captainkirk];
                        pc->mWeights[captainkirk].mVertexId = weight.mVertexId + vertexOffset;
                        pc->mWeights[captainkirk].mWeight = weight.mWeight;
                    }
                }


                vertexOffset += cmesh->mNumVertices;
                boneOffset += cmesh->mNumBones;
            }
            //Todo: delete all current meshes;
            if (current->mNumMeshes > 0 && current->mMeshes) {
                delete[] current->mMeshes;
            }
            // Copy BigMesh to current node
            current->mNumMeshes = 1;
            current->mMeshes = new unsigned int[current->mNumMeshes];
            current->mMeshes[0]= outMeshesList.size()-1;
        }

    }

    DBG("\n===========================\n");
    // Delete old meshes
    for (uint i = 0; i < outScene->mNumMeshes; i++) {
        delete outScene->mMeshes[i];
    }
    delete [] outScene->mMeshes;

    // Write new Meshes
    outScene->mNumMeshes = outMeshesList.size();
    outScene->mMeshes = new aiMesh*[inScene->mNumMeshes];
    for (uint i = 0; i < inScene->mNumMeshes; i++) {
        inScene->mMeshes[i] = outMeshesList.front();
        outMeshesList.pop_front();
    }
    DBG("\n===========================\n" <<
        "Output:  " << inScene->mNumMeshes << " mMeshes\n");

    return true;
}
aiScene *Assimp::Converter::collectMeshesByName(const aiScene *sceneIn)
{
    aiScene * sceneOut = new aiScene();
    DBG("\n===========================" <<
          "Resorting " << sceneIn->mNumMeshes << " mMeshes\n");

    list<aiNode*> nodeList;
    list<aiMesh*> outMeshesList;
    list<aiNode*> outNodeList;

    nodeList.push_back(sceneIn->mRootNode);


    while(!nodeList.empty()) {
        aiNode* current = nodeList.front();
        nodeList.pop_front();
        aiNode* copyNode = new aiNode(current->mName.C_Str(), current->mId.C_Str());

        // Push all Children to the nodeList
        for (uint i = 0; i < current->mNumChildren; i++) {
            nodeList.push_back(current->mChildren[i]);
        }

        DBG("Name: " << current->mName.data << " (" << current->mNumMeshes << ")\n");


        if (current->mNumMeshes > 0) {
            outNodeList.push_back(copyNode);

            DBG("Merging " << current->mNumMeshes << " into one mesh\n");

            // Count Faces/vertices of all meshes
            int facesCount = 0;
            int verticesCount = 0;
            int boneCount = 0;
            for (uint i = 0; i < current->mNumMeshes; i++) {

                aiMesh * cmesh = sceneIn->mMeshes[current->mMeshes[i]];
                facesCount += cmesh->mNumFaces;
                verticesCount += cmesh->mNumVertices;
                boneCount += cmesh->mNumBones;
            }


            DBG("Counting " << facesCount << " faces and " << verticesCount << " vertices\n");

            // Create new Faces and vertex arrays
            aiMesh * bigMesh = new aiMesh();
            outMeshesList.push_back(bigMesh);
            copyNode->mNumMeshes = 1;
            copyNode->mMeshes = new uint[1];
            copyNode->mMeshes[0] = outMeshesList.size()-1;
            copyNode->mTransformation = current->mTransformation;

            bigMesh->mNumVertices = verticesCount;
            bigMesh->mVertices = new aiVector3D[verticesCount];
            bigMesh->mNormals = new aiVector3D[verticesCount];

            bigMesh->mNumFaces = facesCount;
            bigMesh->mFaces = new aiFace[facesCount];

            bigMesh->mNumBones = boneCount;
            bigMesh->mBones =  new aiBone*[boneCount];



            // Add All vertices and faces to the bigMesh
            int vertexOffset = 0;
            int boneOffset = 0;
            int facePos = 0;
            for (uint i = 0; i < current->mNumMeshes; i++) {
                aiMesh * cmesh = sceneIn->mMeshes[current->mMeshes[i]];
                bigMesh->mName = cmesh->mName;

                // Copy the vertex array of the current mesh to the right position (+vertexOffset) in the big mesh
                memcpy(bigMesh->mVertices + vertexOffset, cmesh->mVertices, sizeof(aiVector3D)*cmesh->mNumVertices);
                // copy normals (if there are any)
                if (cmesh->mNormals != NULL) {
                    memcpy(bigMesh->mNormals + vertexOffset, cmesh->mNormals, sizeof(aiVector3D)*cmesh->mNumVertices);
                }

                // Go over all faces and copy the face indices (+vertexOffset) to the big mesh
                for (uint f = 0; f < cmesh->mNumFaces; f++) {
                    aiFace cface = cmesh->mFaces[f];
                    aiFace *bface = &bigMesh->mFaces[facePos++];
                    bface->mNumIndices = cface.mNumIndices;
                    bface->mIndices = new uint[cface.mNumIndices];

                    // Copy the vertex indices of current face
                    for (uint v = 0; v < cface.mNumIndices; v++) {
                        bface->mIndices[v] = cface.mIndices[v] + vertexOffset;
                    }
                }

                // Copy bones

                for (uint cbone = 0; cbone < cmesh->mNumBones; cbone++)
                {
                    aiBone* pc = new aiBone();
                    bigMesh->mBones[boneOffset+cbone] = pc;
                    aiBone* cc = cmesh->mBones[cbone];
                    pc->mName = cc->mName;
                    pc->mOffsetMatrix = cc->mOffsetMatrix;
                    pc->mNumWeights = cc->mNumWeights;
                    pc->mWeights = new aiVertexWeight[cc->mNumWeights];

                    // Copy Vertex Indices and Weights
                    for (unsigned int captainkirk = 0; captainkirk < pc->mNumWeights;++captainkirk)
                    {
                        aiVertexWeight weight = cc->mWeights[captainkirk];
                        pc->mWeights[captainkirk].mVertexId = weight.mVertexId + vertexOffset;
                        pc->mWeights[captainkirk].mWeight = weight.mWeight;
                    }
                }


                vertexOffset += cmesh->mNumVertices;
                boneOffset += cmesh->mNumBones;
            }
        }
    }
    sceneOut->mNumMeshes = outMeshesList.size();
    sceneOut->mMeshes = new aiMesh*[sceneOut->mNumMeshes];
    for (uint i = 0; i < sceneOut->mNumMeshes; i++) {
        sceneOut->mMeshes[i] = outMeshesList.front();
        outMeshesList.pop_front();
    }
    sceneOut->mRootNode = new aiNode("MyRoot");
    sceneOut->mRootNode->mNumChildren = outNodeList.size();
    sceneOut->mRootNode->mChildren = new aiNode*[sceneOut->mRootNode->mNumChildren];

    for (uint i = 0; i < sceneOut->mRootNode->mNumChildren; i++) {
        sceneOut->mRootNode->mChildren[i] = outNodeList.front();
        DBG("Writing to outrootNode: " << sceneOut->mRootNode->mChildren[i]->mName.data << endl);
        sceneOut->mRootNode->mChildren[i]->mParent = sceneOut->mRootNode;
        outNodeList.pop_front();
    }

    DBG("\n===========================\n");

    for (uint i = 0; i < sceneOut->mNumMeshes; i++) {
        DBG("MeshId " << sceneOut->mMeshes[i]->mName.C_Str() << " with faces/verts: " <<  sceneOut->mMeshes[i]->mNumFaces << "/" << sceneOut->mMeshes[i]->mNumVertices << endl);
    }


    for (uint i = 0; i < sceneOut->mRootNode->mNumChildren; i++) {
        aiNode* current = sceneOut->mRootNode->mChildren[i];
        DBG("Node: " << current->mName.data << " with " << current->mNumMeshes << " mesh ID" << current->mMeshes[0] << endl);
    }

    DBG("\n===========================\n");


    // Copy the rest
    //sceneOut->mNumAnimations = sceneIn->mNumAnimations;
   // sceneOut->mAnimations = sceneIn->mAnimations;
    sceneOut->mNumMaterials = sceneIn->mNumMaterials;
    sceneOut->mMaterials = sceneIn->mMaterials;
    sceneOut->mNumJoints = sceneIn->mNumJoints;
    sceneOut->mJoints = sceneIn->mJoints;
    sceneOut->mUpDirection = sceneIn->mUpDirection;
    sceneOut->mFlags = sceneIn->mFlags;
    sceneOut->mNumPhysicsModel = sceneIn->mNumPhysicsModel;
    sceneOut->mPhysicsModel = sceneIn->mPhysicsModel;

    return sceneOut;
}
