cmake_minimum_required(VERSION 2.8.12)

project(ZyColladaLoader)

SET(ASSIMP_HEADER_PATH ${CMAKE_CURRENT_SOURCE_DIR}/../assimp-3.0.1270/include/assimp)
SET(ASSIMP_CODE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/../assimp-3.0.1270/code)

SET( COMPILER_HEADERS
        ${ASSIMP_HEADER_PATH}/Compiler/pushpack1.h
        ${ASSIMP_HEADER_PATH}/Compiler/poppack1.h
        ${ASSIMP_CODE_PATH}/pstdint.h
)

SET( PUBLIC_HEADERS
        ${ASSIMP_HEADER_PATH}/anim.h
        ${ASSIMP_HEADER_PATH}/ai_assert.h
        ${ASSIMP_HEADER_PATH}/camera.h
        ${ASSIMP_HEADER_PATH}/color4.h
        ${ASSIMP_HEADER_PATH}/color4.inl
        ${ASSIMP_HEADER_PATH}/config.h
        ${ASSIMP_HEADER_PATH}/defs.h
        ${ASSIMP_HEADER_PATH}/cfileio.h
        ${ASSIMP_HEADER_PATH}/light.h
        ${ASSIMP_HEADER_PATH}/material.h
        ${ASSIMP_HEADER_PATH}/material.inl
        ${ASSIMP_HEADER_PATH}/matrix3x3.h
        ${ASSIMP_HEADER_PATH}/matrix3x3.inl
        ${ASSIMP_HEADER_PATH}/matrix4x4.h
        ${ASSIMP_HEADER_PATH}/matrix4x4.inl
        ${ASSIMP_HEADER_PATH}/mesh.h
        ${ASSIMP_HEADER_PATH}/postprocess.h
        ${ASSIMP_HEADER_PATH}/quaternion.h
        ${ASSIMP_HEADER_PATH}/quaternion.inl
        ${ASSIMP_HEADER_PATH}/scene.h
        ${ASSIMP_HEADER_PATH}/texture.h
        ${ASSIMP_HEADER_PATH}/types.h
        ${ASSIMP_HEADER_PATH}/vector2.h
        ${ASSIMP_HEADER_PATH}/vector2.inl
        ${ASSIMP_HEADER_PATH}/vector3.h
        ${ASSIMP_HEADER_PATH}/vector3.inl
        ${ASSIMP_HEADER_PATH}/version.h
        ${ASSIMP_HEADER_PATH}/cimport.h
        ${ASSIMP_HEADER_PATH}/importerdesc.h
        ${ASSIMP_HEADER_PATH}/Importer.hpp
        ${ASSIMP_HEADER_PATH}/DefaultLogger.hpp
        ${ASSIMP_HEADER_PATH}/ProgressHandler.hpp
        ${ASSIMP_HEADER_PATH}/IOStream.hpp
        ${ASSIMP_HEADER_PATH}/IOSystem.hpp
        ${ASSIMP_HEADER_PATH}/Logger.hpp
        ${ASSIMP_HEADER_PATH}/LogStream.hpp
        ${ASSIMP_HEADER_PATH}/NullLogger.hpp
        ${ASSIMP_HEADER_PATH}/cexport.h
        ${ASSIMP_HEADER_PATH}/Exporter.hpp
)

SET( Core_SRCS
        ${ASSIMP_CODE_PATH}/Assimp.cpp
        ${ASSIMP_CODE_PATH}/AssimpPCH.cpp
        ${ASSIMP_CODE_PATH}/AssimpPCH.h
)

SET( Boost_SRCS
        ${ASSIMP_CODE_PATH}/BoostWorkaround/boost/math/common_factor_rt.hpp
        ${ASSIMP_CODE_PATH}/BoostWorkaround/boost/foreach.hpp
        ${ASSIMP_CODE_PATH}/BoostWorkaround/boost/format.hpp
        ${ASSIMP_CODE_PATH}/BoostWorkaround/boost/scoped_array.hpp
        ${ASSIMP_CODE_PATH}/BoostWorkaround/boost/scoped_ptr.hpp
        ${ASSIMP_CODE_PATH}/BoostWorkaround/boost/shared_array.hpp
        ${ASSIMP_CODE_PATH}/BoostWorkaround/boost/shared_ptr.hpp
        ${ASSIMP_CODE_PATH}/BoostWorkaround/boost/make_shared.hpp
        ${ASSIMP_CODE_PATH}/BoostWorkaround/boost/static_assert.hpp
        ${ASSIMP_CODE_PATH}/BoostWorkaround/boost/tuple/tuple.hpp
)

SET( Logging_SRCS
        ${ASSIMP_HEADER_PATH}/DefaultLogger.hpp
        ${ASSIMP_HEADER_PATH}/IOStream.hpp
        ${ASSIMP_HEADER_PATH}/LogStream.hpp
        ${ASSIMP_HEADER_PATH}/Logger.hpp
        ${ASSIMP_HEADER_PATH}/NullLogger.hpp
        ${ASSIMP_CODE_PATH}/Win32DebugLogStream.h
        ${ASSIMP_CODE_PATH}/DefaultLogger.cpp
        ${ASSIMP_CODE_PATH}/FileLogStream.h
)

SET( Common_SRCS
        ${ASSIMP_CODE_PATH}/fast_atof.h
        ${ASSIMP_CODE_PATH}/qnan.h
        ${ASSIMP_CODE_PATH}/BaseImporter.cpp
        ${ASSIMP_CODE_PATH}/BaseImporter.h
        ${ASSIMP_CODE_PATH}/BaseProcess.cpp
        ${ASSIMP_CODE_PATH}/BaseProcess.h
        ${ASSIMP_CODE_PATH}/Importer.h
        ${ASSIMP_CODE_PATH}/ScenePrivate.h
        ${ASSIMP_CODE_PATH}/PostStepRegistry.cpp
        ${ASSIMP_CODE_PATH}/ImporterRegistry.cpp
        ${ASSIMP_CODE_PATH}/ByteSwap.h
        ${ASSIMP_CODE_PATH}/DefaultProgressHandler.h
        ${ASSIMP_CODE_PATH}/DefaultIOStream.cpp
        ${ASSIMP_CODE_PATH}/DefaultIOStream.h
        ${ASSIMP_CODE_PATH}/DefaultIOSystem.cpp
        ${ASSIMP_CODE_PATH}/DefaultIOSystem.h
        ${ASSIMP_CODE_PATH}/CInterfaceIOWrapper.h
        ${ASSIMP_CODE_PATH}/Hash.h
        ${ASSIMP_CODE_PATH}/Importer.cpp
        ${ASSIMP_CODE_PATH}/IFF.h
        ${ASSIMP_CODE_PATH}/ParsingUtils.h
        ${ASSIMP_CODE_PATH}/StdOStreamLogStream.h
        ${ASSIMP_CODE_PATH}/StreamReader.h
        ${ASSIMP_CODE_PATH}/StringComparison.h
        ${ASSIMP_CODE_PATH}/SGSpatialSort.cpp
        ${ASSIMP_CODE_PATH}/SGSpatialSort.h
        ${ASSIMP_CODE_PATH}/VertexTriangleAdjacency.cpp
        ${ASSIMP_CODE_PATH}/VertexTriangleAdjacency.h
        ${ASSIMP_CODE_PATH}/GenericProperty.h
        ${ASSIMP_CODE_PATH}/SpatialSort.cpp
        ${ASSIMP_CODE_PATH}/SpatialSort.h
        ${ASSIMP_CODE_PATH}/SceneCombiner.cpp
        ${ASSIMP_CODE_PATH}/SceneCombiner.h
        ${ASSIMP_CODE_PATH}/ScenePreprocessor.cpp
        ${ASSIMP_CODE_PATH}/ScenePreprocessor.h
        ${ASSIMP_CODE_PATH}/SkeletonMeshBuilder.cpp
        ${ASSIMP_CODE_PATH}/SkeletonMeshBuilder.h
        ${ASSIMP_CODE_PATH}/SplitByBoneCountProcess.cpp
        ${ASSIMP_CODE_PATH}/SplitByBoneCountProcess.h
        ${ASSIMP_CODE_PATH}/SmoothingGroups.h
        ${ASSIMP_CODE_PATH}/StandardShapes.cpp
        ${ASSIMP_CODE_PATH}/StandardShapes.h
        ${ASSIMP_CODE_PATH}/TargetAnimation.cpp
        ${ASSIMP_CODE_PATH}/TargetAnimation.h
        ${ASSIMP_CODE_PATH}/RemoveComments.cpp
        ${ASSIMP_CODE_PATH}/RemoveComments.h
        ${ASSIMP_CODE_PATH}/Subdivision.cpp
        ${ASSIMP_CODE_PATH}/Subdivision.h
        ${ASSIMP_CODE_PATH}/Vertex.h
        ${ASSIMP_CODE_PATH}/LineSplitter.h
        ${ASSIMP_CODE_PATH}/TinyFormatter.h
        ${ASSIMP_CODE_PATH}/Profiler.h
        ${ASSIMP_CODE_PATH}/LogAux.h
)

SET( 3DS_SRCS
        ${ASSIMP_CODE_PATH}/3DSConverter.cpp
        ${ASSIMP_CODE_PATH}/3DSHelper.h
        ${ASSIMP_CODE_PATH}/3DSLoader.cpp
        ${ASSIMP_CODE_PATH}/3DSLoader.h
)
SOURCE_GROUP(3DS FILES ${3DS_SRCS})

SET( AC_SRCS
        ${ASSIMP_CODE_PATH}/ACLoader.cpp
        ${ASSIMP_CODE_PATH}/ACLoader.h
)
SOURCE_GROUP( AC FILES ${AC_SRCS})

SET( ASE_SRCS
        ${ASSIMP_CODE_PATH}/ASELoader.cpp
        ${ASSIMP_CODE_PATH}/ASELoader.h
        ${ASSIMP_CODE_PATH}/ASEParser.cpp
        ${ASSIMP_CODE_PATH}/ASEParser.h
)
SOURCE_GROUP( ASE FILES ${ASE_SRCS})

SET( B3D_SRCS
        ${ASSIMP_CODE_PATH}/B3DImporter.cpp
        ${ASSIMP_CODE_PATH}/B3DImporter.h
)
SOURCE_GROUP( B3D FILES ${B3D_SRCS})

SET( BVH_SRCS
        ${ASSIMP_CODE_PATH}/BVHLoader.cpp
        ${ASSIMP_CODE_PATH}/BVHLoader.h
)
SOURCE_GROUP( BVH FILES ${BVH_SRCS})

SET( Collada_SRCS
        ${ASSIMP_CODE_PATH}/ColladaHelper.h
        ${ASSIMP_CODE_PATH}/ColladaLoader.cpp
        ${ASSIMP_CODE_PATH}/ColladaLoader.h
        ${ASSIMP_CODE_PATH}/ColladaParser.cpp
        ${ASSIMP_CODE_PATH}/ColladaParser.h
        ${ASSIMP_CODE_PATH}/ColladaExporter.h
        ${ASSIMP_CODE_PATH}/ColladaExporter.cpp
)
SOURCE_GROUP( Collada FILES ${Collada_SRCS})

SET( DXF_SRCS
        ${ASSIMP_CODE_PATH}/DXFLoader.cpp
        ${ASSIMP_CODE_PATH}/DXFLoader.h
        ${ASSIMP_CODE_PATH}/DXFHelper.h
)
SOURCE_GROUP( DXF FILES ${DXF_SRCS})

SET( CSM_SRCS
        ${ASSIMP_CODE_PATH}/CSMLoader.cpp
        ${ASSIMP_CODE_PATH}/CSMLoader.h
)
SOURCE_GROUP( CSM FILES ${CSM_SRCS})

SET( HMP_SRCS
        ${ASSIMP_CODE_PATH}/HMPFileData.h
        ${ASSIMP_CODE_PATH}/HMPLoader.cpp
        ${ASSIMP_CODE_PATH}/HMPLoader.h
        ${ASSIMP_CODE_PATH}/HalfLifeFileData.h
)
SOURCE_GROUP( HMP FILES ${HMP_SRCS})

SET( Irr_SRCS
        ${ASSIMP_CODE_PATH}/IRRLoader.cpp
        ${ASSIMP_CODE_PATH}/IRRLoader.h
        ${ASSIMP_CODE_PATH}/IRRMeshLoader.cpp
        ${ASSIMP_CODE_PATH}/IRRMeshLoader.h
        ${ASSIMP_CODE_PATH}/IRRShared.cpp
        ${ASSIMP_CODE_PATH}/IRRShared.h
)
SOURCE_GROUP( Irr FILES ${Irr_SRCS})

SET( LWO_SRCS
        ${ASSIMP_CODE_PATH}/LWOAnimation.cpp
        ${ASSIMP_CODE_PATH}/LWOAnimation.h
        ${ASSIMP_CODE_PATH}/LWOBLoader.cpp
        ${ASSIMP_CODE_PATH}/LWOFileData.h
        ${ASSIMP_CODE_PATH}/LWOLoader.cpp
        ${ASSIMP_CODE_PATH}/LWOLoader.h
        ${ASSIMP_CODE_PATH}/LWOMaterial.cpp
)
SOURCE_GROUP( LWO FILES ${LWO_SRCS})

SET( LWS_SRCS
        ${ASSIMP_CODE_PATH}/LWSLoader.cpp
        ${ASSIMP_CODE_PATH}/LWSLoader.h
)
SOURCE_GROUP( LWS FILES ${LWS_SRCS})

SET ( M3_SRCS
        ${ASSIMP_CODE_PATH}/M3Importer.cpp
        ${ASSIMP_CODE_PATH}/M3Importer.h
)
SOURCE_GROUP( M3 FILES ${M3_SRCS} )

SET( MD2_SRCS
        ${ASSIMP_CODE_PATH}/MD2FileData.h
        ${ASSIMP_CODE_PATH}/MD2Loader.cpp
        ${ASSIMP_CODE_PATH}/MD2Loader.h
        ${ASSIMP_CODE_PATH}/MD2NormalTable.h
)
SOURCE_GROUP( MD2 FILES ${MD2_SRCS})

SET( MD3_SRCS
        ${ASSIMP_CODE_PATH}/MD3FileData.h
        ${ASSIMP_CODE_PATH}/MD3Loader.cpp
        ${ASSIMP_CODE_PATH}/MD3Loader.h
)
SOURCE_GROUP( MD3 FILES ${MD3_SRCS})

SET( MD5_SRCS
        ${ASSIMP_CODE_PATH}/MD5Loader.cpp
        ${ASSIMP_CODE_PATH}/MD5Loader.h
        ${ASSIMP_CODE_PATH}/MD5Parser.cpp
        ${ASSIMP_CODE_PATH}/MD5Parser.h
)
SOURCE_GROUP( MD5 FILES ${MD5_SRCS})

SET( MDC_SRCS
        ${ASSIMP_CODE_PATH}/MDCFileData.h
        ${ASSIMP_CODE_PATH}/MDCLoader.cpp
        ${ASSIMP_CODE_PATH}/MDCLoader.h
        ${ASSIMP_CODE_PATH}/MDCNormalTable.h
)
SOURCE_GROUP( MDC FILES ${MDC_SRCS})

SET( MDL_SRCS
        ${ASSIMP_CODE_PATH}/MDLDefaultColorMap.h
        ${ASSIMP_CODE_PATH}/MDLFileData.h
        ${ASSIMP_CODE_PATH}/MDLLoader.cpp
        ${ASSIMP_CODE_PATH}/MDLLoader.h
        ${ASSIMP_CODE_PATH}/MDLMaterialLoader.cpp
)
SOURCE_GROUP( MDL FILES ${MDL_SRCS})

SET( MaterialSystem_SRCS
        ${ASSIMP_CODE_PATH}/MaterialSystem.cpp
        ${ASSIMP_CODE_PATH}/MaterialSystem.h
)
SOURCE_GROUP( MaterialSystem FILES ${MaterialSystem_SRCS})

SET( NFF_SRCS
        ${ASSIMP_CODE_PATH}/NFFLoader.cpp
        ${ASSIMP_CODE_PATH}/NFFLoader.h
)
SOURCE_GROUP( NFF FILES ${NFF_SRCS})

SET( NDO_SRCS
        ${ASSIMP_CODE_PATH}/NDOLoader.cpp
        ${ASSIMP_CODE_PATH}/NDOLoader.h
)
SOURCE_GROUP( NDO FILES ${NDO_SRCS})

SET( OFFFormat_SRCS
        ${ASSIMP_CODE_PATH}/OFFLoader.cpp
        ${ASSIMP_CODE_PATH}/OFFLoader.h
)
SOURCE_GROUP( OFFFormat FILES ${OFFFormat_SRCS})

SET( Obj_SRCS
        ${ASSIMP_CODE_PATH}/ObjFileData.h
        ${ASSIMP_CODE_PATH}/ObjFileImporter.cpp
        ${ASSIMP_CODE_PATH}/ObjFileImporter.h
        ${ASSIMP_CODE_PATH}/ObjFileMtlImporter.cpp
        ${ASSIMP_CODE_PATH}/ObjFileMtlImporter.h
        ${ASSIMP_CODE_PATH}/ObjFileParser.cpp
        ${ASSIMP_CODE_PATH}/ObjFileParser.h
        ${ASSIMP_CODE_PATH}/ObjTools.h

        ${ASSIMP_CODE_PATH}/ObjExporter.h
        ${ASSIMP_CODE_PATH}/ObjExporter.cpp
)
SOURCE_GROUP( Obj FILES ${Obj_SRCS})

SET( Ogre_SRCS
        ${ASSIMP_CODE_PATH}/OgreImporter.hpp
        ${ASSIMP_CODE_PATH}/OgreXmlHelper.hpp
        ${ASSIMP_CODE_PATH}/OgreImporter.cpp
        ${ASSIMP_CODE_PATH}/OgreMaterial.cpp
        ${ASSIMP_CODE_PATH}/OgreMesh.cpp
        ${ASSIMP_CODE_PATH}/OgreSkeleton.cpp
)
SOURCE_GROUP( Ogre FILES ${Ogre_SRCS})

SET( Ply_SRCS
        ${ASSIMP_CODE_PATH}/PlyLoader.cpp
        ${ASSIMP_CODE_PATH}/PlyLoader.h
        ${ASSIMP_CODE_PATH}/PlyParser.cpp
        ${ASSIMP_CODE_PATH}/PlyParser.h
        ${ASSIMP_CODE_PATH}/PlyExporter.cpp
        ${ASSIMP_CODE_PATH}/PlyExporter.h
)
SOURCE_GROUP( Ply FILES ${Ply_SRCS})

SET(MS3D_SRCS
        ${ASSIMP_CODE_PATH}/MS3DLoader.cpp
        ${ASSIMP_CODE_PATH}/MS3DLoader.h
)
SOURCE_GROUP( MS3D FILES ${MS3D_SRCS})

SET(COB_SRCS
        ${ASSIMP_CODE_PATH}/COBLoader.cpp
        ${ASSIMP_CODE_PATH}/COBLoader.h
        ${ASSIMP_CODE_PATH}/COBScene.h
)
SOURCE_GROUP( COB FILES ${COB_SRCS})

SET(BLENDER_SRCS
        ${ASSIMP_CODE_PATH}/BlenderLoader.cpp
        ${ASSIMP_CODE_PATH}/BlenderLoader.h
        ${ASSIMP_CODE_PATH}/BlenderDNA.cpp
        ${ASSIMP_CODE_PATH}/BlenderDNA.h
        ${ASSIMP_CODE_PATH}/BlenderDNA.inl
        ${ASSIMP_CODE_PATH}/BlenderScene.cpp
        ${ASSIMP_CODE_PATH}/BlenderScene.h
        ${ASSIMP_CODE_PATH}/BlenderSceneGen.h
        ${ASSIMP_CODE_PATH}/BlenderIntermediate.h
        ${ASSIMP_CODE_PATH}/BlenderModifier.h
        ${ASSIMP_CODE_PATH}/BlenderModifier.cpp
)
SOURCE_GROUP( BLENDER FILES ${BLENDER_SRCS})

# Does not compile with gcc 7.x - not relevant for the customized assimp library, since we rely on the COLLADA format mainly.
#SET(IFC_SRCS
#	IFCLoader.cpp
#	IFCLoader.h
#	IFCReaderGen.cpp
#	IFCReaderGen.h
#	IFCUtil.h
#	IFCUtil.cpp
#	IFCGeometry.cpp
#	IFCMaterial.cpp
#	IFCProfile.cpp
#	IFCCurve.cpp
#	STEPFile.h
#	STEPFileReader.h
#	STEPFileReader.cpp
#)
#SOURCE_GROUP( IFC FILES ${IFC_SRCS})

SET( XGL_SRCS
        ${ASSIMP_CODE_PATH}/XGLLoader.cpp
        ${ASSIMP_CODE_PATH}/XGLLoader.h
)
SOURCE_GROUP( XGL FILES ${XGL_SRCS})


SET( PostProcessing_SRCS
        ${ASSIMP_CODE_PATH}/Converter.cpp
        ${ASSIMP_CODE_PATH}/Converter.h
        ${ASSIMP_CODE_PATH}/CalcTangentsProcess.cpp
        ${ASSIMP_CODE_PATH}/CalcTangentsProcess.h
        ${ASSIMP_CODE_PATH}/ComputeUVMappingProcess.cpp
        ${ASSIMP_CODE_PATH}/ComputeUVMappingProcess.h
        ${ASSIMP_CODE_PATH}/ConvertToLHProcess.cpp
        ${ASSIMP_CODE_PATH}/ConvertToLHProcess.h
        ${ASSIMP_CODE_PATH}/FindDegenerates.cpp
        ${ASSIMP_CODE_PATH}/FindDegenerates.h
        ${ASSIMP_CODE_PATH}/FindInstancesProcess.cpp
        ${ASSIMP_CODE_PATH}/FindInstancesProcess.h
        ${ASSIMP_CODE_PATH}/FindInvalidDataProcess.cpp
        ${ASSIMP_CODE_PATH}/FindInvalidDataProcess.h
        ${ASSIMP_CODE_PATH}/FixNormalsStep.cpp
        ${ASSIMP_CODE_PATH}/FixNormalsStep.h
        ${ASSIMP_CODE_PATH}/GenFaceNormalsProcess.cpp
        ${ASSIMP_CODE_PATH}/GenFaceNormalsProcess.h
        ${ASSIMP_CODE_PATH}/GenVertexNormalsProcess.cpp
        ${ASSIMP_CODE_PATH}/GenVertexNormalsProcess.h
        ${ASSIMP_CODE_PATH}/PretransformVertices.cpp
        ${ASSIMP_CODE_PATH}/PretransformVertices.h
        ${ASSIMP_CODE_PATH}/ImproveCacheLocality.cpp
        ${ASSIMP_CODE_PATH}/ImproveCacheLocality.h
        ${ASSIMP_CODE_PATH}/JoinVerticesProcess.cpp
        ${ASSIMP_CODE_PATH}/JoinVerticesProcess.h
        ${ASSIMP_CODE_PATH}/LimitBoneWeightsProcess.cpp
        ${ASSIMP_CODE_PATH}/LimitBoneWeightsProcess.h
        ${ASSIMP_CODE_PATH}/RemoveRedundantMaterials.cpp
        ${ASSIMP_CODE_PATH}/RemoveRedundantMaterials.h
        ${ASSIMP_CODE_PATH}/RemoveVCProcess.cpp
        ${ASSIMP_CODE_PATH}/RemoveVCProcess.h
        ${ASSIMP_CODE_PATH}/SortByPTypeProcess.cpp
        ${ASSIMP_CODE_PATH}/SortByPTypeProcess.h
        ${ASSIMP_CODE_PATH}/SplitLargeMeshes.cpp
        ${ASSIMP_CODE_PATH}/SplitLargeMeshes.h
        ${ASSIMP_CODE_PATH}/TerragenLoader.cpp
        ${ASSIMP_CODE_PATH}/TerragenLoader.h
        ${ASSIMP_CODE_PATH}/TextureTransform.cpp
        ${ASSIMP_CODE_PATH}/TextureTransform.h
        ${ASSIMP_CODE_PATH}/TriangulateProcess.cpp
        ${ASSIMP_CODE_PATH}/TriangulateProcess.h
        ${ASSIMP_CODE_PATH}/ValidateDataStructure.cpp
        ${ASSIMP_CODE_PATH}/ValidateDataStructure.h
        ${ASSIMP_CODE_PATH}/OptimizeGraph.cpp
        ${ASSIMP_CODE_PATH}/OptimizeGraph.h
        ${ASSIMP_CODE_PATH}/OptimizeMeshes.cpp
        ${ASSIMP_CODE_PATH}/OptimizeMeshes.h
        ${ASSIMP_CODE_PATH}/DeboneProcess.cpp
        ${ASSIMP_CODE_PATH}/DeboneProcess.h
        ${ASSIMP_CODE_PATH}/ProcessHelper.h
        ${ASSIMP_CODE_PATH}/ProcessHelper.cpp
        ${ASSIMP_CODE_PATH}/PolyTools.h
        ${ASSIMP_CODE_PATH}/MakeVerboseFormat.cpp
        ${ASSIMP_CODE_PATH}/MakeVerboseFormat.h
)
SOURCE_GROUP( PostProcessing FILES ${PostProcessing_SRCS})

SET( Q3D_SRCS
        ${ASSIMP_CODE_PATH}/Q3DLoader.cpp
        ${ASSIMP_CODE_PATH}/Q3DLoader.h
)
SOURCE_GROUP( Q3D FILES ${Q3D_SRCS})

SET( Q3BSP_SRCS
        ${ASSIMP_CODE_PATH}/Q3BSPFileData.h
        ${ASSIMP_CODE_PATH}/Q3BSPFileParser.h
        ${ASSIMP_CODE_PATH}/Q3BSPFileParser.cpp
        ${ASSIMP_CODE_PATH}/Q3BSPFileImporter.h
        ${ASSIMP_CODE_PATH}/Q3BSPFileImporter.cpp
        ${ASSIMP_CODE_PATH}/Q3BSPZipArchive.h
        ${ASSIMP_CODE_PATH}/Q3BSPZipArchive.cpp
)
SOURCE_GROUP( Q3BSP FILES ${Q3BSP_SRCS})

SET( Raw_SRCS
        ${ASSIMP_CODE_PATH}/RawLoader.cpp
        ${ASSIMP_CODE_PATH}/RawLoader.h
)
SOURCE_GROUP( Raw FILES ${Raw_SRCS})

SET( SMD_SRCS
        ${ASSIMP_CODE_PATH}/SMDLoader.cpp
        ${ASSIMP_CODE_PATH}/SMDLoader.h
)
SOURCE_GROUP( SMD FILES ${SMD_SRCS})

SET( STL_SRCS
        ${ASSIMP_CODE_PATH}/STLLoader.cpp
        ${ASSIMP_CODE_PATH}/STLLoader.h
        ${ASSIMP_CODE_PATH}/STLExporter.h
        ${ASSIMP_CODE_PATH}/STLExporter.cpp
)
SOURCE_GROUP( STL FILES ${STL_SRCS})

SET( Unreal_SRCS
        ${ASSIMP_CODE_PATH}/UnrealLoader.cpp
        ${ASSIMP_CODE_PATH}/UnrealLoader.h
)
SOURCE_GROUP( Unreal FILES ${Unreal_SRCS})

SET( XFile_SRCS
        ${ASSIMP_CODE_PATH}/XFileHelper.h
        ${ASSIMP_CODE_PATH}/XFileImporter.cpp
        ${ASSIMP_CODE_PATH}/XFileImporter.h
        ${ASSIMP_CODE_PATH}/XFileParser.cpp
        ${ASSIMP_CODE_PATH}/XFileParser.h
)
SOURCE_GROUP( XFile FILES ${XFile_SRCS})

SET( Exporter_SRCS
        ${ASSIMP_CODE_PATH}/Exporter.cpp
        ${ASSIMP_CODE_PATH}/AssimpCExport.cpp
        ${ASSIMP_CODE_PATH}/BlobIOSystem.h
)
SOURCE_GROUP( Exporter FILES ${Exporter_SRCS})

SET( Extra_SRCS
        ${ASSIMP_CODE_PATH}/MD4FileData.h
)
SOURCE_GROUP( Extra FILES ${Extra_SRCS})

SET( IrrXML_SRCS
        ${ASSIMP_CODE_PATH}/irrXMLWrapper.h
        ${ASSIMP_CODE_PATH}/../contrib/irrXML/CXMLReaderImpl.h
        ${ASSIMP_CODE_PATH}/../contrib/irrXML/heapsort.h
        ${ASSIMP_CODE_PATH}/../contrib/irrXML/irrArray.h
        ${ASSIMP_CODE_PATH}/../contrib/irrXML/irrString.h
        ${ASSIMP_CODE_PATH}/../contrib/irrXML/irrTypes.h
        ${ASSIMP_CODE_PATH}/../contrib/irrXML/irrXML.cpp
        ${ASSIMP_CODE_PATH}/../contrib/irrXML/irrXML.h
)
SOURCE_GROUP( IrrXML FILES ${IrrXML_SRCS})

SET( ConvertUTF_SRCS
        ${ASSIMP_CODE_PATH}/../contrib/ConvertUTF/ConvertUTF.h
        ${ASSIMP_CODE_PATH}/../contrib/ConvertUTF/ConvertUTF.c
)
SOURCE_GROUP( ConvertUTF FILES ${ConvertUTF_SRCS})

SET( Clipper_SRCS
        ${ASSIMP_CODE_PATH}/../contrib/clipper/clipper.hpp
        ${ASSIMP_CODE_PATH}/../contrib/clipper/clipper.cpp
)
SOURCE_GROUP( Clipper FILES ${Clipper_SRCS})


SET( Poly2Tri_SRCS
        ${ASSIMP_CODE_PATH}/../contrib/poly2tri/poly2tri/common/shapes.cc
        ${ASSIMP_CODE_PATH}/../contrib/poly2tri/poly2tri/common/shapes.h
        ${ASSIMP_CODE_PATH}/../contrib/poly2tri/poly2tri/common/utils.h
        ${ASSIMP_CODE_PATH}/../contrib/poly2tri/poly2tri/sweep/advancing_front.h
        ${ASSIMP_CODE_PATH}/../contrib/poly2tri/poly2tri/sweep/advancing_front.cc
        ${ASSIMP_CODE_PATH}/../contrib/poly2tri/poly2tri/sweep/cdt.cc
        ${ASSIMP_CODE_PATH}/../contrib/poly2tri/poly2tri/sweep/cdt.h
        ${ASSIMP_CODE_PATH}/../contrib/poly2tri/poly2tri/sweep/sweep.cc
        ${ASSIMP_CODE_PATH}/../contrib/poly2tri/poly2tri/sweep/sweep.h
        ${ASSIMP_CODE_PATH}/../contrib/poly2tri/poly2tri/sweep/sweep_context.cc
        ${ASSIMP_CODE_PATH}/../contrib/poly2tri/poly2tri/sweep/sweep_context.h
)
SOURCE_GROUP( Poly2Tri FILES ${Poly2Tri_SRCS})

SET( unzip_SRCS
        ${ASSIMP_CODE_PATH}/../contrib/unzip/crypt.h
        ${ASSIMP_CODE_PATH}/../contrib/unzip/ioapi.c
        ${ASSIMP_CODE_PATH}/../contrib/unzip/ioapi.h
        ${ASSIMP_CODE_PATH}/../contrib/unzip/unzip.c
        ${ASSIMP_CODE_PATH}/../contrib/unzip/unzip.h
)
SOURCE_GROUP( unzip FILES ${unzip_SRCS})

SET( assim_src
        # Assimp Files
        ${Core_SRCS}
        ${Common_SRCS}
        ${Logging_SRCS}
        ${Exporter_SRCS}
        ${PostProcessing_SRCS}

        # Model Support
        ${3DS_SRCS}
        ${AC_SRCS}
        ${ASE_SRCS}
        ${B3D_SRCS}
        ${BVH_SRCS}
        ${Collada_SRCS}
        ${DXF_SRCS}
        ${CSM_SRCS}
        ${HMP_SRCS}
        ${Irr_SRCS}
        ${LWO_SRCS}
        ${LWS_SRCS}
        ${M3_SRCS}
        ${MD2_SRCS}
        ${MD3_SRCS}
        ${MD5_SRCS}
        ${MDC_SRCS}
        ${MDL_SRCS}
        ${MaterialSystem_SRCS}
        ${NFF_SRCS}
        ${OFFFormat_SRCS}
        ${Obj_SRCS}
        ${Ogre_SRCS}
        ${Ply_SRCS}
        ${Q3D_SRCS}
        ${Q3BSP_SRCS}
        ${Raw_SRCS}
        ${SMD_SRCS}
        ${STL_SRCS}
        ${Unreal_SRCS}
        ${XFile_SRCS}
        ${Extra_SRCS}
        ${MS3D_SRCS}
        ${COB_SRCS}
        ${BLENDER_SRCS}
        ${NDO_SRCS}
        # ${IFC_SRCS} Does not compile with gcc 7.x
        ${XGL_SRCS}

        # Third-party libraries
        ${IrrXML_SRCS}
        ${ConvertUTF_SRCS}
        ${unzip_SRCS}
        ${Poly2Tri_SRCS}
        ${Clipper_SRCS}
        # Necessary to show the headers in the project when using the VC++ generator:
        ${Boost_SRCS}

        ${PUBLIC_HEADERS}
        ${COMPILER_HEADERS}
)

set(HEADER_FILES
	ColladaTransformHelper.h
        ZyColladaLoader.h
	config.h
)

set(SOURCE_FILES
        ColladaTransformHelper.cpp
        ZyColladaLoader.cpp
        initPlugin.cpp
        ${assim_src}
)

set(README_FILES ColladaSceneLoader.txt)

find_package(SofaBase REQUIRED)
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR})
find_package(Flexible QUIET)
find_package(image QUIET)

add_library(${PROJECT_NAME} SHARED ${HEADER_FILES} ${SOURCE_FILES} ${README_FILES})
target_link_libraries(${PROJECT_NAME} SofaLoader SofaSimulationCommon SofaBaseMechanics SofaRigid SofaBoundaryCondition SofaBaseTopology SofaBaseVisual SofaOpenglVisual SofaDeformable SofaMeshCollision SofaGuiQt)

#if ("${CMAKE_BUILD_TYPE}" STREQUAL "Debug")
#    target_link_libraries(${PROJECT_NAME} ${CMAKE_BINARY_DIR}/lib/libassimpD.so)
#    # target_link_libraries(${PROJECT_NAME} assimpD)
#else()
#    # target_link_libraries(${PROJECT_NAME} assimp)
#    target_link_libraries(${PROJECT_NAME} ${CMAKE_BINARY_DIR}/lib/libassimp.so)
#endif()

set(QT5_NO_LINK_QTMAIN 1)

find_package(Qt5 QUIET COMPONENTS Core Gui Widgets)
if(Qt5Core_FOUND)
    message(STATUS "${PROJECT_NAME}: Using Qt5")
	include_directories(${Qt5Core_INCLUDE_DIRS} ${Qt5Widgets_INCLUDE_DIRS} ${Qt5OpenGL_INCLUDE_DIRS})
	set(EXT_QT_LIBRARIES ${Qt5Core_LIBRARIES} ${Qt5Widgets_LIBRARIES} ${Qt5OpenGL_LIBRARIES})
else()
    message(STATUS "${PROJECT_NAME}: Qt5 not found, using Qt4")

    set(QT4_NO_LINK_QTMAIN 1)
    find_package(Qt4 QUIET COMPONENTS qtcore qtgui REQUIRED)
    include_directories(${QT_INCLUDE_DIRS})

	set(EXT_QT_LIBRARIES ${QT_QTCORE_LIBRARY} ${QT_QTGUI_LIBRARY} ${QT_QTOPENGL_LIBRARY} ${QT_QTXML_LIBRARY})
endif()

target_link_libraries(${PROJECT_NAME} ${EXT_QT_LIBRARIES})

find_package(Boost COMPONENTS regex REQUIRED)
include_directories(${Boost_INCLUDE_DIR})
link_directories(${Boost_LIBRARY_DIRS})

if(SOFA_PLUGIN_OBBTREEGPU_ENABLED)
	add_definitions("-DSOFA_HAVE_PLUGIN_OBBTREEGPU")
	# target_link_libraries(${PROJECT_NAME} ObbTreeGPU PQP)
	# include_directories(
	#	${CMAKE_SOURCE_DIR}/truplugins/ObbTreeGPU
	#	${CMAKE_SOURCE_DIR}/trulibs/PQP
	#	${CMAKE_SOURCE_DIR}/trulibs/PQP/src
	#	${CMAKE_SOURCE_DIR}/trulibs/PQP/include
	#	${CMAKE_SOURCE_DIR}/trulibs/gProximity
	#	${CMAKE_SOURCE_DIR}/trulibs/CyberX3D
	# )	
	
	#if(SOFA-EXTERNAL_GLEW)
#		add_definitions("-DSOFA_HAVE_GLEW")
#	endif()
	
	if(WIN32)
		# include_directories(${CMAKE_SOURCE_DIR}/truplugins/RobotConnector)
		#link_directories(${CMAKE_SOURCE_DIR}/lib/Win32)
		# link_directories(${CMAKE_SOURCE_DIR}/lib/Win64)
	endif()
endif()

set(ZYCOLLADALOADER_COMPILE_FLAGS "-DSOFA_BUILD_ZYCOLLADALOADER")

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/..
                    ${CMAKE_CURRENT_SOURCE_DIR}
                    ${CMAKE_CURRENT_SOURCE_DIR}/../ZySOFAControllers
                    ${CMAKE_SOURCE_DIR}/applications
                    ${CMAKE_SOURCE_DIR}/framework
                    ${CMAKE_SOURCE_DIR}/modules
                    ${CMAKE_BINARY_DIR}/include
                    ${CMAKE_BINARY_DIR}/SofaGui/SofaGuiQt)

include_directories(
                    ${CMAKE_CURRENT_SOURCE_DIR}/../assimp-3.0.1270
                    ${CMAKE_CURRENT_SOURCE_DIR}/../assimp-3.0.1270/include
                    ${CMAKE_CURRENT_SOURCE_DIR}/../assimp-3.0.1270/code
                    ${CMAKE_CURRENT_SOURCE_DIR}/../assimp-3.0.1270/contrib/irrXML
                   )

target_link_libraries(${PROJECT_NAME} ${Boost_REGEX_LIBRARY})

if(Flexible_FOUND)
    target_link_libraries(${PROJECT_NAME} Flexible)
    set(ZYCOLLADALOADER_COMPILE_FLAGS "${TRUCOLLADALOADER_COMPILE_FLAGS} -DSOFA_HAVE_PLUGIN_FLEXIBLE")
else()
    message(STATUS "${PROJECT_NAME}: could not find Flexible")
endif()

if(image_FOUND)
    target_link_libraries(${PROJECT_NAME} image)
    set(ZYCOLLADALOADER_COMPILE_FLAGS "${TRUCOLLADALOADER_COMPILE_FLAGS} -DSOFA_HAVE_PLUGIN_IMAGE")
else()
    message(STATUS "${PROJECT_NAME}: could not find image")
endif()

add_definitions("-DSOFA_BUILD_ZYCOLLADALOADER")
target_link_libraries(${PROJECT_NAME} SofaLoader SofaSimulationCommon SofaBaseMechanics SofaRigid SofaBoundaryCondition SofaBaseTopology SofaBaseVisual SofaDeformable SofaMeshCollision SofaUserInteraction)
target_link_libraries(${PROJECT_NAME} ZySOFAControllers)

if(SOFA_PLUGIN_OBBTREEGPU_ENABLED)
    target_link_libraries(${PROJECT_NAME} SofaConstraint)
endif()

if(SOFA_PLUGIN_ROBOTCONNECTOR_ENABLED AND WIN32)
    target_link_libraries(${PROJECT_NAME} RobotConnector)
endif()

if(SOFA_PLUGIN_QTOGREVIEWER_ENABLED)
    target_link_libraries(${PROJECT_NAME} QtOgreViewer)
endif()

set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "${ZYCOLLADALOADER_COMPILE_FLAGS}")
set_target_properties(${PROJECT_NAME} PROPERTIES DEBUG_POSTFIX "_d")

sofa_create_package(${PROJECT_NAME} "0.0.1" ${PROJECT_NAME} ${PROJECT_NAME})

install(DIRECTORY Demos/ DESTINATION share/sofa/plugins/${PROJECT_NAME})

