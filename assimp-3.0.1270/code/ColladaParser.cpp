/*
---------------------------------------------------------------------------
Open Asset Import Library (assimp)
---------------------------------------------------------------------------

Copyright (c) 2006-2012, assimp team

All rights reserved.

Redistribution and use of this software in source and binary forms, 
with or without modification, are permitted provided that the following 
conditions are met:

* Redistributions of source code must retain the above
copyright notice, this list of conditions and the
following disclaimer.

* Redistributions in binary form must reproduce the above
copyright notice, this list of conditions and the
following disclaimer in the documentation and/or other
materials provided with the distribution.

* Neither the name of the assimp team, nor the names of its
contributors may be used to endorse or promote products
derived from this software without specific prior
written permission of the assimp team.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY 
THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
---------------------------------------------------------------------------
*/

/** @file ColladaParser.cpp
 *  @brief Implementation of the Collada parser helper
 */

#include "AssimpPCH.h"
#ifndef ASSIMP_BUILD_NO_DAE_IMPORTER

#include "ColladaParser.h"
#include "fast_atof.h"
#include "ParsingUtils.h"
#include <deque>



using namespace Assimp;
using namespace Assimp::Collada;

#define TRIANGLE_OFF 20

// ------------------------------------------------------------------------------------------------
// Constructor to be privately used by Importer
ColladaParser::ColladaParser( IOSystem* pIOHandler, const std::string& pFile)
    : mFileName( pFile)
{
    mUnnamedCounter = 0;
    mRootNode = NULL;
    mUnitSize = 1.0f;
    mUpDirection = UP_Z;

    // We assume the newest file format by default
    mFormat = FV_1_5_n;

    // open the file
    boost::scoped_ptr<IOStream> file( pIOHandler->Open( pFile));
    if( file.get() == NULL)
        throw DeadlyImportError( "Failed to open file " + pFile + ".");

    // generate a XML reader for it
    boost::scoped_ptr<CIrrXML_IOStreamReader> mIOWrapper( new CIrrXML_IOStreamReader( file.get()));
    mReader = irr::io::createIrrXMLReader( mIOWrapper.get());
    if( !mReader)
        ThrowException( "Collada: Unable to open file.");

    // start reading
    ReadContents();

	// Second pass: Build list of relative transforms for kinematic models
	ReadRelativeTransforms(pIOHandler, pFile);
}

// ------------------------------------------------------------------------------------------------
// Destructor, private as well
ColladaParser::~ColladaParser()
{
    delete mReader;
    for( NodeLibrary::iterator it = mNodeLibrary.begin(); it != mNodeLibrary.end(); ++it)
        delete it->second;
    for( MeshLibrary::iterator it = mMeshLibrary.begin(); it != mMeshLibrary.end(); ++it)
        delete it->second;
}

// ------------------------------------------------------------------------------------------------
// Read string list from text contents of current element

void ColladaParser::ReadStringsFromTextContent(std::vector<std::string> &stringlist, char* n = " \f\n\r\t") {
    std::cout << "Reading Whitelist" << std::endl;
    const char* content = GetTextContent();
    std::string str(content);
    str = str.substr(str.find_first_not_of(n));
    while (true) {
		int end = str.find_first_of(n);
        std::string token = str.substr(0,end);
        if (token.empty()) break;
        stringlist.push_back(token);
        std::cout << "found: '" << token << "'" << std::endl;
		int newStart = str.find_first_not_of(n,end);
        if (newStart == -1) break;
		str = str.substr(newStart);
    }
}

// ------------------------------------------------------------------------------------------------
// Read bool from text contents of current element

bool ColladaParser::ReadBoolFromTextContent()
{
    const char* cur = GetTextContent();
    return (!ASSIMP_strincmp(cur,"true",4) || '0' != *cur);
}

void ColladaParser::ReadFloatsFromTextContent(float &a, float &b, float &c) {
    const char* content = GetTextContent();

    content = fast_atoreal_move<float>( content, (float&)a);
    SkipSpacesAndLineEnd( &content);

    content = fast_atoreal_move<float>( content, (float&)b);
    SkipSpacesAndLineEnd( &content);

    content = fast_atoreal_move<float>( content, (float&)c);
    SkipSpacesAndLineEnd( &content);
}

void ColladaParser::ReadFloatsFromTextContent(float &a, float &b, float &c, float &d) {
    const char* content = GetTextContent();

    content = fast_atoreal_move<float>( content, (float&)a);
    SkipSpacesAndLineEnd( &content);

    content = fast_atoreal_move<float>( content, (float&)b);
    SkipSpacesAndLineEnd( &content);

    content = fast_atoreal_move<float>( content, (float&)c);
    SkipSpacesAndLineEnd( &content);

    content = fast_atoreal_move<float>( content, (float&)d);
    SkipSpacesAndLineEnd( &content);
}

// ------------------------------------------------------------------------------------------------
// Read float from text contents of current element
float ColladaParser::ReadFloatFromTextContent()
{
    const char* cur = GetTextContent();
    return fast_atof(cur);
}

// ------------------------------------------------------------------------------------------------
// Reads the contents of the file
void ColladaParser::ReadContents()
{
    while( mReader->read())
    {
        // handle the root element "COLLADA"
        if( mReader->getNodeType() == irr::io::EXN_ELEMENT)
        {
            if( IsElement( "COLLADA"))
            {
                // check for 'version' attribute
                const int attrib = TestAttribute("version");
                if (attrib != -1) {
                    const char* version = mReader->getAttributeValue(attrib);

                    if (!::strncmp(version,"1.5",3)) {
                        mFormat =  FV_1_5_n;
                        DefaultLogger::get()->debug("Collada schema version is 1.5.n");
                    }
                    else if (!::strncmp(version,"1.4",3)) {
                        mFormat =  FV_1_4_n;
                        DefaultLogger::get()->debug("Collada schema version is 1.4.n");
                    }
                    else if (!::strncmp(version,"1.3",3)) {
                        mFormat =  FV_1_3_n;
                        DefaultLogger::get()->debug("Collada schema version is 1.3.n");
                    }
                }

                ReadStructure();
            } else
            {
                DefaultLogger::get()->debug( boost::str( boost::format( "Ignoring global element \"%s\".") % mReader->getNodeName()));
                SkipElement();
            }
        } else
        {
            // skip everything else silently
        }
    }
}

// ------------------------------------------------------------------------------------------------
// Reads the structure of the file
void ColladaParser::ReadStructure()
{
    while( mReader->read())
    {
        // beginning of elements
        if( mReader->getNodeType() == irr::io::EXN_ELEMENT)
        {
            if( IsElement( "asset"))
                ReadAssetInfo();
            else if( IsElement( "library_animations"))
                ReadAnimationLibrary();
            else if( IsElement( "library_controllers"))
                ReadControllerLibrary();
            else if( IsElement( "library_images"))
                ReadImageLibrary();
            else if( IsElement( "library_materials"))
                ReadMaterialLibrary();
            else if( IsElement( "library_effects"))
                ReadEffectLibrary();
            else if( IsElement( "library_geometries"))
                ReadGeometryLibrary();
            else if( IsElement( "library_visual_scenes"))
                ReadSceneLibrary();
            else if( IsElement( "library_lights"))
                ReadLightLibrary();
            else if( IsElement( "library_cameras"))
                ReadCameraLibrary();
            else if( IsElement( "library_nodes"))
                ReadSceneNode(NULL); /* some hacking to reuse this piece of code */
            else if( IsElement( "scene"))
                ReadScene();
            else if( IsElement( "library_joints"))
                ReadJointsLibrary();
            else if( IsElement ( "library_kinematics_models"))
                ReadKinematicsModelsLibrary();
            else if ( IsElement ( "library_physics_models"))
                ReadPhysicsModelsLibrary();
            else if( IsElement ( "library_kinematics_scenes"))
                ReadKinematicsScenes();
            else
                SkipElement();
        }
        else if( mReader->getNodeType() == irr::io::EXN_ELEMENT_END)
        {
            break;
        }
    }

    showJointLib();
}

// ------------------------------------------------------------------------------------------------
// Reads asset informations such as coordinate system informations and legal blah
void ColladaParser::ReadAssetInfo()
{
    if( mReader->isEmptyElement())
        return;

    while( mReader->read())
    {
        if( mReader->getNodeType() == irr::io::EXN_ELEMENT)
        {
            if( IsElement( "unit"))
            {
                // read unit data from the element's attributes
                const int attrIndex = TestAttribute( "meter");
                if (attrIndex == -1) {
                    mUnitSize = 1.f;
                }
                else {
                    mUnitSize = mReader->getAttributeValueAsFloat( attrIndex);
                }

                // consume the trailing stuff
                if( !mReader->isEmptyElement())
                    SkipElement();
            }
            else if( IsElement( "up_axis"))
            {
                // read content, strip whitespace, compare
                const char* content = GetTextContent();
                if( strncmp( content, "X_UP", 4) == 0)
                    mUpDirection = UP_X;
                else if( strncmp( content, "Y_UP", 4) == 0)
                    mUpDirection = UP_Y;
                else
                    mUpDirection = UP_Z;

                // check element end
                TestClosing( "up_axis");
            } else
            {
                SkipElement();
            }
        }
        else if( mReader->getNodeType() == irr::io::EXN_ELEMENT_END)
        {
            if( strcmp( mReader->getNodeName(), "asset") != 0)
                ThrowException( "Expected end of \"asset\" element.");

            break;
        }
    }
}

// ------------------------------------------------------------------------------------------------
// Reads the animation library
void ColladaParser::ReadAnimationLibrary()
{
    if (mReader->isEmptyElement())
        return;

    while( mReader->read())
    {
        if( mReader->getNodeType() == irr::io::EXN_ELEMENT)
        {
            if( IsElement( "animation"))
            {
                // delegate the reading. Depending on the inner elements it will be a container or a anim channel
                ReadAnimation( &mAnims);
            } else
            {
                // ignore the rest
                SkipElement();
            }
        }
        else if( mReader->getNodeType() == irr::io::EXN_ELEMENT_END)
        {
            if( strcmp( mReader->getNodeName(), "library_animations") != 0)
                ThrowException( "Expected end of \"library_animations\" element.");

            break;
        }
    }
}

// ------------------------------------------------------------------------------------------------
// Reads an animation into the given parent structure
void ColladaParser::ReadAnimation( Collada::Animation* pParent)
{
    if( mReader->isEmptyElement())
        return;

    // an <animation> element may be a container for grouping sub-elements or an animation channel
    // this is the channel collection by ID, in case it has channels
    typedef std::map<std::string, AnimationChannel> ChannelMap;
    ChannelMap channels;
    // this is the anim container in case we're a container
    Animation* anim = NULL;

    // optional name given as an attribute
    std::string animName;
    int indexName = TestAttribute( "name");
    int indexID = TestAttribute( "id");
    if( indexName >= 0)
        animName = mReader->getAttributeValue( indexName);
    else if( indexID >= 0)
        animName = mReader->getAttributeValue( indexID);
    else
        animName = "animation";

    while( mReader->read())
    {
        if( mReader->getNodeType() == irr::io::EXN_ELEMENT)
        {
            // we have subanimations
            if( IsElement( "animation"))
            {
                // create container from our element
                if( !anim)
                {
                    anim = new Animation;
                    anim->mName = animName;
                    pParent->mSubAnims.push_back( anim);
                }

                // recurse into the subelement
                ReadAnimation( anim);
            }
            else if( IsElement( "source"))
            {
                // possible animation data - we'll never know. Better store it
                ReadSource();
            }
            else if( IsElement( "sampler"))
            {
                // read the ID to assign the corresponding collada channel afterwards.
                int indexID = GetAttribute( "id");
                std::string id = mReader->getAttributeValue( indexID);
                ChannelMap::iterator newChannel = channels.insert( std::make_pair( id, AnimationChannel())).first;

                // have it read into a channel
                ReadAnimationSampler( newChannel->second);
            }
            else if( IsElement( "channel"))
            {
                // the binding element whose whole purpose is to provide the target to animate
                // Thanks, Collada! A directly posted information would have been too simple, I guess.
                // Better add another indirection to that! Can't have enough of those.
                int indexTarget = GetAttribute( "target");
                int indexSource = GetAttribute( "source");
                const char* sourceId = mReader->getAttributeValue( indexSource);
                if( sourceId[0] == '#')
                    sourceId++;
                ChannelMap::iterator cit = channels.find( sourceId);
                if( cit != channels.end())
                    cit->second.mTarget = mReader->getAttributeValue( indexTarget);

                if( !mReader->isEmptyElement())
                    SkipElement();
            }
            else
            {
                // ignore the rest
                SkipElement();
            }
        }
        else if( mReader->getNodeType() == irr::io::EXN_ELEMENT_END)
        {
            if( strcmp( mReader->getNodeName(), "animation") != 0)
                ThrowException( "Expected end of \"animation\" element.");

            break;
        }
    }

    // it turned out to have channels - add them
    if( !channels.empty())
    {
        // special filtering for stupid exporters packing each channel into a separate animation
        if( channels.size() == 1)
        {
            pParent->mChannels.push_back( channels.begin()->second);
        } else
        {
            // else create the animation, if not done yet, and store the channels
            if( !anim)
            {
                anim = new Animation;
                anim->mName = animName;
                pParent->mSubAnims.push_back( anim);
            }
            for( ChannelMap::const_iterator it = channels.begin(); it != channels.end(); ++it)
                anim->mChannels.push_back( it->second);
        }
    }
}

// ------------------------------------------------------------------------------------------------
// Reads an animation sampler into the given anim channel
void ColladaParser::ReadAnimationSampler( Collada::AnimationChannel& pChannel)
{
    while( mReader->read())
    {
        if( mReader->getNodeType() == irr::io::EXN_ELEMENT)
        {
            if( IsElement( "input"))
            {
                int indexSemantic = GetAttribute( "semantic");
                const char* semantic = mReader->getAttributeValue( indexSemantic);
                int indexSource = GetAttribute( "source");
                const char* source = mReader->getAttributeValue( indexSource);
                if( source[0] != '#')
                    ThrowException( "Unsupported URL format");
                source++;

                if( strcmp( semantic, "INPUT") == 0)
                    pChannel.mSourceTimes = source;
                else if( strcmp( semantic, "OUTPUT") == 0)
                    pChannel.mSourceValues = source;

                if( !mReader->isEmptyElement())
                    SkipElement();
            }
            else
            {
                // ignore the rest
                SkipElement();
            }
        }
        else if( mReader->getNodeType() == irr::io::EXN_ELEMENT_END)
        {
            if( strcmp( mReader->getNodeName(), "sampler") != 0)
                ThrowException( "Expected end of \"sampler\" element.");

            break;
        }
    }
}

// ------------------------------------------------------------------------------------------------
// Reads the skeleton controller library
void ColladaParser::ReadControllerLibrary()
{
    if (mReader->isEmptyElement())
        return;

    while( mReader->read())
    {
        if( mReader->getNodeType() == irr::io::EXN_ELEMENT)
        {
            if( IsElement( "controller"))
            {
                // read ID. Ask the spec if it's neccessary or optional... you might be surprised.
                int attrID = GetAttribute( "id");
                std::string id = mReader->getAttributeValue( attrID);

                // create an entry and store it in the library under its ID
                mControllerLibrary[id] = Controller();

                // read on from there
                ReadController( mControllerLibrary[id]);
            } else
            {
                // ignore the rest
                SkipElement();
            }
        }
        else if( mReader->getNodeType() == irr::io::EXN_ELEMENT_END)
        {
            if( strcmp( mReader->getNodeName(), "library_controllers") != 0)
                ThrowException( "Expected end of \"library_controllers\" element.");

            break;
        }
    }
}

// ------------------------------------------------------------------------------------------------
// Reads a controller into the given mesh structure
void ColladaParser::ReadController( Collada::Controller& pController)
{
    while( mReader->read())
    {
        if( mReader->getNodeType() == irr::io::EXN_ELEMENT)
        {
            // two types of controllers: "skin" and "morph". Only the first one is relevant, we skip the other
            if( IsElement( "morph"))
            {
                // should skip everything inside, so there's no danger of catching elements inbetween
                SkipElement();
            }
            else if( IsElement( "skin"))
            {
                // read the mesh it refers to. According to the spec this could also be another
                // controller, but I refuse to implement every bullshit idea they've come up with
                int sourceIndex = GetAttribute( "source");
                pController.mMeshId = mReader->getAttributeValue( sourceIndex) + 1;
            }
            else if( IsElement( "bind_shape_matrix"))
            {
                // content is 16 floats to define a matrix... it seems to be important for some models
                const char* content = GetTextContent();

                // read the 16 floats
                for( unsigned int a = 0; a < 16; a++)
                {
                    // read a number
                    content = fast_atoreal_move<float>( content, pController.mBindShapeMatrix[a]);
                    // skip whitespace after it
                    SkipSpacesAndLineEnd( &content);
                }

                TestClosing( "bind_shape_matrix");
            }
            else if( IsElement( "source"))
            {
                // data array - we have specialists to handle this
                ReadSource();
            }
            else if( IsElement( "joints"))
            {
                ReadControllerJoints( pController);
            }
            else if( IsElement( "vertex_weights"))
            {
                ReadControllerWeights( pController);
            }
            else
            {
                // ignore the rest
                SkipElement();
            }
        }
        else if( mReader->getNodeType() == irr::io::EXN_ELEMENT_END)
        {
            if( strcmp( mReader->getNodeName(), "controller") == 0)
                break;
            else if( strcmp( mReader->getNodeName(), "skin") != 0)
                ThrowException( "Expected end of \"controller\" element.");
        }
    }
}

// ------------------------------------------------------------------------------------------------
// Reads the joint definitions for the given controller
void ColladaParser::ReadControllerJoints( Collada::Controller& pController)
{
    while( mReader->read())
    {
        if( mReader->getNodeType() == irr::io::EXN_ELEMENT)
        {
            // Input channels for joint data. Two possible semantics: "JOINT" and "INV_BIND_MATRIX"
            if( IsElement( "input"))
            {
                int indexSemantic = GetAttribute( "semantic");
                const char* attrSemantic = mReader->getAttributeValue( indexSemantic);
                int indexSource = GetAttribute( "source");
                const char* attrSource = mReader->getAttributeValue( indexSource);

                // local URLS always start with a '#'. We don't support global URLs
                if( attrSource[0] != '#')
                    ThrowException( boost::str( boost::format( "Unsupported URL format in \"%s\"") % attrSource));
                attrSource++;

                // parse source URL to corresponding source
                if( strcmp( attrSemantic, "JOINT") == 0)
                    pController.mJointNameSource = attrSource;
                else if( strcmp( attrSemantic, "INV_BIND_MATRIX") == 0)
                    pController.mJointOffsetMatrixSource = attrSource;
                else
                    ThrowException( boost::str( boost::format( "Unknown semantic \"%s\" in joint data") % attrSemantic));

                // skip inner data, if present
                if( !mReader->isEmptyElement())
                    SkipElement();
            }
            else
            {
                // ignore the rest
                SkipElement();
            }
        }
        else if( mReader->getNodeType() == irr::io::EXN_ELEMENT_END)
        {
            if( strcmp( mReader->getNodeName(), "joints") != 0)
                ThrowException( "Expected end of \"joints\" element.");

            break;
        }
    }
}

// ------------------------------------------------------------------------------------------------
// Reads the joint weights for the given controller
void ColladaParser::ReadControllerWeights( Collada::Controller& pController)
{
    // read vertex count from attributes and resize the array accordingly
    int indexCount = GetAttribute( "count");
    size_t vertexCount = (size_t) mReader->getAttributeValueAsInt( indexCount);
    pController.mWeightCounts.resize( vertexCount);

    while( mReader->read())
    {
        if( mReader->getNodeType() == irr::io::EXN_ELEMENT)
        {
            // Input channels for weight data. Two possible semantics: "JOINT" and "WEIGHT"
            if( IsElement( "input"))
            {
                InputChannel channel;

                int indexSemantic = GetAttribute( "semantic");
                const char* attrSemantic = mReader->getAttributeValue( indexSemantic);
                int indexSource = GetAttribute( "source");
                const char* attrSource = mReader->getAttributeValue( indexSource);
                int indexOffset = TestAttribute( "offset");
                if( indexOffset >= 0)
                    channel.mOffset = mReader->getAttributeValueAsInt( indexOffset);

                // local URLS always start with a '#'. We don't support global URLs
                if( attrSource[0] != '#')
                    ThrowException( boost::str( boost::format( "Unsupported URL format in \"%s\"") % attrSource));
                channel.mAccessor = attrSource + 1;

                // parse source URL to corresponding source
                if( strcmp( attrSemantic, "JOINT") == 0)
                    pController.mWeightInputJoints = channel;
                else if( strcmp( attrSemantic, "WEIGHT") == 0)
                    pController.mWeightInputWeights = channel;
                else
                    ThrowException( boost::str( boost::format( "Unknown semantic \"%s\" in vertex_weight data") % attrSemantic));

                // skip inner data, if present
                if( !mReader->isEmptyElement())
                    SkipElement();
            }
            else if( IsElement( "vcount"))
            {
                // read weight count per vertex
                const char* text = GetTextContent();
                size_t numWeights = 0;
                for( std::vector<size_t>::iterator it = pController.mWeightCounts.begin(); it != pController.mWeightCounts.end(); ++it)
                {
                    if( *text == 0)
                        ThrowException( "Out of data while reading vcount");

                    *it = strtoul10( text, &text);
                    numWeights += *it;
                    SkipSpacesAndLineEnd( &text);
                }

                TestClosing( "vcount");

                // reserve weight count
                pController.mWeights.resize( numWeights);
            }
            else if( IsElement( "v"))
            {
                // read JointIndex - WeightIndex pairs
                const char* text = GetTextContent();

                for( std::vector< std::pair<size_t, size_t> >::iterator it = pController.mWeights.begin(); it != pController.mWeights.end(); ++it)
                {
                    if( *text == 0)
                        ThrowException( "Out of data while reading vertex_weights");
                    it->first = strtoul10( text, &text);
                    SkipSpacesAndLineEnd( &text);
                    if( *text == 0)
                        ThrowException( "Out of data while reading vertex_weights");
                    it->second = strtoul10( text, &text);
                    SkipSpacesAndLineEnd( &text);
                }

                TestClosing( "v");
            }
            else
            {
                // ignore the rest
                SkipElement();
            }
        }
        else if( mReader->getNodeType() == irr::io::EXN_ELEMENT_END)
        {
            if( strcmp( mReader->getNodeName(), "vertex_weights") != 0)
                ThrowException( "Expected end of \"vertex_weights\" element.");

            break;
        }
    }
}

// ------------------------------------------------------------------------------------------------
// Reads the image library contents
void ColladaParser::ReadImageLibrary()
{
    if( mReader->isEmptyElement())
        return;

    while( mReader->read())
    {
        if( mReader->getNodeType() == irr::io::EXN_ELEMENT) {
            if( IsElement( "image"))
            {
                // read ID. Another entry which is "optional" by design but obligatory in reality
                int attrID = GetAttribute( "id");
                std::string id = mReader->getAttributeValue( attrID);

                // create an entry and store it in the library under its ID
                mImageLibrary[id] = Image();

                // read on from there
                ReadImage( mImageLibrary[id]);
            } else
            {
                // ignore the rest
                SkipElement();
            }
        }
        else if( mReader->getNodeType() == irr::io::EXN_ELEMENT_END) {
            if( strcmp( mReader->getNodeName(), "library_images") != 0)
                ThrowException( "Expected end of \"library_images\" element.");

            break;
        }
    }
}

// ------------------------------------------------------------------------------------------------
// Reads an image entry into the given image
void ColladaParser::ReadImage( Collada::Image& pImage)
{
    while( mReader->read())
    {
        if( mReader->getNodeType() == irr::io::EXN_ELEMENT){
            // Need to run different code paths here, depending on the Collada XSD version
            if (IsElement("image")) {
                SkipElement();
            }
            else if(  IsElement( "init_from"))
            {
                if (mFormat == FV_1_4_n)
                {
                    // FIX: C4D exporter writes empty <init_from/> tags
                    if (!mReader->isEmptyElement()) {
                        // element content is filename - hopefully
                        const char* sz = TestTextContent();
                        if (sz)pImage.mFileName = sz;
                        TestClosing( "init_from");
                    }
                    if (!pImage.mFileName.length()) {
                        pImage.mFileName = "unknown_texture";
                    }
                }
                else if (mFormat == FV_1_5_n)
                {
                    // make sure we skip over mip and array initializations, which
                    // we don't support, but which could confuse the loader if
                    // they're not skipped.
                    int attrib = TestAttribute("array_index");
                    if (attrib != -1 && mReader->getAttributeValueAsInt(attrib) > 0) {
                        DefaultLogger::get()->warn("Collada: Ignoring texture array index");
                        continue;
                    }

                    attrib = TestAttribute("mip_index");
                    if (attrib != -1 && mReader->getAttributeValueAsInt(attrib) > 0) {
                        DefaultLogger::get()->warn("Collada: Ignoring MIP map layer");
                        continue;
                    }

                    // TODO: correctly jump over cube and volume maps?
                }
            }
            else if (mFormat == FV_1_5_n)
            {
                if( IsElement( "ref"))
                {
                    // element content is filename - hopefully
                    const char* sz = TestTextContent();
                    if (sz)pImage.mFileName = sz;
                    TestClosing( "ref");
                }
                else if( IsElement( "hex") && !pImage.mFileName.length())
                {
                    // embedded image. get format
                    const int attrib = TestAttribute("format");
                    if (-1 == attrib)
                        DefaultLogger::get()->warn("Collada: Unknown image file format");
                    else pImage.mEmbeddedFormat = mReader->getAttributeValue(attrib);

                    const char* data = GetTextContent();

                    // hexadecimal-encoded binary octets. First of all, find the
                    // required buffer size to reserve enough storage.
                    const char* cur = data;
                    while (!IsSpaceOrNewLine(*cur)) cur++;

                    const unsigned int size = (unsigned int)(cur-data) * 2;
                    pImage.mImageData.resize(size);
                    for (unsigned int i = 0; i < size;++i)
                        pImage.mImageData[i] = HexOctetToDecimal(data+(i<<1));

                    TestClosing( "hex");
                }
            }
            else
            {
                // ignore the rest
                SkipElement();
            }
        }
        else if( mReader->getNodeType() == irr::io::EXN_ELEMENT_END) {
            if( strcmp( mReader->getNodeName(), "image") == 0)
                break;
        }
    }
}

// ------------------------------------------------------------------------------------------------
// Reads the material library
void ColladaParser::ReadMaterialLibrary()
{
    if( mReader->isEmptyElement())
        return;

    while( mReader->read())
    {
        if( mReader->getNodeType() == irr::io::EXN_ELEMENT)
        {
            if( IsElement( "material"))
            {
                // read ID. By now you propably know my opinion about this "specification"
                int attrID = GetAttribute( "id");
                std::string id = mReader->getAttributeValue( attrID);

                // create an entry and store it in the library under its ID
                ReadMaterial(mMaterialLibrary[id] = Material());
            } else
            {
                // ignore the rest
                SkipElement();
            }
        }
        else if( mReader->getNodeType() == irr::io::EXN_ELEMENT_END)
        {
            if( strcmp( mReader->getNodeName(), "library_materials") != 0)
                ThrowException( "Expected end of \"library_materials\" element.");

            break;
        }
    }
}

// ------------------------------------------------------------------------------------------------
// Reads the light library
void ColladaParser::ReadLightLibrary()
{
    if( mReader->isEmptyElement())
        return;

    while( mReader->read())
    {
        if( mReader->getNodeType() == irr::io::EXN_ELEMENT) {
            if( IsElement( "light"))
            {
                // read ID. By now you propably know my opinion about this "specification"
                int attrID = GetAttribute( "id");
                std::string id = mReader->getAttributeValue( attrID);

                // create an entry and store it in the library under its ID
                ReadLight(mLightLibrary[id] = Light());

            } else
            {
                // ignore the rest
                SkipElement();
            }
        }
        else if( mReader->getNodeType() == irr::io::EXN_ELEMENT_END)	{
            if( strcmp( mReader->getNodeName(), "library_lights") != 0)
                ThrowException( "Expected end of \"library_lights\" element.");

            break;
        }
    }
}

// ------------------------------------------------------------------------------------------------
// Reads the camera library
void ColladaParser::ReadCameraLibrary()
{
    if( mReader->isEmptyElement())
        return;

    while( mReader->read())
    {
        if( mReader->getNodeType() == irr::io::EXN_ELEMENT) {
            if( IsElement( "camera"))
            {
                // read ID. By now you propably know my opinion about this "specification"
                int attrID = GetAttribute( "id");
                std::string id = mReader->getAttributeValue( attrID);

                // create an entry and store it in the library under its ID
                Camera& cam = mCameraLibrary[id];
                attrID = TestAttribute( "name");
                if (attrID != -1)
                    cam.mName = mReader->getAttributeValue( attrID);

                ReadCamera(cam);

            } else
            {
                // ignore the rest
                SkipElement();
            }
        }
        else if( mReader->getNodeType() == irr::io::EXN_ELEMENT_END)	{
            if( strcmp( mReader->getNodeName(), "library_cameras") != 0)
                ThrowException( "Expected end of \"library_cameras\" element.");

            break;
        }
    }
}

// ------------------------------------------------------------------------------------------------
// Reads a material entry into the given material
void ColladaParser::ReadMaterial( Collada::Material& pMaterial)
{
    while( mReader->read())
    {
        if( mReader->getNodeType() == irr::io::EXN_ELEMENT) {
            if (IsElement("material")) {
                SkipElement();
            }
            else if( IsElement( "instance_effect"))
            {
                // referred effect by URL
                int attrUrl = GetAttribute( "url");
                const char* url = mReader->getAttributeValue( attrUrl);
                if( url[0] != '#')
                    ThrowException( "Unknown reference format");

                pMaterial.mEffect = url+1;

                SkipElement();
            } else
            {
                // ignore the rest
                SkipElement();
            }
        }
        else if( mReader->getNodeType() == irr::io::EXN_ELEMENT_END) {
            if( strcmp( mReader->getNodeName(), "material") != 0)
                ThrowException( "Expected end of \"material\" element.");

            break;
        }
    }
}

// ------------------------------------------------------------------------------------------------
// Reads a light entry into the given light
void ColladaParser::ReadLight( Collada::Light& pLight)
{
    while( mReader->read())
    {
        if( mReader->getNodeType() == irr::io::EXN_ELEMENT) {
            if (IsElement("light")) {
                SkipElement();
            }
            else if (IsElement("spot")) {
                pLight.mType = aiLightSource_SPOT;
            }
            else if (IsElement("ambient")) {
                pLight.mType = aiLightSource_AMBIENT;
            }
            else if (IsElement("directional")) {
                pLight.mType = aiLightSource_DIRECTIONAL;
            }
            else if (IsElement("point")) {
                pLight.mType = aiLightSource_POINT;
            }
            else if (IsElement("color")) {
                // text content contains 3 floats
                const char* content = GetTextContent();

                content = fast_atoreal_move<float>( content, (float&)pLight.mColor.r);
                SkipSpacesAndLineEnd( &content);

                content = fast_atoreal_move<float>( content, (float&)pLight.mColor.g);
                SkipSpacesAndLineEnd( &content);

                content = fast_atoreal_move<float>( content, (float&)pLight.mColor.b);
                SkipSpacesAndLineEnd( &content);

                TestClosing( "color");
            }
            else if (IsElement("constant_attenuation")) {
                pLight.mAttConstant = ReadFloatFromTextContent();
                TestClosing("constant_attenuation");
            }
            else if (IsElement("linear_attenuation")) {
                pLight.mAttLinear = ReadFloatFromTextContent();
                TestClosing("linear_attenuation");
            }
            else if (IsElement("quadratic_attenuation")) {
                pLight.mAttQuadratic = ReadFloatFromTextContent();
                TestClosing("quadratic_attenuation");
            }
            else if (IsElement("falloff_angle")) {
                pLight.mFalloffAngle = ReadFloatFromTextContent();
                TestClosing("falloff_angle");
            }
            else if (IsElement("falloff_exponent")) {
                pLight.mFalloffExponent = ReadFloatFromTextContent();
                TestClosing("falloff_exponent");
            }
            // FCOLLADA extensions
            // -------------------------------------------------------
            else if (IsElement("outer_cone")) {
                pLight.mOuterAngle = ReadFloatFromTextContent();
                TestClosing("outer_cone");
            }
            // ... and this one is even deprecated
            else if (IsElement("penumbra_angle")) {
                pLight.mPenumbraAngle = ReadFloatFromTextContent();
                TestClosing("penumbra_angle");
            }
            else if (IsElement("intensity")) {
                pLight.mIntensity = ReadFloatFromTextContent();
                TestClosing("intensity");
            }
            else if (IsElement("falloff")) {
                pLight.mOuterAngle = ReadFloatFromTextContent();
                TestClosing("falloff");
            }
            else if (IsElement("hotspot_beam")) {
                pLight.mFalloffAngle = ReadFloatFromTextContent();
                TestClosing("hotspot_beam");
            }
        }
        else if( mReader->getNodeType() == irr::io::EXN_ELEMENT_END) {
            if( strcmp( mReader->getNodeName(), "light") == 0)
                break;
        }
    }
}

// ------------------------------------------------------------------------------------------------
// Reads a camera entry into the given light
void ColladaParser::ReadCamera( Collada::Camera& pCamera)
{
    while( mReader->read())
    {
        if( mReader->getNodeType() == irr::io::EXN_ELEMENT) {
            if (IsElement("camera")) {
                SkipElement();
            }
            else if (IsElement("orthographic")) {
                pCamera.mOrtho = true;
            }
            else if (IsElement("xfov") || IsElement("xmag")) {
                pCamera.mHorFov = ReadFloatFromTextContent();
                TestClosing((pCamera.mOrtho ? "xmag" : "xfov"));
            }
            else if (IsElement("yfov") || IsElement("ymag")) {
                pCamera.mVerFov = ReadFloatFromTextContent();
                TestClosing((pCamera.mOrtho ? "ymag" : "yfov"));
            }
            else if (IsElement("aspect_ratio")) {
                pCamera.mAspect = ReadFloatFromTextContent();
                TestClosing("aspect_ratio");
            }
            else if (IsElement("znear")) {
                pCamera.mZNear = ReadFloatFromTextContent();
                TestClosing("znear");
            }
            else if (IsElement("zfar")) {
                pCamera.mZFar = ReadFloatFromTextContent();
                TestClosing("zfar");
            }
        }
        else if( mReader->getNodeType() == irr::io::EXN_ELEMENT_END) {
            if( strcmp( mReader->getNodeName(), "camera") == 0)
                break;
        }
    }
}

// ------------------------------------------------------------------------------------------------
// Reads the effect library
void ColladaParser::ReadEffectLibrary()
{
    if (mReader->isEmptyElement()) {
        return;
    }

    while( mReader->read())
    {
        if( mReader->getNodeType() == irr::io::EXN_ELEMENT) {
            if( IsElement( "effect"))
            {
                // read ID. Do I have to repeat my ranting about "optional" attributes?
                // Alex: .... no, not necessary. Please shut up and leave more space for
                // me to complain about the fucking Collada spec with its fucking
                // 'optional' attributes ...
                int attrID = GetAttribute( "id");
                std::string id = mReader->getAttributeValue( attrID);

                // create an entry and store it in the library under its ID
                mEffectLibrary[id] = Effect();
                // read on from there
                ReadEffect( mEffectLibrary[id]);
            } else
            {
                // ignore the rest
                SkipElement();
            }
        }
        else if( mReader->getNodeType() == irr::io::EXN_ELEMENT_END) {
            if( strcmp( mReader->getNodeName(), "library_effects") != 0)
                ThrowException( "Expected end of \"library_effects\" element.");

            break;
        }
    }
}

// ------------------------------------------------------------------------------------------------
// Reads an effect entry into the given effect
void ColladaParser::ReadEffect( Collada::Effect& pEffect)
{
    // for the moment we don't support any other type of effect.
    while( mReader->read())
    {
        if( mReader->getNodeType() == irr::io::EXN_ELEMENT)
        {
            if( IsElement( "profile_COMMON"))
                ReadEffectProfileCommon( pEffect);
            else
                SkipElement();
        }
        else if( mReader->getNodeType() == irr::io::EXN_ELEMENT_END)
        {
            if( strcmp( mReader->getNodeName(), "effect") != 0)
                ThrowException( "Expected end of \"effect\" element.");

            break;
        }
    }
}

// ------------------------------------------------------------------------------------------------
// Reads an COMMON effect profile
void ColladaParser::ReadEffectProfileCommon( Collada::Effect& pEffect)
{
    while( mReader->read())
    {
        if( mReader->getNodeType() == irr::io::EXN_ELEMENT)
        {
            if( IsElement( "newparam"))	{
                // save ID
                int attrSID = GetAttribute( "sid");
                std::string sid = mReader->getAttributeValue( attrSID);
                pEffect.mParams[sid] = EffectParam();
                ReadEffectParam( pEffect.mParams[sid]);
            }
            else if( IsElement( "technique") || IsElement( "extra"))
            {
                // just syntactic sugar
            }

            /* Shading modes */
            else if( IsElement( "phong"))
                pEffect.mShadeType = Shade_Phong;
            else if( IsElement( "constant"))
                pEffect.mShadeType = Shade_Constant;
            else if( IsElement( "lambert"))
                pEffect.mShadeType = Shade_Lambert;
            else if( IsElement( "blinn"))
                pEffect.mShadeType = Shade_Blinn;

            /* Color + texture properties */
            else if( IsElement( "emission"))
                ReadEffectColor( pEffect.mEmissive, pEffect.mTexEmissive);
            else if( IsElement( "ambient"))
                ReadEffectColor( pEffect.mAmbient, pEffect.mTexAmbient);
            else if( IsElement( "diffuse"))
                ReadEffectColor( pEffect.mDiffuse, pEffect.mTexDiffuse);
            else if( IsElement( "specular"))
                ReadEffectColor( pEffect.mSpecular, pEffect.mTexSpecular);
            else if( IsElement( "reflective")) {
                ReadEffectColor( pEffect.mReflective, pEffect.mTexReflective);
            }
            else if( IsElement( "transparent")) {
                ReadEffectColor( pEffect.mTransparent,pEffect.mTexTransparent);
            }
            else if( IsElement( "shininess"))
                ReadEffectFloat( pEffect.mShininess);
            else if( IsElement( "reflectivity"))
                ReadEffectFloat( pEffect.mReflectivity);

            /* Single scalar properties */
            else if( IsElement( "transparency"))
                ReadEffectFloat( pEffect.mTransparency);
            else if( IsElement( "index_of_refraction"))
                ReadEffectFloat( pEffect.mRefractIndex);

            // GOOGLEEARTH/OKINO extensions
            // -------------------------------------------------------
            else if( IsElement( "double_sided"))
                pEffect.mDoubleSided = ReadBoolFromTextContent();

            // FCOLLADA extensions
            // -------------------------------------------------------
            else if( IsElement( "bump")) {
                aiColor4D dummy;
                ReadEffectColor( dummy,pEffect.mTexBump);
            }

            // MAX3D extensions
            // -------------------------------------------------------
            else if( IsElement( "wireframe"))	{
                pEffect.mWireframe = ReadBoolFromTextContent();
                TestClosing( "wireframe");
            }
            else if( IsElement( "faceted"))	{
                pEffect.mFaceted = ReadBoolFromTextContent();
                TestClosing( "faceted");
            }
            else
            {
                // ignore the rest
                SkipElement();
            }
        }
        else if( mReader->getNodeType() == irr::io::EXN_ELEMENT_END) {
            if( strcmp( mReader->getNodeName(), "profile_COMMON") == 0)
            {
                break;
            }
        }
    }
}

// ------------------------------------------------------------------------------------------------
// Read texture wrapping + UV transform settings from a profile==Maya chunk
void ColladaParser::ReadSamplerProperties( Sampler& out )
{
    if (mReader->isEmptyElement()) {
        return;
    }

    while( mReader->read())
    {
        if( mReader->getNodeType() == irr::io::EXN_ELEMENT) {

            // MAYA extensions
            // -------------------------------------------------------
            if( IsElement( "wrapU"))		{
                out.mWrapU = ReadBoolFromTextContent();
                TestClosing( "wrapU");
            }
            else if( IsElement( "wrapV"))	{
                out.mWrapV = ReadBoolFromTextContent();
                TestClosing( "wrapV");
            }
            else if( IsElement( "mirrorU"))		{
                out.mMirrorU = ReadBoolFromTextContent();
                TestClosing( "mirrorU");
            }
            else if( IsElement( "mirrorV"))	{
                out.mMirrorV = ReadBoolFromTextContent();
                TestClosing( "mirrorV");
            }
            else if( IsElement( "repeatU"))	{
                out.mTransform.mScaling.x = ReadFloatFromTextContent();
                TestClosing( "repeatU");
            }
            else if( IsElement( "repeatV"))	{
                out.mTransform.mScaling.y = ReadFloatFromTextContent();
                TestClosing( "repeatV");
            }
            else if( IsElement( "offsetU"))	{
                out.mTransform.mTranslation.x = ReadFloatFromTextContent();
                TestClosing( "offsetU");
            }
            else if( IsElement( "offsetV"))	{
                out.mTransform.mTranslation.y = ReadFloatFromTextContent();
                TestClosing( "offsetV");
            }
            else if( IsElement( "rotateUV"))	{
                out.mTransform.mRotation = ReadFloatFromTextContent();
                TestClosing( "rotateUV");
            }
            else if( IsElement( "blend_mode"))	{

                const char* sz = GetTextContent();
                // http://www.feelingsoftware.com/content/view/55/72/lang,en/
                // NONE, OVER, IN, OUT, ADD, SUBTRACT, MULTIPLY, DIFFERENCE, LIGHTEN, DARKEN, SATURATE, DESATURATE and ILLUMINATE
                if (0 == ASSIMP_strincmp(sz,"ADD",3))
                    out.mOp = aiTextureOp_Add;

                else if (0 == ASSIMP_strincmp(sz,"SUBTRACT",8))
                    out.mOp = aiTextureOp_Subtract;

                else if (0 == ASSIMP_strincmp(sz,"MULTIPLY",8))
                    out.mOp = aiTextureOp_Multiply;

                else  {
                    DefaultLogger::get()->warn("Collada: Unsupported MAYA texture blend mode");
                }
                TestClosing( "blend_mode");
            }
            // OKINO extensions
            // -------------------------------------------------------
            else if( IsElement( "weighting"))	{
                out.mWeighting = ReadFloatFromTextContent();
                TestClosing( "weighting");
            }
            else if( IsElement( "mix_with_previous_layer"))	{
                out.mMixWithPrevious = ReadFloatFromTextContent();
                TestClosing( "mix_with_previous_layer");
            }
            // MAX3D extensions
            // -------------------------------------------------------
            else if( IsElement( "amount"))	{
                out.mWeighting = ReadFloatFromTextContent();
                TestClosing( "amount");
            }
        }
        else if( mReader->getNodeType() == irr::io::EXN_ELEMENT_END) {
            if( strcmp( mReader->getNodeName(), "technique") == 0)
                break;
        }
    }
}

// ------------------------------------------------------------------------------------------------
// Reads an effect entry containing a color or a texture defining that color
void ColladaParser::ReadEffectColor( aiColor4D& pColor, Sampler& pSampler)
{
    if (mReader->isEmptyElement())
        return;

    // Save current element name
    const std::string curElem = mReader->getNodeName();

    while( mReader->read())
    {
        if( mReader->getNodeType() == irr::io::EXN_ELEMENT) {
            if( IsElement( "color"))
            {
                // text content contains 4 floats
                const char* content = GetTextContent();

                content = fast_atoreal_move<float>( content, (float&)pColor.r);
                SkipSpacesAndLineEnd( &content);

                content = fast_atoreal_move<float>( content, (float&)pColor.g);
                SkipSpacesAndLineEnd( &content);

                content = fast_atoreal_move<float>( content, (float&)pColor.b);
                SkipSpacesAndLineEnd( &content);

                content = fast_atoreal_move<float>( content, (float&)pColor.a);
                SkipSpacesAndLineEnd( &content);
                TestClosing( "color");
            }
            else if( IsElement( "texture"))
            {
                // get name of source textur/sampler
                int attrTex = GetAttribute( "texture");
                pSampler.mName = mReader->getAttributeValue( attrTex);

                // get name of UV source channel
                attrTex = GetAttribute( "texcoord");
                pSampler.mUVChannel = mReader->getAttributeValue( attrTex);
                //SkipElement();
            }
            else if( IsElement( "technique"))
            {
                const int _profile = GetAttribute( "profile");
                const char* profile = mReader->getAttributeValue( _profile );

                // Some extensions are quite useful ... ReadSamplerProperties processes
                // several extensions in MAYA, OKINO and MAX3D profiles.
                if (!::strcmp(profile,"MAYA") || !::strcmp(profile,"MAX3D") || !::strcmp(profile,"OKINO"))
                {
                    // get more information on this sampler
                    ReadSamplerProperties(pSampler);
                }
                else SkipElement();
            }
            else if( !IsElement( "extra"))
            {
                // ignore the rest
                SkipElement();
            }
        }
        else if( mReader->getNodeType() == irr::io::EXN_ELEMENT_END){
            if (mReader->getNodeName() == curElem)
                break;
        }
    }
}

// ------------------------------------------------------------------------------------------------
// Reads an effect entry containing a float
void ColladaParser::ReadEffectFloat( float& pFloat)
{
    while( mReader->read())
    {
        if( mReader->getNodeType() == irr::io::EXN_ELEMENT){
            if( IsElement( "float"))
            {
                // text content contains a single floats
                const char* content = GetTextContent();
                content = fast_atoreal_move<float>( content, pFloat);
                SkipSpacesAndLineEnd( &content);

                TestClosing( "float");
            } else
            {
                // ignore the rest
                SkipElement();
            }
        }
        else if( mReader->getNodeType() == irr::io::EXN_ELEMENT_END){
            break;
        }
    }
}

// ------------------------------------------------------------------------------------------------
// Reads an effect parameter specification of any kind 
void ColladaParser::ReadEffectParam( Collada::EffectParam& pParam)
{
    while( mReader->read())
    {
        if( mReader->getNodeType() == irr::io::EXN_ELEMENT) {
            if( IsElement( "surface"))
            {
                // image ID given inside <init_from> tags
                TestOpening( "init_from");
                const char* content = GetTextContent();
                pParam.mType = Param_Surface;
                pParam.mReference = content;
                TestClosing( "init_from");

                // don't care for remaining stuff
                SkipElement( "surface");
            }
            else if( IsElement( "sampler2D"))
            {
                // surface ID is given inside <source> tags
                TestOpening( "source");
                const char* content = GetTextContent();
                pParam.mType = Param_Sampler;
                pParam.mReference = content;
                TestClosing( "source");

                // don't care for remaining stuff
                SkipElement( "sampler2D");
            } else
            {
                // ignore unknown element
                SkipElement();
            }
        }
        else if( mReader->getNodeType() == irr::io::EXN_ELEMENT_END) {
            break;
        }
    }
}

// ------------------------------------------------------------------------------------------------
// Reads the geometry library contents
void ColladaParser::ReadGeometryLibrary()
{
    if( mReader->isEmptyElement())
        return;

    while( mReader->read())
    {
        if( mReader->getNodeType() == irr::io::EXN_ELEMENT)
        {
            if( IsElement( "geometry"))
            {
                // read ID. Another entry which is "optional" by design but obligatory in reality
                int indexID = GetAttribute( "id");
                std::string id = mReader->getAttributeValue( indexID);

                // TODO: (thom) support SIDs
                // ai_assert( TestAttribute( "sid") == -1);

                // create a mesh and store it in the library under its ID
                Mesh* mesh = new Mesh(id);
                mMeshLibrary[id] = mesh;

                // read on from there
                ReadGeometry( mesh);
            } else
            {
                // ignore the rest
                SkipElement();
            }
        }
        else if( mReader->getNodeType() == irr::io::EXN_ELEMENT_END)
        {
            if( strcmp( mReader->getNodeName(), "library_geometries") != 0)
                ThrowException( "Expected end of \"library_geometries\" element.");

            break;
        }
    }
}

// ------------------------------------------------------------------------------------------------
// Reads a geometry from the geometry library.
void ColladaParser::ReadGeometry( Collada::Mesh* pMesh)
{
    if( mReader->isEmptyElement())
        return;

    while( mReader->read())
    {
        if( mReader->getNodeType() == irr::io::EXN_ELEMENT)
        {
            if( IsElement( "mesh"))
            {
                // read on from there
                ReadMesh( pMesh);
            } else
            {
                // ignore the rest
                SkipElement();
            }
        }
        else if( mReader->getNodeType() == irr::io::EXN_ELEMENT_END)
        {
            if( strcmp( mReader->getNodeName(), "geometry") != 0)
                ThrowException( "Expected end of \"geometry\" element.");

            break;
        }
    }
}

// ------------------------------------------------------------------------------------------------
// Reads a mesh from the geometry library
void ColladaParser::ReadMesh( Mesh* pMesh)
{
    if( mReader->isEmptyElement())
        return;

    while( mReader->read())
    {
        if( mReader->getNodeType() == irr::io::EXN_ELEMENT)
        {
            if( IsElement( "source"))
            {
                // we have professionals dealing with this
                ReadSource();
            }
            else if( IsElement( "vertices"))
            {
                // read per-vertex mesh data
                ReadVertexData( pMesh);
            }
            else if( IsElement( "triangles") || IsElement( "lines") || IsElement( "linestrips")
                     || IsElement( "polygons") || IsElement( "polylist") || IsElement( "trifans") || IsElement( "tristrips"))
            {
                // read per-index mesh data and faces setup
                ReadIndexData( pMesh);
            } else
            {
                // ignore the rest
                SkipElement();
            }
        }
        else if( mReader->getNodeType() == irr::io::EXN_ELEMENT_END)
        {
            if( strcmp( mReader->getNodeName(), "technique_common") == 0)
            {
                // end of another meaningless element - read over it
            }
            else if( strcmp( mReader->getNodeName(), "mesh") == 0)
            {
                // end of <mesh> element - we're done here
                break;
            } else
            {
                // everything else should be punished
                ThrowException( "Expected end of \"mesh\" element.");
            }
        }
    }
}

// ------------------------------------------------------------------------------------------------
// Reads a source element 
void ColladaParser::ReadSource()
{
    int indexID = GetAttribute( "id");
    std::string sourceID = mReader->getAttributeValue( indexID);

    while( mReader->read())
    {
        if( mReader->getNodeType() == irr::io::EXN_ELEMENT)
        {
            if( IsElement( "float_array") || IsElement( "IDREF_array") || IsElement( "Name_array"))
            {
                ReadDataArray();
            }
            else if( IsElement( "technique_common"))
            {
                // I don't fucking care for your profiles bullshit
            }
            else if( IsElement( "accessor"))
            {
                ReadAccessor( sourceID);
            } else
            {
                // ignore the rest
                SkipElement();
            }
        }
        else if( mReader->getNodeType() == irr::io::EXN_ELEMENT_END)
        {
            if( strcmp( mReader->getNodeName(), "source") == 0)
            {
                // end of <source> - we're done
                break;
            }
            else if( strcmp( mReader->getNodeName(), "technique_common") == 0)
            {
                // end of another meaningless element - read over it
            } else
            {
                // everything else should be punished
                ThrowException( "Expected end of \"source\" element.");
            }
        }
    }
}

// ------------------------------------------------------------------------------------------------
// Reads a data array holding a number of floats, and stores it in the global library
void ColladaParser::ReadDataArray()
{
    std::string elmName = mReader->getNodeName();
    bool isStringArray = (elmName == "IDREF_array" || elmName == "Name_array");
    bool isEmptyElement = mReader->isEmptyElement();

    // read attributes
    int indexID = GetAttribute( "id");
    std::string id = mReader->getAttributeValue( indexID);
    int indexCount = GetAttribute( "count");
    unsigned int count = (unsigned int) mReader->getAttributeValueAsInt( indexCount);
    const char* content = TestTextContent();

    // read values and store inside an array in the data library
    mDataLibrary[id] = Data();
    Data& data = mDataLibrary[id];
    data.mIsStringArray = isStringArray;

    // some exporters write empty data arrays, but we need to conserve them anyways because others might reference them
    if (content)
    {
        if( isStringArray)
        {
            data.mStrings.reserve( count);
            std::string s;

            for( unsigned int a = 0; a < count; a++)
            {
                if( *content == 0)
                    ThrowException( "Expected more values while reading IDREF_array contents.");

                s.clear();
                while( !IsSpaceOrNewLine( *content))
                    s += *content++;
                data.mStrings.push_back( s);

                SkipSpacesAndLineEnd( &content);
            }
        } else
        {
            data.mValues.reserve( count);

            for( unsigned int a = 0; a < count; a++)
            {
                if( *content == 0)
                    ThrowException( "Expected more values while reading float_array contents.");

                float value;
                // read a number
                content = fast_atoreal_move<float>( content, value);
                data.mValues.push_back( value);
                // skip whitespace after it
                SkipSpacesAndLineEnd( &content);
            }
        }
    }

    // test for closing tag
    if( !isEmptyElement )
        TestClosing( elmName.c_str());
}

// ------------------------------------------------------------------------------------------------
// Reads an accessor and stores it in the global library
void ColladaParser::ReadAccessor( const std::string& pID)
{
    // read accessor attributes
    int attrSource = GetAttribute( "source");
    const char* source = mReader->getAttributeValue( attrSource);
    if( source[0] != '#')
        ThrowException( boost::str( boost::format( "Unknown reference format in url \"%s\".") % source));
    int attrCount = GetAttribute( "count");
    unsigned int count = (unsigned int) mReader->getAttributeValueAsInt( attrCount);
    int attrOffset = TestAttribute( "offset");
    unsigned int offset = 0;
    if( attrOffset > -1)
        offset = (unsigned int) mReader->getAttributeValueAsInt( attrOffset);
    int attrStride = TestAttribute( "stride");
    unsigned int stride = 1;
    if( attrStride > -1)
        stride = (unsigned int) mReader->getAttributeValueAsInt( attrStride);

    // store in the library under the given ID
    mAccessorLibrary[pID] = Accessor();
    Accessor& acc = mAccessorLibrary[pID];
    acc.mCount = count;
    acc.mOffset = offset;
    acc.mStride = stride;
    acc.mSource = source+1; // ignore the leading '#'
    acc.mSize = 0; // gets incremented with every param

    // and read the components
    while( mReader->read())
    {
        if( mReader->getNodeType() == irr::io::EXN_ELEMENT)
        {
            if( IsElement( "param"))
            {
                // read data param
                int attrName = TestAttribute( "name");
                std::string name;
                if( attrName > -1)
                {
                    name = mReader->getAttributeValue( attrName);

                    // analyse for common type components and store it's sub-offset in the corresponding field

                    /* Cartesian coordinates */
                    if( name == "X") acc.mSubOffset[0] = acc.mParams.size();
                    else if( name == "Y") acc.mSubOffset[1] = acc.mParams.size();
                    else if( name == "Z") acc.mSubOffset[2] = acc.mParams.size();

                    /* RGBA colors */
                    else if( name == "R") acc.mSubOffset[0] = acc.mParams.size();
                    else if( name == "G") acc.mSubOffset[1] = acc.mParams.size();
                    else if( name == "B") acc.mSubOffset[2] = acc.mParams.size();
                    else if( name == "A") acc.mSubOffset[3] = acc.mParams.size();

                    /* UVWQ (STPQ) texture coordinates */
                    else if( name == "S") acc.mSubOffset[0] = acc.mParams.size();
                    else if( name == "T") acc.mSubOffset[1] = acc.mParams.size();
                    else if( name == "P") acc.mSubOffset[2] = acc.mParams.size();
                    //	else if( name == "Q") acc.mSubOffset[3] = acc.mParams.size();
                    /* 4D uv coordinates are not supported in Assimp */

                    /* Generic extra data, interpreted as UV data, too*/
                    else if( name == "U") acc.mSubOffset[0] = acc.mParams.size();
                    else if( name == "V") acc.mSubOffset[1] = acc.mParams.size();
                    //else
                    //	DefaultLogger::get()->warn( boost::str( boost::format( "Unknown accessor parameter \"%s\". Ignoring data channel.") % name));
                }

                // read data type
                int attrType = TestAttribute( "type");
                if( attrType > -1)
                {
                    // for the moment we only distinguish between a 4x4 matrix and anything else.
                    // TODO: (thom) I don't have a spec here at work. Check if there are other multi-value types
                    // which should be tested for here.
                    std::string type = mReader->getAttributeValue( attrType);
                    if( type == "float4x4")
                        acc.mSize += 16;
                    else
                        acc.mSize += 1;
                }

                acc.mParams.push_back( name);

                // skip remaining stuff of this element, if any
                SkipElement();
            } else
            {
                ThrowException( "Unexpected sub element in tag \"accessor\".");
            }
        }
        else if( mReader->getNodeType() == irr::io::EXN_ELEMENT_END)
        {
            if( strcmp( mReader->getNodeName(), "accessor") != 0)
                ThrowException( "Expected end of \"accessor\" element.");

            break;
        }
    }
}

// ------------------------------------------------------------------------------------------------
// Reads input declarations of per-vertex mesh data into the given mesh
void ColladaParser::ReadVertexData( Mesh* pMesh)
{
    // extract the ID of the <vertices> element. Not that we care, but to catch strange referencing schemes we should warn about
    int attrID= GetAttribute( "id");
    pMesh->mVertexID = mReader->getAttributeValue( attrID);

    // a number of <input> elements
    while( mReader->read())
    {
        if( mReader->getNodeType() == irr::io::EXN_ELEMENT)
        {
            if( IsElement( "input"))
            {
                ReadInputChannel( pMesh->mPerVertexData);
            } else
            {
                ThrowException( "Unexpected sub element in tag \"vertices\".");
            }
        }
        else if( mReader->getNodeType() == irr::io::EXN_ELEMENT_END)
        {
            if( strcmp( mReader->getNodeName(), "vertices") != 0)
                ThrowException( "Expected end of \"vertices\" element.");

            break;
        }
    }
}

// ------------------------------------------------------------------------------------------------
// Reads input declarations of per-index mesh data into the given mesh
void ColladaParser::ReadIndexData( Mesh* pMesh)
{
    std::vector<size_t> vcount;
    std::vector<InputChannel> perIndexData;

    // read primitive count from the attribute
    int attrCount = GetAttribute( "count");
    size_t numPrimitives = (size_t) mReader->getAttributeValueAsInt( attrCount);

    // material subgroup
    int attrMaterial = TestAttribute( "material");
    SubMesh subgroup;
    if( attrMaterial > -1)
        subgroup.mMaterial = mReader->getAttributeValue( attrMaterial);
    subgroup.mNumFaces = numPrimitives;



    // distinguish between polys and triangles
    std::string elementName = mReader->getNodeName();
    PrimitiveType primType = Prim_Invalid;
    if( IsElement( "lines"))
        primType = Prim_Lines;
    else if( IsElement( "linestrips"))
        primType = Prim_LineStrip;
    else if( IsElement( "polygons"))
        primType = Prim_Polygon;
    else if( IsElement( "polylist"))
        primType = Prim_Polylist;
    else if( IsElement( "triangles"))
        primType = Prim_Triangles;
    else if( IsElement( "trifans"))
        primType = Prim_TriFans;
    else if( IsElement( "tristrips"))
        primType = Prim_TriStrips;

    ai_assert( primType != Prim_Invalid);

    // For TriStrips and TriFans we will add the subgroup later, when we now how many triangle primitives we have
    if (primType != Prim_TriStrips && primType != Prim_TriFans) {
        pMesh->mSubMeshes.push_back(subgroup);
    }

    // also a number of <input> elements, but in addition a <p> primitive collection and propably index counts for all primitives
    size_t lastFaceNum =  pMesh->mFaceSize.size();

    while( mReader->read())
    {

        if( mReader->getNodeType() == irr::io::EXN_ELEMENT)
        {


            if( IsElement( "input"))
            {
                ReadInputChannel( perIndexData);
            }
            else if( IsElement( "vcount"))
            {
                if( !mReader->isEmptyElement())
                {
                    if (numPrimitives)	// It is possible to define a mesh without any primitives
                    {
                        // case <polylist> - specifies the number of indices for each polygon
                        const char* content = GetTextContent();
                        vcount.reserve( numPrimitives);
                        for( unsigned int a = 0; a < numPrimitives; a++)
                        {
                            if( *content == 0)
                                ThrowException( "Expected more values while reading vcount contents.");
                            // read a number
                            vcount.push_back( (size_t) strtoul10( content, &content));
                            // skip whitespace after it
                            SkipSpacesAndLineEnd( &content);
                        }
                    }

                    TestClosing( "vcount");
                }
            }
            else if( IsElement( "p"))
            {
                if( !mReader->isEmptyElement())
                {
                    // now here the actual fun starts - these are the indices to construct the mesh data from
                    ReadPrimitives( pMesh, perIndexData, numPrimitives, vcount, primType);
                }
            } else
            {
                ThrowException( "Unexpected sub element in tag \"vertices\".");
            }

        }
        else if( mReader->getNodeType() == irr::io::EXN_ELEMENT_END)
        {
            if( mReader->getNodeName() != elementName)
                ThrowException( boost::str( boost::format( "Expected end of \"%s\" element.") % elementName));

            break;
        }

    }

    // Add number of subfaces for TriStrips and TriFans
    if (primType == Prim_TriStrips || primType == Prim_TriFans) {
        size_t newFaces = pMesh->mFaceSize.size() - lastFaceNum;
        subgroup.mNumFaces = newFaces;
        pMesh->mSubMeshes.push_back( subgroup);
    }

    // Test for plausibility
#ifdef DEBUG
    int facecount = 0;
    for(int i = 0; i < pMesh->mSubMeshes.size(); i++) {
        facecount += ((SubMesh)pMesh->mSubMeshes.at(i)).mNumFaces;
    }
    if(pMesh->mFaceSize.size() != facecount) {
        DBG ("ASSERTION FAILED: " <<pMesh->mFaceSize.size() << "==" << facecount << "\n");
        throw "Inalid face count";
    }
#endif
}

// ------------------------------------------------------------------------------------------------
// Reads a single input channel element and stores it in the given array, if valid 
void ColladaParser::ReadInputChannel( std::vector<InputChannel>& poChannels)
{
    InputChannel channel;

    // read semantic
    int attrSemantic = GetAttribute( "semantic");
    std::string semantic = mReader->getAttributeValue( attrSemantic);
    channel.mType = GetTypeForSemantic( semantic);

    // read source
    int attrSource = GetAttribute( "source");
    const char* source = mReader->getAttributeValue( attrSource);
    if( source[0] != '#')
        ThrowException( boost::str( boost::format( "Unknown reference format in url \"%s\".") % source));
    channel.mAccessor = source+1; // skipping the leading #, hopefully the remaining text is the accessor ID only

    // read index offset, if per-index <input>
    int attrOffset = TestAttribute( "offset");
    if( attrOffset > -1)
        channel.mOffset = mReader->getAttributeValueAsInt( attrOffset);

    // read set if texture coordinates
    if(channel.mType == IT_Texcoord || channel.mType == IT_Color){
        int attrSet = TestAttribute("set");
        if(attrSet > -1){
            attrSet = mReader->getAttributeValueAsInt( attrSet);
            if(attrSet < 0)
                ThrowException( boost::str( boost::format( "Invalid index \"%i\" for set attribute") % (attrSet)));

            channel.mIndex = attrSet;
        }
    }

    // store, if valid type
    if( channel.mType != IT_Invalid)
        poChannels.push_back( channel);

    // skip remaining stuff of this element, if any
    SkipElement();
}

// ------------------------------------------------------------------------------------------------
// Helper function to calculate buckets for TriFans And TriStrips
// TriStrips:
//    [0 1 2] [3 1 2] [3 4 2] [3 4 5] ...
// TriFans:
//    [0 2 1] [0 2 3] [0 4 3] [0 4 5] ...
// LineStrips
//    [0 1] [2 1] [2 3]
// Hence the function returns for the values i = 0, 1, 2, 3, 4
// TriStrips:  0, 1, 2, 0, 1, 2, ...
// TriFans:    0, 2, 1, 2, 1, 2, ...
// LineStrips:
size_t calculateDelta(size_t i, size_t offset, PrimitiveType prim) {
    switch(prim) {
    case Prim_TriStrips:
        return (i % 3) * offset;
    case Prim_TriFans:
        return  i == 0 ? 0 : ((i % 2) + 1) * offset;
    case Prim_LineStrip:
        return (i % 2) * offset;
    default:
        throw "Error using calculateDelta for undefined PrimitiveType.";
    }
}
// Because the function above will calculate the triangle Deltas
// R = clockwise L = counter-clockwise
// TriStrips: L R L R
// TriFans:   R L R L
// if read from i= [0,1,2]
// we need this function to fix this
// TriFans:
//   if current is even: for [0,1,2] return [0,1,2]
//   if it's odd: return [2,1,0]
// TriStrips: The other way round
size_t calculateCounterClockDelta(size_t i, size_t current, PrimitiveType prim) {
    switch (prim) {
    case Prim_TriStrips:
        if (current % 2 == 0) return i;
        else {
            switch(i) {
            case 2: return 0;
            case 1: return 1;
            case 0: return 2;
            }
        }
        break;
    case Prim_TriFans:
        if (current % 2 == 0){
            switch(i) {
            case 2: return 0;
            case 1: return 1;
            case 0: return 2;
            }
        }
        else return i;
        break;
    default:
        throw "Error using calculateCounterClockDelta for undefined PrimitiveType.";
    }
    throw "We should never be here";
    return 0;
}

// ------------------------------------------------------------------------------------------------
// Reads a <p> primitive index list and assembles the mesh data into the given mesh
void ColladaParser::ReadPrimitives( Mesh* pMesh, std::vector<InputChannel>& pPerIndexChannels, 
                                    size_t pNumPrimitives, const std::vector<size_t>& pVCount, PrimitiveType pPrimType)
{

    // determine number of indices coming per vertex
    // find the offset index for all per-vertex channels
    size_t numOffsets = 1;
    size_t perVertexOffset = SIZE_MAX; // invalid value
    BOOST_FOREACH( const InputChannel& channel, pPerIndexChannels)
    {
        numOffsets = std::max( numOffsets, channel.mOffset+1);
        if( channel.mType == IT_Vertex)
            perVertexOffset = channel.mOffset;
    }

    // determine the expected number of indices
    size_t expectedPointCount = 0;
    switch( pPrimType)
    {
    case Prim_Polylist:
    {
        BOOST_FOREACH( size_t i, pVCount)
                expectedPointCount += i;
        break;
    }
    case Prim_Lines:
        expectedPointCount = 2 * pNumPrimitives;
        break;
    case Prim_Triangles:
        expectedPointCount = 3 * pNumPrimitives;
        break;
    default:
        // other primitive types don't state the index count upfront... we need to guess
        break;
    }

    // and read all indices into a temporary array
    std::vector<size_t> indices;
    if( expectedPointCount > 0)
        indices.reserve( expectedPointCount * numOffsets);

    if (pNumPrimitives > 0)	// It is possible to not contain any indicies
    {
        const char* content = GetTextContent();
        while( *content != 0)
        {
            // read a value.
            // Hack: (thom) Some exporters put negative indices sometimes. We just try to carry on anyways.
            int value = std::max( 0, strtol10( content, &content));
            indices.push_back( size_t( value));
            // skip whitespace after it
            SkipSpacesAndLineEnd( &content);
        }
    }

    // complain if the index count doesn't fit
    if( expectedPointCount > 0 && indices.size() != expectedPointCount * numOffsets)
        ThrowException( "Expected different index count in <p> element.");
    else if( expectedPointCount == 0 && (indices.size() % numOffsets) != 0)
        ThrowException( "Expected different index count in <p> element.");

    // find the data for all sources
    for( std::vector<InputChannel>::iterator it = pMesh->mPerVertexData.begin(); it != pMesh->mPerVertexData.end(); ++it)
    {
        InputChannel& input = *it;
        if( input.mResolved)
            continue;

        // find accessor
        input.mResolved = &ResolveLibraryReference( mAccessorLibrary, input.mAccessor);
        // resolve accessor's data pointer as well, if neccessary
        const Accessor* acc = input.mResolved;
        if( !acc->mData)
            acc->mData = &ResolveLibraryReference( mDataLibrary, acc->mSource);
    }
    // and the same for the per-index channels
    for( std::vector<InputChannel>::iterator it = pPerIndexChannels.begin(); it != pPerIndexChannels.end(); ++it)
    {
        InputChannel& input = *it;
        if( input.mResolved)
            continue;

        // ignore vertex pointer, it doesn't refer to an accessor
        if( input.mType == IT_Vertex)
        {
            // warn if the vertex channel does not refer to the <vertices> element in the same mesh
            if( input.mAccessor != pMesh->mVertexID)
                ThrowException( "Unsupported vertex referencing scheme. I fucking hate Collada.");
            continue;
        }

        // find accessor
        input.mResolved = &ResolveLibraryReference( mAccessorLibrary, input.mAccessor);
        // resolve accessor's data pointer as well, if neccessary
        const Accessor* acc = input.mResolved;
        if( !acc->mData)
            acc->mData = &ResolveLibraryReference( mDataLibrary, acc->mSource);
    }


    // now assemble vertex data according to those indices
    std::vector<size_t>::const_iterator idx = indices.begin();

    // For continued primitives, the given count does not come all in one <p>, but only one primitive per <p>
    size_t numPrimitives = pNumPrimitives;

    if( pPrimType == Prim_TriFans || pPrimType == Prim_Polygon || pPrimType == Prim_TriStrips)
        numPrimitives = 1;

    pMesh->mFaceSize.reserve( numPrimitives);
    pMesh->mFacePosIndices.reserve( indices.size() / numOffsets);

    for( size_t a = 0; a < numPrimitives; a++)
    {
        // determine number of points for this primitive
        size_t numPoints = 0;
        switch( pPrimType)
        {
        case Prim_Lines:
            numPoints = 2;
            break;
        case Prim_Triangles:
            numPoints = 3;
            break;
        case Prim_Polylist:
            numPoints = pVCount[a];
            break;
        case Prim_Polygon:
            numPoints = indices.size() / numOffsets;
            break;
        case Prim_TriFans:
        case Prim_TriStrips:
        {
            numPoints = indices.size() / numOffsets;

            ai_assert( numOffsets < TRIANGLE_OFF && perVertexOffset < TRIANGLE_OFF);
            size_t triIndices[TRIANGLE_OFF*3];

            for( size_t b = 0; b < numPoints; b++)
            {
                // This Delta is used to fill the buckets in triIndices ... see calculateDelta for more info.
                size_t DELTA = calculateDelta(b, TRIANGLE_OFF, pPrimType);

                // read all indices for this vertex. Yes, in a hacky local array
                for( size_t offsets = 0; offsets < numOffsets; ++offsets)
                    triIndices[offsets+DELTA] = *idx++;

                // for the first complete triangle and all following
                if (b>1) {
                    // create new face: triangle
                    pMesh->mFaceSize.push_back(3);

                    // Add three points to the mesh
                    for (int i = 0; i < 3; i++) {
                        DELTA = calculateCounterClockDelta(i,b,pPrimType) * TRIANGLE_OFF;
                        // extract per-vertex channels using the global per-vertex offset
                        for( std::vector<InputChannel>::iterator it = pMesh->mPerVertexData.begin(); it != pMesh->mPerVertexData.end(); ++it)
                            ExtractDataObjectFromChannel( *it, triIndices[perVertexOffset+DELTA], pMesh);
                        // and extract per-index channels using there specified offset
                        for( std::vector<InputChannel>::iterator it = pPerIndexChannels.begin(); it != pPerIndexChannels.end(); ++it)
                            ExtractDataObjectFromChannel( *it, triIndices[it->mOffset+DELTA], pMesh);

                        // store the vertex-data index for later assignment of bone vertex weights
                        pMesh->mFacePosIndices.push_back( triIndices[perVertexOffset+DELTA]);
                    }
                }
            }
            goto doneReadPrimitives; // Yes, we use a goto here to cleanup. Apple does it. Why shouldn't we do it :P
        }
        case Prim_LineStrip:
            //ThrowException( "Unsupported primitive type: LineStrip");

            numPoints = indices.size() / numOffsets;

            ai_assert( numOffsets < TRIANGLE_OFF && perVertexOffset < TRIANGLE_OFF);
            size_t triIndices[TRIANGLE_OFF*2];

            for( size_t b = 0; b < numPoints; b++)
            {
                // This Delta is used to fill the buckets in triIndices ... see calculateDelta for more info.
                size_t DELTA = calculateDelta(b, TRIANGLE_OFF, pPrimType);

                // read all indices for this vertex. Yes, in a hacky local array
                for( size_t offsets = 0; offsets < numOffsets; ++offsets)
                    triIndices[offsets+DELTA] = *idx++;

                // for the first complete line and all following
                if (b>0) {
                    DBG("Line\n");
                    // create new face: line
                    pMesh->mFaceSize.push_back(2);

                    // Add three points to the mesh
                    for (int i = 0; i < 2; i++) {
                        DELTA = i * TRIANGLE_OFF;
                        // extract per-vertex channels using the global per-vertex offset
                        for( std::vector<InputChannel>::iterator it = pMesh->mPerVertexData.begin(); it != pMesh->mPerVertexData.end(); ++it)
                            ExtractDataObjectFromChannel( *it, triIndices[perVertexOffset+DELTA], pMesh);
                        // and extract per-index channels using there specified offset
                        for( std::vector<InputChannel>::iterator it = pPerIndexChannels.begin(); it != pPerIndexChannels.end(); ++it)
                            ExtractDataObjectFromChannel( *it, triIndices[it->mOffset+DELTA], pMesh);

                        // store the vertex-data index for later assignment of bone vertex weights
                        pMesh->mFacePosIndices.push_back( triIndices[perVertexOffset+DELTA]);
                    }
                }
            }
            goto doneReadPrimitives;
            break;
        default:
            // LineStrip and TriStrip not supported due to expected index unmangling (e.g. someone beeing lazy)
            ThrowException( "Unsupported primitive type");
            break;
        }

        // store the face size to later reconstruct the face from
        pMesh->mFaceSize.push_back( numPoints);

        ai_assert( numOffsets < 20 && perVertexOffset < 20);
        size_t vindex[20];

        // gather that number of vertices
        for( size_t b = 0; b < numPoints; b++)
        {
            // read all indices for this vertex. Yes, in a hacky local array
            for( size_t offsets = 0; offsets < numOffsets; ++offsets)
                vindex[offsets] = *idx++;

            // extract per-vertex channels using the global per-vertex offset
            for( std::vector<InputChannel>::iterator it = pMesh->mPerVertexData.begin(); it != pMesh->mPerVertexData.end(); ++it)
                ExtractDataObjectFromChannel( *it, vindex[perVertexOffset], pMesh);
            // and extract per-index channels using there specified offset
            for( std::vector<InputChannel>::iterator it = pPerIndexChannels.begin(); it != pPerIndexChannels.end(); ++it)
                ExtractDataObjectFromChannel( *it, vindex[it->mOffset], pMesh);

            // store the vertex-data index for later assignment of bone vertex weights
            pMesh->mFacePosIndices.push_back( vindex[perVertexOffset]);
        }
    }

doneReadPrimitives:
    // if I ever get my hands on that guy who invented this steaming pile of indirection...
    TestClosing( "p");
}

// ------------------------------------------------------------------------------------------------
// Extracts a single object from an input channel and stores it in the appropriate mesh data array 
void ColladaParser::ExtractDataObjectFromChannel( const InputChannel& pInput, size_t pLocalIndex, Mesh* pMesh)
{
    // ignore vertex referrer - we handle them that separate
    if( pInput.mType == IT_Vertex)
        return;

    const Accessor& acc = *pInput.mResolved;
    if( pLocalIndex >= acc.mCount)
        ThrowException( boost::str( boost::format( "Invalid data index (%d/%d) in primitive specification") % pLocalIndex % acc.mCount));

    // get a pointer to the start of the data object referred to by the accessor and the local index
    const float* dataObject = &(acc.mData->mValues[0]) + acc.mOffset + pLocalIndex* acc.mStride;

    // assemble according to the accessors component sub-offset list. We don't care, yet,
    // what kind of object exactly we're extracting here
    float obj[4];
    for( size_t c = 0; c < 4; ++c)
        obj[c] = dataObject[acc.mSubOffset[c]];

    // now we reinterpret it according to the type we're reading here
    switch( pInput.mType)
    {
    case IT_Position: // ignore all position streams except 0 - there can be only one position
        if( pInput.mIndex == 0)
            pMesh->mPositions.push_back( aiVector3D( obj[0], obj[1], obj[2]));
        else
            DefaultLogger::get()->error("Collada: just one vertex position stream supported");
        break;
    case IT_Normal:
        // pad to current vertex count if necessary
        if( pMesh->mNormals.size() < pMesh->mPositions.size()-1)
            pMesh->mNormals.insert( pMesh->mNormals.end(), pMesh->mPositions.size() - pMesh->mNormals.size() - 1, aiVector3D( 0, 1, 0));

        // ignore all normal streams except 0 - there can be only one normal
        if( pInput.mIndex == 0)
            pMesh->mNormals.push_back( aiVector3D( obj[0], obj[1], obj[2]));
        else
            DefaultLogger::get()->error("Collada: just one vertex normal stream supported");
        break;
    case IT_Tangent:
        // pad to current vertex count if necessary
        if( pMesh->mTangents.size() < pMesh->mPositions.size()-1)
            pMesh->mTangents.insert( pMesh->mTangents.end(), pMesh->mPositions.size() - pMesh->mTangents.size() - 1, aiVector3D( 1, 0, 0));

        // ignore all tangent streams except 0 - there can be only one tangent
        if( pInput.mIndex == 0)
            pMesh->mTangents.push_back( aiVector3D( obj[0], obj[1], obj[2]));
        else
            DefaultLogger::get()->error("Collada: just one vertex tangent stream supported");
        break;
    case IT_Bitangent:
        // pad to current vertex count if necessary
        if( pMesh->mBitangents.size() < pMesh->mPositions.size()-1)
            pMesh->mBitangents.insert( pMesh->mBitangents.end(), pMesh->mPositions.size() - pMesh->mBitangents.size() - 1, aiVector3D( 0, 0, 1));

        // ignore all bitangent streams except 0 - there can be only one bitangent
        if( pInput.mIndex == 0)
            pMesh->mBitangents.push_back( aiVector3D( obj[0], obj[1], obj[2]));
        else
            DefaultLogger::get()->error("Collada: just one vertex bitangent stream supported");
        break;
    case IT_Texcoord:
        // up to 4 texture coord sets are fine, ignore the others
        if( pInput.mIndex < AI_MAX_NUMBER_OF_TEXTURECOORDS)
        {
            // pad to current vertex count if necessary
            if( pMesh->mTexCoords[pInput.mIndex].size() < pMesh->mPositions.size()-1)
                pMesh->mTexCoords[pInput.mIndex].insert( pMesh->mTexCoords[pInput.mIndex].end(),
                        pMesh->mPositions.size() - pMesh->mTexCoords[pInput.mIndex].size() - 1, aiVector3D( 0, 0, 0));

            pMesh->mTexCoords[pInput.mIndex].push_back( aiVector3D( obj[0], obj[1], obj[2]));
            if (0 != acc.mSubOffset[2] || 0 != acc.mSubOffset[3]) /* hack ... consider cleaner solution */
                pMesh->mNumUVComponents[pInput.mIndex]=3;
        }	else
        {
            DefaultLogger::get()->error("Collada: too many texture coordinate sets. Skipping.");
        }
        break;
    case IT_Color:
        // up to 4 color sets are fine, ignore the others
        if( pInput.mIndex < AI_MAX_NUMBER_OF_COLOR_SETS)
        {
            // pad to current vertex count if necessary
            if( pMesh->mColors[pInput.mIndex].size() < pMesh->mPositions.size()-1)
                pMesh->mColors[pInput.mIndex].insert( pMesh->mColors[pInput.mIndex].end(),
                        pMesh->mPositions.size() - pMesh->mColors[pInput.mIndex].size() - 1, aiColor4D( 0, 0, 0, 1));

            pMesh->mColors[pInput.mIndex].push_back( aiColor4D( obj[0], obj[1], obj[2], obj[3]));
        } else
        {
            DefaultLogger::get()->error("Collada: too many vertex color sets. Skipping.");
        }

        break;
    default:
        // IT_Invalid and IT_Vertex
        ai_assert(false && "shouldn't ever get here");
    }
}

// ------------------------------------------------------------------------------------------------
// Reads the library of node hierarchies and scene parts
void ColladaParser::ReadSceneLibrary()
{
    if( mReader->isEmptyElement())
        return;

    while( mReader->read())
    {
        if( mReader->getNodeType() == irr::io::EXN_ELEMENT)
        {
            // a visual scene - generate root node under its ID and let ReadNode() do the recursive work
            if( IsElement( "visual_scene"))
            {
                // read ID. Is optional according to the spec, but how on earth should a scene_instance refer to it then?
                int indexID = GetAttribute( "id");
                const char* attrID = mReader->getAttributeValue( indexID);

                // read name if given.
                int indexName = TestAttribute( "name");
                const char* attrName = "unnamed";
                if( indexName > -1)
                    attrName = mReader->getAttributeValue( indexName);

                // create a node and store it in the library under its ID
                Node* node = new Node;
                node->mID = attrID;
                node->mName = attrName;
                mNodeLibrary[node->mID] = node;

                ReadSceneNode( node);
            } else
            {
                // ignore the rest
                SkipElement();
            }
        }
        else if( mReader->getNodeType() == irr::io::EXN_ELEMENT_END)
        {
            if( strcmp( mReader->getNodeName(), "library_visual_scenes") == 0)
                //ThrowException( "Expected end of \"library_visual_scenes\" element.");

                break;
        }
    }
}

// ------------------------------------------------------------------------------------------------
// Reads a scene node's contents including children and stores it in the given node
void ColladaParser::ReadSceneNode( Node* pNode)
{
    // quit immediately on <bla/> elements
    if( mReader->isEmptyElement())
        return;

    while( mReader->read())
    {
        if( mReader->getNodeType() == irr::io::EXN_ELEMENT)
        {
            if( IsElement( "node"))
            {
                Node* child = new Node;
                int attrID = TestAttribute( "id");
                if( attrID > -1)
                    child->mID = mReader->getAttributeValue( attrID);
                int attrSID = TestAttribute( "sid");
                if( attrSID > -1)
                    child->mSID = mReader->getAttributeValue( attrSID);

                int attrName = TestAttribute( "name");
                if( attrName > -1)
                    child->mName = mReader->getAttributeValue( attrName);

                // TODO: (thom) support SIDs
                // ai_assert( TestAttribute( "sid") == -1);

                if (pNode)
                {
                    pNode->mChildren.push_back( child);
                    child->mParent = pNode;
                }
                else
                {
                    // no parent node given, probably called from <library_nodes> element.
                    // create new node in node library
                    mNodeLibrary[child->mID] = child;
                }

                // read on recursively from there
                ReadSceneNode( child);
                continue;
            }
            // For any further stuff we need a valid node to work on
            else if (!pNode)
                continue;

            if( IsElement( "lookat"))
                ReadNodeTransformation( pNode->mTransforms, TF_LOOKAT);
            else if( IsElement( "matrix"))
                ReadNodeTransformation( pNode->mTransforms, TF_MATRIX);
            else if( IsElement( "rotate"))
                ReadNodeTransformation( pNode->mTransforms, TF_ROTATE);
            else if( IsElement( "scale"))
                ReadNodeTransformation( pNode->mTransforms, TF_SCALE);
            else if( IsElement( "skew"))
                ReadNodeTransformation( pNode->mTransforms, TF_SKEW);
            else if( IsElement( "translate"))
                ReadNodeTransformation( pNode->mTransforms, TF_TRANSLATE);
            else if( IsElement( "render") && pNode->mParent == NULL && 0 == pNode->mPrimaryCamera.length())
            {
                // ... scene evaluation or, in other words, postprocessing pipeline,
                // or, again in other words, a turing-complete description how to
                // render a Collada scene. The only thing that is interesting for
                // us is the primary camera.
                int attrId = TestAttribute("camera_node");
                if (-1 != attrId)
                {
                    const char* s = mReader->getAttributeValue(attrId);
                    if (s[0] != '#')
                        DefaultLogger::get()->error("Collada: Unresolved reference format of camera");
                    else
                        pNode->mPrimaryCamera = s+1;
                }
            }
            else if( IsElement( "instance_node"))
            {
                // find the node in the library
                int attrID = TestAttribute( "url");
                if( attrID != -1)
                {
                    const char* s = mReader->getAttributeValue(attrID);
                    if (s[0] != '#') {
                        std::cerr << "File Reference not supported: " << s << std::endl;
                        filesToParse.push_back(s);
                        DefaultLogger::get()->error("Collada: Unresolved reference format of node");

                        pNode->mNodeInstances.push_back(NodeInstance());
                        pNode->mNodeInstances.back().mNode = s;

                    } else
                    {
                        pNode->mNodeInstances.push_back(NodeInstance());
                        pNode->mNodeInstances.back().mNode = s+1;
                    }
                }
            }
            else if( IsElement( "instance_geometry") || IsElement( "instance_controller"))
            {
                // Reference to a mesh or controller, with possible material associations
                ReadNodeGeometry( pNode);
            }
            else if( IsElement( "instance_light"))
            {
                // Reference to a light, name given in 'url' attribute
                int attrID = TestAttribute("url");
                if (-1 == attrID)
                    DefaultLogger::get()->warn("Collada: Expected url attribute in <instance_light> element");
                else
                {
                    const char* url = mReader->getAttributeValue( attrID);
                    if( url[0] != '#')
                        ThrowException( "Unknown reference format in <instance_light> element");

                    pNode->mLights.push_back(LightInstance());
                    pNode->mLights.back().mLight = url+1;
                }
            }
            else if( IsElement( "instance_camera"))
            {
                // Reference to a camera, name given in 'url' attribute
                int attrID = TestAttribute("url");
                if (-1 == attrID)
                    DefaultLogger::get()->warn("Collada: Expected url attribute in <instance_camera> element");
                else
                {
                    const char* url = mReader->getAttributeValue( attrID);
                    if( url[0] != '#')
                        ThrowException( "Unknown reference format in <instance_camera> element");

                    pNode->mCameras.push_back(CameraInstance());
                    pNode->mCameras.back().mCamera = url+1;
                }
            }
            else
            {
                // skip everything else for the moment
                SkipElement();
            }
        }
        else if( mReader->getNodeType() == irr::io::EXN_ELEMENT_END) {
            break;
        }
    }
}

// ------------------------------------------------------------------------------------------------
// Reads a node transformation entry of the given type and adds it to the given node's transformation list.
void ColladaParser::ReadNodeTransformation(std::vector<Transform>& transVec, TransformType pType, irr::io::IrrXMLReader* reader, bool transformsOnly)
{
	std::cout << "ReadNodeTransformation(): Got transformation vector count = " << transVec.size() << std::endl;

	irr::io::IrrXMLReader* mReader = this->mReader;
	if (reader != NULL && transformsOnly)
		mReader = reader;

    if( mReader->isEmptyElement())
        return;

    std::string tagName = mReader->getNodeName();

    Transform tf;
    tf.mType = pType;

    // read SID
    int indexSID = TestAttribute( "sid", mReader);
    if( indexSID >= 0)
        tf.mID = mReader->getAttributeValue( indexSID);

    // how many parameters to read per transformation type
    static const unsigned int sNumParameters[] = { 9, 4, 3, 3, 7, 16 };
    const char* content = GetTextContent(mReader);

    // read as many parameters and store in the transformation
    for( unsigned int a = 0; a < sNumParameters[pType]; a++)
    {
        // read a number
        content = fast_atoreal_move<float>( content, tf.f[a]);
        // skip whitespace after it
        SkipSpacesAndLineEnd( &content);
    }

    // place the transformation at the queue of the node
    transVec.push_back( tf);

    // and consume the closing tag
    TestClosing( tagName.c_str(), mReader);
}

// ------------------------------------------------------------------------------------------------
// Reads a node transformation entry of the given type and adds it to the given node's transformation list.
void ColladaParser::ReadLinkTransformation(std::vector<Transform>& transVec, TransformType pType, Collada::Link& link, RelativeTransformStack& relativeTransforms, bool isAttachment, irr::io::IrrXMLReader* reader, int attachmentIndex)
{
	std::cout << "ReadLinkTransformation(): Got transformation vector count = " << transVec.size() << ", isAttachment = " << isAttachment << std::endl;
	ReadNodeTransformation(transVec, pType, reader, true);
	
	int registeredTransforms = 0;
	for (KinematicsModelsRelativeTransformStack::iterator ct_it = relativeTransforms.transformStack.begin(); ct_it != relativeTransforms.transformStack.end(); ++ct_it)
	{
		if (ct_it->first.compare(link.id) == 0)
			registeredTransforms++;
	}

	std::cout << " registered for link " << link.id << ": " << registeredTransforms << std::endl;
	std::cout << "  already have registered transforms, insert with index " << registeredTransforms << std::endl;

	RelativeTransformEntry entry;
	entry.elementId = link.id;
	entry.index = registeredTransforms;
	entry.attachmentIndices.push_back(attachmentIndex);
	//entry.isAttachment = isAttachment;
	entry.transform = transVec.back();

	//std::cout << "  isAttachment in entry = " << entry.isAttachment << std::endl;

	relativeTransforms.transformStack.push_back(std::make_pair(link.id, entry));
}

// ------------------------------------------------------------------------------------------------
// Processes bind_vertex_input and bind elements
void ColladaParser::ReadMaterialVertexInputBinding( Collada::SemanticMappingTable& tbl)
{
    while( mReader->read())
    {
        if( mReader->getNodeType() == irr::io::EXN_ELEMENT)	{
            if( IsElement( "bind_vertex_input"))
            {
                Collada::InputSemanticMapEntry vn;

                // effect semantic
                int n = GetAttribute("semantic");
                std::string s = mReader->getAttributeValue(n);

                // input semantic
                n = GetAttribute("input_semantic");
                vn.mType = GetTypeForSemantic( mReader->getAttributeValue(n) );

                // index of input set
                n = TestAttribute("input_set");
                if (-1 != n)
                    vn.mSet = mReader->getAttributeValueAsInt(n);

                tbl.mMap[s] = vn;
            }
            else if( IsElement( "bind")) {
                DefaultLogger::get()->warn("Collada: Found unsupported <bind> element");
            }
        }
        else if( mReader->getNodeType() == irr::io::EXN_ELEMENT_END)	{
            if( strcmp( mReader->getNodeName(), "instance_material") == 0)
                break;
        }
    }
}

// ------------------------------------------------------------------------------------------------
// Reads a mesh reference in a node and adds it to the node's mesh list
void ColladaParser::ReadNodeGeometry( Node* pNode)
{
    // referred mesh is given as an attribute of the <instance_geometry> element
    int attrUrl = GetAttribute("url");
    // TP anfang
    int visualMod = TestAttribute("isVisualModel");
    // TP ende
    const char* url = mReader->getAttributeValue( attrUrl);
    if( url[0] != '#')
        ThrowException( "Unknown reference format");

    Collada::MeshInstance instance;
    instance.mMeshOrController = url+1; // skipping the leading #

    // TP anfang
    if (visualMod > -1)
    {
        //const char* visualAtt = mReader->getAttributeValue(visualMod); // TODO: make the attribute value do something reasonable
        instance.isVisualMesh = true; 
    }
    else 
    {
        instance.isVisualMesh = false;
    }
    // TP ende

    if( !mReader->isEmptyElement())
    {
        // read material associations. Ignore additional elements inbetween
        while( mReader->read())
        {
            if( mReader->getNodeType() == irr::io::EXN_ELEMENT)
            {
                if( IsElement( "instance_material"))
                {
                    // read ID of the geometry subgroup and the target material
                    int attrGroup = GetAttribute( "symbol");
                    std::string group = mReader->getAttributeValue( attrGroup);
                    int attrMaterial = GetAttribute( "target");
                    const char* urlMat = mReader->getAttributeValue( attrMaterial);
                    Collada::SemanticMappingTable s;
                    if( urlMat[0] == '#')
                        urlMat++;

                    s.mMatName = urlMat;

                    // resolve further material details + THIS UGLY AND NASTY semantic mapping stuff
                    if( !mReader->isEmptyElement())
                        ReadMaterialVertexInputBinding(s);

                    // store the association
                    instance.mMaterials[group] = s;
                }
            }
            else if( mReader->getNodeType() == irr::io::EXN_ELEMENT_END)
            {
                if( strcmp( mReader->getNodeName(), "instance_geometry") == 0
                        || strcmp( mReader->getNodeName(), "instance_controller") == 0)
                    break;
            }
        }
    }

    // store it
    pNode->mMeshes.push_back( instance);
}

// ------------------------------------------------------------------------------------------------
// Reads the collada scene
void ColladaParser::ReadScene()
{
    if( mReader->isEmptyElement())
        return;

    while( mReader->read())
    {
        if( mReader->getNodeType() == irr::io::EXN_ELEMENT)	{
            if( IsElement ( "instance_kinematics_scene")) {
                ReadInstanceKinematicsScene();

            } else if( IsElement( "instance_visual_scene"))
            {
                // should be the first and only occurence
                if( mRootNode)
                    ThrowException( "Invalid scene containing multiple root nodes");

                // read the url of the scene to instance. Should be of format "#some_name"
                int urlIndex = GetAttribute( "url");
                const char* url = mReader->getAttributeValue( urlIndex);
                if( url[0] != '#')
                    ThrowException( "Unknown reference format");

                // find the referred scene, skip the leading #
                NodeLibrary::const_iterator sit = mNodeLibrary.find( url+1);
                if( sit == mNodeLibrary.end())
                    ThrowException( "Unable to resolve visual_scene reference \"" + std::string(url) + "\".");
                mRootNode = sit->second;
            } else	{
                SkipElement();
            }
        }
        else if( mReader->getNodeType() == irr::io::EXN_ELEMENT_END){
            break;
        }
    }
}

std::string ColladaParser::GetUnnamed(std::string name) {
    std::stringstream ss;
    ss << name << mUnnamedCounter++;
    return ss.str();
}

void ColladaParser::ReadKinematicsModelsLibrary() {

    std::cout << "Read Kinematics Models" << std::endl;
    while( mReader->read())
    {
        if( mReader->getNodeType() == irr::io::EXN_ELEMENT)	{
            if( IsOpenElement( "kinematics_model")) {
                KinematicsModel kinmod;
				KinematicsModel kinmod_container;

                kinmod.id = mReader->getAttributeValue("id") ? mReader->getAttributeValue("id") : GetUnnamed("kinmod_unnamed_");
                kinmod.name = mReader->getAttributeValue("name") ? mReader->getAttributeValue("name") : kinmod.id;

				kinmod_container.id = kinmod.id;
				kinmod_container.name = kinmod.name;

				std::cout << " --> read Kinematics Model: " << kinmod.id << " -- name = " << kinmod.name << std::endl;

                while( mReader->read())
                {
                    if( mReader->getNodeType() == irr::io::EXN_ELEMENT)	{
                        if( IsOpenElement( "technique_common")) {

                            while( mReader->read())
                            {
                                if( mReader->getNodeType() == irr::io::EXN_ELEMENT)	{
                                    if( IsOpenElement( "instance_joint")) {
                                        InstanceJoint ij;
                                        ij.kinmodId = kinmod.id;
                                        ij.kinmodName = kinmod.name;
                                        ij.url = mReader->getAttributeValue("url");
                                        ij.sid = kinmod.id + "/" + mReader->getAttributeValue("sid");
                                        mInstanceJointLibrary[ij.sid] = ij;

										kinmod_container.jointInstances.push_back(ij.url);

                                    } else	if( IsOpenElement( "link")){

										std::string rootLinkSid = mReader->getAttributeValue("sid") ? mReader->getAttributeValue("sid") : GetUnnamed("auto_link_");
										std::string rootLinkName = mReader->getAttributeValue("name") ? mReader->getAttributeValue("name") : rootLinkSid;
										kinmod.rootLinkName = rootLinkName;

										RelativeTransformStack relativeTransforms;

										std::cout << "     --> read links..." << std::endl;
										ReadLink(kinmod.id, kinmod.name, false, NULL, relativeTransforms);

#if 0
										std::cout << "         After reading links: Registered relativeTransforms = " << relativeTransforms.transformStack.size() << std::endl;

										std::cout << "BEGIN RELATIVE TRANSFORMS DUMP" << std::endl;

										for (KinematicsModelsRelativeTransformStack::iterator it = relativeTransforms.transformStack.begin(); it != relativeTransforms.transformStack.end(); ++it)
										{
											std::string key = it->first;
											
											std::cout << "          - Link " << key << ": " << std::endl;

											for (KinematicsModelsRelativeTransformStack::iterator tr_it = relativeTransforms.transformStack.begin(); tr_it != relativeTransforms.transformStack.end(); ++tr_it)
											{
												if (tr_it->first.compare(key) == 0)
												{
													RelativeTransformEntry& rt_entry = tr_it->second;
													if (rt_entry.transform.mType == TF_TRANSLATE)
														std::cout << "           " << "* " << rt_entry.elementId << ": translate by " << rt_entry.transform.f[0] << "," << rt_entry.transform.f[1] << "," << rt_entry.transform.f[2] << std::endl;
													else if (rt_entry.transform.mType == TF_ROTATE)
														std::cout << "           " << "* " << rt_entry.elementId << ": rotate around axis " << rt_entry.transform.f[0] << "," << rt_entry.transform.f[1] << "," << rt_entry.transform.f[2] << " by " << rt_entry.transform.f[3] << " deg." << std::endl;
												}
											}
										}

										std::cout << "END   RELATIVE TRANSFORMS DUMP" << std::endl;

										mKinematicsModelsRelativeTransforms.insert(std::make_pair(kinmod.id, relativeTransforms));
#endif

                                    } else {
                                        SkipElement();
                                    }
                                } else if( mReader->getNodeType() == irr::io::EXN_ELEMENT_END) {
                                    if(IsCloseElement("technique_common")) break;
                                }
                            }

                        } else if (IsOpenElement("technique"))  {
                            std::string profile = mReader->getAttributeValue("profile");
                            if (profile.compare("Zyklio") == 0) {

                                while( mReader->read())
                                {
                                    if( mReader->getNodeType() == irr::io::EXN_ELEMENT)	{
                                        if( IsOpenElement( "tool_attachment")) {
                                            ToolAttachment ta;
                                            ta.link =  mReader->getAttributeValue("link")?mReader->getAttributeValue("link") : "ERROR: attribute link missing";
                                            ta.name =  mReader->getAttributeValue("name")?mReader->getAttributeValue("name") : "No Name";
                                            ta.toolId =  mReader->getAttributeValue("tool")?mReader->getAttributeValue("tool") : "#unnamed_tool";
                                            ta.toolId = ta.toolId.substr(1);

                                            while( mReader->read())
                                            {
                                                if( mReader->getNodeType() == irr::io::EXN_ELEMENT)	{
                                                    if( IsOpenElement( "rotate")) {
                                                        ReadNodeTransformation( ta.mTransforms, TF_ROTATE);
                                                    } else if( IsOpenElement( "translate")) {
                                                        ReadNodeTransformation( ta.mTransforms, TF_TRANSLATE);
                                                    } else {
                                                        SkipElement();
                                                    }
                                                } else if( mReader->getNodeType() == irr::io::EXN_ELEMENT_END){
                                                    if (IsCloseElement("tool_attachment")) break;
                                                }
                                            }

                                            kinmod.toolAttachment[ta.link] = ta;
                                        } else	{
                                            SkipElement();
                                        }
                                    } else if( mReader->getNodeType() == irr::io::EXN_ELEMENT_END){
                                        if (IsCloseElement("technique")) break;
                                    }
                                }

                            } else {
                                SkipElement();
                            }

                        } else {
                            SkipElement();
                        }
                    } else if( mReader->getNodeType() == irr::io::EXN_ELEMENT_END) {
                        if(IsCloseElement("kinematics_model")) break;
                    }
                }
                mKinematicsModelLibrary[kinmod.id] = kinmod;
				mKinematicsModelLibraryContainers[kinmod_container.id] = kinmod_container;


            } else	{
                SkipElement();
            }
        }  else if( mReader->getNodeType() == irr::io::EXN_ELEMENT_END) {
            if (IsCloseElement("library_kinematics_models")) break;
        }
    }
    std::cout << "Done Read Kinematics Models" << std::endl;
}


void ColladaParser::ReadAttachment(Link &l, std::string kinMod, std::string kinModName, std::string jointN, std::vector<Transform> translations, bool transformsOnly, irr::io::IrrXMLReader* reader, RelativeTransformStack relativeTransforms, int attachmentIndex) {
    
	irr::io::IrrXMLReader* mReader = this->mReader;
	if (transformsOnly)
		mReader = reader;
	
	if( mReader->isEmptyElement()) {
        return;
    }

	int link_transforms = 0;
	int link_attachments = 0;
    while( mReader->read())
    {
        if( mReader->getNodeType() == irr::io::EXN_ELEMENT)	{

            // TODO: Read Translation and rotation
            if ( IsOpenElement( "translate", mReader)) {
				if (!transformsOnly)
					ReadNodeTransformation( translations, TF_TRANSLATE);
				else
					ReadLinkTransformation(translations, TF_TRANSLATE, l, relativeTransforms, true, mReader, attachmentIndex);
				
				link_transforms++;
			}
			else if (IsOpenElement("rotate", mReader)) {
				if (!transformsOnly)
					ReadNodeTransformation( translations, TF_ROTATE);
				else
					ReadLinkTransformation(translations, TF_ROTATE, l, relativeTransforms, true, mReader, attachmentIndex);
				
				link_transforms++;
			}
			else if (IsOpenElement("link", mReader)) {
				l.postLinks.push_back(ReadLink(kinMod, kinModName, transformsOnly, mReader, relativeTransforms, jointN, l.id, translations, attachmentIndex));
            } else {
                SkipElement(mReader);
            }
        } else if( mReader->getNodeType() == irr::io::EXN_ELEMENT_END) {
			if (IsCloseElement("attachment_full", mReader)) break;
        }
    }
}

std::string ColladaParser::ReadLink(std::string kinMod, std::string kinModName, bool transformsOnly, irr::io::IrrXMLReader* reader, RelativeTransformStack relativeTransforms, std::string joint, std::string preLink, std::vector<Transform> translations, int attachmentIndex) {

	std::cout << "ReadLink(" << kinMod << ", " << kinModName << ", " << joint << ", " << preLink << ")" << std::endl;

	irr::io::IrrXMLReader* mReader = this->mReader;
	if (transformsOnly)
		mReader = reader;

#if 0
	std::cout << " Transforms passed: " << translations.size() << std::endl;
	int t_idx = 0;

	aiMatrix4x4 res;

	for (std::vector<Transform>::const_iterator it = translations.begin(); it != translations.end(); ++it)
	{
		const Transform& tf = *it;
		std::cout << "  * " << t_idx << " of type";
		t_idx++;

		switch (tf.mType)
		{
		case TF_LOOKAT:
		{
			std::cout << " TF_LOOKAT: ";
			aiVector3D pos(tf.f[0], tf.f[1], tf.f[2]);
			aiVector3D dstPos(tf.f[3], tf.f[4], tf.f[5]);
			aiVector3D up = aiVector3D(tf.f[6], tf.f[7], tf.f[8]).Normalize();
			aiVector3D dir = aiVector3D(dstPos - pos).Normalize();
			aiVector3D right = (dir ^ up).Normalize();

			std::cout << " up = " << up.x << "," << up.y << "," << up.z << "; dir = " << dir.x << "," << dir.y << "," << dir.z << "; right = " << right.x << "," << right.y << "," << right.z;
			
			res *= aiMatrix4x4(
				right.x, up.x, -dir.x, pos.x,
				right.y, up.y, -dir.y, pos.y,
				right.z, up.z, -dir.z, pos.z,
				0, 0, 0, 1);

			break;
		}
		case TF_ROTATE:
		{
			std::cout << " TF_ROTATE: ";
			aiMatrix4x4 rot;
			float angle = tf.f[3] * float(AI_MATH_PI) / 180.0f;
			aiVector3D axis(tf.f[0], tf.f[1], tf.f[2]);
			aiMatrix4x4::Rotation(angle, axis, rot);
			res *= rot;
			
			std::cout << " axis = " << axis.x << "," << axis.y << "," << axis.z << "; angle in radians = " << angle << "; angle in degrees = " << tf.f[3];

			break;
		}
		case TF_TRANSLATE:
		{
			std::cout << " TF_TRANSLATE: ";
			aiMatrix4x4 trans;
			aiMatrix4x4::Translation(aiVector3D(tf.f[0], tf.f[1], tf.f[2]), trans);
			res *= trans;
			
			std::cout << " translate by " << tf.f[0] << "," << tf.f[1] << "," << tf.f[2] << " to " << res.a4 << "," << res.b4 << "," << res.c4;

			break;
		}
		case TF_SCALE:
		{
			std::cout << " TF_SCALE: ";
			aiMatrix4x4 scale(tf.f[0], 0.0f, 0.0f, 0.0f, 0.0f, tf.f[1], 0.0f, 0.0f, 0.0f, 0.0f, tf.f[2], 0.0f,
				0.0f, 0.0f, 0.0f, 1.0f);
			res *= scale;
			
			std::cout << " scale by " << tf.f[0] << "," << tf.f[1] << "," << tf.f[2];

			break;
		}
		case TF_SKEW:
			// TODO: (thom)
			std::cout << " TF_SKEW: NOT IMPLEMENTED";
			break;
		case TF_MATRIX:
		{
			std::cout << " TF_MATRIX: " << std::endl;
			aiMatrix4x4 mat(tf.f[0], tf.f[1], tf.f[2], tf.f[3], tf.f[4], tf.f[5], tf.f[6], tf.f[7],
				tf.f[8], tf.f[9], tf.f[10], tf.f[11], tf.f[12], tf.f[13], tf.f[14], tf.f[15]);

			res *= mat;

			std::cout << "      " << tf.f[0] << "," << tf.f[1] << "," << tf.f[2] << "," << tf.f[3] << std::endl;
			std::cout << "      " << tf.f[4] << "," << tf.f[5] << "," << tf.f[6] << "," << tf.f[7] << std::endl;
			std::cout << "      " << tf.f[8] << "," << tf.f[9] << "," << tf.f[10] << "," << tf.f[11] << std::endl;
			std::cout << "      " << tf.f[12] << "," << tf.f[13] << "," << tf.f[14] << "," << tf.f[15] << std::endl;

			break;
		}
		default:
			break;
		}
		std::cout << std::endl;
		std::cout << "    --> total translation = " << res.a4 << "," << res.b4 << "," << res.c4 << std::endl;
		aiQuaternion quat(aiMatrix3x3(res.a1, res.b1, res.c1,
									  res.a2, res.b2, res.c2,
									  res.a3, res.b3, res.c3));
		std::cout << "    --> total rotation    = " << quat.x << "," << quat.y << "," << quat.z << "," << quat.w << std::endl;
	}
#endif

    std::string jointN = "";
    Link l;
    l.sid = mReader->getAttributeValue("sid") ? mReader->getAttributeValue("sid") : GetUnnamed("auto_link_");
    l.name = mReader->getAttributeValue("name") ? mReader->getAttributeValue("name"): l.sid;
    l.preLink = preLink;
    l.joint = joint;
    l.id = joint.empty()? kinMod : joint;
    l.kinmodId = kinMod;
    l.kinmodName = kinModName;

    // Copy Translation
    aiQuaternion rotation;
    aiVector3D position;
    aiMatrix4x4 nextTrans = CalculateResultTransform(translations);
    nextTrans.DecomposeNoScaling(rotation, position);
    l.trans_x = position.x;
    l.trans_y = position.y;
    l.trans_z = position.z;
    l.quat_w = rotation.w;
    l.quat_x = rotation.x;
    l.quat_y = rotation.y;
    l.quat_z = rotation.z;

    if( mReader->isEmptyElement()) {
        if (mLinkLibrary.find(l.id) != mLinkLibrary.end() && !transformsOnly) {
            std::cerr << "ERROR: Dublicate Link ID: " << l.id << std::endl;
            return "";
        }
		
		if (!transformsOnly)
			mLinkLibrary[l.id] = l;
        
		return l.id;
    }

	int link_transforms = 0;
	int link_attachments = 0;
    while( mReader->read())
    {
        if( mReader->getNodeType() == irr::io::EXN_ELEMENT)	{

            if ( IsOpenElement( "translate", mReader)) {
				if (!transformsOnly)
					ReadNodeTransformation( translations, TF_TRANSLATE);
				else
					ReadLinkTransformation(translations, TF_TRANSLATE, l, relativeTransforms, false, mReader, attachmentIndex);
				
				link_transforms++;
			}
			else if (IsOpenElement("rotate", mReader)) {
				if (!transformsOnly)
					ReadNodeTransformation( translations, TF_ROTATE);
				else
					ReadLinkTransformation(translations, TF_ROTATE, l, relativeTransforms, false, mReader, attachmentIndex);
				
				link_transforms++;
			}
			else if (IsOpenElement("attachment_full", mReader)) {
                jointN = mReader->getAttributeValue("joint");
				ReadAttachment(l, kinMod, kinModName, jointN, translations, transformsOnly, (transformsOnly ? mReader : NULL), relativeTransforms, link_attachments++);

            } else	{
				SkipElement(mReader);
            }
        } else if( mReader->getNodeType() == irr::io::EXN_ELEMENT_END) {
			if (IsCloseElement("link", mReader)) break;
        }
    }
    if (mLinkLibrary.find(l.id) != mLinkLibrary.end()) 
	{
		if (!transformsOnly)
		{
			std::cerr << "ERROR: Dublicate Link ID: " << l.id << std::endl;
			return "";
		}
    }

	// Store relative transform from root to current link; probably redundant, but easier to work with if available in each link. 
	// Don't have to reconstruct the transform history from a steaming pile of indirection :)

	for (int k = 0; k < translations.size(); ++k)
		l.relativeAncestorTransforms.push_back(translations[k]);

	//std::cout << "After reading link " << l.id << ": transformations counts -- before = " << numCurrentTransformations << ", transformations now: " << translations.size() << " -- difference = " << translations.size() - numCurrentTransformations << std::endl;
	std::cout << "After reading link " << l.id << ": translate/rotate elements of link = " << link_transforms << std::endl;
	for (int k = translations.size() - 1; k >= 0; --k)
	{
		if (translations[k].mType == TF_TRANSLATE)
			std::cout << " translate: " << translations[k].f[0] << "," << translations[k].f[1] << "," << translations[k].f[2] << std::endl;
		else if (translations[k].mType == TF_ROTATE)
		{
			float angle = translations[k].f[3] * float(AI_MATH_PI) / 180.0f;
			std::cout << " rotate: " << angle << " radians/" << translations[k].f[3] << " deg. around axis " << translations[k].f[0] << "," << translations[k].f[1] << "," << translations[k].f[2] << std::endl;
		}
	}
	
	if (!transformsOnly)
		mLinkLibrary[l.id] = l;
    
	return l.id;
}


void ColladaParser::ReadKinematicsScenes() {

    if( mReader->isEmptyElement())
        return;

    while( mReader->read())
    {
        if( mReader->getNodeType() == irr::io::EXN_ELEMENT)	{

            if( IsOpenElement( "kinematics_scene")) {

                // Read Attr: id(opt), name(opt)
				InstanceKinematicsModel kmi_container;
                InstanceKinematicsModel kmi;
                kmi.id = mReader->getAttributeValue("id");
				kmi_container.id = kmi.id;

                while( mReader->read())
                {
                    if( mReader->getNodeType() == irr::io::EXN_ELEMENT)	{

                        if( IsOpenElement( "instance_kinematics_model")) {
                            // Read Attr: url sid(opt) name(opt)
                            std::string url = mReader->getAttributeValue("url");
                            kmi.url = url.substr(1);
                            kmi.sid = mReader->getAttributeValue("sid");

							kmi_container.url = kmi.url;
							kmi_container.sid = kmi.sid;

                            while( mReader->read())
                            {
                                if( mReader->getNodeType() == irr::io::EXN_ELEMENT)	{
                                    if( IsOpenElement( "newparam")) {
                                        // read attrib sid
                                        kmi.param = mReader->getAttributeValue("sid");
										
                                        while( mReader->read())
                                        {
                                            if( mReader->getNodeType() == irr::io::EXN_ELEMENT)	{
                                                if( IsOpenElement( "SIDREF")) {
                                                    kmi.ref = GetTextContent();

                                                    TestClosing("SIDREF");
                                                    mInstanceKinematicsModel[kmi.param] = kmi;
													kmi_container.jointInstances.push_back(kmi.ref);
                                                } else	{
                                                    SkipElement();
                                                }
                                            } else if( mReader->getNodeType() == irr::io::EXN_ELEMENT_END){
                                                if (IsCloseElement("newparam")) break;
                                            }
                                        }

                                    } else if( IsOpenElement( "extra"))  {

                                        while( mReader->read())
                                        {
                                            if( mReader->getNodeType() == irr::io::EXN_ELEMENT)	{
                                                if( IsOpenElement( "technique")) {
                                                    std::string profile = mReader->getAttributeValue("profile");
                                                    if (profile.compare("Zyklio") == 0) {

                                                        while( mReader->read())
                                                        {
                                                            if( mReader->getNodeType() == irr::io::EXN_ELEMENT)	{
                                                                if( IsOpenElement( "robot")) {
                                                                    kmi.robotModel.isRobot = true;
                                                                    kmi.robotModel.name = mReader->getAttributeValue("name")?mReader->getAttributeValue("name") : "Unnamed Robot";
                                                                    kmi.robotModel.control = mReader->getAttributeValue("control")?mReader->getAttributeValue("control"): "";
                                                                    kmi.robotModel.control_robot_config = mReader->getAttributeValue("control_robot_config")?mReader->getAttributeValue("control_robot_config"):"";
                                                                    kmi.robotModel.control_host = mReader->getAttributeValue("control_host")?mReader->getAttributeValue("control_host"):"";

                                                                } else	{
                                                                    SkipElement();
                                                                }
                                                            } else if( mReader->getNodeType() == irr::io::EXN_ELEMENT_END){
                                                                if (IsCloseElement("technique")) break;
                                                            }
                                                        }

                                                    } else {
                                                        SkipElement();
                                                    }

                                                } else	{
                                                    SkipElement();
                                                }
                                            } else if( mReader->getNodeType() == irr::io::EXN_ELEMENT_END){
                                                if (IsCloseElement("extra")) break;
                                            }
                                        }

                                    } else	{
                                        SkipElement();
                                    }
                                } else if( mReader->getNodeType() == irr::io::EXN_ELEMENT_END){
                                    if (IsCloseElement("instance_kinematics_model")) break;
                                }
                            }

                        } else	{
                            SkipElement();
                        }
                    } else if( mReader->getNodeType() == irr::io::EXN_ELEMENT_END){
                        if (IsCloseElement("kinematics_scene")) break;
                    }
                }

				mInstanceKinematicsModelContainers[kmi_container.id] = kmi_container;

            } else	{
                SkipElement();
            }
        } else if( mReader->getNodeType() == irr::io::EXN_ELEMENT_END){
            if (IsCloseElement("library_kinematics_scenes")) break;
        }
    }
}


std::string ColladaParser::ReadParam(std::string closeelement) {

    std::string retval;
    while( mReader->read())
    {
        if( mReader->getNodeType() == irr::io::EXN_ELEMENT)	{
            if( IsOpenElement( "param")) {
                retval = GetTextContent();
                TestClosing("param");
            } else	{
                SkipElement();
            }
        } else if( mReader->getNodeType() == irr::io::EXN_ELEMENT_END){
            if (IsCloseElement(closeelement.c_str())) break;
        }
    }
    return retval;
}

void ColladaParser::ReadInstanceKinematicsScene() {

    if( mReader->isEmptyElement())
        return;

    InstanceKinematicsScene ikm;
    std::string url = mReader->getAttributeValue("url");
    ikm.url = url.substr(1);


    while( mReader->read())
    {
        if( mReader->getNodeType() == irr::io::EXN_ELEMENT)	{
            if( IsOpenElement( "bind_kinematics_model")) {
                ikm.node = mReader->getAttributeValue("node");

                ikm.param = ReadParam("bind_kinematics_model");

				ikm.bind_kinematics_model = ikm.param;

                mInstanceKinematicsScene.push_back(ikm);

            } else if( IsOpenElement( "bind_joint_axis")) {
                std::string node = mReader->getAttributeValue("target");
                unsigned int l = node.find_first_of('/');
                if (l< node.size()) node = node.substr(0,l);
                ikm.node = node;
                while( mReader->read())
                {
                    if( mReader->getNodeType() == irr::io::EXN_ELEMENT)	{
                        if( IsOpenElement( "axis")) {
                            // Create Joint Axes for last
                            ikm.param = ReadParam("axis");
                        } else	if (IsOpenElement("value")){
                            ikm.axisValue = ReadParam("value");
                        }
                    } else if( mReader->getNodeType() == irr::io::EXN_ELEMENT_END){
                        if (IsCloseElement("bind_joint_axis")) break;
                    }
                }
                mInstanceKinematicsScene.push_back(ikm);

            } else	{
                SkipElement();
            }
        } else if( mReader->getNodeType() == irr::io::EXN_ELEMENT_END){
            if (IsCloseElement("instance_kinematics_scene")) break;
        }
    }

}

void ColladaParser::ReadRotation(float & out_x, float & out_y, float & out_z) {
    const char* content = GetTextContent();
    float x,y,z = 0;
    float angle = 0;

    content = fast_atoreal_move<float>( content, (float&)x);
    SkipSpacesAndLineEnd( &content);

    content = fast_atoreal_move<float>( content, (float&)y);
    SkipSpacesAndLineEnd( &content);

    content = fast_atoreal_move<float>( content, (float&)z);
    SkipSpacesAndLineEnd( &content);

    content = fast_atoreal_move<float>( content, (float&)angle);
    SkipSpacesAndLineEnd( &content);

    if (x) {
        out_x = angle;
    } else if (y) {
        out_y = angle;
    } else if (z) {
        out_z = angle;
    }
    TestClosing("rotate");

}


void ColladaParser::ReadTranslation(float & out_x, float & out_y, float & out_z) {
    const char* content = GetTextContent();

    content = fast_atoreal_move<float>( content, (float&)out_x);
    SkipSpacesAndLineEnd( &content);

    content = fast_atoreal_move<float>( content, (float&)out_y);
    SkipSpacesAndLineEnd( &content);

    content = fast_atoreal_move<float>( content, (float&)out_z);
    SkipSpacesAndLineEnd( &content);

    TestClosing("translate");
}

void ColladaParser::ReadShape(RigidBody &p) {
    while( mReader->read())
    {

        if( mReader->getNodeType() == irr::io::EXN_ELEMENT)	{

            if( IsOpenElement( "instance_geometry")) {

                // read attribute: url
                int attrID = GetAttribute( "url");
                if (attrID > -1) {
                    std::string url = mReader->getAttributeValue( attrID);
                    p.meshId = url.substr(1);
                }
            } else if( IsOpenElement( "rotate")) {
                ReadNodeTransformation( p.mTransforms, TF_ROTATE);
            } else if( IsOpenElement( "translate")) {
                ReadNodeTransformation( p.mTransforms, TF_TRANSLATE);
            } else	{
                SkipElement();
            }
        } else if( mReader->getNodeType() == irr::io::EXN_ELEMENT_END){
            if (IsCloseElement("shape")) break;
        }
    }
}

void ColladaParser::ReadPhysicsModelsLibrary() {
    DBG("Read PhysicsModelsLibrary\n");
    if( mReader->isEmptyElement())
        return;

    while( mReader->read())
    {
        if( mReader->getNodeType() == irr::io::EXN_ELEMENT)	{
            if( IsOpenElement( "physics_model")) {
                // could read attributes: id, name

                while( mReader->read())
                {
                    if( mReader->getNodeType() == irr::io::EXN_ELEMENT)	{
                        if( IsOpenElement( "rigid_body")) {
                            RigidBody p;
                            p.mSid = mReader->getAttributeValue("sid")?mReader->getAttributeValue("sid"):"";

                            while( mReader->read())
                            {
                                if( mReader->getNodeType() == irr::io::EXN_ELEMENT)	{
                                    if( IsOpenElement( "technique_common")) {

                                        while( mReader->read())
                                        {
                                            if( mReader->getNodeType() == irr::io::EXN_ELEMENT)	{
                                                if( IsOpenElement( "dynamic")) {
                                                    const char* content = GetTextContent();
                                                    p.dynamic = (strcmp(content, "true") == 0);

                                                    TestClosing("dynamic");
                                                } else if( IsOpenElement( "mass")) {
                                                    const char* content = GetTextContent();

                                                    content = fast_atoreal_move<float>( content, (float&)p.mass);
                                                    SkipSpacesAndLineEnd( &content);

                                                    TestClosing("mass");
                                                } else if( IsOpenElement( "mass_frame")) {

                                                    while( mReader->read())
                                                    {
                                                        if( mReader->getNodeType() == irr::io::EXN_ELEMENT)	{
                                                            if( IsOpenElement( "translate")) {
                                                                ReadTranslation(p.mMassFrame_x, p.mMassFrame_y, p.mMassFrame_z);
                                                            } else {
                                                                SkipElement();
                                                            }
                                                        } else if( mReader->getNodeType() == irr::io::EXN_ELEMENT_END){
                                                            if (IsCloseElement("mass_frame")) break;
                                                        }
                                                    }

                                                } else if( IsOpenElement( "shape")) {
                                                    if (p.meshId == "") {
                                                        // Read the main Shape
                                                        ReadShape(p);
                                                        std::cout << "physics model has shape with meshId: " << p.meshId  << std::endl;
                                                    } else {
                                                        // Read sub-shapes.
                                                        RigidBody subP;
                                                        ReadShape(subP);
                                                        p.subShapes.push_back(subP);
                                                    }
                                                    // tst Anfang
                                                    // tst Ende
                                                } else {
                                                    SkipElement();
                                                }
                                            } else if( mReader->getNodeType() == irr::io::EXN_ELEMENT_END){
                                                if (IsCloseElement("technique_common")) break;
                                            }
                                        }

                                    } else if(IsOpenElement("technique")) {
                                        std::string profile = mReader->getAttributeValue("profile");
                                        if (profile.compare("Zyklio") == 0) {

                                            while( mReader->read())
                                            {
                                                if( mReader->getNodeType() == irr::io::EXN_ELEMENT)	{
                                                    if( IsOpenElement( "tool")) {
                                                        p.mTool->isTool = true;
                                                        p.mTool->mName = mReader->getAttributeValue("name")?mReader->getAttributeValue("name") : "Unnamed Tool";
                                                        p.mTool->mDescription = mReader->getAttributeValue("description")?mReader->getAttributeValue("description") : "No description";
                                                        p.mTool->mId = mReader->getAttributeValue("id")?mReader->getAttributeValue("id") : "unnamed_tool";


                                                        while( mReader->read())
                                                        {
                                                            if( mReader->getNodeType() == irr::io::EXN_ELEMENT)	{
                                                                if( IsOpenElement( "rotate")) {
                                                                    ReadNodeTransformation( p.mTool->mTransforms, TF_ROTATE);
                                                                } else if( IsOpenElement( "translate")) {
                                                                    ReadNodeTransformation( p.mTool->mTransforms, TF_TRANSLATE);
                                                                } else {
                                                                    SkipElement();
                                                                }
                                                            } else if( mReader->getNodeType() == irr::io::EXN_ELEMENT_END){
                                                                if (IsCloseElement("tool")) break;
                                                            }
                                                        }

                                                    } else if( IsOpenElement( "collides_with")) {
                                                        ReadStringsFromTextContent(p.mWhitelist);
                                                        TestClosing("collides_with");

                                                    // TP anfang
                                                    } else if (IsOpenElement("visual_model")) {
                                                        std::cout << "Reading visual model" << std::endl;
                                                        const char* content = GetTextContent();
                                                        char* n = " \f\n\r\t";
                                                        std::string str(content);
                                                        str = str.substr(0,str.find_first_of(n));
                                                        if (!str.empty())
                                                        {
                                                            std::cout << "found: '" << str << "'" << std::endl;
                                                            p.mVisualModel = str;
                                                        }
                                                        //ReadStringsFromTextContent(p.mvisualmodel);
                                                        TestClosing("visual_model");
                                                    // TP ende

                                                    } else if( IsOpenElement( "ghost" )){
                                                        if (mReader->getAttributeValue("id")) {
                                                            Ghost ghost;
                                                            ghost.mId = mReader->getAttributeValue("id");
                                                            ghost.mTolerance = mReader->getAttributeValueAsFloat("tolerance")?mReader->getAttributeValueAsFloat("tolerance"):0.05;
                                                            p.mGhosts.push_back(ghost);
                                                        }
                                                    } else {
                                                        SkipElement();
                                                    }
                                                } else if( mReader->getNodeType() == irr::io::EXN_ELEMENT_END){
                                                    if (IsCloseElement("technique")) break;
                                                }
                                            }

                                        } else {
                                            SkipElement();
                                        }

                                    } else	{
                                        SkipElement();
                                    }
                                } else if( mReader->getNodeType() == irr::io::EXN_ELEMENT_END){
                                    if (IsCloseElement("rigid_body")) break;
                                }
                            }

                            mPhysicsModelLibrary[p.meshId] = p;
                        } else if (IsOpenElement("rigid_constraint")) {
                            RigidBodyConstraint c;
                            c.mSid = mReader->getAttributeValue("sid") ? mReader->getAttributeValue("sid") : GetUnnamed("rigid_body_constraint_");
                            c.mName = mReader->getAttributeValue("name") ? mReader->getAttributeValue("name") : c.mSid;

                            while( mReader->read())
                            {
                                if( mReader->getNodeType() == irr::io::EXN_ELEMENT)	{
                                    if( IsOpenElement( "ref_attachment")) {
                                        c.mRigidBody1 = mReader->getAttributeValue("rigid_body");

                                        while( mReader->read())
                                        {
                                            if( mReader->getNodeType() == irr::io::EXN_ELEMENT)	{
                                                if( IsOpenElement( "rotate")) {
                                                    ReadNodeTransformation( c.mRB1Transforms, TF_ROTATE);
                                                } else if( IsOpenElement( "translate")) {
                                                    ReadNodeTransformation(  c.mRB1Transforms, TF_TRANSLATE);
                                                } else if ( IsOpenElement( "extra")) {
                                                    SkipElement();
                                                } else	{
                                                    SkipElement();
                                                }
                                            } else if( mReader->getNodeType() == irr::io::EXN_ELEMENT_END){
                                                if (IsCloseElement("ref_attachment")) break;
                                            }
                                        }

                                    } else if( IsOpenElement( "attachment")) {
                                        c.mRigidBody2 = mReader->getAttributeValue("rigid_body");

                                        while( mReader->read())
                                        {
                                            if( mReader->getNodeType() == irr::io::EXN_ELEMENT)	{
                                                if( IsOpenElement( "rotate")) {
                                                    ReadNodeTransformation( c.mRB2Transforms, TF_ROTATE);
                                                } else if( IsOpenElement( "translate")) {
                                                    ReadNodeTransformation(  c.mRB2Transforms, TF_TRANSLATE);
                                                } else if ( IsOpenElement( "extra")) {
                                                    SkipElement();
                                                } else	{
                                                    SkipElement();
                                                }
                                            } else if( mReader->getNodeType() == irr::io::EXN_ELEMENT_END){
                                                if (IsCloseElement("attachment")) break;
                                            }
                                        }

                                    } else if (IsOpenElement("technique_common")) {

                                        while( mReader->read())
                                        {
                                            if( mReader->getNodeType() == irr::io::EXN_ELEMENT)	{
                                                if( IsOpenElement( "interpenetrate")) {
                                                    c.interpenetrate = ReadBoolFromTextContent();
                                                    TestClosing("interpenetrate");

                                                } else if( IsOpenElement( "limits")) {

                                                    while( mReader->read())
                                                    {
                                                        if( mReader->getNodeType() == irr::io::EXN_ELEMENT)	{
                                                            if( IsOpenElement( "linear")) {
                                                                c.mType = RigidBodyConstraint::LINEAR;

                                                                while( mReader->read())
                                                                {
                                                                    if( mReader->getNodeType() == irr::io::EXN_ELEMENT)	{
                                                                        if( IsOpenElement( "min")) {
                                                                            ReadFloatsFromTextContent(c.min_x, c.min_y, c.min_z);
                                                                            TestClosing("min");
                                                                        } else if( IsOpenElement( "max")) {
                                                                            ReadFloatsFromTextContent(c.max_x, c.max_y, c.max_z);
                                                                            TestClosing("max");
                                                                        } else {
                                                                            SkipElement();
                                                                        }
                                                                    } else if( mReader->getNodeType() == irr::io::EXN_ELEMENT_END){
                                                                        if (IsCloseElement("linear")) break;
                                                                    }
                                                                }

                                                            } else if (IsOpenElement("swing_cone_and_twist")) {
                                                                c.mType = RigidBodyConstraint::SWING_CONE_AND_TWIST;

                                                                while( mReader->read())
                                                                {
                                                                    if( mReader->getNodeType() == irr::io::EXN_ELEMENT)	{
                                                                        if( IsOpenElement( "min")) {
                                                                            ReadFloatsFromTextContent(c.min_x, c.min_y, c.min_z);
                                                                            TestClosing("min");
                                                                        } else if( IsOpenElement( "max")) {
                                                                            ReadFloatsFromTextContent(c.max_x, c.max_y, c.max_z);
                                                                            TestClosing("max");
                                                                        } else {
                                                                            SkipElement();
                                                                        }
                                                                    } else if( mReader->getNodeType() == irr::io::EXN_ELEMENT_END){
                                                                        if (IsCloseElement("swing_cone_and_twist")) break;
                                                                    }
                                                                }

                                                            } else {
                                                                SkipElement();
                                                            }
                                                        } else if( mReader->getNodeType() == irr::io::EXN_ELEMENT_END){
                                                            if (IsCloseElement("limits")) break;
                                                        }
                                                    }

                                                } else	{
                                                    SkipElement();
                                                }
                                            } else if( mReader->getNodeType() == irr::io::EXN_ELEMENT_END){
                                                if (IsCloseElement("technique_common")) break;
                                            }
                                        }

                                    } else if (IsOpenElement("technique")) {
                                        std::string profile = mReader->getAttributeValue("profile");
                                        if (profile.compare("Zyklio") == 0) {

                                            while( mReader->read())
                                            {
                                                if( mReader->getNodeType() == irr::io::EXN_ELEMENT)	{
                                                    if( IsOpenElement( "actuator")) {

                                                        c.pos_0 = mReader->getAttributeValueAsFloat("pos0");
                                                        c.pos_1 = mReader->getAttributeValueAsFloat("pos1");
                                                        c.delta_t = mReader->getAttributeValueAsFloat("delta_t");
                                                        c.binaryIO = mReader->getAttributeValueAsInt("binaryIO");
                                                        c.force_sensitive = (mReader->getAttributeValueAsInt("force_sensitive") == 1);

                                                    } else	{
                                                        SkipElement();
                                                    }
                                                } else if( mReader->getNodeType() == irr::io::EXN_ELEMENT_END){
                                                    if (IsCloseElement("technique")) break;
                                                }
                                            }

                                        }
                                    } else	{
                                        SkipElement();
                                    }
                                } else if( mReader->getNodeType() == irr::io::EXN_ELEMENT_END){
                                    if (IsCloseElement("rigid_constraint"))  {
                                        mRigidBodyConstraintLibrary[c.mSid] = c;
                                        break;
                                    }
                                }
                            }


                        } else {
                            SkipElement();
                        }
                    } else if( mReader->getNodeType() == irr::io::EXN_ELEMENT_END){
                        if (IsCloseElement("physics_model")) break;
                    }
                }

            } else	{
                SkipElement();
            }
        } else if( mReader->getNodeType() == irr::io::EXN_ELEMENT_END){
            if (IsCloseElement("library_physics_models")) break;
        }
    }
    DBG("DONE reading PhysicsModelsLibrary\n");
}

void ColladaParser::ReadJointsLibrary()
{

    DBG("Read Joints\n");
    if( mReader->isEmptyElement())
        return;

    while( mReader->read())
    {
        if( mReader->getNodeType() == irr::io::EXN_ELEMENT)	{
            if( IsOpenElement( "joint"))
            {
                Joint j;
                j.id = mReader->getAttributeValue( "id");
                j.name = mReader->getAttributeValue( "name");

                // read the rest of the element
                while (mReader->read()) {
                    if( mReader->getNodeType() == irr::io::EXN_ELEMENT)	{
                        if( IsOpenElement( "revolute") || IsOpenElement("prismatic")) {
                            if (IsOpenElement("revolute")) {
                                j.type = Joint::REVOLUTE;
                            } else {
                                j.type = Joint::PRISMATIC;
                            }
                            j.sid = mReader->getAttributeValue("sid");


                            while (mReader->read()) {
                                if( mReader->getNodeType() == irr::io::EXN_ELEMENT)	{
                                    if (IsOpenElement( "axis")) {
                                        const char* content = GetTextContent();

                                        content = fast_atoreal_move<float>( content, (float&)j.axis_x);
                                        SkipSpacesAndLineEnd( &content);

                                        content = fast_atoreal_move<float>( content, (float&)j.axis_y);
                                        SkipSpacesAndLineEnd( &content);

                                        content = fast_atoreal_move<float>( content, (float&)j.axis_z);
                                        SkipSpacesAndLineEnd( &content);

                                        TestClosing("axis");

                                    } else if (IsOpenElement ("limits")) {
                                        while (mReader->read()) {
                                            if( mReader->getNodeType() == irr::io::EXN_ELEMENT)	{
                                                if (IsOpenElement( "min")) {
                                                    const char* content = GetTextContent();
                                                    fast_atoreal_move<float>( content, (float&)j.min);
                                                    TestClosing("min");
                                                } else if (IsOpenElement( "max")) {
                                                    const char* content = GetTextContent();
                                                    fast_atoreal_move<float>( content, (float&)j.max);
                                                    TestClosing("max");
                                                }
                                            } else if( mReader->getNodeType() == irr::io::EXN_ELEMENT_END) {
                                                if (IsCloseElement("limits")) break;
                                            }
                                        }
                                    }
                                }
                                else if( mReader->getNodeType() == irr::io::EXN_ELEMENT_END){
                                    if (j.type == Joint::REVOLUTE && IsCloseElement("revolute")) break;
                                    if (j.type == Joint::PRISMATIC && IsCloseElement("prismatic")) break;
                                }
                            }
                            mJointLibrary[j.id] = j;
                        }
                        else SkipElement();
                    } else if( mReader->getNodeType() == irr::io::EXN_ELEMENT_END){
                        if (IsCloseElement("joint")) break;
                    }
                }
            }
        } else if( mReader->getNodeType() == irr::io::EXN_ELEMENT_END){
            if (IsCloseElement("library_joints")) break;
        }
    }
    DBG("Done reading Joints. Read: " << mJointLibrary.size() << std::endl);
}


void ColladaParser::showJointLib() {
    std::cout << "============================================================================\n";
    std::cout << "Geometry Library\n";
    std::cout << "Number of Geometries: " << mMeshLibrary.size() << std::endl;
    for (MeshLibrary::iterator it = mMeshLibrary.begin(); it != mMeshLibrary.end(); ++it) {
        Mesh *m = it->second;
        std::cout << "    Mesh       id: " << m->mId  << std::endl;
    }
    std::cout << "Number of Nodes: " << mNodeLibrary.size() << std::endl;
    for (NodeLibrary::iterator it = mNodeLibrary.begin(); it != mNodeLibrary.end(); ++it) {
        Node *m = it->second;
        std::cout << "    Node       id: " << m->mID  << "  children: " << m->mChildren.size() << std::endl;
        for (int i = 0; i < m->mChildren.size(); i++) {
            Node * n = m->mChildren.at(i);
            std::cout << "    ChildNode  id: " << n->mID  << "  children: " << m->mChildren.size() << std::endl;
        }
    }


    std::cout << "============================================================================\n";
    std::cout << "Joints Library\n";

    std::cout << "\nNumber of Models: " << mKinematicsModelLibrary.size() << std::endl;
    for (KinematicsModelLibrary::iterator it = mKinematicsModelLibrary.begin(); it != mKinematicsModelLibrary.end(); ++it) {
        KinematicsModel &m = it->second;
        std::cout << "    KinModel    id: " << m.id << "  name: " << m.name << std::endl;
    }

    std::cout << "\nNumber of Joints: " << mJointLibrary.size() << std::endl;
    for (JointLibrary::iterator it = mJointLibrary.begin(); it != mJointLibrary.end(); ++it) {
        Joint &j = it->second;
        std::cout << "    Joint       id: " << j.id << "  name: " << j.name <<  "  sid:" << j.sid << std::endl;
        std::cout << "              axis: " << j.axis_x << "/" << j.axis_y << "/" << j.axis_z << std::endl;
        std::cout << "           min/max: " << j.min << "/" << j.max << std::endl;
    }

    std::cout << "\nNumber of JointInstances: " << mInstanceJointLibrary.size() << std::endl;
    for (InstanceJointLibrary::iterator it = mInstanceJointLibrary.begin(); it != mInstanceJointLibrary.end(); ++it) {
        InstanceJoint &j = it->second;
        std::cout << "    InstJoint  sid: " << j.sid << "  url: " << j.url  << "  name: " << j.kinmodName << std::endl;
    }

    std::cout << "\nNumber of Links: " << mLinkLibrary.size() << std::endl;
    for (LinkLibrary::iterator it = mLinkLibrary.begin(); it != mLinkLibrary.end(); ++it) {
        Link &l = it->second;
        std::cout << "    Link        id: " << l.id << "  sid: " << l.sid << "  name: " << l.name  << "  joint: " << l.joint << std::endl;
        std::cout << "       translation: " << l.trans_x << "/" << l.trans_y << "/" << l.trans_z << std::endl;
        std::cout << "           preLink: " << l.preLink << "  postLinks: ";
        for (unsigned int i = 0; i < l.postLinks.size(); i++) std::cout << l.postLinks.at(i) << ", ";
        std::cout << std::endl;
    }


    std::cout << "\nPhysicsModels: " << mPhysicsModelLibrary.size() << std::endl;
    for (PhysicsModelLibrary::iterator it = mPhysicsModelLibrary.begin(); it != mPhysicsModelLibrary.end(); ++it) {
        RigidBody &l = it->second;
        std::cout << "      PhysicsModel: " << l.meshId << "  mass: " << l.mass  << "  dynamic: " << l.dynamic << std::endl;
        std::cout << "         MassFrame: " << l.mMassFrame_x << "/" << l.mMassFrame_y << "/" << l.mMassFrame_z << std::endl;
        std::cout << "\n   SubModels: " << l.subShapes.size() << std::endl;
    }

    std::cout << "\nKinematicsModelInstances: " << mInstanceKinematicsModel.size() << std::endl;
    for (InstanceKinematicsModelLibrary::iterator it = mInstanceKinematicsModel.begin(); it != mInstanceKinematicsModel.end(); ++it) {
        InstanceKinematicsModel &k = it->second;
        std::cout << "      KinematicsParam: " << k.param << std::endl;
        std::cout << "        KinematicsREF: " << k.ref   << std::endl;
    }

    std::cout << "\nRigidBodyConstraints: " << mRigidBodyConstraintLibrary.size() << std::endl;
    for (RigidBodyConstraintLibrary::iterator it = mRigidBodyConstraintLibrary.begin(); it != mRigidBodyConstraintLibrary.end(); ++it) {
        RigidBodyConstraint &c = it->second;
        std::cout << "      RigidBodyRef: " << c.mRigidBody1 << "   Body2: "<< c.mRigidBody2 << std::endl;
        std::cout << "     type, min/max: " << c.mType << " (" << c.min_x << " " << c.min_y << " " << c.min_z<< " / " << c.max_x << " " << c.max_y << " " << c.max_z << ")"  << std::endl;
    }
    std::cout << "============================================================================\n";

}



// ------------------------------------------------------------------------------------------------
// Aborts the file reading with an exception
void ColladaParser::ThrowException( const std::string& pError) const
{
    throw DeadlyImportError( boost::str( boost::format( "Collada: %s - %s") % mFileName % pError));
}

// ------------------------------------------------------------------------------------------------
// Skips all data until the end node of the current element
void ColladaParser::SkipElement(irr::io::IrrXMLReader* reader /* = NULL */)
{

    if (reader == NULL) {
        reader = mReader;
    }
    // nothing to skip if it's an <element />
    if (reader->isEmptyElement())
        return;

    // reroute
    std::string elementName = reader->getNodeName();
    std::cout << "Skipping Element: " << elementName << std::endl;

    SkipElement(reader->getNodeName(), reader);
}

// ------------------------------------------------------------------------------------------------
// Skips all data until the end node of the given element
void ColladaParser::SkipElement(const char* pElement, irr::io::IrrXMLReader* reader  /* = NULL */)
{
    if (reader == NULL) {
        reader = mReader;
    }

    // copy the current node's name because it'a pointer to the reader's internal buffer,
    // which is going to change with the upcoming parsing
    std::string element = pElement;
    while (reader->read())
    {
        if (reader->getNodeType() == irr::io::EXN_ELEMENT_END)
            if (reader->getNodeName() == element)
                break;
    }
}

// ------------------------------------------------------------------------------------------------
// Tests for an opening element of the given name, throws an exception if not found
void ColladaParser::TestOpening(const char* pName, irr::io::IrrXMLReader* reader)
{
	irr::io::IrrXMLReader* mReader = this->mReader;
	if (reader != NULL)
		mReader = reader;

    // read element start
    if( !mReader->read())
        ThrowException( boost::str( boost::format( "Unexpected end of file while beginning of \"%s\" element.") % pName));
    // whitespace in front is ok, just read again if found
    if( mReader->getNodeType() == irr::io::EXN_TEXT)
        if( !mReader->read())
            ThrowException( boost::str( boost::format( "Unexpected end of file while reading beginning of \"%s\" element.") % pName));

    if( mReader->getNodeType() != irr::io::EXN_ELEMENT || strcmp( mReader->getNodeName(), pName) != 0)
        ThrowException( boost::str( boost::format( "Expected start of \"%s\" element.") % pName));
}

// ------------------------------------------------------------------------------------------------
// Tests for the closing tag of the given element, throws an exception if not found
void ColladaParser::TestClosing(const char* pName, irr::io::IrrXMLReader* reader)
{
	irr::io::IrrXMLReader* mReader = this->mReader;
	if (reader != NULL)
		mReader = reader;

    // check if we're already on the closing tag and return right away
    if( mReader->getNodeType() == irr::io::EXN_ELEMENT_END && strcmp( mReader->getNodeName(), pName) == 0)
        return;

    // if not, read some more
    if( !mReader->read())
        ThrowException( boost::str( boost::format( "Unexpected end of file while reading end of \"%s\" element.") % pName));
    // whitespace in front is ok, just read again if found
    if( mReader->getNodeType() == irr::io::EXN_TEXT)
        if( !mReader->read())
            ThrowException( boost::str( boost::format( "Unexpected end of file while reading end of \"%s\" element.") % pName));

    // but this has the be the closing tag, or we're lost
    if( mReader->getNodeType() != irr::io::EXN_ELEMENT_END || strcmp( mReader->getNodeName(), pName) != 0)
        ThrowException( boost::str( boost::format( "Expected end of \"%s\" element.") % pName));
}

// ------------------------------------------------------------------------------------------------
// Returns the index of the named attribute or -1 if not found. Does not throw, therefore useful for optional attributes
int ColladaParser::GetAttribute(const char* pAttr, irr::io::IrrXMLReader* reader) const
{
	irr::io::IrrXMLReader* mReader = this->mReader;
	if (reader != NULL)
		mReader = reader;

    int index = TestAttribute( pAttr, reader);
    if( index != -1)
        return index;

    // attribute not found -> throw an exception
    std::string error = boost::str(boost::format("Expected attribute \"%s\" at element \"%s\".") % pAttr % mReader->getNodeName());
    std::cerr << error << std::endl;
    ThrowException(error);
    return -1;
}

// ------------------------------------------------------------------------------------------------
// Tests the present element for the presence of one attribute, returns its index or throws an exception if not found
int ColladaParser::TestAttribute(const char* pAttr, irr::io::IrrXMLReader* reader) const
{
	irr::io::IrrXMLReader* mReader = this->mReader;
	if (reader != NULL)
		mReader = reader;

    for( int a = 0; a < mReader->getAttributeCount(); a++)
        if( strcmp( mReader->getAttributeName( a), pAttr) == 0)
            return a;

    return -1;
}

// ------------------------------------------------------------------------------------------------
// Reads the text contents of an element, throws an exception if not given. Skips leading whitespace.
const char* ColladaParser::GetTextContent(irr::io::IrrXMLReader* reader)
{
    const char* sz = TestTextContent(reader);
    if(!sz) {
        ThrowException( "Invalid contents in element \"n\".");
    }
    return sz;
}

// ------------------------------------------------------------------------------------------------
// Reads the text contents of an element, returns NULL if not given. Skips leading whitespace.
const char* ColladaParser::TestTextContent(irr::io::IrrXMLReader* reader)
{
	irr::io::IrrXMLReader* mReader = this->mReader;
	if (reader != NULL)
		mReader = reader;

    // present node should be the beginning of an element
    if( mReader->getNodeType() != irr::io::EXN_ELEMENT || mReader->isEmptyElement())
        return NULL;

    // read contents of the element
    if( !mReader->read() )
        return NULL;
    if( mReader->getNodeType() != irr::io::EXN_TEXT)
        return NULL;

    // skip leading whitespace
    const char* text = mReader->getNodeData();
    SkipSpacesAndLineEnd( &text);

    return text;
}

// ------------------------------------------------------------------------------------------------
// Calculates the resulting transformation fromm all the given transform steps
aiMatrix4x4 ColladaParser::CalculateResultTransform( const std::vector<Transform>& pTransforms) const
{
    aiMatrix4x4 res;

    for( std::vector<Transform>::const_iterator it = pTransforms.begin(); it != pTransforms.end(); ++it)
    {
        const Transform& tf = *it;
        switch( tf.mType)
        {
        case TF_LOOKAT:
        {
            aiVector3D pos( tf.f[0], tf.f[1], tf.f[2]);
            aiVector3D dstPos( tf.f[3], tf.f[4], tf.f[5]);
            aiVector3D up = aiVector3D( tf.f[6], tf.f[7], tf.f[8]).Normalize();
            aiVector3D dir = aiVector3D( dstPos - pos).Normalize();
            aiVector3D right = (dir ^ up).Normalize();

            res *= aiMatrix4x4(
                        right.x, up.x, -dir.x, pos.x,
                        right.y, up.y, -dir.y, pos.y,
                        right.z, up.z, -dir.z, pos.z,
                        0, 0, 0, 1);
            break;
        }
        case TF_ROTATE:
        {
            aiMatrix4x4 rot;
            float angle = tf.f[3] * float( AI_MATH_PI) / 180.0f;
            aiVector3D axis( tf.f[0], tf.f[1], tf.f[2]);
            aiMatrix4x4::Rotation( angle, axis, rot);
            res *= rot;
            break;
        }
        case TF_TRANSLATE:
        {
            aiMatrix4x4 trans;
            aiMatrix4x4::Translation( aiVector3D( tf.f[0], tf.f[1], tf.f[2]), trans);
            res *= trans;
            break;
        }
        case TF_SCALE:
        {
            aiMatrix4x4 scale( tf.f[0], 0.0f, 0.0f, 0.0f, 0.0f, tf.f[1], 0.0f, 0.0f, 0.0f, 0.0f, tf.f[2], 0.0f,
                    0.0f, 0.0f, 0.0f, 1.0f);
            res *= scale;
            break;
        }
        case TF_SKEW:
            // TODO: (thom)
            ai_assert( false);
            break;
        case TF_MATRIX:
        {
            aiMatrix4x4 mat( tf.f[0], tf.f[1], tf.f[2], tf.f[3], tf.f[4], tf.f[5], tf.f[6], tf.f[7],
                    tf.f[8], tf.f[9], tf.f[10], tf.f[11], tf.f[12], tf.f[13], tf.f[14], tf.f[15]);
            res *= mat;
            break;
        }
        default:
            ai_assert( false);
            break;
        }
    }

    return res;
}

// ------------------------------------------------------------------------------------------------
// Determines the input data type for the given semantic string
Collada::InputType ColladaParser::GetTypeForSemantic( const std::string& pSemantic)
{
    if( pSemantic == "POSITION")
        return IT_Position;
    else if( pSemantic == "TEXCOORD")
        return IT_Texcoord;
    else if( pSemantic == "NORMAL")
        return IT_Normal;
    else if( pSemantic == "COLOR")
        return IT_Color;
    else if( pSemantic == "VERTEX")
        return IT_Vertex;
    else if( pSemantic == "BINORMAL" || pSemantic ==  "TEXBINORMAL")
        return IT_Bitangent;
    else if( pSemantic == "TANGENT" || pSemantic == "TEXTANGENT")
        return IT_Tangent;

    DefaultLogger::get()->warn( boost::str( boost::format( "Unknown vertex input type \"%s\". Ignoring.") % pSemantic));
    return IT_Invalid;
}

void ColladaParser::ReadRelativeTransforms(IOSystem* pIOHandler, const std::string& pFile)
{
	// open the file
	boost::scoped_ptr<IOStream> file(pIOHandler->Open(pFile));
	if (file.get() != NULL)
	{
		// generate a XML reader for it
		boost::scoped_ptr<CIrrXML_IOStreamReader> mIOWrapper(new CIrrXML_IOStreamReader(file.get()));
		irr::io::IrrXMLReader* mReader = irr::io::createIrrXMLReader(mIOWrapper.get());
		if (mReader)
		{
			while (mReader->read())
			{
				// handle the root element "COLLADA"
				if (mReader->getNodeType() == irr::io::EXN_ELEMENT)
				{
					std::cout << "PARSER HAS AN ELEMENT NODE: " << mReader->getNodeType() << ", name = " << mReader->getNodeName() << std::endl;

					if (IsElement("COLLADA", mReader))
					{
						while (mReader->read())
						{
							// beginning of elements
							if (mReader->getNodeType() == irr::io::EXN_ELEMENT)
							{
								if (IsElement("library_kinematics_models", mReader))
								{
									std::cout << "FOUND library_kinematics_models" << std::endl;
									ReadTransformsFromKinematicsLibrary(mReader);
								}

							}
							else if (mReader->getNodeType() == irr::io::EXN_ELEMENT_END)
							{
								continue;
							}
						}
					}
					else
					{
						DefaultLogger::get()->debug(boost::str(boost::format("Ignoring global element \"%s\".") % mReader->getNodeName()));
						SkipElement(mReader);
					}
				}
				else
				{
						// skip everything else silently
				}
			}
		}
	}
}

void ColladaParser::ReadTransformsFromKinematicsLibrary(irr::io::IrrXMLReader* mReader)
{
	std::cout << "=== ReadTransformsFromKinematicsLibrary() ===" << std::endl;
	
	while (mReader->read())
	{
		std::cout << " -> node: " << mReader->getNodeName() << ", type = " << mReader->getNodeType() << std::endl;
		if (mReader->getNodeType() == irr::io::EXN_ELEMENT)
		{
			if (IsOpenElement("kinematics_model", mReader))
			{
				KinematicsModel kinmod;
				KinematicsModel kinmod_container;

				kinmod.id = mReader->getAttributeValue("id") ? mReader->getAttributeValue("id") : GetUnnamed("kinmod_unnamed_");
				kinmod.name = mReader->getAttributeValue("name") ? mReader->getAttributeValue("name") : kinmod.id;

				kinmod_container.id = kinmod.id;
				kinmod_container.name = kinmod.name;

				std::cout << " --> read Kinematics Model: " << kinmod.id << " -- name = " << kinmod.name << std::endl;

				while (mReader->read())
				{
					if (mReader->getNodeType() == irr::io::EXN_ELEMENT)
					{
						if (IsOpenElement("technique_common", mReader)) 
						{
							while (mReader->read())
							{
								if (mReader->getNodeType() == irr::io::EXN_ELEMENT)
								{
									if (IsOpenElement("link", mReader))
									{

										std::string rootLinkSid = mReader->getAttributeValue("sid") ? mReader->getAttributeValue("sid") : GetUnnamed("auto_link_");
										std::string rootLinkName = mReader->getAttributeValue("name") ? mReader->getAttributeValue("name") : rootLinkSid;
										kinmod.rootLinkName = rootLinkName;

										RelativeTransformStack relativeTransforms;
										std::vector<Collada::Transform> relativeTranslations;

										std::cout << "     --> read links..." << std::endl;
										ReadLink(kinmod.id, kinmod.name, true, mReader, relativeTransforms, "", "", relativeTranslations, -1);

										std::cout << "         After reading links: Registered relativeTransforms = " << relativeTransforms.transformStack.size() << std::endl;

										std::cout << "BEGIN RELATIVE TRANSFORMS DUMP" << std::endl;
#if 0
										for (KinematicsModelsRelativeTransformStack::iterator it = relativeTransforms.transformStack.begin(); it != relativeTransforms.transformStack.end(); ++it)
										{
											std::string key = it->first;

											std::cout << "          - Link " << key << ": "  << std::endl;
											LinkLibrary::const_iterator link_it = this->mLinkLibrary.find(key);
											if (link_it != mLinkLibrary.end())
											{
												std::cout << "           -> postLinks aka. attachments: " << link_it->second.postLinks.size() << std::endl;
												for (KinematicsModelsRelativeTransformStack::iterator tr_it = relativeTransforms.transformStack.begin(); tr_it != relativeTransforms.transformStack.end(); ++tr_it)
												{
													if (tr_it->first.compare(key) == 0)
													{
														RelativeTransformEntry& rt_entry = tr_it->second;
														if (rt_entry.transform.mType == TF_TRANSLATE)
															std::cout << "           " << "* " << rt_entry.elementId << ": translate by " << rt_entry.transform.f[0] << "," << rt_entry.transform.f[1] << "," << rt_entry.transform.f[2] << std::endl;
														else if (rt_entry.transform.mType == TF_ROTATE)
															std::cout << "           " << "* " << rt_entry.elementId << ": rotate around axis " << rt_entry.transform.f[0] << "," << rt_entry.transform.f[1] << "," << rt_entry.transform.f[2] << " by " << rt_entry.transform.f[3] << " deg." << std::endl;

														//std::cout << "            attachmentIndices: " << rt_entry.attachmentIndices.size() << " -- ";

														for (int l = 0; l < rt_entry.attachmentIndices.size(); ++l)
														{
															std::cout << "            attachment index " << l << " = " << rt_entry.attachmentIndices[l] << ": ";
															if (rt_entry.attachmentIndices[l] < link_it->second.postLinks.size())
															{
																std::cout << " belongs to link " << link_it->second.postLinks[rt_entry.attachmentIndices[l]] << std::endl;
																rt_entry.attachmentTransformations.push_back(std::make_pair(rt_entry.attachmentIndices[l], link_it->second.postLinks[rt_entry.attachmentIndices[l]]));
															}
															else
															{
																std::cout << " NO MATCHING LINK FOUND!" << std::endl;
															}
														}
													}
												}
											}
										}

										std::cout << "END   RELATIVE TRANSFORMS DUMP" << std::endl;
#endif
										std::vector<std::string> processedLinkIds;
										for (KinematicsModelsRelativeTransformStack::iterator it = relativeTransforms.transformStack.begin(); it != relativeTransforms.transformStack.end(); ++it)
										{
											std::string key = it->first;

											if (std::find(processedLinkIds.begin(), processedLinkIds.end(), key) == processedLinkIds.end())
												processedLinkIds.push_back(key);
											else
												continue;

											LinkLibrary::const_iterator link_it = this->mLinkLibrary.find(key);
											if (link_it != mLinkLibrary.end())
											{
												
												for (KinematicsModelsRelativeTransformStack::iterator tr_it = relativeTransforms.transformStack.begin(); tr_it != relativeTransforms.transformStack.end(); ++tr_it)
												{
													if (tr_it->first.compare(key) == 0)
													{
														RelativeTransformEntry& rt_entry = tr_it->second;
														for (int l = 0; l < rt_entry.attachmentIndices.size(); ++l)
														{
															rt_entry.indicesToPostLinks.insert(std::make_pair(rt_entry.attachmentIndices[l], link_it->second.postLinks[rt_entry.attachmentIndices[l]]));
														}
													}
												}

												/*std::cout << "             - Link " << key << " post-link transform indices: " << std::endl;
												for (std::multimap<unsigned int, std::string>::const_iterator pl_it = rt_entry.indicesToPostLinks.begin(); pl_it != rt_entry.indicesToPostLinks.end(); ++pl_it)
												{
													std::cout << "               - " << pl_it->first << ": to post-link " << pl_it->second << std::endl;
												}*/
											}
										}

										std::cout << "    sanity check: Links in model = " << processedLinkIds.size() << std::endl;

										kinModel_RelativeTransforms.insert(std::make_pair(kinmod.name, relativeTransforms));
									}
									else
									{
										SkipElement(mReader);
									}
								}
								else if (mReader->getNodeType() == irr::io::EXN_ELEMENT_END)
								{ 
									if (IsCloseElement("technique_common", mReader))
										break;
								}
							}

						}
						else
						{
							SkipElement(mReader);
						}
					}
					else if (mReader->getNodeType() == irr::io::EXN_ELEMENT_END)
					{
						if (IsCloseElement("kinematics_model", mReader))
							break;
					}
				}
			}
			else	{
				SkipElement(mReader);
			}
		}
		else if (mReader->getNodeType() == irr::io::EXN_ELEMENT_END) {
			if (IsCloseElement("library_kinematics_models", mReader)) break;
		}
	}
	std::cout << "Done Read Kinematics Models, relative transforms" << std::endl;

	// Re-organize the link-to-transform data structure to "flatten" it out
	std::cout << "==== Relative transformations dump: " << kinModel_RelativeTransforms.size() << " entries ====" << std::endl;
	std::map<std::string, std::map<std::string, std::vector<std::pair<unsigned int, std::string> > > > linksToRelativePost;
	for (std::map<std::string, RelativeTransformStack>::iterator kit = kinModel_RelativeTransforms.begin(); kit != kinModel_RelativeTransforms.end(); ++kit)
	{
		RelativeTransformStack& rt_stack = kit->second;

		if (linksToRelativePost.find(kit->first) == linksToRelativePost.end())
			linksToRelativePost.insert(std::make_pair(kit->first, std::map<std::string, std::vector<std::pair<unsigned int, std::string> > >()));

		for (KinematicsModelsRelativeTransformStack::iterator rt_it = rt_stack.transformStack.begin(); rt_it != rt_stack.transformStack.end(); ++rt_it)
		{
			RelativeTransformEntry& rt_entry = rt_it->second;

			if (linksToRelativePost[kit->first].find(rt_it->first) == linksToRelativePost[kit->first].end())
				linksToRelativePost[kit->first].insert(std::make_pair(rt_it->first, std::vector<std::pair<unsigned int, std::string> >()));

			for (std::multimap<unsigned int, std::string>::const_iterator a_it = rt_entry.indicesToPostLinks.begin(); a_it != rt_entry.indicesToPostLinks.end(); ++a_it)
			{
				std::cout << "   - " << a_it->first << " to attachment " << a_it->second << std::endl;
				linksToRelativePost[kit->first][rt_it->first].push_back(std::make_pair(a_it->first, a_it->second));
			}
		}
	}
	
	// Re-fill the "flattened" link-to-transform structure into the RelativeTransformStack structure
	for (std::map<std::string, std::map<std::string, std::vector<std::pair<unsigned int, std::string> > > >::iterator model_it = linksToRelativePost.begin(); model_it != linksToRelativePost.end(); ++model_it)
	{
		std::cout << " * Model: " << model_it->first << std::endl;
		std::map<std::string, std::vector<std::pair<unsigned int, std::string> > >& linksInModel = model_it->second;
		
		RelativeTransformStack& modelTransformStack = kinModel_RelativeTransforms[model_it->first];

		for (std::map<std::string, std::vector<std::pair<unsigned int, std::string> > >::iterator link_it = linksInModel.begin(); link_it != linksInModel.end(); ++link_it)
		{
			std::vector<unsigned int> matchingLinkIndices;
			for (size_t m = 0; m < modelTransformStack.transformStack.size(); ++m)
			{
				if (modelTransformStack.transformStack[m].first.compare(link_it->first) == 0)
					matchingLinkIndices.push_back(m);
			}
			
			std::cout << "  - Link post-links and indices: " << link_it->first << "; registered transforms = " << link_it->second.size() << "; matching entry indices in transformStack = " << matchingLinkIndices.size() << std::endl;
			for (int m = 0; m < link_it->second.size(); ++m)
			{
				std::cout << "    - " << link_it->second[m].first << ": " << link_it->second[m].second << std::endl;
				if (m < matchingLinkIndices.size() && matchingLinkIndices[m] < modelTransformStack.transformStack.size())
				{
					std::cout << "       match to index entry in transformStack: " << matchingLinkIndices[m] << " = " << modelTransformStack.transformStack.at(matchingLinkIndices[m]).first << std::endl;
					std::cout << "       Type: ";
					Assimp::RelativeTransformEntry& transformEntry = modelTransformStack.transformStack.at(matchingLinkIndices[m]).second;
					Collada::Transform& transform = transformEntry.transform;
					if (transform.mType == TF_TRANSLATE)
						std::cout << "Translation: " << transform.f[0] << "," << transform.f[1] << "," << transform.f[2];
					else if (transform.mType == TF_ROTATE)
						std::cout << "Rotation   : axis = " << transform.f[0] << "," << transform.f[1] << "," << transform.f[2] << ", angle = " << transform.f[3];

					std::cout << std::endl;

					if (transformEntry.indicesToPostLinks.find(link_it->second[m].first) == transformEntry.indicesToPostLinks.end())
						transformEntry.indicesToPostLinks.insert(std::make_pair(link_it->second[m].first, link_it->second[m].second));

					std::cout << "        indicesToPostLinks now has: " << transformEntry.indicesToPostLinks.size() << " entries." << std::endl;
					for (std::multimap<unsigned int, std::string>::const_iterator pl_it = transformEntry.indicesToPostLinks.begin(); pl_it != transformEntry.indicesToPostLinks.end(); ++pl_it)
					{
						std::cout << "          - " << pl_it->first << " -- " << pl_it->second << std::endl;
					}
				}
			}
		}
	}

	std::cout << "===================================================" << std::endl;
}

#endif // !! ASSIMP_BUILD_NO_DAE_IMPORTER

/** SNIPPETS */

/*
 *
    while( mReader->read())
    {
        if( mReader->getNodeType() == irr::io::EXN_ELEMENT)	{
            if( IsOpenElement( "XXX")) {

            } else {
                SkipElement();
            }
        } else if( mReader->getNodeType() == irr::io::EXN_ELEMENT_END){
            if (IsCloseElement("YYY")) break;
        }
    }
    */

