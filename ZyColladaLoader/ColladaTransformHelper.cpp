#include "ColladaTransformHelper.h"

#include <sofa/core/ObjectFactory.h>
#include <sofa/core/objectmodel/ClassInfo.h>

#ifdef _WIN32
#include <gl/GL.h>
#include <gl/glut.h>
#else
#include <GL/gl.h>
#include <GL/glut.h>
#endif

#include <boost/regex.hpp>
#include <boost/algorithm/string.hpp>

using namespace sofa::defaulttype;
using namespace sofa::component::loader;

SOFA_DECL_CLASS(ColladaTransformHelper)

int ColladaTransformHelperClass = sofa::core::RegisterObject("Collada transform helper.")
.add< ColladaTransformHelper >()
;

ColladaTransformHelper::ColladaTransformHelper():
m_bUsePlainGL(initData(&m_bUsePlainGL, true, "usePlainGL", "Use pure OpenGL calls for drawing")),
m_bDrawArrows(initData(&m_bDrawArrows, false, "drawArrows", "Draw coordinate markers in joint hierarchies")),
m_bDisplayText(initData(&m_bDisplayText, true, "displayText", "Display text infos for joint hierarchies"))
{

}

ColladaTransformHelper::~ColladaTransformHelper()
{

}

void ColladaTransformHelper::addObjectTransform(const std::string& objectName, const Vector3& position, const Quaternion& orientation)
{
	ObjectTransform objTransform(position, orientation);
	objTransform.setName(objectName);
	this->m_transforms[objectName] = objTransform;
}

void ColladaTransformHelper::addJointTransform(const std::string& objectName, const std::string& parentTransform, const std::string& transformName, const Vector3& position, const Quaternion& orientation)
{
	std::cout << "ColladaTransformHelper::addJointTransform(" << objectName << "," << parentTransform << "," << transformName << "," << position << "," << orientation << ")" << std::endl;

	/*size_t slashPos_parent =  parentTransform.find("/");
	std::cout << " parentTransform parts: " << parentTransform.substr(0, slashPos_parent) << ", " << parentTransform.substr(slashPos_parent + 1, parentTransform.size() - 1) << std::endl;
	std::string jointRootName = parentTransform.substr(0, slashPos_parent);
	std::string jointName = parentTransform.substr(slashPos_parent + 1, parentTransform.size() - 1);*/

	std::string name_to_search = objectName;
	/*if (this->m_modelToRootJointMappings.find(objectName) != this->m_modelToRootJointMappings.end())
		name_to_search = this->m_modelToRootJointMappings[objectName];*/

	if (this->m_parentJointTransforms.find(name_to_search) != this->m_parentJointTransforms.end())
	{
		ObjectTransform& objectTransform = this->m_parentJointTransforms[name_to_search];

		ObjectTransform* parent_tr = this->findChildTransformRecursive(name_to_search, parentTransform);
		if (parent_tr != NULL)
		{
			std::cout << "  FOUND a possible parent transform: " << parent_tr->getName() << std::endl;
			if (!parent_tr->hasChildTransform(transformName))
			{
				ObjectTransform child(position, orientation, parent_tr, &objectTransform);
				child.setName(transformName);
				objectTransform.addChildTransform(child);
			}
			else
			{
				std::vector<std::string> child_transforms = parent_tr->getChildTransformNames();
				std::cout << "  FOUND NO POSSIBLE PARENT TRANSFORM! Searched for: " << transformName << ", candidates = " << child_transforms.size() << std::endl;				
				std::cout << "   names: ";
				for (size_t m = 0; m < child_transforms.size(); ++m)
					std::cout << child_transforms[m] << ";";
				
				std::cout << std::endl;
			}
		}
		else
		{
			std::cout << "  NO possible parent transform found!" << std::endl;
			ObjectTransform orphan(position, orientation, NULL, &objectTransform);
			this->m_orphanJointTransforms.insert(std::make_pair(transformName, orphan));
		}
	}
}

bool ColladaTransformHelper::findChildTransformRecursiveHelper(ObjectTransform* transform, const std::string& transformName, ObjectTransform*& result)
{
	std::cout << " findChildTransformRecursiveHelper(" << transformName << ")" << std::endl;

	if (result != NULL)
		return true;

	for (std::map<std::string, ObjectTransform>::iterator it = transform->m_childTransforms.begin(); it != transform->m_childTransforms.end(); it++)
	{
		std::cout << "  compare: " << it->first << " == " << transformName << std::endl;
		if (it->first.compare(transformName) == 0)
		{
			std::cout << "  Target transform " << transformName << " found in " << it->second.getName() << std::endl;
			result = &(it->second);
			return true;
		}
		findChildTransformRecursiveHelper(&(it->second), transformName, result);
	}

	std::cout << "  Target transform " << transformName << " NOT FOUND in " << transform->getName() << std::endl;
	return false;
}

ObjectTransform* ColladaTransformHelper::findChildTransformRecursive(const std::string& hierarchyName, const std::string& transformName)
{
	std::cout << "ColladaTransformHelper::findChildTransformRecursive('" << hierarchyName << "', '" << transformName << "')" << std::endl;

	std::map<std::string, ObjectTransform>::const_iterator hr_it = this->m_parentJointTransforms.find(hierarchyName);
	bool partialNameMatch = false;
	if (hr_it == this->m_parentJointTransforms.end())
	{
		for (std::map<std::string, ObjectTransform>::const_iterator cur_hr_it = this->m_parentJointTransforms.begin(); cur_hr_it != this->m_parentJointTransforms.end(); ++cur_hr_it)
		{
			std::string hierarchy_name = cur_hr_it->first;
			
			boost::regex namePartSplit("/");

			boost::sregex_token_iterator i(hierarchy_name.begin(), hierarchy_name.end(), namePartSplit, -1);
			boost::sregex_token_iterator j;
			std::vector<std::string> nameParts;
			while (i != j)
			{
				nameParts.push_back(*i);
				++i;
			}

			if (nameParts.size() >= 1)
			{
				if (nameParts.at(0).compare(hierarchyName) == 0)
				{
					partialNameMatch = true;
				}
			}
		}
	}

	if (!partialNameMatch && hr_it == this->m_parentJointTransforms.end())
	{
		std::cout << " no hierarchy named " << hierarchyName << std::endl;
		return NULL;
	}

	if (hierarchyName.compare(transformName) == 0)
	{
		std::cout << " looking for hierarchy root transform ... returning from m_parentJointTransforms" << std::endl;
		return &(this->m_parentJointTransforms[hierarchyName]);
	}

	/*if (this->m_modelToRootJointMappings.find(hierarchyName) != this->m_modelToRootJointMappings.end())
	{
		std::cout << " looking for hierarchy root transform, mapped root joint name ... returning from m_parentJointTransforms" << std::endl;
		return &(this->m_parentJointTransforms[this->m_modelToRootJointMappings[hierarchyName]]);
	}*/

	std::vector<std::string> trNames = this->m_parentJointTransforms[hierarchyName].getChildTransformNames();
	std::cout << " TransformNames in hierarchy " << hierarchyName << ": " << trNames.size() << std::endl;
	bool trNameFound = false;
    for (size_t k = 0; k < trNames.size(); k++)
	{
		std::cout << " * " << trNames[k] << " -- compare with " << transformName << " result: " << trNames[k].compare(transformName) << std::endl;
		if (trNames[k].compare(transformName) == 0)
		{
			trNameFound = true;
			break;
		}
	}

	if (!trNameFound)
	{
		std::cout << " no child transform named " << transformName << " in hierarchy " << hierarchyName << std::endl;
		return NULL;
	}

	ObjectTransform* result = NULL;
	bool ret = findChildTransformRecursiveHelper(&(this->m_parentJointTransforms[hierarchyName]), transformName, result);

	std::cout << "Recursive search result: " << ret << std::endl;
	if (result != NULL)
		std::cout << " ObjectTransform found: " << result->getName() << std::endl;
	else
		std::cout << " NO ObjectTransform found" << std::endl;

	return result;
}

void ColladaTransformHelper::dumpTransformHierarchy(ObjectTransform& tr)
{
	std::cout << " dumpTransformHierarchy(" << tr.getName() << ")" << std::endl;
	std::vector<std::string> childTransforms = tr.getChildTransformNames();
    for (size_t k = 0; k < childTransforms.size(); k++)
	{
		ObjectTransform* child_tr = tr.getChildTransform(childTransforms[k]);
		std::cout << "  * child " << child_tr->getName() << ": " << child_tr->getTranslation() << " -- " << child_tr->getQuaternion() << std::endl;
		if (child_tr->getNumChildTransforms() > 0)
			dumpTransformHierarchy(*child_tr);
	}
}

void ColladaTransformHelper::dumpJointHierarchies()
{
	for (std::map<std::string, ObjectTransform>::iterator it = m_parentJointTransforms.begin();
		it != m_parentJointTransforms.end(); it++)
	{
		std::cout << "==== Joint hierarchy: " << it->first << " ====" << std::endl;
		ObjectTransform& tr = it->second;
		std::vector<std::string> childTransforms = tr.getChildTransformNames();
        for (size_t k = 0; k < childTransforms.size(); k++)
		{
			ObjectTransform* child_tr = tr.getChildTransform(childTransforms[k]);
			if (child_tr != NULL)
			{
				std::cout << "  * child " << k << " -- " << child_tr->getName() << ": " << child_tr->getTranslation() << " -- " << child_tr->getQuaternion() << std::endl;
				ObjectTransform* tr_parent = child_tr->getParentTransform();
				std::cout << "   path to root: ";
				do
				{
					std::cout << tr_parent->getName() <<  " -- ";
					tr_parent = tr_parent->getParentTransform();
				} while (tr_parent != NULL && tr_parent->getParentTransform() != NULL);
				std::cout << std::endl;
				dumpTransformHierarchy(*child_tr);
			}
			else
			{
				std::cout << "  * child " << k << " -- INVALID ObjectTransform pointer!" << std::endl;
			}
		}
	}
}

void ColladaTransformHelper::createJointParentTransform(const std::string& objectName, const Vector3& position, const Quaternion& orientation, const std::string& rootJointName)
{
	ObjectTransform jointParentTransform(position, orientation);
	jointParentTransform.setName(objectName);
	this->m_parentJointTransforms[objectName] = jointParentTransform;

	this->m_modelToRootJointMappings[objectName] = rootJointName;

	/*if (!rootJointName.empty())
		m_parentJointTransforms[rootJointName] = jointParentTransform;*/
}

bool ColladaTransformHelper::hasJointTransform(const std::string& objectName, const std::string& jointName)
{
	std::cout << "ColladaTransformHelper::hasJointTransform(" << objectName << "," << jointName << ")" << std::endl;
	if (this->m_parentJointTransforms.find(objectName) == this->m_parentJointTransforms.end())
	{
		std::cout << " no joint hierarchy named " << objectName << " exists, return false." << std::endl;
		return false;
	}

	std::cout << " search for child " << jointName << " in joint hierarchy " << objectName << std::endl;
	ObjectTransform* joint_tr = this->findChildTransformRecursive(objectName, jointName);

	std::cout << " search result = " << (joint_tr != NULL);
	if (joint_tr != NULL)
		std::cout << ", transform found = " << joint_tr->getName() << std::endl;
	else
		std::cout << ", no matching transform found!" << std::endl;

	return (joint_tr != NULL);
}

ObjectTransform* ColladaTransformHelper::getJointTransform(const std::string& objectName, const std::string& jointName)
{
	std::cout << "ColladaTransformHelper::getJointTransform(" << objectName << "," << jointName << ")" << std::endl;
	if (this->m_parentJointTransforms.find(objectName) == this->m_parentJointTransforms.end())
	{
		std::cout << " no joint hierarchy named " << objectName << " exists, return false." << std::endl;
		return false;
	}

	std::cout << " search for child " << jointName << " in joint hierarchy " << objectName << std::endl;
	ObjectTransform* joint_tr = this->findChildTransformRecursive(objectName, jointName);

	std::cout << " search result = " << (joint_tr != NULL);
	if (joint_tr != NULL)
		std::cout << ", transform found = " << joint_tr->getName() << std::endl;
	else
		std::cout << ", no matching transform found!" << std::endl;

	return joint_tr;
}

void ColladaTransformHelper::drawCoordinateMarkerGL(float lineLength, float lineWidth, const Vec4f& xColor, const Vec4f& yColor, const Vec4f& zColor)
{
	glLineWidth(lineWidth);
	glBegin(GL_LINES);

	glColor4f(xColor.x(), xColor.y(), xColor.z(), xColor.w());
	glVertex3d(0, 0, 0);
	glVertex3d(lineLength, 0, 0);

	glColor4f(yColor.x(), yColor.y(), yColor.z(), yColor.w());
	glVertex3d(0, 0, 0);
	glVertex3d(0, lineLength, 0);

	glColor4f(zColor.x(), zColor.y(), zColor.z(), zColor.w());
	glVertex3d(0, 0, 0);
	glVertex3d(0, 0, lineLength);

	glEnd();
	glLineWidth(1.0f);
}

void ColladaTransformHelper::drawJointHierarchies(const sofa::core::visual::VisualParams* vparams)
{
	for (std::map<std::string, ObjectTransform>::iterator it = this->m_parentJointTransforms.begin(); it != this->m_parentJointTransforms.end(); ++it)
	{
		ObjectTransform& transform = it->second;
		const Vector3 translation = transform.getTranslation();
		const Quaternion orientation = transform.getQuaternion();

		std::vector<std::string> allChildNames = transform.getChildTransformNames();

		glPushMatrix();

		glPushAttrib(GL_ENABLE_BIT);
		glEnable(GL_COLOR_MATERIAL);

		int numChildren = 0;
        for (size_t k = 0; k < allChildNames.size(); k++)
		{
			if (k != 3)
				continue;

			ObjectTransform* child = this->findChildTransformRecursive(it->first, allChildNames[k]);
			if (child)
			{
				std::vector<Vector3> parentTransforms;
				std::vector<Quaternion> parentRotations;
				std::vector<std::string> parentTransformNames;
				parentTransforms.push_back(child->getTranslation());
				parentRotations.push_back(child->getQuaternion());
				parentTransformNames.push_back(child->getName());

				ObjectTransform* tr_parent = child->getParentTransform();
				if (tr_parent != NULL)
				{
					do
					{
						parentTransforms.push_back(tr_parent->getTranslation());
						parentRotations.push_back(tr_parent->getQuaternion());
						parentTransformNames.push_back(tr_parent->getName());

						tr_parent = tr_parent->getParentTransform();
					} while (tr_parent != NULL && tr_parent->getParentTransform() != NULL);
				}

				std::reverse(parentTransforms.begin(), parentTransforms.end());
				std::reverse(parentRotations.begin(), parentRotations.end());
				std::reverse(parentTransformNames.begin(), parentTransformNames.end());

				Vector3 rootTranslation;

				glLineWidth(3.0f);
				glBegin(GL_LINES);
				std::cout << " --> Draw lines for joint hierarchy " << transform.getName() << ": " << parentTransforms.size() << std::endl;
				std::cout << "    ";
				for (int m = 0; m < parentTransformNames.size(); m++)
					std::cout << parentTransformNames[m] << ";";
					
				std::cout << std::endl;
				
				for (int m = 0; m < parentTransforms.size(); m++)
				{
					if (k % 3 == 0)
						glColor4d(1, 0, 0, 0.75);
					else if (k % 3 == 1)
						glColor4d(0, 1, 0, 0.75);
					else if (k % 3 == 2)
						glColor4d(0, 0, 1, 0.75);

					glVertex3d(rootTranslation.x(), rootTranslation.y(), rootTranslation.z());
					
					if (k % 3 == 0)
						glColor4d(0, 1, 0, 0.75);
					else if (k % 3 == 1)
						glColor4d(0, 0, 1, 0.75);
					else if (k % 3 == 2)
						glColor4d(1, 0, 0, 0.75);
					
					Vector3 parent_tr = parentRotations[m].rotate(parentTransforms[m]);

					glVertex3d(rootTranslation.x() + parent_tr.x(), rootTranslation.y() + parent_tr.y(), rootTranslation.z() + parent_tr.z());

					std::cout << "     * parentTransforms[" << m << "] from '" << parentTransformNames[m] << "' = " << parentTransforms[m] << "; draw line : " << rootTranslation << " to " << (rootTranslation + parentTransforms[m]) << std::endl;

					rootTranslation += parent_tr;
				}
				glEnd();
				glLineWidth(1.0f);
				/*glTranslated(rootTranslation.x(), rootTranslation.y(), rootTranslation.z());
				drawCoordinateMarkerGL(12.0, 6.0);
				glTranslated(-rootTranslation.x(), -rootTranslation.y(), -rootTranslation.z());*/
			
				numChildren++;

				if (numChildren > 6)
					break;
			}
		}

		glPopAttrib();
		glPopMatrix();

		for (int k = 0; k < allChildNames.size(); k++)
		{
			ObjectTransform* child = this->findChildTransformRecursive(it->first, allChildNames[k]);
			if (child)
			{
				vparams->drawTool()->draw3DText(child->getTranslation(), 0.001f, Vec4f(1, 1, 1, 0.25), child->getName().c_str());
			}
		}
	}
}

void ColladaTransformHelper::draw(const sofa::core::visual::VisualParams* vparams)
{
	for (std::map<std::string, ObjectTransform>::iterator it = m_transforms.begin(); it != m_transforms.end(); ++it)
	{
		const Vector3& translation = it->second.getTranslation();
		const Quaternion& orientation = it->second.getQuaternion();
		
		if (m_bUsePlainGL.getValue())
		{
			glPushMatrix();

			glPushAttrib(GL_ENABLE_BIT);
			glEnable(GL_COLOR_MATERIAL);

			glLineWidth(5.0f);
			glBegin(GL_LINE);
			glColor4f(1, 0.5, 0.5, 0.5);
			glVertex3d(0, 0, 0);
			glColor4f(0.5, 1, 0.5, 0.5);
			glVertex3d(translation[0], translation[1], translation[2]);
			glEnd();
			glLineWidth(1.0);

			glPopAttrib();
			glPopMatrix();
		}
		else
		{
			vparams->drawTool()->drawArrow(Vector3(0, 0, 0), translation, 3.0f, Vec4f(0.8, 0.2, 0.2, 0.25));
		}

		/*glPushMatrix();

		glPushAttrib(GL_ENABLE_BIT);
		glEnable(GL_COLOR_MATERIAL);
		
		if (m_bUsePlainGL.getValue())
			drawCoordinateMarkerGL(15.0f, 8.0f);

		if (m_bDisplayText.getValue())
		{
			Mat<4, 4, GLfloat> modelviewM;
			float scale = 0.0002f;
			glScalef(scale, scale, scale);

			// Makes text always face the viewer by removing the scene rotation
			// get the current modelview matrix
			glGetFloatv(GL_MODELVIEW_MATRIX, modelviewM.ptr());
			modelviewM.transpose();

			sofa::defaulttype::Vec3f temp = modelviewM.transform(translation);

			glLoadIdentity();

			glTranslatef(temp[0], temp[1], temp[2]);
			glScalef(scale, scale, scale);

			std::stringstream transform_data;
			transform_data << it->first << ": " << translation;
			std::string output = transform_data.str();
			const char *s = output.c_str();

			glColor4f(1, 1, 1, 0.5);
			while (*s)
			{
				glutStrokeCharacter(GLUT_STROKE_ROMAN, *s);
				s++;
			}

			glTranslated(-translation.x(), -translation.y(), -translation.z());
		}*/

		/*if (!m_bUsePlainGL.getValue())
		{
			if (m_bDrawArrows.getValue())
				vparams->drawTool()->drawFrame(translation, orientation, Vec3f(3, 3, 3));
		}
		else
		{
			glTranslated(translation.x(), translation.y(), translation.z());
			drawCoordinateMarkerGL(3.0f, 2.0f);
			glTranslated(-translation.x(), -translation.y(), -translation.z());
		}*/


		/*for (std::map<std::string, ObjectTransform>::iterator it = this->m_parentJointTransforms.begin(); it != this->m_parentJointTransforms.end(); ++it)
		{
			const Vector3& joint_translation = it->second.getTranslation();
			const Quaternion& joint_orientation = it->second.getQuaternion();

			if (!m_bUsePlainGL.getValue())
			{
				vparams->drawTool()->drawArrow(Vector3(0, 0, 0), joint_translation, 3.0f, Vec4f(0.2, 0.8, 0.2, 0.5));
			}
			else
			{
				glPushMatrix();

				glPushAttrib(GL_ENABLE_BIT);
				glEnable(GL_COLOR_MATERIAL);

				glBegin(GL_LINES);
				
				glColor4d(0.8, 0.2, 0.2, 0.75);
				glVertex3d(0, 0, 0);
				
				glColor4d(0.2, 0.8, 0.2, 0.75);
				glVertex3d(translation.x(), translation.y(), translation.z());

				glColor4d(1, 1, 1, 0.75);
				glVertex3d(translation.x(), translation.y(), translation.z());

				glColor4d(1, 1, 1, 1);
				glVertex3d(joint_translation.x(), joint_translation.y(), joint_translation.z());

				glEnd();

				glPopAttrib();
				glPopMatrix();
			}

			glPushMatrix();

			glPushAttrib(GL_ENABLE_BIT);
			glEnable(GL_COLOR_MATERIAL);

			glTranslated(translation.x() + joint_translation.x(), translation.y() + joint_translation.y(), translation.z() + joint_translation.z());

			if (m_bUsePlainGL.getValue())
				drawCoordinateMarkerGL(25.0f, 12.5f);

			if (m_bDisplayText.getValue())
			{
				Mat<4, 4, GLfloat> modelviewM;
				float scale = 0.005f;
				glScalef(scale, scale, scale);

				// Makes text always face the viewer by removing the scene rotation
				// get the current modelview matrix
				glGetFloatv(GL_MODELVIEW_MATRIX, modelviewM.ptr());
				modelviewM.transpose();

				sofa::defaulttype::Vec3f temp = modelviewM.transform(translation + joint_translation);

				glLoadIdentity();

				glTranslatef(temp[0], temp[1], temp[2]);
				glScalef(scale, scale, scale);

				std::stringstream transform_data;
				transform_data << it->first << ": " << translation << " -- ";
				if (it->second.getNumChildTransforms() > 0)
				{
					transform_data << "Child transforms: " << it->second.getNumChildTransforms() << " -- ";
					std::vector<std::string> childNames = it->second.getChildTransformNames();
					transform_data << "Names: ";
					for (std::vector<std::string>::const_iterator child_it = childNames.begin(); child_it != childNames.end(); ++child_it)
						transform_data << (*child_it) << ";";
				}

				std::string output = transform_data.str();
				const char *s = output.c_str();

				glColor4f(0.2, 1, 0.2, 0.75);
				while (*s)
				{
					glutStrokeCharacter(GLUT_STROKE_ROMAN, *s);
					s++;
				}

				glTranslated(-translation.x() - joint_translation.x(), -translation.y() - joint_translation.y(), -translation.z() - joint_translation.z());

				glPopAttrib();
				glPopMatrix();
			}

			if (!m_bUsePlainGL.getValue() && m_bDrawArrows.getValue())
				vparams->drawTool()->drawFrame(translation, orientation, Vec3f(9, 9, 9));
		

		glPopAttrib();
		glPopMatrix();}*/
	}

	drawJointHierarchies(vparams);
}
