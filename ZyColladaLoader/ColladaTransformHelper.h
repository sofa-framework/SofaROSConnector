#ifndef SOFA_COMPONENT_LOADER_ColladaTransformHelper_H
#define SOFA_COMPONENT_LOADER_ColladaTransformHelper_H

#include <ZyColladaLoader/config.h>

#include <sofa/core/objectmodel/BaseObject.h>
#include <sofa/defaulttype/Vec.h>

#include <sofa/core/visual/VisualParams.h>

namespace sofa
{
	namespace component
	{
		namespace loader
		{
			using namespace sofa::defaulttype;
			class ObjectTransform							   
			{
				public:
					ObjectTransform(const Vector3& translation = Vector3(0, 0, 0), const Quaternion& quaternion = Quaternion(0, 0, 0, 1), ObjectTransform* parentTransform = NULL, ObjectTransform* rootTransform = NULL) :
						m_translation(translation), m_rotationQuat(quaternion), m_parentTransform(parentTransform), m_parentRelative(false), m_rootTransform(rootTransform)
					{
						if (m_parentTransform != NULL)
							m_parentRelative = true;
					}

					ObjectTransform(const ObjectTransform& other)
					{
						if (this != &other)
						{
							m_translation = other.m_translation;
							m_rotationQuat = other.m_rotationQuat;
							m_parentTransform = other.m_parentTransform;
							m_rootTransform = other.m_rootTransform;
							m_parentRelative = other.m_parentRelative;
							m_name = other.m_name;

							for (std::vector<std::string>::const_iterator it = other.m_allChildTransformNames.begin(); it != other.m_allChildTransformNames.end(); it++)
								m_allChildTransformNames.push_back((*it));

							for (std::map<std::string, ObjectTransform>::const_iterator it = other.m_childTransforms.begin(); it != other.m_childTransforms.end(); it++)
								m_childTransforms.insert(std::make_pair(it->first, it->second));
						}
					}

					ObjectTransform& operator=(const ObjectTransform& other)
					{
						if (this != &other)
						{
							m_translation = other.m_translation;
							m_rotationQuat = other.m_rotationQuat;
							m_parentTransform = other.m_parentTransform;
							m_rootTransform = other.m_rootTransform;
							m_parentRelative = other.m_parentRelative;
							m_name = other.m_name;

							for (std::map<std::string, ObjectTransform>::const_iterator it = other.m_childTransforms.begin(); it != other.m_childTransforms.end(); it++)
								m_childTransforms.insert(std::make_pair(it->first, it->second));

							for (std::vector<std::string>::const_iterator it = other.m_allChildTransformNames.begin(); it != other.m_allChildTransformNames.end(); it++)
								m_allChildTransformNames.push_back((*it));
						}
						return (*this);
					}

					const Vector3& getTranslation() { return m_translation; }
					const Quaternion& getQuaternion() { return m_rotationQuat; }

					void setParentTransform(ObjectTransform* tr) { m_parentTransform = tr; }
					ObjectTransform* getParentTransform() { return m_parentTransform; }


					void addChildTransform(const ObjectTransform& childTransform)
					{
						m_allChildTransformNames.push_back(childTransform.getName());
						std::cout << "==> addChildTransform " << this->getName() << ": " << childTransform.getName() << "; now childCount = " << m_allChildTransformNames.size() << std::endl;
						for (std::vector<std::string>::const_iterator it = m_allChildTransformNames.begin(); it != m_allChildTransformNames.end(); it++)
							std::cout << " * " << (*it) << std::endl;

						if (m_rootTransform != NULL)
							m_rootTransform->m_allChildTransformNames.push_back(childTransform.getName());


						const_cast<ObjectTransform*>(&childTransform)->setParentTransform(this);
						m_childTransforms[childTransform.getName()] = childTransform;
					}

					bool hasChildTransform(const std::string& childName)
					{
						return (m_childTransforms.find(childName) != m_childTransforms.end());
					}

					ObjectTransform* getChildTransform(const std::string& childName)
					{
						if (m_childTransforms.find(childName) != m_childTransforms.end())
							return &(m_childTransforms[childName]);

						return NULL;
					}

					std::vector<std::string> getChildTransformNames()
					{
						/*std::vector<std::string> transformNames;
						for (std::map<std::string, ObjectTransform>::const_iterator it = m_childTransforms.begin(); it != m_childTransforms.end(); it++)
							transformNames.push_back(it->first);

						return transformNames;*/
						return m_allChildTransformNames;
					}
					
					int getNumChildTransforms() const { return m_childTransforms.size(); }

					const std::string& getName() const { return m_name; }
					void setName(const std::string& name) { m_name = name; }

				private:
					friend class ColladaTransformHelper;
					Vector3 m_translation;
					Quaternion m_rotationQuat;
					bool m_parentRelative;

					std::string m_name;

					ObjectTransform* m_parentTransform;
					ObjectTransform* m_rootTransform;
					std::map<std::string, ObjectTransform> m_childTransforms;

					std::vector<std::string> m_allChildTransformNames;
			};

			class ColladaTransformHelper: public sofa::core::objectmodel::BaseObject
			{
				public:
					SOFA_CLASS(ColladaTransformHelper, sofa::core::objectmodel::BaseObject);

					ColladaTransformHelper();
					virtual ~ColladaTransformHelper();

					void addObjectTransform(const std::string&, const Vector3&, const Quaternion&);

					void createJointParentTransform(const std::string&, const Vector3&, const Quaternion&, const std::string& rootJointName = "");
					void addJointTransform(const std::string&, const std::string&, const std::string&, const Vector3&, const Quaternion&);

					bool hasJointTransform(const std::string&, const std::string&);
					ObjectTransform* getJointTransform(const std::string&, const std::string&);

					virtual void draw(const core::visual::VisualParams*);

					void dumpJointHierarchies();

					Data<bool> m_bUsePlainGL;
					Data<bool> m_bDrawArrows;
					Data<bool> m_bDisplayText;

				protected:
					void dumpTransformHierarchy(ObjectTransform&);
					ObjectTransform* findChildTransformRecursive(const std::string&, const std::string&);
					bool findChildTransformRecursiveHelper(ObjectTransform*, const std::string&, ObjectTransform*&);
					
					std::map<std::string, ObjectTransform> m_transforms;

					std::map<std::string, ObjectTransform> m_parentJointTransforms;
					std::map<std::string, ObjectTransform> m_orphanJointTransforms;

					std::map<std::string, std::string> m_modelToRootJointMappings;

					void drawCoordinateMarkerGL(float lineLength, float lineWidth, const Vec4f& xColor = Vec4f(1, 0, 0, 1), const Vec4f& yColor = Vec4f(0, 1, 0, 1), const Vec4f& zColor = Vec4f(0, 0, 1, 1));
					void drawJointHierarchies(const core::visual::VisualParams*);
			};
		}
	}
}

#endif //SOFA_COMPONENT_LOADER_ColladaTransformHelper_H
