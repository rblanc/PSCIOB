/*
 * This file is part of the psciob library.
 *
 * Copyright (c) 2012, Remi Blanc, University of Bordeaux
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met: 
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer. 
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution. 
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
* \file BaseScene.h
* \author Rémi Blanc 
* \date 29. February 2012
*/

#ifndef BASESCENE_H_
#define BASESCENE_H_


//IDEA: create a class replacing the current "PixelSet", proposing a compact run-length encoding of the pixel set - with respect to a given "image frame"
//for ParametricObject, this PixelSet should remain tied to the minimal image containing the object
//for the scene force all TAssociatedData containers to contain 2 such PixelSet 
//	=> 1 translating the PixelSet of the object in the scene frame
//	=> and 1 for the in-context representation (dealing with overlaps, and the specific scene policy)
// this would certainly be useful for Image-based scenes
// Furthermore, sensors may be highly interested in having the possibility to write into such pixel sets to speed-up the computation / recomputation of metrics.



//TODO: methods behind TestObjectInsertionAcceptance & TestObjectInsertionAcceptanceIgnoringObjectID may be writing some interesting data
//it would be good to have another (protected) version of them that takes an ObjectInScene object as input, so they could save this data for future use.
//keep the original interface for user convenience
//but AddObject and Modify object should call these new methods to avoid recomputing the same stuff again...


#include <vector>
#include <deque>
#include <set>
#include <map>

#include "BoundingBoxesUtils.h"
#include "ITKUtils.h"
#include "VTKUtils.h"

#include "itkBinaryThresholdImageFilter.h"
#include "vtkAppendPolyData.h"

#include "ParametricObject.h"
#include "ObjectInSceneDataContainers.h"
#include "ObjectTypesLibrary.h"
#include "SceneGlobalPrior_Base.h"
#include "ObjectInteractionManager.h"

#include <vtkSmartPointer.h>
#include <vtkAppendPolyData.h>
#include <vtkRenderer.h> 
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkPolyDataMapper.h> 
#include <vtkActor.h> 
#include <vtkActorCollection.h>
#include <vtkProperty.h> 

#include "SensorSceneInterface.h"

namespace psciob {

/** \class BaseScene
* \brief BaseScene
* base class to handle a collection of ParametricObject, associating a unique identifier to each object
* attach various pdfs and likelihoods through a ObjectTypesLibrary
* monitor interaction through an ObjectInteractionManager, etc...
* \sa ParametricObject
* \sa SensorSceneInterface
* \sa ObjectInSceneDataContainers
* \sa ObjectTypesLibrary
* \sa SceneGlobalPrior_Base
* \sa ObjectInteractionManager
*/


//ABSTRACT CLASS
template <unsigned int VDimension, class TAppearance = unsigned char, class TObjectId = unsigned short, class TAssociatedData = ObjectPriorCostContainer, class TInteractionData = VTKIntersectionContainer>
class BaseScene : public itk::LightObject {
public:
	template<class TScene, class TOutputImage> friend class ImageSensor_Base;
	template<class TScene> friend class ObjectInteractionManager;
	template<class TScene> friend class SceneObjectIterator;

	/** Standard class typedefs. */
	typedef BaseScene                     Self;
	typedef itk::LightObject              Superclass;
	typedef itk::SmartPointer<Self>       Pointer;
	typedef itk::SmartPointer<const Self> ConstPointer;
	/** Base Types to which all inherited classes can refer */
	typedef Self                          BaseClass;
	typedef Pointer                       BaseClassPointer;
	/** Run-time type information (and related methods). */
	itkTypeMacro(BaseScene, itk::LightObject);

	/** Dimensionality of the scene, and of the objects within 
	* (pure 2D objects cannot be cast in 3D space, this would necessitate providing a third coordinate - even if it would be a constant) 
	*/
	static const unsigned int Dimension = VDimension;

	/** Base type of deformable objects */
	typedef ParametricObject<VDimension, TAppearance>	DeformableObjectType;

	/** Type of the object identifier - typically, this depends on the expected number of objects simultaneously present inside the scene
	* <255 => unsigned char is fine
	* otherwise unsigned short should be fine with up to 65535 objects
	* The IDType is also used as the pixel type for the label image
	*/
	typedef TObjectId                               IDType;

	/** Type of the Label Image that may be produced out of the scene */
	typedef itk::Image<IDType, Dimension>			LabelImageType;
	/** type of the label objects contained in the scene */
	typedef itk::LabelObject<IDType, Dimension>	    LabelObjectType;
	/** type of the label map representing the scene */
	typedef itk::LabelMap<LabelObjectType>			LabelMapType;

	/** Type of the Binary Image that may be produced out of the scene */
	typedef typename DeformableObjectType::BinaryImageType   BinaryImageType;
	typedef typename DeformableObjectType::BinaryPixelType   BinaryPixelType;

	/** Type of the Textured Image that may be produced out of the scene */
	typedef typename DeformableObjectType::TexturedPixelType TexturedPixelType;
	typedef typename DeformableObjectType::TexturedImageType TexturedImageType;

	/** Type of LabelObject and LabelMap internally used by the DeformableObjects */
	typedef typename DeformableObjectType::LabelObjectType   ObjectLabelObjectType;
	typedef typename DeformableObjectType::LabelMapType      ObjectLabelMapType;

	/** Helper class that lists the different types of objects present in the scene, associating each type with specific PDFs over their parameters 
	* It is possible to add types to the library using the method SetObjectsLibrary
	*/
	typedef ObjectTypesLibrary<DeformableObjectType>         ObjectTypesLibraryType;			
	/** Base type of the class used to manage the interactions between objects (identify interactions, and compute their strength) 
	* At construction time, an instance of ObjectsWithoutInteraction is used
	* It can be replaced using the method SetInteractionManager
	*/
	typedef ObjectInteractionManager<Self>                   ObjectInteractionManagerType;	
	/** Base type of the class used to compute a global prior out of the scene
	* At construction time, an instance of SceneFlatPrior is used
	* It can be replaced using the method SetGlobalPrior
	*/
	typedef SceneGlobalPrior_Base<Self> ScenePriorType;

	/** Base type for the object prior normalization function */
	typedef GenericParametricFunctions<double, double>       PriorNormalizationFunctionType;

	/** Type of the structure defining the data associated to each individual object */
	typedef TAssociatedData             AssociatedDataType;

	/** Type of the structure defining the data associated to each individual object */
	typedef TInteractionData            InteractionDataType;

	/** Type of the structure listing all interactions an object may have with other in the scene */
	typedef std::map<IDType, InteractionDataType>       ObjectInteractionMapType;
	typedef typename ObjectInteractionMapType::iterator ObjectInteractionIterator;
	typedef std::pair<IDType, InteractionDataType>      ObjectInteractionPairType;

	/** Base structure wrapping an object when it is added in the scene 
	* id: unique identifier referencing an object
	* obj: pointer to the actual object
	* objectTypeId: identifier for the type of object
	* priority: in cases of overlaps, higher priority (then higher label) means being drawn over the other objects
	* objectData: template containers hosting further object-related information, managed partly by the scene, the sensor, cost functions, ...
	* interactionData: std::map of template containers hosting information about interactions between an objects and others, managed by the m_interactionManager, possibly the sensor
	*/
	class ObjectInScene {
	public:
		ObjectInScene() : id(0), priority(0), actorFlag(false), sceneOffContextLabelObjectFlag(false), mapper(NULL), actor(NULL), sceneOffContextLabelObject(NULL) {}

		IDType id;
		typename DeformableObjectType::Pointer obj;
		unsigned char objectTypeId; //we assume there won't be more than 255 different object types in a scene
		unsigned char priority;
		AssociatedDataType objectData;
		ObjectInteractionMapType interactionData;

		vtkSmartPointer<vtkActor> actor; bool actorFlag; //updated automatically by the VTKScene
		vtkSmartPointer<vtkPolyDataMapper> mapper;
		typename LabelObjectType::Pointer sceneOffContextLabelObject; bool sceneOffContextLabelObjectFlag; // updated automatically by the LabelMapScene (AddObject & ModifyObjectParameters)
	};

	/** Type of the object collection */
	typedef std::vector<ObjectInScene>       ObjectSetType;

	/** Enumeration of possible boundary conditions 
	* BOUNDARYCONDITIONS_VOID: (DEFAULT) no boundary conditions, objects may be partially outside, the scene will not care
	* BOUNDARYCONDITIONS_SOLID: the scene should forbid objects that partially cross the boundaries of the bounding box
	* BOUNDARYCONDITIONS_PERIODIC: objects are wrapping over to the other side if they partially cross the boundary, and may overlap with objects there...
	*/
	typedef enum  {
		BOUNDARYCONDITIONS_VOID,
		BOUNDARYCONDITIONS_SOLID,
		BOUNDARYCONDITIONS_PERIODIC
	} SceneBoundaryConditionCode;

	/** Enumeration of possible overlap status
	* Not all scene implementation may care about this...
	* OBJECT_FULLYOUTSIDE: object bounding box is outside the scene bounding box, this should never be allowed.
	* OBJECT_NOOVERLAP: object is fully inside the scene, and not overlapping any other object
	* OBJECT_OVERLAPOBJECT: object is fully inside the scene, but overlaps at least one other object
	* OBJECT_OVERLAPSCENE: object is only partially inside the scene, but not overlapping other objects
	* OBJECT_OVERLAPOBJECTANDSCENE: object is only partially inside the scene, and overlaps at least one other object
	*/
	typedef enum {
		OBJECT_FULLYOUTSIDE,
		OBJECT_NOOVERLAP,
		OBJECT_OVERLAPOBJECT,
		OBJECT_OVERLAPSCENE,
		OBJECT_OVERLAPOBJECTANDSCENE,
	} SceneObjectOverlapCode ;

	/** Enumeration of scene Policies for the insertion of objects
	* ACCEPTALL: (default) accept all objects, unless they are fully outside the scene
	* the other are not yet implemented...
	* FORBIDALLOVERLAPS: prevent any object from entering if it intersects something (other objects, possibly the scene boundaries
	* FORBIDPARTIALLYOUTSIDE: prevent any object from entering if it intersects the scene boundaries (IDEA: add a mask
	*/
	typedef enum { //find generic names which may hold both for image or mesh representations?
		ACCEPTALL,
		FORBIDALLOVERLAPS,
		FORBIDPARTIALLYOUTSIDE,
	} SceneObjectInsertionPolicyCode;

	/** */
	typedef SensorSceneInterface<BaseClass>             SensorInterfaceType;
	typedef typename std::vector<SensorInterfaceType *> SensorInterfaceListType;

	/** Give Information about Self */
	virtual std::string GetClassName() const = 0;
	virtual void PrintInfo()           const = 0;

	/** Set physical bounding Box and pixel spacing
	* the actual bounding box will be slightly modify, to ensure that the origin will be at the center of a pixel (even if outside the actual grid)
	*/
	void SetPhysicalDimensions(const vnl_vector<double> &physicalBoundingBox, double spacing = 1.0);
	void SetPhysicalDimensions( typename BinaryImageType::PointType origin, typename BinaryImageType::SpacingType spacing, typename BinaryImageType::SizeType size);
	void SetPhysicalDimensions(const vnl_vector<double> &physicalBoundingBox, const vnl_vector<double> &spacing);

	/** Get information about the scene physical dimensions */
	const vnl_vector<double> &                    GetSceneBoundingBox() const { return m_sceneBBox; }
	const typename BinaryImageType::SpacingType & GetSceneSpacing()     const { return m_sceneSpacing; }
	const typename BinaryImageType::RegionType &  GetSceneImageRegion() const { return m_sceneImageRegion; }
	const typename BinaryImageType::PointType &   GetSceneOrigin()      const { return m_sceneOrigin; }

	/** Get the boundary condition policy code */	
	inline SceneBoundaryConditionCode GetBoundaryConditionPolicyCode() const { return m_BoundaryConditionManagement; }
	/** Set the boundary condition policy code 
	* \todo not functional yet
	* NOT IMPLEMENTED
	* prerequisites: implement the mechanisms related to the non-default BC !!!
	* requirements: check if objects are already present... if yes, updating things would probably be quite painful 
	*				 interactions should be updated
	*				 scene policy may exlude objects, but how to decide which to remove?
	*				 lots of update ; or forbid the modification if this is too complicated
	*/	
	void SetBoundaryConditionPolicyCode(SceneBoundaryConditionCode code) {
		if (m_BoundaryConditionManagement == code) return;
		throw DeformableModelException("SetBoundaryConditionPolicyCode : not implemented, see documentation for more details / implementation ideas");
	}

	/** Get the boundary condition policy code */	
	inline SceneObjectInsertionPolicyCode GetObjectInsertionPolicyCode() const { return m_insertionPolicy; }
	/** Set the object insertion policy code 
	* \todo not functional yet
	* NOT IMPLEMENTED
	* prerequisites: implement the mechanisms related to the non-default Boundary Conditions, and the object overlap codes
	* requirements: check if objects are already present in the scene... if yes, updating things would probably be quite painful 
	*				 interactions should be updated
	*				 scene policy may exlude objects, but how to decide which to remove?
	*				 lots of update ; or forbid the modification if this is too complicated
	*/	
	void SetObjectInsertionPolicyCode(SceneObjectInsertionPolicyCode code) {
		if (m_insertionPolicy == code) return;
		throw DeformableModelException("SetObjectInsertionPolicyCode : not implemented, see documentation for more details / implementation ideas");
	}

	/** Test if an object intersects another object, or the scene boundary, ... 
	* The implementation essentially depends whether the scene prefers using image representations, pixel sets, or meshes... 
	*/
	inline SceneObjectOverlapCode TestOverlap(DeformableObjectType *object)                            { ObjectInScene tmpObj; tmpObj.obj = object; return TestOverlap_Internal(&tmpObj); }
	inline SceneObjectOverlapCode TestOverlapIgnoringObjectID(DeformableObjectType *object, IDType id) { ObjectInScene tmpObj; tmpObj.obj = object; return TestOverlapIgnoringObjectID_Internal(&tmpObj, id); }

	/** Test if an object can be inserted, depending on the specific scene policy and boundary conditions 
	* for the moment, overlaps, and special boundary conditions are not implemented
	* IDEA: when they are implemented, and involved in the test, it would be wise to save a temporary object caching the overlap information if 
	*/
	inline bool TestObjectInsertionAcceptance(DeformableObjectType *object)                            { ObjectInScene tmpObj; tmpObj.obj = object; return TestObjectInsertionAcceptance_Internal(&tmpObj); }

	/** Test if an object can be inserted depending on the specific scene policy, ignoring an existing scene object 
	* same comments as for TestObjectInsertionAcceptance
	*/
	bool TestObjectInsertionAcceptanceIgnoringObjectID(DeformableObjectType *object, IDType id)        { ObjectInScene tmpObj; tmpObj.obj = object; return TestObjectInsertionAcceptanceIgnoringObjectID_Internal(&tmpObj, id); }

	/** Test whether the object is fully inside the scene, based on their respective bounding boxes 
	* It is sufficient to check the axis-aligned bounding boxes.
	*/
	inline bool TestObjectFullyInside(DeformableObjectType *object)                                     { ObjectInScene tmpObj; tmpObj.obj = object; return TestObjectFullyInside_Internal(&tmpObj); }

	/** Test whether the object is fully outside the scene
	* This does not restrict to the bounding boxes (imagine a 'diagonal line' just outside a corner...)
	*/
	inline bool TestObjectFullyOutside(DeformableObjectType *object)                                    { ObjectInScene tmpObj; tmpObj.obj = object; return TestObjectFullyOutside_Internal(&tmpObj); }

protected:
	//internal version using an ObjectInScene, so that data can be cached
	virtual      SceneObjectOverlapCode TestOverlap_Internal(ObjectInScene *object)                            = 0;
	virtual      SceneObjectOverlapCode TestOverlapIgnoringObjectID_Internal(ObjectInScene *object, IDType id) = 0;
	inline bool  TestObjectInsertionAcceptance_Internal(ObjectInScene *object);
	inline bool  TestObjectInsertionAcceptanceIgnoringObjectID_Internal(ObjectInScene *object, IDType id);
	inline bool  TestObjectFullyInside_Internal(ObjectInScene *object);
	virtual bool TestObjectFullyOutside_Internal(ObjectInScene *object) = 0;
public:

	/** Clear all objects but keep the helpers classes, scene policy parameters, etc... unchanged */
	void ClearScene();

	/** Removes all objects	
	* the only difference with ClearScene is to keep the object library (registered types and associated pdfs)
	*/
	void RemoveAllObjects();

	/** Get Number of objects */
	inline unsigned int GetNumberOfObjects()    const { return m_nbObjectsInScene; }

	/** Get total number of parameters (sum of the number of parameters of each object) */
	inline unsigned int GetNumberOfParameters() const { return m_totalNumberOfParameters; }

	/** get the first valid id ; convenience method for initializing other classes... */
	inline IDType FirstID();

	/** Print Information about an object */
	void PrintInfoObject(IDType id)	const;

	/** Get a pointer to an object 
	* returns a NULL pointer if the id is invalied
	* \warning: the object should not be modified by the user
	* \todo : make public const method, and protected non const methods for lib classes that may use it.
	*/
	inline ObjectInScene* GetObject(IDType id);

	/** Get a pointer to the (offcontext = ignoring any potential overlaps) pixel set of the object
	* the representation is that of a labelObject, expressed with respect to the scene origin, spacing and size
	*/
	LabelObjectType* GetObjectLabelObject(IDType id);

	/** Get a pointer to a (offcontext = ignoring any potential overlaps) label map containing the 
	* labelObject corresponding to the requested object...
	*/
	void GetObjectLabelMap(IDType id, LabelMapType* labelMap);

	/** Get a copy of the object parameters ; really useful??
	* \attention No check is performed about the validity of the id
	*/
	inline vnl_vector<double> GetParametersOfObject(IDType id) const { return m_arrayObjects[id-1].obj->GetParameters(); }

	/** Basics of adding an object to the scene
	* IMPORTANT: the object passed as an argument is CLONED inside the scene, so that the user can modify the pointer as he wills, this will not affect the scene in any way.
	* Internally calls a virtual function implementing scene specific checks about the validity of the new object, in case the object is valid:
	* A unique identifier is automatically assigned to the object, unless the proposed id is available. The chosen id is returned as output. 
	* 0 means failure to add the object
	* \attention the proposed ID is only accepted if it is at the beginning, or the end of the list of freed IDs, or outside the current range of used IDs
	*			 using it may break performances, creating many 'holes' in the list of objects, slowing down its traversal
	*/
	IDType AddObject(DeformableObjectType *object, IDType proposedID = 0);

	/** Removes the specified object from the container, updating the other objects if they were in relation. 
	* Returns false if the given id is unused 
	*/
	bool RemoveObject(IDType id);

	/** Modifies an object already present in the scene by updating its parameters to the specified values.
	* returns false if something went wrong and the modification couldn't be applied (non-existing object, invalid parameters, new object forbidden by scene policy...)
	*/
	bool ModifyObjectParameters(IDType id, const vnl_vector<double> &p);


	/** Convert an object to a specified type - return false if something prevented 
	* the type modification (e.g. if modified object does not fit scene policy)
	* NOT IMPLEMENTED
	* requirements: (+id check) check the new type is valid, and different from the previous 
	*				 what should be the parameters of the new object? <- MAIN question...
	*				 test whether this new object could be accepted (as in ModifyObjectParameters)
	*				 erase the old and draw the new (as in ModifyObjectParameters)
	*				 update the scene (priors, interaction, ...)
	*/
	bool ModifyObjectType(IDType id, unsigned int targetType) {
		throw DeformableModelException("ModifyObjectType : not implemented, see documentation for more details / implementation ideas");
	}


	/** full clone of the scene
	* not implemented 
	* need to clone the objectTypeLibrary, sceneGlobalPrior, InteractionPrior ...
	*/
	Self* GetSceneCopy() {
		throw DeformableModelException("ModifyObjectType : not implemented, see documentation for more details / implementation ideas");
	}

	/////** USEFULL ??? */
	////virtual SCENEDRAWMODE		GetSceneInsertionMode() = 0;
	////virtual TexturedPixelType	GetSceneDefaultAppearance() = 0;


	/** ObjectSet and Object */
	ObjectSetType* GetObjectSet() const { return const_cast<ObjectSetType*>(&m_objectSet); }

	/** Get an array containing all the objects in the scene, the array must be allocated first, using new */
	void GetObjectSetCopy(ObjectSetType &objectSet);

	/** Replace the object set by another 
	* keep the priors, objectTypeLibrary, ...
	*/
	void SetObjectSet(const ObjectSetType &objectset);

	/** Add a set of objects to the current collection, including any associated data */
	void AddObjectInSceneCollection(const ObjectSetType &objectSet);

	/** Add a set of raw objects to the current collection */
	void AddObjectCollection(const std::vector<DeformableObjectType *> &objectSet);

	/** Set a scene from another */
	void SetScene(Self* scene);

	/** Fuse an ObjectTypeLibrary with the current scene library */
	void FuseObjectTypesLibrary(ObjectTypesLibraryType *library);

	/** Get a pointer to the object type library */
	ObjectTypesLibraryType* GetObjectTypesLibrary() { return m_objectTypesLibrary.GetPointer(); } 

	/* Interaction Manager - integrated to the scene so it can cache the list of interacting objects during construction
	* \todo at the moment, the function add all objects to the interaction manager one by one ; interactions will therefore be computed twice...
	*       a solution would be to duplicate the AddObject method of the interaction manager ~> AddObjectIncrementally ; which would only test interaction with objects with a label lower than the current one... instead of all objects
	*/
	void SetInteractionManager(ObjectInteractionManagerType* i)	{ 
		m_interactionManager = i; m_interactionManager->SetScene(this);
		for (unsigned i=0 ; i<m_arrayObjects.size() ; i++) { if (m_arrayObjects[i].id!=0) { m_arrayObjects[i].interactionData.clear(); } }
		for (unsigned i=0 ; i<m_arrayObjects.size() ; i++) { if (m_arrayObjects[i].id!=0) { m_interactionManager->AddObject(&m_arrayObjects[i]); } }
	}
	ObjectInteractionManagerType* GetInteractionManager() const	{ return const_cast<ObjectInteractionManagerType*>(m_interactionManager.GetPointer()); }

	/** Scene Prior 
	* \todo as for SetInteractionManager, this class should update the object/scene prior...
	*/
	void SetGlobalPrior(ScenePriorType *p) { m_scenePrior = p; m_scenePrior->SetScene(this); }
	ScenePriorType* GetGlobalPrior() const { return const_cast<ScenePriorType*>(m_scenePrior.GetPointer()); }

	/** Get the global prior cost */
	inline double GetGlobalPriorCost()     { return m_scenePrior->GetValue(); }

	/** Set Object Prior normalization cost functions
	* works on the PDF_OBJECTLIKELIHOOD associated with the object type...
	* \todo: the current implementation is not flexible enough when multiple object types are present in the scene
	*        it would be necessary to define one function for each type of object ; or to use a common mechanism (e.g. always normalize based on the current PDF maximum loglikelihood, ...
	*/
	void SetObjectPriorNormalizationFunction(PriorNormalizationFunctionType *fct) { m_priorNormalizationFunction = fct; }

	/** Get Object Prior normalization cost functions */
	PriorNormalizationFunctionType * GetObjectPriorNormalizationFunction() { return m_priorNormalizationFunction; }

	/** Get Prior Cost for an object */
	inline double GetObjectPriorCost(IDType id) {
		if (!m_arrayObjects[id-1].objectData.priorCostFlag) { 
			m_arrayObjects[id-1].objectData.priorCost = m_priorNormalizationFunction->Evaluate( m_objectTypesLibrary->GetObjectPDF( m_arrayObjects[id-1].objectTypeId, PDF_OBJECTLIKELIHOOD )->GetLogLikelihood(m_arrayObjects[id-1].obj->GetParameters()) );
			m_arrayObjects[id-1].objectData.priorCostFlag = true;
		}
		return m_arrayObjects[id-1].objectData.priorCost; 
	}	

	/** Get Interaction Cost for an object */
	inline double GetObjectTotalInteractionCost(IDType id) {
		double sum = 0;
		InteractionDataType interactData;
		if ( m_arrayObjects[id-1].interactionData.size()>0 ) {			
			for (ObjectInteractionMapType::iterator mapit = m_arrayObjects[id-1].interactionData.begin() ; mapit!=m_arrayObjects[id-1].interactionData.end() ; mapit++) {
				if (! mapit->second.interactionCostFlag) { //should not be necessary... interaction should be always uptodate
					std::cout<<"interaction is not uptodate... SHOULD NEVER HAPPEN!"<<std::endl;
				}
				sum += mapit->second.interactionCost;
			}
		}
		return sum;
	}

	/** Invalidate all the datacosts of all objects - useful when the user wants to change the different metric after previous computations */
	void InvalidateObjectDataCosts() {
		for (unsigned i=0 ; i<m_arrayObjects.size() ; i++) { 
			m_arrayObjects[i].objectData.InvalidateData();
		}
	}

	///** Invalidate all In Context data */
	//void InvalidateObjectDataCosts() {
	//	for (unsigned i=0 ; i<m_arrayObjects.size() ; i++) { 
	//		m_arrayObjects[i].objectData.InvalidateInContextCache();
	//	}
	//}

	///** Invalidate all the datacosts of all objects - useful when the user wants to change the different metric after previous computations */
	//void InvalidateObjectDataCosts(IDType id) {
	//	if ( (id <=0) || (id > m_arrayObjects.size()) ) return;
	//	m_arrayObjects[id-1].objectData.dataCostFlag = false; 
	//	m_arrayObjects[id-1].objectData.offContextDataCostFlag = false;
	//}

	/** Get the vtk representation of the scene - through a rendered */
	vtkRenderer* GetSceneRenderer() {
		if (!m_rendererFlag) {
			m_renderer = vtkSmartPointer<vtkRenderer>::New(); //reset the renderer -- at least with an older vtk version, I experienced leakages and problems with renderers, add / removing objects, ...
			m_renderer->SetBackground(0,0,0);                 //                      it would be worth investigating this further.
			for (unsigned i=0 ; i<m_arrayObjects.size() ; i++) {
				if ( m_arrayObjects[i].id == 0 ) continue;
				if (!m_arrayObjects[i].actorFlag) {
					vtkSmartPointer<vtkPolyDataMapper> map = vtkSmartPointer<vtkPolyDataMapper>::New(); 
					map->SetInput( m_arrayObjects[i].obj->GetObjectAsVTKPolyData() ); 
					m_arrayObjects[i].actor->SetMapper(map); 
					unsigned tmpid = m_arrayObjects[i].id%(255*255), R = m_arrayObjects[i].id/(255*255), G = tmpid/255, B = tmpid%255;
					m_arrayObjects[i].actor->GetProperty( )->SetColor(R/255.0,G/255.0,B/255.0); 
					m_arrayObjects[i].actorFlag = true;
				}
				m_renderer->AddActor(m_arrayObjects[i].actor);
			}						
			m_rendererFlag = true;
		}
		return m_renderer.GetPointer();
	}

	/** Get the itk::LabelMap representing the scene */
	LabelMapType* GetSceneAsLabelMap() {
		if (!m_labelMapFlag) {
			m_labelMap->ClearLabels();
			m_labelMap->SetRegions( m_sceneImageRegion );
			m_labelMap->SetSpacing( m_sceneSpacing );
			m_labelMap->SetOrigin( m_sceneOrigin );
			for (unsigned i=0 ; i<m_arrayObjects.size() ; i++) {
				if ( m_arrayObjects[i].id == 0 ) continue;
				//warning, this may need to be modified conditionally on the boundary conditions (in particular periodic boundaries)
				InsertSingleObjectLabelMapIntoAnother<DeformableObjectType::LabelMapType, LabelMapType>(m_arrayObjects[i].obj->GetObjectAsLabelMap(), m_labelMap, m_arrayObjects[i].id);
				m_arrayObjects[i].sceneOffContextLabelObject = m_labelMap->GetLabelObject( m_arrayObjects[i].id );
				m_arrayObjects[i].sceneOffContextLabelObjectFlag = true;
			}
			m_labelMapFlag = true;
		}
		return m_labelMap.GetPointer();
	}


	/** Get a single vtkPolyData representing the scene, appending all entries */
	vtkPolyData* GetSceneAsVTKPolyData() {
		m_appendPolyDataFilter->RemoveAllInputs();
		for (unsigned i=0 ; i<m_arrayObjects.size() ; i++) {
			if ( m_arrayObjects[i].id != 0 ) {
				m_appendPolyDataFilter->AddInput( m_arrayObjects[i].obj->GetObjectAsVTKPolyData() );
			}
		}
		m_appendPolyDataFilter->Update();
		return m_appendPolyDataFilter->GetOutput();
	}

	/** */
	virtual vtkPolyData* GetSceneAsTexturedVTKPolyData() {
		throw DeformableModelException("GetSceneAsTexturedVTKPolyData NOT IMPLEMENTED YET!");
	}

	/** the default implementation thresholds the label image generated by GetSceneAsLabelImage() */
	virtual BinaryImageType* GetSceneAsBinaryImage() {
		if (!m_binaryImageFlag) {
			itk::BinaryThresholdImageFilter<LabelImageType, BinaryImageType>::Pointer thresholdFilter = itk::BinaryThresholdImageFilter<LabelImageType, BinaryImageType>::New();
			thresholdFilter->SetInput( GetSceneAsLabelImage() );
			thresholdFilter->SetLowerThreshold(1);
			thresholdFilter->SetInsideValue(255); thresholdFilter->SetOutsideValue(0);
			thresholdFilter->Update();
			m_binaryImage = thresholdFilter->GetOutput();
			m_binaryImageFlag = true;
		}		
		return m_binaryImage.GetPointer();
	}
	/** */
	virtual TexturedImageType* GetSceneAsTexturedImage() {
		throw DeformableModelException("GetSceneAsTexturedImage NOT IMPLEMENTED YET!");
	}
	
	/** */
	LabelImageType* GetSceneAsLabelImage() {
		if (!m_labelImageFlag) {			
			GetSceneAsLabelMap(); //make sure the label map is uptodate
			typedef itk::LabelMapToLabelImageFilter< LabelMapType, LabelImageType> LabelMapToLabelImageFilterType;
			LabelMapToLabelImageFilterType::Pointer labelMapToLabelImageFilter = LabelMapToLabelImageFilterType::New();
	
			labelMapToLabelImageFilter->SetInput( m_labelMap );
			labelMapToLabelImageFilter->Update();
			m_labelImage = labelMapToLabelImageFilter->GetOutput();

			m_labelImageFlag = true;
		}
		return m_labelImage.GetPointer();
	}

	/** Opens a window for interactive visualization of the scene */
	void InteractiveSceneVisualization();

	/** Load/Save the scene information to a file -> TODO 
	* NOT IMPEMENTED
	* requirements: find a way to write / read the type of object ...
	*/
	bool SaveSceneToFile(std::string filename);
	// Actually, in this case, all types related to scene have already been decided... so it is unclear what should be done here... 
	bool LoadSceneFromFile(std::string filename);


	/** Connect a sensor to the scene, so it can be informed of modifications of the scene 
	* checks if the sensor is already connected, and ignore the command in this case.
	*/
	void ConnectSensor(SensorInterfaceType *s) {
		bool alreadyThere = false;
		for (unsigned i=0 ; i<m_listConnectedSensors.size() ; i++) {
			if (m_listConnectedSensors[i]==s) { alreadyThere=true; return; }
		}
		if (!alreadyThere) m_listConnectedSensors.push_back(s);
	}

	/** Disconnect a sensor from the scene */
	void DisconnectSensor(SensorInterfaceType *s) {
		for (unsigned i=0 ; i<m_listConnectedSensors.size() ; i++) {
			if (m_listConnectedSensors[i]==s) {
				m_listConnectedSensors[i] = m_listConnectedSensors.back();
				m_listConnectedSensors.pop_back();
				break;
			}
		}
	}

	/** Get list of connected sensors 
	* \todo output should be const
	*/
	SensorInterfaceListType GetListOfConnectedSensors() {
		return m_listConnectedSensors;
	}


	unsigned long m_addCounter, m_remCounter, m_modCounter;
protected:	
	BaseScene();
	virtual ~BaseScene() { };	

	ObjectInScene m_tmpObject;

	IDType m_maxIDReached, m_nbObjectsInScene;
	ObjectSetType m_arrayObjects;
	std::deque<IDType> m_freedIDs;

	typename ObjectTypesLibraryType::Pointer		m_objectTypesLibrary;	//keeps track of the different types of objects, and all associated pdfs
	typename ObjectInteractionManagerType::Pointer	m_interactionManager;	//
	typename ScenePriorType::Pointer				m_scenePrior;
	PriorNormalizationFunctionType::Pointer         m_priorNormalizationFunction; //

	unsigned int m_totalNumberOfParameters;

	vnl_vector<double> m_sceneBBox;
	typename BinaryImageType::RegionType	m_sceneImageRegion;
	typename BinaryImageType::SpacingType	m_sceneSpacing;
	typename BinaryImageType::PointType		m_sceneOrigin;
	typename BinaryImageType::SizeType		m_sceneSize;

	typename LabelMapType::Pointer      m_labelMap;      bool m_labelMapFlag; //don't forget to invalidate it in DrawObject/EraseObject in the corresponding child classes, maintained by the LabelMapScene but not by the VTKScene
	vtkSmartPointer<vtkRenderer>        m_renderer;      bool m_rendererFlag; //don't forget to invalidate it in DrawObject/EraseObject in the corresponding child classes, maintained by the VTKScene but not by the LabelMapScene
	vtkSmartPointer<vtkAppendPolyData>  m_appendPolyDataFilter;
	typename BinaryImageType::Pointer   m_binaryImage;   bool m_binaryImageFlag;
	typename LabelImageType::Pointer    m_labelImage;    bool m_labelImageFlag;
	//typename TexturedImageType::Pointer m_texturedImage; bool m_texturedImageFlag;

	SceneBoundaryConditionCode m_BoundaryConditionManagement; //this should also affect the interaction manager!
	SceneObjectInsertionPolicyCode m_insertionPolicy;

	// make these private, called by GetSceneAsLabelImage / BinaryImage / TexturesImage ...
	//	/** Allocate the memory for the images - depends on which ones are requested...
	// * \todo make the interface less tied to an image representation - rather consider a physical bounding box in the base class ; not allocating memory for the image
	//*/
	//void AllocateImage(vnl_vector<double> physicalBoundingBox, vnl_vector<double> spacing) = 0;
	//void AllocateImage(vnl_vector<double> physicalBoundingBox, double s)  { vnl_vector<double> sp(Dimension); sp.fill(s);	AllocateImage(physicalBoundingBox, sp); }



	// "behind the scenes" object manipulation, protected from the public interface
	//tell if the behavior is acceptable or not, depending on the specific scene policy (boundary conditions, object intersections...) ; typically called just after
	//perhaps this can be implemented in the base class ...
	bool CodeToBool(SceneObjectOverlapCode code) const {
		switch(code) {	
		case OBJECT_FULLYOUTSIDE: return false;
		case OBJECT_OVERLAPOBJECT:
		case OBJECT_OVERLAPOBJECTANDSCENE: 
		case OBJECT_NOOVERLAP:
		case OBJECT_OVERLAPSCENE: return true;
		default: return false;	//should never happen
		}		
	}

	// Allocate memory for internal representations
	virtual void AllocateFromPhysicalDimension() {}

	// draws the object ; 
	virtual void DrawObjectInScene(IDType id)    = 0;	//this method be implemented only in the leaf class when all insertion policies have been defined (do I write in the label image, in both the label and the textured image, do I authorize object overlaps, etc...)
	virtual void EraseObjectFromScene(IDType id) = 0;
	virtual void UpdateObjectInScene(IDType id, ObjectInScene *newObject) = 0;	//probably specific to the type of scene (image / meshes) ; used to complement a shape that used to be overlapping with something that got erased...


	// Track the modifications in the scene (in particular to inform the sensor which regions have changed, and need to be updated)
	SensorInterfaceListType m_listConnectedSensors;
	void TrackModifications(typename BinaryImageType::RegionType region);

private:
	BaseScene(const Self&);                 //purposely not implemented
	const Self & operator=( const Self & ); //purposely not implemented

	/** private class in charge of finding an available ID 
	* it adds empty entries into the m_arrayObjects and updates m_maxIDReached if necessary 
	*/
	IDType FindAvailableID(IDType preferedID = 0);
};



/** \class ObjectIterator
* \brief iterator traversing the set of objects in the scene
*/
template<class SceneType>
class SceneObjectIterator {
public:
	SceneObjectIterator(SceneType *scene) { m_scene = scene; m_iterator = 0; }
	/** Move the iterator to the first object */
	void GoToBegin() {
		m_iterator = 0;
		if (m_scene->m_arrayObjects.size()==0) return;
		while ( m_scene->m_arrayObjects[m_iterator].id==0 ) { if ( ++m_iterator==m_scene->m_arrayObjects.size() ) return; }
	}
	/** Checks whether the iterator is at the end (which is an invalid position!) */
	bool IsAtEnd() { 
		if ( m_iterator >= m_scene->m_arrayObjects.size() ) return true; 
		return false;
	}
	/** Checks whether the iterator is at the beginning */
	bool IsAtBegin() { 
		//if ( m_iterator == 0 ) return true;
		//check if there are positions lower than this one with non-zero id
		for (unsigned i=0 ; i<m_iterator ; i++) { if ( m_scene->m_arrayObjects[i].id!=0 ) return false; }
		return true;
	}
	/** Advance the iterator to the next object */
	SceneObjectIterator operator++() { 
		//make sure we are not already at the end
		if (m_iterator>=m_scene->m_arrayObjects.size()-1) { m_iterator = m_scene->m_arrayObjects.size(); return *this; }

		//advance the iterator until a valid object is found, or the end of the list is reached
		while ( m_scene->m_arrayObjects[++m_iterator].id==0 ) { 
			if (m_iterator>=m_scene->m_arrayObjects.size()-1) {	
				m_iterator = m_scene->m_arrayObjects.size(); 
				return *this; 
			}
		}
		return *this; 
	}
	/** Move the iterator to the previous object */
	SceneObjectIterator operator--() { 
		//make sure we are not already at the beginning
		SceneType::IDType tmp_begin = 0; while ( m_scene->m_arrayObjects[tmp_begin].id==0 ) { if ( ++tmp_begin==m_scene->m_arrayObjects.size() ) return *this; }
		if (m_iterator==tmp_begin) { return *this; }
		
		//rewind the iterator until a valid object is found, or the beginning of the list is reached
		while ( m_scene->m_arrayObjects[--m_iterator].id==0 ) { 
			if (m_iterator<=tmp_begin) { return *this; }
		}
		return *this;
	}
	/** Get a pointer to the object structure within the scene */
	inline typename SceneType::ObjectInScene* GetObjectInScene() { return (&(m_scene->m_arrayObjects[m_iterator])); }
	/** Get a pointer to the parametric object */
	inline typename SceneType::DeformableObjectType* GetObject() { return (m_scene->m_arrayObjects[m_iterator].obj); }
	/** Returns the id (or label) of the current object */
	inline typename SceneType::IDType GetID() { return (m_iterator+1); }

	/** Advance 1 time, similar as ++ */
	inline void Advance() {
		//make sure we are not already at the end
		if (m_iterator>=m_scene->m_arrayObjects.size()-1) { m_iterator = m_scene->m_arrayObjects.size(); return; }

		//advance the iterator until a valid object is found, or the end of the list is reached
		while ( m_scene->m_arrayObjects[++m_iterator].id==0 ) { 
			if (m_iterator>=m_scene->m_arrayObjects.size()-1) {	
				m_iterator = m_scene->m_arrayObjects.size(); 
				return; 
			}
		}
		return; 
	}

	/** Advance n times */
	void Advance(unsigned int n) {
		for (unsigned i=0 ; i<n ; i++) Advance();
	}
private:
	typename SceneType::IDType m_iterator;
	typename SceneType* m_scene;
};


} // namespace psciob


#include "BaseScene.txx"

#endif /* BaseScene_H_ */





