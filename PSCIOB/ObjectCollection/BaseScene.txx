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
* \file BaseScene.txx
* \author Rémi Blanc 
* \date 29. February 2012
*/


#include "BaseScene.h"
#include "ObjectsWithoutInteraction.h"	//default, in case nothing is provided...
#include "SceneFlatPrior.h"

namespace psciob {


//
// Constructor
//
template<unsigned int VDimension, class TAppearance, class TObjectId, class TAssociatedData, class TInteractionData>
BaseScene<VDimension, TAppearance, TObjectId, TAssociatedData, TInteractionData>::BaseScene() : m_maxIDReached(0), m_nbObjectsInScene(0), m_totalNumberOfParameters(0),
m_labelMapFlag(false), m_rendererFlag(false), m_labelImageFlag(false) {
	m_appendPolyDataFilter = vtkSmartPointer<vtkAppendPolyData>::New();
	m_addCounter=0; m_remCounter=0; m_modCounter=0;
	//new object types library
	m_objectTypesLibrary = ObjectTypesLibraryType::New();

	m_BoundaryConditionManagement = BOUNDARYCONDITIONS_VOID; //default: no boundaries conditions
	m_insertionPolicy = ACCEPTALL; //default: accept all objects provided they are not fully outside the scene

	m_trackChanges = false;

	//main scene representations
	m_labelMap = LabelMapType::New();
	m_renderer = vtkSmartPointer<vtkRenderer>::New(); m_renderer->SetBackground(0,0,0);

	//interaction manager
	m_interactionManager = static_cast<ObjectInteractionManagerType*>(ObjectsWithoutInteraction<Self>::New().GetPointer());	//by default: no interactions
	m_interactionManager->SetScene(this);

	//global prior
	m_scenePrior = static_cast<ScenePriorType*>(SceneFlatPrior<Self>::New().GetPointer());	//by default: no prior on the scene statistics.
	m_scenePrior->SetScene(this);

	//normalization function for the prior, by default, it is the identity function...
	m_priorNormalizationFunction = GPF_IdentityFunction<double, double>::New();

}


//
// SetPhysicalDimensions
//
template<unsigned int VDimension, class TAppearance, class TObjectId, class TAssociatedData, class TInteractionData>
void 
BaseScene<VDimension, TAppearance, TObjectId, TAssociatedData, TInteractionData>::SetPhysicalDimensions(const vnl_vector<double> &physicalBoundingBox, const vnl_vector<double> &spacing) {	
	ITKGridImageInformationFromPhysicalBoundingBoxAndSpacing<Dimension>(physicalBoundingBox, spacing, &m_sceneOrigin, &m_sceneImageRegion);
	m_sceneSize = m_sceneImageRegion.GetSize();
	for (unsigned i=0 ; i<Dimension ; i++) { m_sceneSpacing[i] = spacing(i); }
	m_sceneBBox = BoundingBoxFromITKImageInformation<BinaryImageType>( m_sceneOrigin, m_sceneSpacing, m_sceneImageRegion );

	m_labelMap->SetRegions( m_sceneImageRegion );
	m_labelMap->SetSpacing( m_sceneSpacing );
	m_labelMap->SetOrigin( m_sceneOrigin );
}

template<unsigned int VDimension, class TAppearance, class TObjectId, class TAssociatedData, class TInteractionData>
void 
BaseScene<VDimension, TAppearance, TObjectId, TAssociatedData, TInteractionData>::SetPhysicalDimensions(const vnl_vector<double> &physicalBoundingBox, double spacing = 1) {
	vnl_vector<double> sp(Dimension); sp.fill(spacing);
	SetPhysicalDimensions(physicalBoundingBox, sp);
}


//
// TestObjectInsertionAcceptance_Internal
//
template<unsigned int VDimension, class TAppearance, class TObjectId, class TAssociatedData, class TInteractionData>
inline
bool 
BaseScene<VDimension, TAppearance, TObjectId, TAssociatedData, TInteractionData>::TestObjectInsertionAcceptance_Internal(ObjectInScene *object) { 
	switch (m_insertionPolicy) {
		case ACCEPTALL: //accept all objects, as long as they are not fully outside
			return (!TestObjectFullyOutside_Internal(object)); 
			break;
		case FORBIDPARTIALLYOUTSIDE: //accept all objects, as long as they are fully inside the scene
			//IDEA: expand this to use a mask?
			return (TestObjectFullyInside_Internal(object));
			break;
		case FORBIDALLOVERLAPS: //check for overlaps, and for boundary conditions...
			return ( CodeToBool(TestOverlap_Internal(object)) ); 
			break;
		default: return false; //never happens
			break;
	}
}

//
// TestObjectInsertionAcceptanceIgnoringObjectID_Internal
//
template<unsigned int VDimension, class TAppearance, class TObjectId, class TAssociatedData, class TInteractionData>
inline
bool 
BaseScene<VDimension, TAppearance, TObjectId, TAssociatedData, TInteractionData>::TestObjectInsertionAcceptanceIgnoringObjectID_Internal(ObjectInScene *object, IDType id) { 
	switch (m_insertionPolicy) {
		case ACCEPTALL: //accept all objects, as long as they are not fully outside
			return (!TestObjectFullyOutside_Internal(object)); 
			break;
		case FORBIDPARTIALLYOUTSIDE: //accept all objects, as long as they are fully inside the scene
			//IDEA: expand this to use a mask?
			return (TestObjectFullyInside_Internal(object));
			break;
		case FORBIDALLOVERLAPS: //check for overlaps, and for boundary conditions...
			return ( CodeToBool(TestOverlapIgnoringObjectID_Internal(object, id)) ); 
			break;
		default: return false; //never happens
			break;
	}
}

//
// TestObjectFullyInside
//

template<unsigned int VDimension, class TAppearance, class TObjectId, class TAssociatedData, class TInteractionData>
inline
bool 
BaseScene<VDimension, TAppearance, TObjectId, TAssociatedData, TInteractionData>::TestObjectFullyInside_Internal(ObjectInScene *object) {
	return TestBoundingBoxFullyInsideAnother( object->obj->GetPhysicalBoundingBox(), m_sceneBBox );
}


//
// ClearScene
//
template<unsigned int VDimension, class TAppearance, class TObjectId, class TAssociatedData, class TInteractionData>
void 
BaseScene<VDimension, TAppearance, TObjectId, TAssociatedData, TInteractionData>::ClearScene() {
	RemoveAllObjects();
	m_objectTypesLibrary->Clear();
}

//
// RemoveAllObjects
//
template<unsigned int VDimension, class TAppearance, class TObjectId, class TAssociatedData, class TInteractionData>
void 
BaseScene<VDimension, TAppearance, TObjectId, TAssociatedData, TInteractionData>::RemoveAllObjects() {
	SceneObjectIterator<Self> it(this);
	for (it.GoToBegin() ; !it.IsAtEnd() ; ++it) this->RemoveObject(it.GetID());
	m_maxIDReached = 0;
	m_nbObjectsInScene = 0;
	m_totalNumberOfParameters = 0;
	m_arrayObjects.clear();
	m_freedIDs.clear();

	m_scenePrior->Clear();
	m_interactionManager->Clear();	
}

//
// FirstID
//
template<unsigned int VDimension, class TAppearance, class TObjectId, class TAssociatedData, class TInteractionData>
TObjectId 
BaseScene<VDimension, TAppearance, TObjectId, TAssociatedData, TInteractionData>::FirstID() {
	if (m_nbObjectsInScene==0) return 0;
	unsigned int i = 0;
	while ( m_arrayObjects[i].id==0 ) { if ( ++i==m_arrayObjects.size() ) return 0; }
	return m_arrayObjects[i].id;
}


//
// PrintInfoObject
//
template<unsigned int VDimension, class TAppearance, class TObjectId, class TAssociatedData, class TInteractionData>
void 
BaseScene<VDimension, TAppearance, TObjectId, TAssociatedData, TInteractionData>::PrintInfoObject(IDType id) const { 
	if ( (id <=0) || (id >= m_arrayObjects.size()) ) { std::cout<<"Invalid ID: "<<id<<std::endl; return; }
	if ( m_arrayObjects[id-1].id == 0 ) { std::cout<<"No Object with ID: "<<id<<std::endl; return; }
	std::cout<<"Object id: "<<m_arrayObjects[id-1].id<<", type index: "<<m_arrayObjects[id-1].indexObjectType<<", object name: "<<m_arrayObjects[id-1].obj->GetClassName()<<std::endl; 
}

//
// GetObject
//
template<unsigned int VDimension, class TAppearance, class TObjectId, class TAssociatedData, class TInteractionData>
inline typename BaseScene<VDimension, TAppearance, TObjectId, TAssociatedData, TInteractionData>::ObjectInScene* 
BaseScene<VDimension, TAppearance, TObjectId, TAssociatedData, TInteractionData>::GetObject(IDType id) { 
	if ( (id <=0) || (id > m_arrayObjects.size()) ) return NULL;
	if ( m_arrayObjects[id-1].id == 0 ) return NULL;
	return (&(m_arrayObjects[id-1])); 
}

//
//GetObjectLabelObject
//
template<unsigned int VDimension, class TAppearance, class TObjectId, class TAssociatedData, class TInteractionData>
typename BaseScene<VDimension, TAppearance, TObjectId, TAssociatedData, TInteractionData>::LabelObjectType* 
BaseScene<VDimension, TAppearance, TObjectId, TAssociatedData, TInteractionData>::GetObjectLabelObject(IDType id) {
	if ( (id <=0) || (id > m_arrayObjects.size()) ) return NULL;
	if ( m_arrayObjects[id-1].id == 0 ) return NULL;
	if (!m_arrayObjects[id-1].sceneOffContextLabelObjectFlag) {
		LabelMapType::Pointer labelMap = LabelMapType::New();
		labelMap->SetRegions( m_sceneImageRegion );
		labelMap->SetSpacing( m_sceneSpacing );
		labelMap->SetOrigin( m_sceneOrigin );
		InsertSingleObjectLabelMapIntoAnother<DeformableObjectType::LabelMapType, LabelMapType>(m_arrayObjects[id-1].obj->GetObjectAsLabelMap(), labelMap, id);
		m_arrayObjects[id-1].sceneOffContextLabelObject = m_labelMap->GetLabelObject( id );
		m_arrayObjects[id-1].sceneOffContextLabelObjectFlag = true;
	}
	return m_arrayObjects[id-1].sceneOffContextLabelObject;
}

/** Get a pointer to a (offcontext = ignoring any potential overlaps) label map containing the 
* labelObject corresponding to the requested object...
*/
template<unsigned int VDimension, class TAppearance, class TObjectId, class TAssociatedData, class TInteractionData>
void
BaseScene<VDimension, TAppearance, TObjectId, TAssociatedData, TInteractionData>::GetObjectLabelMap(IDType id, LabelMapType *labelMap) {
	if ( (id <=0) || (id > m_arrayObjects.size()) ) return;
	if ( m_arrayObjects[id-1].id == 0 ) return;
	labelMap->ClearLabels();
	labelMap->SetRegions( m_sceneImageRegion );
	labelMap->SetSpacing( m_sceneSpacing );
	labelMap->SetOrigin( m_sceneOrigin );
	if (!m_arrayObjects[id-1].sceneOffContextLabelObjectFlag) {
		InsertSingleObjectLabelMapIntoAnother<DeformableObjectType::LabelMapType, LabelMapType>(m_arrayObjects[id-1].obj->GetObjectAsLabelMap(), labelMap, id);
		m_arrayObjects[id-1].sceneOffContextLabelObject = m_labelMap->GetLabelObject( id );
		m_arrayObjects[id-1].sceneOffContextLabelObjectFlag = true;
	}
	else { labelMap->AddLabelObject(m_arrayObjects[id-1].sceneOffContextLabelObject); }
}

//
//
// FindAvailableID (private)
//
//

template<unsigned int VDimension, class TAppearance, class TObjectId, class TAssociatedData, class TInteractionData>
inline
TObjectId
BaseScene<VDimension, TAppearance, TObjectId, TAssociatedData, TInteractionData>::FindAvailableID(TObjectId preferedID) {
	TObjectId id;
	if ( preferedID > m_maxIDReached ) { 
		id = m_maxIDReached+1;		
		while (id!=preferedID) { 
			m_arrayObjects.push_back( ObjectInScene() );	//then create some empty entries between the current max and the preferedID
			m_freedIDs.push_back(id);					//and indicate it at the end of the prefered insertion locations
		}
		m_maxIDReached = preferedID;
		return preferedID;
	}
	else {
		if (!m_freedIDs.empty()) {
			//give a chance to the preferedID, in case it is at the end of the list
			if ( (preferedID!=0) && (preferedID==m_freedIDs.back()) )  { m_freedIDs.pop_back();  return preferedID; }
			//otherwise, just take the first available item and don't care about the preferedID
			id = m_freedIDs.front(); m_freedIDs.pop_front(); return id;
		}			
		//reaching this point means that the list of freedIDs is empty and no id could be found yet
		m_arrayObjects.push_back( ObjectInScene() );
		return (++m_maxIDReached); //this increments m_maxIDReached, and return the new value id		
	}
}

//
//
// AddObject
//
//

template<unsigned int VDimension, class TAppearance, class TObjectId, class TAssociatedData, class TInteractionData>
TObjectId
BaseScene<VDimension, TAppearance, TObjectId, TAssociatedData, TInteractionData>::AddObject(DeformableObjectType *object, IDType proposedID) {
	m_addCounter++;
	//make sure the object has the expected spacing
	object->SetImageSpacing( m_sceneSpacing );
	//Create a structure hosting the object candidate
	ObjectInScene newObject;
	newObject.objectTypeId = m_objectTypesLibrary->RegisterObjectType(object);
	newObject.obj = object->CreateClone();

	//Add to the list of objects in the scene
	newObject.id = FindAvailableID(proposedID);
	if (!TestObjectInsertionAcceptance_Internal(&newObject)) { //TODO: for pixelsetscene, test that the resolution is not too coarse, and that the object really leaves a trace in at least 1 pixel ; currently it can happen that it does not...
		//remove the object if it cannot be drawn and exit
		m_freedIDs.push_front(newObject.id);
		return 0;
	}
	m_arrayObjects[newObject.id-1] = newObject;

	//draw it in the scene, and update the total number of parameters...
	DrawObjectInScene(newObject.id);

	m_nbObjectsInScene++;
	m_totalNumberOfParameters += newObject.obj->GetNumberOfParameters();

	//update the scene prior and object interaction manager
	m_scenePrior->AddObject(&m_arrayObjects[newObject.id-1]);
	m_interactionManager->AddObject(&m_arrayObjects[newObject.id-1]);

	//track changes for the sensor
	if (m_trackChanges) {
		LabelImageType::RegionType	regionInScene;
		ITKImageRegionFromBoundingBox<Dimension>(m_sceneSpacing, m_sceneOrigin, m_sceneSize, m_arrayObjects[newObject.id-1].obj->GetPhysicalBoundingBox(), &regionInScene);
		TrackModifications(regionInScene);
	}

	//any other action to take? if some actions are added here, their 'undoing' shall be also added to the RemoveObject method

	return newObject.id;
}


//
//
// RemoveObject
//
//

template<unsigned int VDimension, class TAppearance, class TObjectId, class TAssociatedData, class TInteractionData>
bool 
BaseScene<VDimension, TAppearance, TObjectId, TAssociatedData, TInteractionData>::RemoveObject(IDType id) {
	if ( (id <=0) || (id > m_arrayObjects.size()) ) return false;
	if ( m_arrayObjects[id-1].id == 0 ) return false;
	m_remCounter++;
	//track changes for the sensor
	if (m_trackChanges) {
		LabelImageType::RegionType regionInScene;
		ITKImageRegionFromBoundingBox<Dimension>(m_sceneSpacing, m_sceneOrigin, m_sceneSize, m_arrayObjects[id-1].obj->GetPhysicalBoundingBox(), &regionInScene);
		TrackModifications(regionInScene);
	}

	//update the scene prior and object interaction manager
	m_scenePrior->RemoveObject(&m_arrayObjects[id-1]);
	m_interactionManager->RemoveObject(&m_arrayObjects[id-1]);

	//
	EraseObjectFromScene(id);

	//delete this object from the container and push the id in the list of freed IDs
	m_nbObjectsInScene--;
	m_totalNumberOfParameters -= m_arrayObjects[id-1].obj->GetNumberOfParameters();
	//m_arrayObjects[id-1].id = 0;
	//m_arrayObjects[id-1].obj = 0; //dereference the object so that memory may be freed immediatly
	m_arrayObjects[id-1] = ObjectInScene();
	m_freedIDs.push_front(id);

	return true;
}


//
//
// ModifyObjectParameters
//
//

template<unsigned int VDimension, class TAppearance, class TObjectId, class TAssociatedData, class TInteractionData>
bool 
BaseScene<VDimension, TAppearance, TObjectId, TAssociatedData, TInteractionData>::ModifyObjectParameters(IDType id, const vnl_vector<double> &p) {
	//assume the id is correct?
//std::cout<<"ModifyObjectParameters: id = "<<id<<", old params = "<<m_arrayObjects[id-1].obj->GetParameters()<<", new params = "<<p<<std::endl;

	if ( (id <=0) || (id > m_arrayObjects.size()) ) return false;
	if ( m_arrayObjects[id-1].id == 0 ) return false;
	m_modCounter++;
	//test if the parameters are different at all and return if nothing needs to be updated
	if ( (m_arrayObjects[id-1].obj->GetParameters() - p).inf_norm() <TINY ) return true;

	//Create a new structure hosting the object candidate
	ObjectInScene newObject;
	newObject.objectTypeId = m_arrayObjects[id-1].objectTypeId;
	newObject.priority = m_arrayObjects[id-1].priority;
	newObject.id = id;
	newObject.obj = m_arrayObjects[id-1].obj->CreateCopy();
	//copy the objectData and invalidate what needs to be invalidated! this is important to safeguard data that should not be modified regardless of object displacements...
	newObject.objectData =  m_arrayObjects[id-1].objectData; newObject.objectData.InvalidateData();	
	if (!newObject.obj->SetParameters( p )) return false;

	//Check for insertion acceptance
	if (!TestObjectInsertionAcceptanceIgnoringObjectID_Internal(&newObject, id)) return false;
	//ok for the modification 
	//track changes for the sensor
	if (m_trackChanges) {
		LabelImageType::RegionType	regionInScene;
		ITKImageRegionFromBoundingBox<Dimension>(m_sceneSpacing, m_sceneOrigin, m_sceneSize, m_arrayObjects[id-1].obj->GetPhysicalBoundingBox(), &regionInScene);
		TrackModifications(regionInScene);
		ITKImageRegionFromBoundingBox<Dimension>(m_sceneSpacing, m_sceneOrigin, m_sceneSize, newObject.obj->GetPhysicalBoundingBox(), &regionInScene);
		TrackModifications(regionInScene);
	}

	//inform the scene prior and interaction manager
	m_scenePrior->ModifyObjectParameters(&m_arrayObjects[id-1], &newObject);
	m_interactionManager->ModifyObjectParameters(&m_arrayObjects[id-1], &newObject);

	//Update the object representation in the scene
	UpdateObjectInScene(id, &newObject);
	m_arrayObjects[id-1] = newObject;
	
	return true;
}

//
// GetObjectSetCopy
//
template<unsigned int VDimension, class TAppearance, class TObjectId, class TAssociatedData, class TInteractionData>
void 
BaseScene<VDimension, TAppearance, TObjectId, TAssociatedData, TInteractionData>::GetObjectSetCopy(ObjectSetType &objectSet) {
	ObjectInScene newObject;
	objectSet.clear();
	for (unsigned i=0 ; i<m_arrayObjects.size() ; i++) {
		if (m_arrayObjects[i].id == i+1) {
			newObject.id = m_arrayObjects[i].id;
			newObject.obj = m_arrayObjects[i].obj->CreateClone();
			newObject.objectTypeId = m_arrayObjects[i].objectTypeId;
			newObject.priority = m_arrayObjects[i].priority;
			newObject.objectData = m_arrayObjects[i].objectData;
			newObject.interactionData = m_arrayObjects[i].interactionData;
			objectSet.push_back(newObject);
		}
		else {
			newObject = ObjectInScene();
			objectSet.push_back(newObject);
		}
	}
}


//
//SetObjectSet
//add a bunch of objects to the current scene
//
template<unsigned int VDimension, class TAppearance, class TObjectId, class TAssociatedData, class TInteractionData>
void 
BaseScene<VDimension, TAppearance, TObjectId, TAssociatedData, TInteractionData>::SetObjectSet(const ObjectSetType &objectSet) {
	this->RemoveAllObjects();
	this->AddObjectInSceneCollection(objectSet);
}
//
//add a bunch of objects to the current scene
//
template<unsigned int VDimension, class TAppearance, class TObjectId, class TAssociatedData, class TInteractionData>
void 
BaseScene<VDimension, TAppearance, TObjectId, TAssociatedData, TInteractionData>::AddObjectInSceneCollection(const ObjectSetType &objectSet) {
	IDType id;
	for (unsigned i=0 ; i<objectSet.size() ; i++) { 
		if (objectSet[i].id!=0) {
			id = AddObject( objectSet[i].obj ); 
			if (id!=0) {
				m_arrayObjects[id-1].objectData = objectSet[i].objectData;
				m_arrayObjects[id-1].interactionData = objectSet[i].interactionData;
			}
		}
	}
}

//
//add a bunch of objects to the current scene
//
template<unsigned int VDimension, class TAppearance, class TObjectId, class TAssociatedData, class TInteractionData>
void 
BaseScene<VDimension, TAppearance, TObjectId, TAssociatedData, TInteractionData>::AddObjectCollection(const std::vector<DeformableObjectType *> &objectSet) {
	IDType id;
	for (unsigned i=0 ; i<objectSet.size() ; i++) { 
		if (objectSet[i].id!=0) {
			id = AddObject( objectSet[i].obj ); 
		}
	}
}

//
//SetScene
//clear and set from another
//
template<unsigned int VDimension, class TAppearance, class TObjectId, class TAssociatedData, class TInteractionData>
void 
BaseScene<VDimension, TAppearance, TObjectId, TAssociatedData, TInteractionData>::SetScene(Self* scene) {
	ClearScene();
	FuseObjectTypesLibrary( scene->GetObjectTypesLibrary() );
	SetInteractionManager( scene->GetInteractionManager() ); //or rather make a copy?
	SetGlobalPrior( scene->GetGlobalPrior() ); //or rather make a copy?

	AddObjectCollection( scene->GetObjectSet() );
}

//
// Library of object types
//
template<unsigned int VDimension, class TAppearance, class TObjectId, class TAssociatedData, class TInteractionData>
void 
BaseScene<VDimension, TAppearance, TObjectId, TAssociatedData, TInteractionData>::FuseObjectTypesLibrary(ObjectTypesLibraryType *library) {
	for (unsigned i=0 ; i<library->GetNumberOfEntries() ; i++) { 
		m_objectTypesLibrary->RegisterEntry( library->GetObjectEntry(i) ); 
	}
}


//
// InteractiveSceneVisualization
//
template<unsigned int VDimension, class TAppearance, class TObjectId, class TAssociatedData, class TInteractionData>
void 
BaseScene<VDimension, TAppearance, TObjectId, TAssociatedData, TInteractionData>::InteractiveSceneVisualization() {
	vtkRenderer *renderer = vtkRenderer::New();
	renderer->SetBackground(0.7,0.7,0.7); 
	vtkRenderWindow *renderWindow = vtkRenderWindow::New(); 
	renderWindow->AddRenderer(renderer); 
	vtkRenderWindowInteractor *iren = vtkRenderWindowInteractor::New(); 
	iren->SetRenderWindow(renderWindow); 
	for (unsigned i=0 ; i<m_arrayObjects.size() ; i++) {
		if (m_arrayObjects[i].id==0) continue;
		vtkSmartPointer<vtkPolyDataMapper> map = vtkSmartPointer<vtkPolyDataMapper>::New(); 
		map->SetInput( m_arrayObjects[i].obj->GetObjectAsVTKPolyData() ); 
		vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New(); 
		actor->SetMapper(map); 

		renderer->AddActor(actor);
	}

	renderWindow->Render(); 		
	iren->Start(); 

	iren->Delete();
	renderer->Delete();
	renderWindow->Delete();
}


//
// Save / Load scene to /from file
//
template<unsigned int VDimension, class TAppearance, class TObjectId, class TAssociatedData, class TInteractionData>
bool 
BaseScene<VDimension, TAppearance, TObjectId, TAssociatedData, TInteractionData>::SaveSceneToFile(std::string filename)	{
	throw DeformableModelException("SaveSceneToFile: not implemented -- TODO ");
}

template<unsigned int VDimension, class TAppearance, class TObjectId, class TAssociatedData, class TInteractionData>
bool 
BaseScene<VDimension, TAppearance, TObjectId, TAssociatedData, TInteractionData>::LoadSceneFromFile(std::string filename) {
	throw DeformableModelException("LoadSceneFromFile: not implemented -- TODO ");
}





//
//
////ModifyObjectParameters
//template<class TObject, class TLabel>
//bool BaseScene<TObject, TLabel>::ModifyObjectParameters(TLabel label, const vnl_vector<double> &p) {
//	//first check wether the parameters are different at all !!
//	if ( (p - this->GetParametersOfObject(label)).inf_norm() < TINY ) { return true; } //the parameters are exactly the same -> can exit right now!*
//	//then test whether the proposed parameters are valid object parameters
//	DeformableObjectType::Pointer test_object = m_objectTypesLibrary->GetObjectEntry(m_objectSet[label-1]->indexObjectType)->objectGenerator->GetNewObject();
//	if (!test_object->SetParameters(p)) {return false;}
//	
//	//now, test whether the scene accepts such an object 
//	//WARNING: there may be problems if a scene wants to forbid the insertion of an object which overlaps something...
//	//there should be a TestObjectInsertionAcceptanceExcludingObject(test_object ,label);
//	if (!TestObjectInsertionAcceptance(test_object)) return 0;
//
//	DeformableObjectType::Pointer oldObject = m_objectSet[label-1]->object;
//	//erase the old object
//	EraseObjectFromScene( label );
//
//	//invalidate the data costs
//	m_objectSet[label-1]->priorCostFlag = false; m_objectSet[label-1]->dataCostFlag = false; m_objectSet[label-1]->offContextDataCostFlag = false; //obj->sumInteractionCostFlag=false;
//	m_objectSet[label-1]->sensedOffContextPixelSetFlag = false;
//
//	m_interactionManager->RemoveObject(m_objectSet[label-1]);
//	m_objectSet[label-1]->interactions.clear();
//	
//	//draw the new one in place - first update the object
//	m_objectSet[label-1]->object = test_object;
//
//	DrawObjectInScene(label);
//	m_interactionManager->AddObject(m_objectSet[label-1]); 
//
//	//TEST ...
//	//updating the scene prior must be called while both objects are available...
//	m_scenePrior->Update_ModifyingObject(oldObject, m_objectSet[label-1]->object);
//
//	return true;
//}
//



//
////write a simple vtk representation from scratch each time the function is called; object deletion would be difficult to handle otherwise
//template<class TObject, class TLabel>
//void BaseScene<TObject, TLabel>::WriteSimpleVTKRepresentation(std::string filename) {
//	vtkSmartPointer<vtkPolyData> tmpPolyData;
//	vtkSmartPointer<vtkAppendPolyData> appendPolyDataFilter = vtkSmartPointer<vtkAppendPolyData>::New();
//	appendPolyDataFilter->SetNumberOfInputs(m_nbObjectsInSceneInScene);
//	appendPolyDataFilter->SetUserManagedInputs(m_nbObjectsInSceneInScene);
//
//	for (unsigned i=0 ; i<m_nbObjectsInSceneInScene ; i++) {
//		tmpPolyData = m_objectSet[ m_objectLabels[i]-1 ]->object->GetObjectAsVTKPolyData();
//		//appendPolyDataFilter->SetInputByNumber(i, tmpPolyData);
//		appendPolyDataFilter->AddInput(tmpPolyData);
//	}
//
//	appendPolyDataFilter->Update();
//	WriteMirrorPolyDataToFile(filename, appendPolyDataFilter->GetOutput());
//}


//
template<unsigned int VDimension, class TAppearance, class TObjectId, class TAssociatedData, class TInteractionData>
void 
BaseScene<VDimension, TAppearance, TObjectId, TAssociatedData, TInteractionData>::TrackModifications(typename BinaryImageType::RegionType region) {
	if ( m_modifiedRegions.empty() ) {m_modifiedRegions.push_back(region);}
	else { 
		//TODO: this is fairly suboptimal..., currently, I change m_modifiedRegions to the axis-aligned bbox that contains both the old and new regions.
		//OPTIMIZATION: if the regions don't intersect, add a new region to the vector, otherwise merge some regions 
		// this can become a bit complex because different existing elements of the vector may need to be merged after a while, etc...
		// other possibility is, on the contrary, to increase the number of regions, extracting minimal rectangular parts
		//  _______             _______
		//  |  1  |             |  1| |
		//  |   __|__           |   | |__
		//  |___|_|  |     =>   |___|2| 3|
		//      |  2 |              | |  |
		//      |____|              |_|__|
		LabelImageType::IndexType modifiedStart = m_modifiedRegions[0].GetIndex();
		LabelImageType::SizeType  modifiedSize = m_modifiedRegions[0].GetSize();		
		const LabelImageType::IndexType &currentStart = region.GetIndex();
		const LabelImageType::SizeType  &currentSize  = region.GetSize();
		unsigned int modifiedEnd;
		for (unsigned i=0 ; i<Dimension ; i++) {
			modifiedEnd = std::max<unsigned>(modifiedStart[i] + modifiedSize[i], currentStart[i] + currentSize[i]);
			modifiedStart[i] = std::min<unsigned>( modifiedStart[i], currentStart[i] );
			modifiedSize[i] = modifiedEnd - modifiedStart[i];
		}
		m_modifiedRegions[0].SetIndex(modifiedStart); m_modifiedRegions[0].SetSize(modifiedSize);
		
		//m_modifiedRegions;
	}
}




} // namespace psciob

