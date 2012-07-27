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
* \file LabelMapScene.txx
* \author Rémi Blanc 
* \date 27. February 2012
*/

// make a labelMapScene on the same model
//intersection managed by pixel sets
//Draw -> set the label value to the id ...
//Erase -> nothing special.

#include "LabelMapScene.h"

namespace psciob {


template<unsigned int VDimension, class TAppearance, class TObjectId, class TAssociatedData, class TInteractionData>
LabelMapScene<VDimension, TAppearance, TObjectId, TAssociatedData, TInteractionData>::LabelMapScene() : BaseScene() {
	m_labelMapFlag = true; //always true for LabelMapScenes, because it is updated at each modification
	m_dummyLabelMap = LabelMapType::New();
}


//
// SetPhysicalDimensions
//
template<unsigned int VDimension, class TAppearance, class TObjectId, class TAssociatedData, class TInteractionData>
void 
LabelMapScene<VDimension, TAppearance, TObjectId, TAssociatedData, TInteractionData>::SetPhysicalDimensions(const vnl_vector<double> &physicalBoundingBox, const vnl_vector<double> &spacing) {	
	ITKGridImageInformationFromPhysicalBoundingBoxAndSpacing<Dimension>(physicalBoundingBox, spacing, &m_sceneOrigin, &m_sceneImageRegion);
	m_sceneSize = m_sceneImageRegion.GetSize();
	for (unsigned i=0 ; i<Dimension ; i++) { m_sceneSpacing[i] = spacing(i); }
	m_sceneBBox = BoundingBoxFromITKImageInformation<BinaryImageType>( m_sceneOrigin, m_sceneSpacing, m_sceneImageRegion );

	m_labelMap->SetRegions( m_sceneImageRegion ); m_dummyLabelMap->SetRegions( m_sceneImageRegion );
	m_labelMap->SetSpacing( m_sceneSpacing );     m_dummyLabelMap->SetSpacing( m_sceneSpacing );
	m_labelMap->SetOrigin(  m_sceneOrigin );      m_dummyLabelMap->SetOrigin(  m_sceneOrigin );
}


//
// TestObjectFullyOutside
//
template<unsigned int VDimension, class TAppearance, class TObjectId, class TAssociatedData, class TInteractionData>
inline 
bool 
LabelMapScene<VDimension, TAppearance, TObjectId, TAssociatedData, TInteractionData>::TestObjectFullyOutside_Internal(ObjectInScene *object) {
	if (!TestBoundingBoxesIntersection(m_sceneBBox, object->obj->GetPhysicalBoundingBox())) return true;
	m_dummyLabelMap->ClearLabels();

	object->obj->SetImageSpacing(m_sceneSpacing);
	if (!object->sceneOffContextLabelObjectFlag) {
		if (!InsertSingleObjectLabelMapIntoAnother<DeformableObjectType::LabelMapType, LabelMapType>(object->obj->GetObjectAsLabelMap(), m_dummyLabelMap)) return true;
		object->sceneOffContextLabelObject = m_dummyLabelMap->GetNthLabelObject(0);
		object->sceneOffContextLabelObjectFlag = true;
	}
	m_dummyLabelMap->ClearLabels();
	return false;
}


//
// Test overlaps
//
template<unsigned int VDimension, class TAppearance, class TObjectId, class TAssociatedData, class TInteractionData>
typename LabelMapScene<VDimension, TAppearance, TObjectId, TAssociatedData, TInteractionData>::SceneObjectOverlapCode
LabelMapScene<VDimension, TAppearance, TObjectId, TAssociatedData, TInteractionData>::TestOverlap_Internal(ObjectInScene *object) {
	throw DeformableModelException(" LabelMapScene::TestOverlap NOT IMPLEMENTED YET"); 
/*
	vnl_vector<double> BB2 = object->GetPhysicalBoundingBox();

	bool partiallyOutside=false, objectOverlap=false;
	for (unsigned i=0 ; i<Dimension ; i++) {
		if ( BB2(2*i+1)<m_sceneBBox( 2*i ) ) return OBJECT_FULLYOUTSIDE;
		if ( BB2( 2*i )>m_sceneBBox(2*i+1) ) return OBJECT_FULLYOUTSIDE;
		if ( BB2( 2*i )<m_sceneBBox( 2*i ) ) partiallyOutside=true;
		if ( BB2(2*i+1)>m_sceneBBox(2*i+1) ) partiallyOutside=true;
	}*/

	//TODO: update VTK as soon as the boolean operation for meshes is part of the release... (it is in V5.9.something)
	//http://www.vtk.org/Wiki/VTK/Examples/Cxx/PolyData/IntersectionPolyDataFilter
	//or better... remove this functionality from all scenes, and let an interaction manager in charge of this task!

	////get the pixel set, updating it if necessary
	//object->SetPixelSetImageInformation(m_sceneOrigin, m_sceneSize,  m_sceneSpacing); //make sure the image information is uptodate
	//ObjectPixelSetType::Pointer objectPixelSet = object->GetObjectAsLabelMap();	//get the pixel set / updating if necessary
	//ObjectPixelSetType::SetType *set2 = objectPixelSet->GetPixelSet();
	//if (set2->size()==0) return OBJECT_FULLYOUTSIDE;
	//ObjectPixelSetType::SetType *set1;
	////browse all objects already in the scene 
	////for (ObjectSetType::iterator it = m_objectSet.begin(); it != m_objectSet.end() ; it++) { 
	//for (unsigned i=0 ; i<m_nbObjectsInScene ; i++) { 
	//	//if ( TestBoundingBoxesIntersection(BB2, it->second->object->GetPhysicalBoundingBox()) ) { //if the bboxes intersect, then check the intersection of the pixel sets
	//	if ( TestBoundingBoxesIntersection(BB2, m_objectSet[m_objectLabels[i]-1]->object->GetPhysicalBoundingBox()) ) { //if the bboxes intersect, then check the intersection of the pixel sets
	//		set1 = m_objectSet[m_objectLabels[i]-1]->object->GetObjectAsLabelMap()->GetPixelSet();
	//		//by convention, the sets should already be sorted, look at each pixel in set1, and try to find it in set2
	//		objectOverlap = TestSetIntersection(set1->begin(), set1->end(), set2->begin(), set2->end());
	//	}
	//	if (objectOverlap) break;
	//}

	//if (partiallyOutside) {
	//	if (objectOverlap)	return OBJECT_OVERLAPOBJECTANDSCENE;
	//	else				return OBJECT_OVERLAPSCENE;
	//}
	//else {
	//	if (objectOverlap)	return OBJECT_OVERLAPOBJECT;
	//	else				return OBJECT_NOOVERLAP;
	//}
}
//
//TestOverlapIgnoringObjectID
//
template<unsigned int VDimension, class TAppearance, class TObjectId, class TAssociatedData, class TInteractionData>
typename LabelMapScene<VDimension, TAppearance, TObjectId, TAssociatedData, TInteractionData>::SceneObjectOverlapCode
LabelMapScene<VDimension, TAppearance, TObjectId, TAssociatedData, TInteractionData>::TestOverlapIgnoringObjectID_Internal(ObjectInScene *object, TObjectId id) {
	throw DeformableModelException(" LabelMapScene::TestOverlap NOT IMPLEMENTED YET"); 
}



//DrawObjectInScene	
template<unsigned int VDimension, class TAppearance, class TObjectId, class TAssociatedData, class TInteractionData>
void 
LabelMapScene<VDimension, TAppearance, TObjectId, TAssociatedData, TInteractionData>::DrawObjectInScene(TObjectId id) {	//this method be implemented only in the leaf class when all insertion policies have been defined (do I write in the label image, in both the label and the textured image, do I authorize object overlaps, etc...) 
	//invalidate the vtk renderer and labelImage flags
	m_rendererFlag = false;
	m_labelImageFlag = false;

	//update the object representation
	if (!m_arrayObjects[id-1].sceneOffContextLabelObjectFlag) {
		if (!InsertSingleObjectLabelMapIntoAnother<DeformableObjectType::LabelMapType, LabelMapType>(m_arrayObjects[id-1].obj->GetObjectAsLabelMap(), m_labelMap, id)) {
			throw DeformableModelException("LabelImageScene::DrawObjectInScene -- problem inserting the object in the map... SHOULD NEVER HAPPEN!!");
		}
		m_arrayObjects[id-1].sceneOffContextLabelObject = m_labelMap->GetLabelObject( id );
		m_arrayObjects[id-1].sceneOffContextLabelObjectFlag = true;
	}
	else {
		m_arrayObjects[id-1].sceneOffContextLabelObject->SetLabel(id); //make sure this is the right label.
		m_labelMap->AddLabelObject(m_arrayObjects[id-1].sceneOffContextLabelObject); 
	}
}

template<unsigned int VDimension, class TAppearance, class TObjectId, class TAssociatedData, class TInteractionData>
void 
LabelMapScene<VDimension, TAppearance, TObjectId, TAssociatedData, TInteractionData>::EraseObjectFromScene(TObjectId id) {
	//invalidate the vtk renderer and labelImage flags
	m_rendererFlag = false;
	m_labelImageFlag = false;

	//remove the labelobject from the map.
	m_labelMap->RemoveLabel( id );
	//not necessary to modify m_arrayObjects[id-1], it will be reinitialized.
}

template<unsigned int VDimension, class TAppearance, class TObjectId, class TAssociatedData, class TInteractionData>
void 
LabelMapScene<VDimension, TAppearance, TObjectId, TAssociatedData, TInteractionData>::UpdateObjectInScene(IDType id, ObjectInScene *newObject) {
	//Invalidate global scene representations
	m_rendererFlag = false; 
	m_labelImageFlag = false;

	//update the LabelObject and LabelMap representation
	m_labelMap->RemoveLabel(id);
	if (!newObject->sceneOffContextLabelObjectFlag) {
		if (!InsertSingleObjectLabelMapIntoAnother<DeformableObjectType::LabelMapType, LabelMapType>(newObject->obj->GetObjectAsLabelMap(), m_labelMap, id)) {
			throw DeformableModelException("LabelImageScene::UpdateObjectInScene -- problem updating the object in the map... SHOULD NEVER HAPPEN!!");
		}
		newObject->sceneOffContextLabelObject = m_labelMap->GetLabelObject( id );
		newObject->sceneOffContextLabelObjectFlag = true;
		m_arrayObjects[id-1].sceneOffContextLabelObject = m_labelMap->GetLabelObject( id );
		m_arrayObjects[id-1].sceneOffContextLabelObjectFlag = true;
	}
	else {
		newObject->sceneOffContextLabelObject->SetLabel(id); //make sure this is the right label.
		m_labelMap->AddLabelObject(newObject->sceneOffContextLabelObject); 
		m_arrayObjects[id-1].sceneOffContextLabelObject = newObject->sceneOffContextLabelObject;
		m_arrayObjects[id-1].sceneOffContextLabelObjectFlag = true;
	}

	//
	m_arrayObjects[id-1].actorFlag = false;
}

} // namespace psciob

