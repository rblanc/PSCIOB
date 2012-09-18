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
* \file VTKScene.txx
* \author Rémi Blanc 
* \date 27. February 2012
*/

// make a labelMapScene on the same model
//intersection managed by pixel sets
//Draw -> set the label value to the id ...
//Erase -> nothing special.

#include "VTKScene.h"

namespace psciob {



template<unsigned int VDimension, class TAppearance, class TObjectId, class TAssociatedData, class TInteractionData>
VTKScene<VDimension, TAppearance, TObjectId, TAssociatedData, TInteractionData>::VTKScene() : BaseScene() {
	m_rendererFlag = true; //always true for VTKScenes, because it is updated at each modification
}

//
// TestObjectFullyOutside
//

template<unsigned int VDimension, class TAppearance, class TObjectId, class TAssociatedData, class TInteractionData>
inline 
bool 
VTKScene<VDimension, TAppearance, TObjectId, TAssociatedData, TInteractionData>::TestObjectFullyOutside_Internal(ObjectInScene *object) {
	//std::cout<<"VTKScene::TestObjectFullyOutside - not really valid, the bbox may interesect, but the object may still be fully outside"<<std::endl;
	return ( !TestBoundingBoxesIntersection(m_sceneBBox, object->obj->GetPhysicalBoundingBox()) );
}


//
// Test overlaps
//
template<unsigned int VDimension, class TAppearance, class TObjectId, class TAssociatedData, class TInteractionData>
typename VTKScene<VDimension, TAppearance, TObjectId, TAssociatedData, TInteractionData>::SceneObjectOverlapCode
VTKScene<VDimension, TAppearance, TObjectId, TAssociatedData, TInteractionData>::TestOverlap_Internal(ObjectInScene *object) {
	throw DeformableModelException(" VTKScene::TestOverlap NOT IMPLEMENTED YET"); 
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
typename VTKScene<VDimension, TAppearance, TObjectId, TAssociatedData, TInteractionData>::SceneObjectOverlapCode
VTKScene<VDimension, TAppearance, TObjectId, TAssociatedData, TInteractionData>::TestOverlapIgnoringObjectID_Internal(ObjectInScene *object, TObjectId id) {
	throw DeformableModelException(" VTKScene::TestOverlap NOT IMPLEMENTED YET"); 
}



//DrawObjectInScene	
template<unsigned int VDimension, class TAppearance, class TObjectId, class TAssociatedData, class TInteractionData>
void 
VTKScene<VDimension, TAppearance, TObjectId, TAssociatedData, TInteractionData>::DrawObjectInScene(TObjectId id) {	//this method be implemented only in the leaf class when all insertion policies have been defined (do I write in the label image, in both the label and the textured image, do I authorize object overlaps, etc...) 
	//Invalidate the labelMap representations
	m_labelMapFlag = false;
	m_labelImageFlag = false;
	m_binaryImageFlag = false;

	//Update the renderer representation
	//create an actor, assign a pseudo-color dependent on the id
	if(!m_arrayObjects[id-1].mapper) {
		m_arrayObjects[id-1].mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
		m_arrayObjects[id-1].actor = vtkSmartPointer<vtkActor>::New(); 
		m_arrayObjects[id-1].actor->GetProperty()->SetAmbient(1); 
		m_arrayObjects[id-1].actor->GetProperty()->SetDiffuse(0); 
		m_arrayObjects[id-1].actor->GetProperty()->SetSpecular(0);
		m_arrayObjects[id-1].actor->SetMapper(m_arrayObjects[id-1].mapper);
	}
	m_arrayObjects[id-1].mapper->SetInput( m_arrayObjects[id-1].obj->GetObjectAsVTKPolyData() ); 
	unsigned tmpid = id%(255*255), R = id/(255*255), G = tmpid/255, B = tmpid%255;
	m_arrayObjects[id-1].actor->GetProperty( )->SetColor(R/255.0,G/255.0,B/255.0); 
	m_renderer->AddActor(m_arrayObjects[id-1].actor);
	m_arrayObjects[id-1].actorFlag = true;

}

template<unsigned int VDimension, class TAppearance, class TObjectId, class TAssociatedData, class TInteractionData>
void 
VTKScene<VDimension, TAppearance, TObjectId, TAssociatedData, TInteractionData>::EraseObjectFromScene(TObjectId id) {
	//Invalidate the labelMap and labelImage representations
	m_labelMapFlag = false;
	m_labelImageFlag = false;
	m_binaryImageFlag = false;

	//Update the renderer representation
	m_renderer->RemoveActor(m_arrayObjects[id-1].actor);
}

template<unsigned int VDimension, class TAppearance, class TObjectId, class TAssociatedData, class TInteractionData>
void 
VTKScene<VDimension, TAppearance, TObjectId, TAssociatedData, TInteractionData>::UpdateObjectInScene(IDType id, ObjectInScene *newObject) {
	//Invalidate global scene representations
	//m_rendererFlag = true; //this status is preserved
	m_labelMapFlag = false;
	m_labelImageFlag = false;
	m_binaryImageFlag = false;

	//update the mapper input... it is necessary to instanciate the new objects
	if(!m_arrayObjects[id-1].mapper) {
		m_arrayObjects[id-1].mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
		m_arrayObjects[id-1].actor = vtkSmartPointer<vtkActor>::New(); 
		m_arrayObjects[id-1].actor->GetProperty()->SetAmbient(1); 
		m_arrayObjects[id-1].actor->GetProperty()->SetDiffuse(0); 
		m_arrayObjects[id-1].actor->GetProperty()->SetSpecular(0);
		m_arrayObjects[id-1].actor->SetMapper(m_arrayObjects[id-1].mapper);
	}
	m_arrayObjects[id-1].mapper->SetInput( newObject->obj->GetObjectAsVTKPolyData() );
	m_arrayObjects[id-1].actorFlag = true;
	m_arrayObjects[id-1].sceneOffContextLabelObjectFlag = false;
}

} // namespace psciob

