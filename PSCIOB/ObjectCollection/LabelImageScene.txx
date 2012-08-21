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
* \file LabelImageScene.txx
* \author Rémi Blanc 
* \date 27. February 2012
*/

#include "LabelImageScene.h"
#include "LabelImageObjectOverlap.h"
#include "LabelMapUtils.h"

namespace psciob {


template<unsigned int VDimension, class TAppearance, class TObjectId, class TAssociatedData, class TInteractionData>
LabelImageScene<VDimension, TAppearance, TObjectId, TAssociatedData, TInteractionData>::LabelImageScene() : BaseScene() {
	m_labelMapFlag = true;   //always true for a LabelImageScene, because it is updated at each modification
	m_labelImageFlag = true; //always true for a LabelImageScene, because it is updated at each modification
	m_labelImage = LabelImageType::New();
	m_dummyLabelMap = LabelMapType::New();

	m_removedPixels = LabelObjectType::New(); m_addedPixels = LabelObjectType::New(); m_redrawPixels = LabelObjectType::New();

	m_interactionManager = static_cast<ObjectInteractionManagerType*>(LabelImageObjectOverlap<BaseScene>::New().GetPointer());
	m_interactionManager->SetScene(this);	
}

//
// SetPhysicalDimensions
//
template<unsigned int VDimension, class TAppearance, class TObjectId, class TAssociatedData, class TInteractionData>
void 
LabelImageScene<VDimension, TAppearance, TObjectId, TAssociatedData, TInteractionData>::AllocateFromPhysicalDimension {
	m_labelMap->SetRegions( m_sceneImageRegion ); m_labelImage->SetRegions( m_sceneImageRegion); m_dummyLabelMap->SetRegions( m_sceneImageRegion );
	m_labelMap->SetSpacing( m_sceneSpacing );     m_labelImage->SetSpacing( m_sceneSpacing );    m_dummyLabelMap->SetSpacing( m_sceneSpacing );
	m_labelMap->SetOrigin(  m_sceneOrigin );      m_labelImage->SetOrigin( m_sceneOrigin );      m_dummyLabelMap->SetOrigin(  m_sceneOrigin );
	m_labelImage->Allocate();
	m_labelImage->FillBuffer(0);
}

//
// TestObjectFullyOutside_Internal
//

template<unsigned int VDimension, class TAppearance, class TObjectId, class TAssociatedData, class TInteractionData>
inline 
bool 
LabelImageScene<VDimension, TAppearance, TObjectId, TAssociatedData, TInteractionData>::TestObjectFullyOutside_Internal(ObjectInScene *object) {
	if (!TestBoundingBoxesIntersection(m_sceneBBox, object->obj->GetPhysicalBoundingBox())) return true;
	m_dummyLabelMap->ClearLabels();
	//LabelMapType::Pointer labelMap = LabelMapType::New();
	//labelMap->SetRegions( m_sceneImageRegion );
	//labelMap->SetSpacing( m_sceneSpacing );
	//labelMap->SetOrigin( m_sceneOrigin );

	object->obj->SetImageSpacing(m_sceneSpacing);
	if (!object->sceneOffContextLabelObjectFlag) {
		//if (!InsertSingleObjectLabelMapIntoAnother<DeformableObjectType::LabelMapType, LabelMapType>(object->obj->GetObjectAsLabelMap(), labelMap)) return true;
		//object->sceneOffContextLabelObject = labelMap->GetNthLabelObject(0);
		if (!InsertSingleObjectLabelMapIntoAnother<DeformableObjectType::LabelMapType, LabelMapType>(object->obj->GetObjectAsLabelMap(), m_dummyLabelMap)) return true;
		object->sceneOffContextLabelObject = m_dummyLabelMap->GetNthLabelObject(0);
		object->sceneOffContextLabelObjectFlag = true;
	}
	//m_dummyLabelMap->ClearLabels();
	return false;
}


//
// Test overlaps
//
template<unsigned int VDimension, class TAppearance, class TObjectId, class TAssociatedData, class TInteractionData>
typename LabelImageScene<VDimension, TAppearance, TObjectId, TAssociatedData, TInteractionData>::SceneObjectOverlapCode
LabelImageScene<VDimension, TAppearance, TObjectId, TAssociatedData, TInteractionData>::TestOverlap_Internal(ObjectInScene *object) {
	throw DeformableModelException(" LabelImageScene::TestOverlap NOT IMPLEMENTED YET"); 
/*
	vnl_vector<double> BB2 = object->GetPhysicalBoundingBox();

	bool partiallyOutside=false, objectOverlap=false;
	for (unsigned i=0 ; i<Dimension ; i++) {
		if ( BB2(2*i+1)<m_sceneBBox( 2*i ) ) return OBJECT_FULLYOUTSIDE;
		if ( BB2( 2*i )>m_sceneBBox(2*i+1) ) return OBJECT_FULLYOUTSIDE;
		if ( BB2( 2*i )<m_sceneBBox( 2*i ) ) partiallyOutside=true;
		if ( BB2(2*i+1)>m_sceneBBox(2*i+1) ) partiallyOutside=true;
	}*/

	//TODO: just check if the pixels whether the pixels are occupied already...

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
typename LabelImageScene<VDimension, TAppearance, TObjectId, TAssociatedData, TInteractionData>::SceneObjectOverlapCode
LabelImageScene<VDimension, TAppearance, TObjectId, TAssociatedData, TInteractionData>::TestOverlapIgnoringObjectID_Internal(ObjectInScene *object, TObjectId id) {
	throw DeformableModelException(" LabelImageScene::TestOverlap NOT IMPLEMENTED YET"); 
}



//DrawObjectInScene	
template<unsigned int VDimension, class TAppearance, class TObjectId, class TAssociatedData, class TInteractionData>
void 
LabelImageScene<VDimension, TAppearance, TObjectId, TAssociatedData, TInteractionData>::DrawObjectInScene(TObjectId id) {	//this method be implemented only in the leaf class when all insertion policies have been defined (do I write in the label image, in both the label and the textured image, do I authorize object overlaps, etc...) 
	//invalidate the vtk renderer flags
	m_rendererFlag = false;

	//update the object representation, label map and label image
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

	LabelImageType::PixelType *labelBuffer = m_labelImage->GetBufferPointer();

	//Draw the object into the Label Image; check for overlaps, maintain the interaction list uptodate
	std::set<IDType> detectedOverlaps;
	LabelObjectType *labelObj = m_arrayObjects[id-1].sceneOffContextLabelObject;
	for (unsigned i=0 ; i<labelObj->GetNumberOfLines() ; i++) {
		LabelImageType::OffsetValueType add = m_labelImage->ComputeOffset(labelObj->GetLine(i).GetIndex());
		for ( unsigned j=0 ; j < labelObj->GetLine(i).GetLength() ; j++, add++) {
			if (labelBuffer[add]!=0) {
				if (labelBuffer[add]!=id) detectedOverlaps.insert(labelBuffer[add]);    //mark this object as an overlap
				else throw DeformableModelException("LabelImageScene::DrawObjectInScene -- object is writting on itself... SHOULD NEVER HAPPEN!!");
				if (labelBuffer[add]<id) labelBuffer[add]=id; //update the label if necessary: the highest label has precedence
			}
			else { labelBuffer[add] = id; }
		}
	}

	//Register the overlaps - be careful, an overlap may hide another...
	if (!detectedOverlaps.empty()) { 
		//list all overlapping objects ; for each of these, mark its own overlaps as potential overlaps
		IDType distantLabel;
		std::set<IDType> potentialOverlaps;
		std::set<IDType>::iterator labelIt; 
		ObjectInteractionIterator distantIt;
		
		for (labelIt = detectedOverlaps.begin() ; labelIt!= detectedOverlaps.end() ; labelIt++) {
			distantLabel = *labelIt; 
			if (distantLabel!=id) potentialOverlaps.insert(distantLabel);
			//looking for potential additional overlaps (for the current object) in the interaction list of the local object - in case it is obscuring an overlap with the current object
			for (distantIt = m_arrayObjects[distantLabel-1].interactionData.begin() ; distantIt != m_arrayObjects[distantLabel-1].interactionData.end() ; distantIt++) {
				if (distantIt->first != id) { potentialOverlaps.insert(distantIt->first); }
			}
		}

		//check if potential Overlaps are real overlaps, and register them if necessary
		for (labelIt = potentialOverlaps.begin() ; labelIt != potentialOverlaps.end() ; labelIt++) {
			distantLabel = *labelIt;
			//this function automatically detect overlaps, identify the 'duplicate' pixels and registers the interaction in the interaction map
			InteractionDataType interactionData;
			m_interactionManager->ComputePairWiseObjectInteractionData(&m_arrayObjects[id-1], &m_arrayObjects[distantLabel-1], interactionData); 
			if ( interactionData.interactionCostFlag ) { //Register the interaction, bilaterally: store the information in both objects
				m_arrayObjects[id-1].interactionData[distantLabel] = interactionData;
				m_arrayObjects[distantLabel-1].interactionData[id] = interactionData;
			}
		}
	}

}

template<unsigned int VDimension, class TAppearance, class TObjectId, class TAssociatedData, class TInteractionData>
void 
LabelImageScene<VDimension, TAppearance, TObjectId, TAssociatedData, TInteractionData>::EraseObjectFromScene(TObjectId id) {
	//invalidate the vtk renderer
	m_rendererFlag = false;

	//remove the labelobject from the map.
	m_labelMap->RemoveLabel( id );

	//Erase the object, reveal objects that used to overlap...
	LabelImageType::PixelType *labelBuffer = m_labelImage->GetBufferPointer();
	LabelObjectType *labelObj = m_arrayObjects[id-1].sceneOffContextLabelObject;
	for (unsigned i=0 ; i<labelObj->GetNumberOfLines() ; i++) {
		LabelImageType::OffsetValueType add = m_labelImage->ComputeOffset(labelObj->GetLine(i).GetIndex());
		for ( unsigned j=0 ; j < labelObj->GetLine(i).GetLength() ; j++, add++) {
			if (labelBuffer[add]==id) labelBuffer[add] = 0;
		}
	}

	//Redraw objects that may have been hidden by the current object...
	IDType distantLabel;
	for (ObjectInteractionIterator mapit = m_arrayObjects[id-1].interactionData.begin() ; mapit != m_arrayObjects[id-1].interactionData.end() ; mapit++ ) {
		distantLabel = mapit->first;
		//look at the intersection set, and assign these pixels to the other object if necessary
		for (unsigned i=0 ; i<mapit->second.intersectionObject->GetNumberOfLines() ; i++) {
			LabelImageType::OffsetValueType add = m_labelImage->ComputeOffset(mapit->second.intersectionObject->GetLine(i).GetIndex());
			for ( unsigned j=0 ; j < mapit->second.intersectionObject->GetLine(i).GetLength() ; j++, add++) {
				if (labelBuffer[add]<distantLabel) labelBuffer[add] = distantLabel;
			}
		}
		//finally, remove the interaction entry from the distant object
		if ( !m_arrayObjects[distantLabel-1].interactionData.erase( id ) ) throw DeformableModelException("LabelImageScene::EraseObjectFromScene found a non-bilateral interaction... should never happen!"); 
	}
	//not necessary to modify m_arrayObjects[id-1], it will be reinitialized.
}

template<unsigned int VDimension, class TAppearance, class TObjectId, class TAssociatedData, class TInteractionData>
void 
LabelImageScene<VDimension, TAppearance, TObjectId, TAssociatedData, TInteractionData>::UpdateObjectInScene(IDType id, ObjectInScene *newObject) {
	//invalidate the vtk renderer flags
	m_rendererFlag = false;
	m_arrayObjects[id-1].actorFlag = false;

	//update the LabelObject and LabelMap representation
	m_labelMap->RemoveLabel(id);
	if (!newObject->sceneOffContextLabelObjectFlag) {
		if (!InsertSingleObjectLabelMapIntoAnother<DeformableObjectType::LabelMapType, LabelMapType>(newObject->obj->GetObjectAsLabelMap(), m_labelMap, id)) {
			throw DeformableModelException("LabelImageScene::UpdateObjectInScene -- problem updating the object in the map... SHOULD NEVER HAPPEN!!");
		}
		newObject->sceneOffContextLabelObject = m_labelMap->GetLabelObject( id );
		newObject->sceneOffContextLabelObjectFlag = true;
	}
	else { 
		newObject->sceneOffContextLabelObject->SetLabel(id); //make sure this is the right label.
		m_labelMap->AddLabelObject(newObject->sceneOffContextLabelObject); 
	}

	//get a pointer to the labelImage to be updated
	LabelImageType::PixelType *labelBuffer = m_labelImage->GetBufferPointer();

	//get the set differences between the old and new objects... 
	GetLabelObjectDifference<LabelObjectType>(m_arrayObjects[id-1].sceneOffContextLabelObject, newObject->sceneOffContextLabelObject, m_removedPixels);
	GetLabelObjectDifference<LabelObjectType>(newObject->sceneOffContextLabelObject, m_arrayObjects[id-1].sceneOffContextLabelObject, m_addedPixels);
	//now, I can update the label object in the scene representation.
	m_arrayObjects[id-1].sceneOffContextLabelObject = m_labelMap->GetLabelObject( id );
	m_arrayObjects[id-1].sceneOffContextLabelObjectFlag = true;

	//Erase the pixels that are no longer occupied by the object
	for (unsigned i=0 ; i<m_removedPixels->GetNumberOfLines() ; i++) {
		LabelImageType::OffsetValueType add = m_labelImage->ComputeOffset(m_removedPixels->GetLine(i).GetIndex());
		for ( unsigned j=0 ; j < m_removedPixels->GetLine(i).GetLength() ; j++, add++) {
			if (labelBuffer[add]==id) labelBuffer[add] = 0;
		}
	}

	//redraw any object that used to interact with the old object, and which has pixels in this removed set
	std::vector<IDType> distantObjectsMarkedAsNoLongerInteracting;
	std::set<IDType> detectedOverlaps; //initialized with the list of known overlapping objects
	IDType distantLabel;
	for (ObjectInteractionIterator mapit = m_arrayObjects[id-1].interactionData.begin() ; mapit != m_arrayObjects[id-1].interactionData.end() ; mapit++ ) {
		distantLabel = mapit->first;
		//check if the intersection between those 2 objects contains pixels that are removed
		GetLabelObjectIntersection<LabelObjectType>( mapit->second.intersectionObject, m_removedPixels, m_redrawPixels );

		//redraw the distant object if necessary
		int tmpsize = 0;
		for (unsigned i=0 ; i<m_redrawPixels->GetNumberOfLines() ; i++) {
			LabelImageType::OffsetValueType add = m_labelImage->ComputeOffset(m_redrawPixels->GetLine(i).GetIndex()); tmpsize+=m_redrawPixels->GetLine(i).GetLength();
			for ( unsigned j=0 ; j < m_redrawPixels->GetLine(i).GetLength() ; j++, add++) {
				if (labelBuffer[add]<distantLabel) labelBuffer[add] = distantLabel;
			}
		}
		if (tmpsize==mapit->second.nbOverlappingPixels) {distantObjectsMarkedAsNoLongerInteracting.push_back(distantLabel);}
		else { //the interaction structure of all objects that used to interact are either marked for deletion, or as out-dated
			//there could be an optimization potential by updating only the necessary object pairs, but dealing with 'hidden' overlaps might not be so straightforward
			detectedOverlaps.insert(distantLabel);
			mapit->second.interactionCostFlag = false;
			m_arrayObjects[distantLabel-1].interactionData[id].interactionCostFlag = false; //should not be necessary as the object should be deleted (replaced by the new)
		}
		//objects that are not involved here may not need to be updated at this point
	}

	//remove already object that are known not to interact anymore from the interaction map.
	for (unsigned i=0 ; i<distantObjectsMarkedAsNoLongerInteracting.size() ; i++) {
		m_arrayObjects[id-1].interactionData.erase( distantObjectsMarkedAsNoLongerInteracting[i] );
		m_arrayObjects[distantObjectsMarkedAsNoLongerInteracting[i]-1].interactionData.erase( id );
	}
	//add the new pixels ; look for new overlaps...	
	for (unsigned i=0 ; i<m_addedPixels->GetNumberOfLines() ; i++) {
		LabelImageType::OffsetValueType add = m_labelImage->ComputeOffset(m_addedPixels->GetLine(i).GetIndex());
		for ( unsigned j=0 ; j < m_addedPixels->GetLine(i).GetLength() ; j++, add++) {
			distantLabel = labelBuffer[add];
			if (distantLabel!=0) {
				detectedOverlaps.insert(distantLabel);      //tick this object for updating its interaction structure, if necessary.
				if (distantLabel<id) labelBuffer[add] = id; //update the label in the labelImage if necessary
				m_arrayObjects[distantLabel-1].interactionData[id].interactionCostFlag = false;
				m_arrayObjects[id-1].interactionData[distantLabel].interactionCostFlag = false;
			}
			else { labelBuffer[add] = id; }
		}
	}

	//the old and new interaction all need to be updated ; in practice, only those that were marked as out of date will be recomputed
	if (!detectedOverlaps.empty()) { 
		//list all overlapping objects ; for each of these, mark its own overlaps as potential overlaps
		std::set<IDType> potentialOverlaps;
		ObjectInteractionIterator distantIt;
		std::set<IDType>::iterator labelIt; 
		for (labelIt = detectedOverlaps.begin() ; labelIt!= detectedOverlaps.end() ; labelIt++) {
			distantLabel = *labelIt; potentialOverlaps.insert(distantLabel);
			//looking for potential additional overlaps (for the current object) in the interaction list of the local object - in case it is obscuring an overlap with the current object
			for (distantIt = m_arrayObjects[distantLabel-1].interactionData.begin() ; distantIt != m_arrayObjects[distantLabel-1].interactionData.end() ; distantIt++) {
				if (id != distantIt->first) { potentialOverlaps.insert(distantIt->first); }
			}
		}
		//check if potential Overlaps are real overlaps, and register them if necessary
		for (labelIt = potentialOverlaps.begin() ; labelIt != potentialOverlaps.end() ; labelIt++) {
			distantLabel = *labelIt;
			//this function automatically detect overlaps, identify the 'duplicate' pixels and registers the interaction in the interaction map
			InteractionDataType interactionData;
			m_interactionManager->ComputePairWiseObjectInteractionData(newObject, &m_arrayObjects[distantLabel-1], interactionData); 
			if ( interactionData.interactionCostFlag ) {
				//Register the interaction, bilaterally: store the information in both objects
				newObject->interactionData[distantLabel] = interactionData;
				m_arrayObjects[distantLabel-1].interactionData[id] = interactionData;
			}
			else { m_interactionManager->UnregisterInteraction(newObject, &m_arrayObjects[distantLabel-1]); }
		}
	}

}

} // namespace psciob

