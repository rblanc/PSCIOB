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
* \file LabelObjectOverlap.h
* \author Rémi Blanc 
* \date 18. June 2012
*/

#ifndef __LABELOBJECTOVERLAP_H_
#define __LABELOBJECTOVERLAP_H_

#include "ObjectInteractionManager.h"

namespace psciob {


/**\class LabelObjectOverlap
* \brief LabelObjectOverlap: detect objects intersection based on their LabelObject representations (with respect to the scene image frame)
* requires a scene with an Interaction container authorizing LabelObjectIntersections (e.g. LabelObjectIntersectionContainer)
*
* the cost computed is the ratio : volume of intersection / (volume 1 + volume 2)
*/

//CONCRETE CLASS
template<class TScene>
class LabelObjectOverlap : public ObjectInteractionManager<TScene> {
public:
	/** Standard class typedefs. */
	typedef LabelObjectOverlap          Self;
	typedef ObjectInteractionManager       Superclass;
	typedef itk::SmartPointer<Self>        Pointer;
	typedef itk::SmartPointer<const Self>  ConstPointer;
	/** Run-time type information (and related methods). */
	itkTypeMacro(LabelObjectOverlap, ObjectInteractionManager);
	itkNewMacro(LabelObjectOverlap);

	typedef typename SceneType::LabelMapType LabelMapType;

	/** Attach the scene to the InteractionManager ; the user doesn't need to call this function
	* \todo make it protected and use friendship to enable usage by the Scene
	* this version also dimensions an internal labelmap.
	*/
	virtual void SetScene(SceneType* scene) { 
		m_scene = scene; m_tmpMapFlag = false;
	}

	/** Clear any scene-related cached information, as the scene is getting cleared itself 
	* use friendship to hide this function from the public interface...
	*/
	virtual void Clear() {m_tmpMapFlag = false;}

	/** given two arbitrary objects, compute what their interaction 
	* does not perform a bounding box check, which should therefore be made prior to calling this method...
	* check the method: TestBoundingBoxesIntersection_NoCheck (or TestBoundingBoxesIntersection for a safer version)
	*/
	void ComputePairWiseObjectInteractionData(DeformableObjectType *object1, DeformableObjectType *object2, InteractionDataType &interactionData) {
		//1: construct a temporary label map, the same size as the scene, to represent the objects
		m_tmpMap->ClearLabels();
		if (!m_tmpMapFlag) { m_tmpMap->SetRegions( m_scene->GetSceneImageRegion() ); m_tmpMap->SetSpacing( m_scene->GetSceneSpacing() ); m_tmpMap->SetOrigin( m_scene->GetSceneOrigin() ); m_tmpMapFlag=true; }
		if (!InsertSingleObjectLabelMapIntoAnother<DeformableObjectType::LabelMapType, LabelMapType>(object1->GetObjectAsLabelMap(), m_tmpMap, 1)) { throw DeformableModelException("LabelImageObjectOverlap::ComputePairWiseObjectInteractionData -- problem inserting the object in the map... SHOULD NEVER HAPPEN!!"); }
		if (!InsertSingleObjectLabelMapIntoAnother<DeformableObjectType::LabelMapType, LabelMapType>(object2->GetObjectAsLabelMap(), m_tmpMap, 2)) { throw DeformableModelException("LabelImageObjectOverlap::ComputePairWiseObjectInteractionData -- problem inserting the object in the map... SHOULD NEVER HAPPEN!!"); }

		//2: compute the intersection
		if (!interactionData.intersectionObject) interactionData.intersectionObject = InteractionDataType::LabelObjectType::New();
		interactionData.nbOverlappingPixels = GetLabelObjectIntersection<LabelObjectType>(m_tmpMap->GetLabelObject(1), m_tmpMap->GetLabelObject(2), interactionData.intersectionObject);

		if (interactionData.nbOverlappingPixels>0) {
			interactionData.interactionCostFlag = true;
			interactionData.interactionCost = m_costFunction->Evaluate(m_tmpMap->GetLabelObject(1)->Size(), m_tmpMap->GetLabelObject(2)->Size(), interactionData.nbOverlappingPixels); 
		}
		else {interactionData.interactionCostFlag = false; interactionData.nbOverlappingPixels=0;}

		//reset the tmpMap.
		m_tmpMap->ClearLabels();
	}

	/** given two objects from the scene, compute what their interaction cost would be 
	* provided in case some information needs to be cached. the default version is an alias for ComputePairWiseObjectInteractionData(DeformableObjectType *object1, DeformableObjectType *object2)
	*/
	void ComputePairWiseObjectInteractionData(ObjectInScene *object1, ObjectInScene *object2, InteractionDataType &interactionData) {
		if (!object1->sceneOffContextLabelObjectFlag) {
			if (!m_tmpMapFlag) { m_tmpMap->SetRegions( m_scene->GetSceneImageRegion() ); m_tmpMap->SetSpacing( m_scene->GetSceneSpacing() ); m_tmpMap->SetOrigin( m_scene->GetSceneOrigin() ); m_tmpMapFlag=true; }
			if (!InsertSingleObjectLabelMapIntoAnother<DeformableObjectType::LabelMapType, LabelMapType>(object1->obj->GetObjectAsLabelMap(), m_tmpMap, object1->id)) { throw DeformableModelException("LabelImageObjectOverlap::ComputePairWiseObjectInteractionData -- problem inserting the object in the map... SHOULD NEVER HAPPEN!!"); }
			object1->sceneOffContextLabelObjectFlag = true;
			object1->sceneOffContextLabelObject = m_tmpMap->GetLabelObject(object1->id);
			m_tmpMap->ClearLabels();
		}
		if (!object2->sceneOffContextLabelObjectFlag) {
			if (!m_tmpMapFlag) { m_tmpMap->SetRegions( m_scene->GetSceneImageRegion() ); m_tmpMap->SetSpacing( m_scene->GetSceneSpacing() ); m_tmpMap->SetOrigin( m_scene->GetSceneOrigin() ); m_tmpMapFlag=true; }
			if (!InsertSingleObjectLabelMapIntoAnother<DeformableObjectType::LabelMapType, LabelMapType>(object2->obj->GetObjectAsLabelMap(), m_tmpMap, object2->id)) { throw DeformableModelException("LabelImageObjectOverlap::ComputePairWiseObjectInteractionData -- problem inserting the object in the map... SHOULD NEVER HAPPEN!!"); }
			object2->sceneOffContextLabelObjectFlag = true;
			object2->sceneOffContextLabelObject = m_tmpMap->GetLabelObject(object2->id);
			m_tmpMap->ClearLabels();
		}

		if (!interactionData.intersectionObject) interactionData.intersectionObject = InteractionDataType::LabelObjectType::New();
		interactionData.nbOverlappingPixels = GetLabelObjectIntersection<LabelObjectType>(object1->sceneOffContextLabelObject, object2->sceneOffContextLabelObject, interactionData.intersectionObject);
		if (interactionData.nbOverlappingPixels>0) {
			interactionData.interactionCostFlag = true;
			interactionData.interactionCost = m_costFunction->Evaluate(object1->sceneOffContextLabelObject->Size(), object2->sceneOffContextLabelObject->Size(), interactionData.nbOverlappingPixels); 
		}
		else {interactionData.interactionCostFlag = false; interactionData.nbOverlappingPixels=0;}
	}


protected:	
	LabelObjectOverlap() : ObjectInteractionManager() {
		m_tmpMap = LabelMapType::New(); m_tmpMapFlag = false;
	};
	virtual ~LabelObjectOverlap() {};	

	typename LabelMapType::Pointer m_tmpMap; bool m_tmpMapFlag;
private:
	LabelObjectOverlap(const Self&);	//purposely not implemented
	const Self & operator=( const Self & );	//purposely not implemented
};


} // namespace psciob

#endif /* __LABELOBJECTOVERLAP_H_ */


