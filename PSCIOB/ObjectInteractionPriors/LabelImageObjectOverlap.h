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
* \file LabelImageObjectOverlap.h
* \author Rémi Blanc 
* \date 21. October 2011
*/

#ifndef __LABELIMAGEOBJECTOVERLAP_H_
#define __LABELIMAGEOBJECTOVERLAP_H_

#include "ObjectInteractionManager.h"

namespace psciob {


/**\class LabelImageObjectOverlap
* \brief LabelImageObjectOverlap: INTERACTIONS ARE DETECTED BY THE SCENE ITSELF => do not search for them
* However, this class is used to compute the set of overlapping pixels between two objects
* and to associate a cost to an interaction. 
* The cost is computed from the measures of volume of both sets, and the volume of the intersection (measured as a number of pixels or voxels)
* The default measure is the Dice index
* \todo add a check that the scene is really a template based on LabelImageScene, at compile time already...
*/

//CONCRETE CLASS
template<class TScene>
class LabelImageObjectOverlap : public ObjectInteractionManager<TScene> {
public:
	/** Standard class typedefs. */
	typedef LabelImageObjectOverlap       Self;
	typedef ObjectInteractionManager      Superclass;
	typedef itk::SmartPointer<Self>       Pointer;
	typedef itk::SmartPointer<const Self> ConstPointer;
	/** Run-time type information (and related methods). */
	itkTypeMacro(LabelImageObjectOverlap, ObjectInteractionManager);
	itkNewMacro(LabelImageObjectOverlap);

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

	/** Add an object to the Interaction Manager -> look for interactions with other objects of the scene 
	* This action is performed by the LabelImageScene scene
	*/
	inline void AddObject(ObjectInScene *objectPtr) {}

	/** Remove an object 
	* the base is to delete all interactions this object could have with other objects 
	* any other action is in charge of specialized classes
	* This action is performed by the LabelImageScene scene
	*/
	inline void RemoveObject(ObjectInScene *objectPtr) {}

	/** Modifies the parameter of an object	
	* This action is performed by the LabelImageScene scene
	*/
	inline void ModifyObjectParameters(ObjectInScene *objectPtr, ObjectInScene *newObject) {}


	/** given two arbitrary objects, compute what their interaction 
	* does not perform a bounding box check, which should therefore be made prior to calling this method...
	* check the method: TestBoundingBoxesIntersection_NoCheck (or TestBoundingBoxesIntersection for a safer version)
	*/
	void ComputePairWiseObjectInteractionData(DeformableObjectType *object1, DeformableObjectType *object2, InteractionDataType &interactionData) {
		//1: construct a temporary label map, the same size as the scene, to represent the objects
		//OPTIMIZATION: implement a dummy labelmap to avoid this New?? the function is not intended to be used by the library, so this is not so important
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
			InsertSingleObjectLabelMapIntoAnother<DeformableObjectType::LabelMapType, LabelMapType>(object1->obj->GetObjectAsLabelMap(), m_tmpMap, object1->id);
			object1->sceneOffContextLabelObjectFlag = true;
			object1->sceneOffContextLabelObject = m_tmpMap->GetLabelObject(object1->id);
			m_tmpMap->ClearLabels();
		}
		if (!object2->sceneOffContextLabelObjectFlag) {
			if (!m_tmpMapFlag) { m_tmpMap->SetRegions( m_scene->GetSceneImageRegion() ); m_tmpMap->SetSpacing( m_scene->GetSceneSpacing() ); m_tmpMap->SetOrigin( m_scene->GetSceneOrigin() ); m_tmpMapFlag=true; }
			InsertSingleObjectLabelMapIntoAnother<DeformableObjectType::LabelMapType, LabelMapType>(object2->obj->GetObjectAsLabelMap(), m_tmpMap, object2->id);
			object2->sceneOffContextLabelObjectFlag = true;
			object2->sceneOffContextLabelObject = m_tmpMap->GetLabelObject(object2->id);
			m_tmpMap->ClearLabels();
		}


		//compute the intersection between the objects
		if (!interactionData.intersectionObject) interactionData.intersectionObject = InteractionDataType::LabelObjectType::New();
		interactionData.nbOverlappingPixels = GetLabelObjectIntersection<LabelObjectType>(object1->sceneOffContextLabelObject, object2->sceneOffContextLabelObject, interactionData.intersectionObject);
		
		//ok. if the intersection is not empty, compute the cost.
		if (interactionData.nbOverlappingPixels>0) {
			interactionData.interactionCostFlag = true;
			interactionData.interactionCost = m_costFunction->Evaluate(object1->sceneOffContextLabelObject->Size(), object2->sceneOffContextLabelObject->Size(), interactionData.nbOverlappingPixels); 
		}
		else {interactionData.interactionCostFlag = false; interactionData.nbOverlappingPixels=0;}
	}

	/** Turn off interaction monitoring 
	* Remove all existing interaction data, and stop monitoring them
	* This can be useful for collective re-arrangement algorithms, where multiple objects are modified 'simultaneously'
	*/
	void TurnOffInteractionManagement() {
		m_monitorInteractionsOnTheFly=true;
	}
	
	/** Turn on interaction monitoring (nothing happens if it is already ON)
	* Recompute all existing interaction data, and turn on their on-the-fly monitoring
	* This can be useful for collective re-arrangement algorithms, where multiple objects are modified 'simultaneously'	
	*/
	void TurnOnInteractionManagement() {
		if (!m_monitorInteractionsOnTheFly) {
			m_monitorInteractionsOnTheFly=true;
			SceneObjectIterator<SceneType> it1(m_scene), it2(m_scene);
			for (it1.GoToBegin() ; !it1.IsAtEnd() ; ++it1) {
				for (it2 = ++it1 ; !it2.IsAtEnd() ; ++it2) {
					if ( !TestBoundingBoxesIntersection_NoCheck(it1.GetObject()->obj->GetPhysicalBoundingBox(), it2.GetObject()->obj->GetPhysicalBoundingBox()) ) continue;
					//if the bboxes intersect, then do the exact test.
					InteractionDataType interactionData;
					ComputePairWiseObjectInteractionData(it1.GetObject(), it2.GetObject(), interactionData);
					if ( !interactionData.interactionCostFlag ) continue;
					//Register the interaction, bilaterally: store the information in both objects
					it1.GetObject()->interactionData[it2.GetID()] = interactionData;
					it2.GetObject()->interactionData[it1.GetID()] = interactionData;
				}
			}			
		}
	}

protected:	
	LabelImageObjectOverlap() : ObjectInteractionManager() { m_tmpMap = LabelMapType::New(); m_tmpMapFlag = false; }
	virtual ~LabelImageObjectOverlap() {}

	typename LabelMapType::Pointer m_tmpMap; bool m_tmpMapFlag;
private:
	LabelImageObjectOverlap(const Self&);	//purposely not implemented
	const Self & operator=( const Self & );	//purposely not implemented
};


} // namespace psciob

#endif /* __LABELIMAGEOBJECTOVERLAP_H_ */
