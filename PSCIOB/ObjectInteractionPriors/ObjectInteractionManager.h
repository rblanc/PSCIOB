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
* \file ObjectInteractionManager.h
* \author Rémi Blanc 
* \date 21. October 2011
*/

#ifndef OBJECTINTERACTIONMANAGER_H_
#define OBJECTINTERACTIONMANAGER_H_


#include "BaseScene.h"
#include "PairWiseIntersectionCostFunctions.h"

namespace psciob {

/**\class ObjectInteractionManager 
* \brief ObjectInteractionManager: Base class for managing interactions between objects
* 
* this class works together with a scene, so that it may share data with, cache some values, etc...
*
*/


//ABSTRACT CLASS
template<class TScene>
class ObjectInteractionManager : public itk::LightObject {
public:
	/** Standard class typedefs. */
	typedef ObjectInteractionManager		Self;
	typedef itk::LightObject				Superclass;
	typedef itk::SmartPointer<Self>			Pointer;
	typedef itk::SmartPointer<const Self>	ConstPointer;
	/** Run-time type information (and related methods). */
	itkTypeMacro(ObjectInteractionManager,itk::LightObject);

	typedef TScene        SceneType;

	typedef typename SceneType::DeformableObjectType      DeformableObjectType;
	typedef typename SceneType::IDType                    IDType;
	typedef typename SceneType::ObjectInScene             ObjectInScene;
	typedef typename SceneType::ObjectInteractionMapType  ObjectInteractionMapType;
	typedef typename SceneType::ObjectInteractionPairType ObjectInteractionPairType;
	typedef typename SceneType::BinaryImageType           ObjectImageType;
	typedef typename SceneType::LabelMapType              LabelMapType;
	typedef typename SceneType::LabelObjectType           LabelObjectType;

	/** Type of information associated to an interaction between 2 objects */
	typedef typename SceneType::InteractionDataType       InteractionDataType;
	typedef typename SceneType::ObjectInteractionMapType  ObjectInteractionMapType;
	typedef typename SceneType::ObjectInteractionPairType ObjectInteractionPairType;

	/** Attach the scene to the InteractionManager ; the user doesn't need to call this function
	* \todo make it protected and use friendship to enable usage by the Scene
	*/
	virtual void SetScene(SceneType* scene) { m_scene = scene; }

	/** Clear any scene-related cached information, as the scene is getting cleared itself 
	* use friendship to hide this function from the public interface...
	*/
	virtual void Clear() {}

	/** Specifies a cost function to be used to compute the cost of the intersection */
	virtual void SetIntersectionCostFunction(PairWiseIntersectionCostFunction *f) { m_costFunction = f; }


	/** Add an object to the Interaction Manager */
	virtual void AddObject(ObjectInScene *objectPtr) {
		if (!m_monitorInteractionsOnTheFly) return;
		//test all objects of the scene
		SceneObjectIterator<SceneType> it(m_scene);
		for (it.GoToBegin() ; !it.IsAtEnd() ; ++it) { 
			if ( objectPtr->id != it.GetID() ) {
				//first, check for bbox
				if ( !TestBoundingBoxesIntersection_NoCheck(objectPtr->obj->GetPhysicalBoundingBox(), it.GetObject()->GetPhysicalBoundingBox()) ) continue;
				//if the bboxes intersect, then do the exact test.
				InteractionDataType interactionData;
				ComputePairWiseObjectInteractionData(objectPtr, it.GetObjectInScene(), interactionData);
				if ( !interactionData.interactionCostFlag ) continue;
				//Register the interaction, bilaterally: store the information in both objects
				objectPtr->interactionData[it.GetID()] = interactionData;
				it.GetObjectInScene()->interactionData[objectPtr->id] = interactionData;
			}
		}
	}

	/** Remove an object: the base is to delete all interactions this object has with others (non-bilaterally, because the object is expected to be deleted anyway)
	* May be overloaded by child classes
	*/
	virtual void RemoveObject(ObjectInScene *objectPtr) {
		if (!m_monitorInteractionsOnTheFly) return;
		for (ObjectInteractionMapType::iterator it = objectPtr->interactionData.begin() ; it != objectPtr->interactionData.end() ; ++it) {
			//??keep this sanity check??
			//if ( m_scene->GetObject( it->first )->interactionData.erase( objectPtr->id ) ==0 ) throw DeformableModelException( "ObjectInteractionManager::RemoveObject found a non-bilateral interaction... should never happen!" + stringify(objectPtr->id) + " and " + stringify(it->first) ); 
			m_scene->GetObject( it->first )->interactionData.erase( objectPtr->id ); //this would be enough if I remove the sanity check.
		}
		//objectPtr->interactionData.clear(); //
	}

	/** Update the interaction of an object after a change in its parameters */
	virtual void ModifyObjectParameters(ObjectInScene *objectPtr, ObjectInScene *newObject) {
		if (!m_monitorInteractionsOnTheFly) return;
		//test all objects of the scene
		SceneObjectIterator<SceneType> it(m_scene);
		for (it.GoToBegin() ; !it.IsAtEnd() ; ++it) { 
			if ( objectPtr->id != it.GetID() ) {
				//first, check if their bbox intersect
				if ( !TestBoundingBoxesIntersection_NoCheck(newObject->obj->GetPhysicalBoundingBox(), it.GetObject()->GetPhysicalBoundingBox()) ) { UnregisterInteraction(objectPtr, it.GetObjectInScene()); continue; }
				//if the bboxes intersect, then do the exact test.
				InteractionDataType interactionData;
				ComputePairWiseObjectInteractionData(newObject, it.GetObjectInScene(), interactionData);
				if ( !interactionData.interactionCostFlag ) { UnregisterInteraction(objectPtr, it.GetObjectInScene()); continue; }
				//Register the interaction, bilaterally: store the information in both objects
				newObject->interactionData[it.GetID()] = interactionData;
				it.GetObjectInScene()->interactionData[objectPtr->id] = interactionData;
			}
		}
	}


	/** given two objects from the scene, return their interaction data (DOES NOT COMPUTE ANYTHING) */
	virtual InteractionDataType GetPairWiseInteractionData(ObjectInScene *object1, ObjectInScene *object2) {
		ObjectInteractionMapType::iterator it1 = object1->interactionData.find(object2->id);
		if (it1==object1->interactionData.end()) return m_emptyInteraction;
		else {			
			if (it1->second.interactionCostFlag) { 
				//???check that the interaction is correctly registered bilaterally???
				if (!object2->interactionData[object1->id].interactionCostFlag) throw DeformableModelException("ObjectInteractionManager::GetPairWiseInteractionData... found a non-bilateral interaction ... SHOULD NEVER HAPPEN"); 
				return it1->second; 
			}
			else { return m_emptyInteraction; }
		}
	}

	/** Given an arbitrary object, look in the scene with which object may interact with it, based on their axes aligned bounding boxes*/
	virtual std::vector<IDType> IdentifyInteractingObjectsInScene(DeformableObjectType *object, IDType ignoredLabel = 0) {
		std::vector<IDType> listPotentiallyInteractingObjects;
		SceneObjectIterator<SceneType> it(m_scene);
		for (it.GoToBegin() ; !it.IsAtEnd() ; ++it) {
			if ( it.GetID() != ignoredLabel ) {
				if ( TestBoundingBoxesIntersection_NoCheck(object->GetPhysicalBoundingBox(), it.GetObject()->GetPhysicalBoundingBox()) )
					listPotentiallyInteractingObjects.push_back(it.GetID());
			}
		}
		return listPotentiallyInteractingObjects;
	}

	/** given two arbitrary objects, compute how they interact
	* does not perform a bounding box check, which should therefore be made prior to calling this method...
	* ( see the function: TestBoundingBoxesIntersection_NoCheck (or TestBoundingBoxesIntersection for a safer version) )
	*/
	virtual void ComputePairWiseObjectInteractionData(DeformableObjectType *object1, DeformableObjectType *object2, InteractionDataType &interactionData) = 0;

	/** given two objects from the scene, compute how they interact 
	* does not perform a bounding box check, which should therefore be made prior to calling this method...
	* ( see the function: TestBoundingBoxesIntersection_NoCheck (or TestBoundingBoxesIntersection for a safer version) )
	* \warning the scene should be set before calling this function (no checks are performed)
	*/
	virtual inline void ComputePairWiseObjectInteractionData(ObjectInScene *object1, ObjectInScene *object2, InteractionDataType &interactionData) {
		return ComputePairWiseObjectInteractionData(object1->obj, object2->obj, interactionData);
	}

	/** if there used to be an interaction between those 2 objects, erase it. */
	inline void UnregisterInteraction(ObjectInScene *object1, ObjectInScene *object2) {
		//CHECK if the interaction used to exist, and unregister it otherwise.
		ObjectInteractionMapType::iterator it1 = object1->interactionData.find(object2->id);
		if ( it1!=object1->interactionData.end() ) { //the interaction used to exist
			ObjectInteractionMapType::iterator it2 = object2->interactionData.find(object1->id);
			if (it2!=object2->interactionData.end()) {
				//erase both interactions
				object1->interactionData.erase(it1);
				object2->interactionData.erase(it2);
			}
			else throw DeformableModelException("ObjectInteractionManager::UnregisterInteraction... found a non-bilateral interaction ... SHOULD NEVER HAPPEN"); 
		}
	}

	/** Turn off interaction monitoring 
	* Remove all existing interaction data, and stop monitoring them
	* This can be useful for collective re-arrangement algorithms, where multiple objects are modified 'simultaneously'
	*/
	virtual void TurnOffInteractionManagement() {
		m_monitorInteractionsOnTheFly=true;
		SceneObjectIterator<SceneType> it(m_scene);
		for (it.GoToBegin() ; !it.IsAtEnd() ; ++it) {
			it.GetObjectInScene()->interactionData.clear();
		}
		
	}
	
	/** Turn on interaction monitoring (nothing happens if it is already ON)
	* Recompute all existing interaction data, and turn on their on-the-fly monitoring
	* This can be useful for collective re-arrangement algorithms, where multiple objects are modified 'simultaneously'	
	*/
	virtual void TurnOnInteractionManagement() {
		if (!m_monitorInteractionsOnTheFly) {
			m_monitorInteractionsOnTheFly=true;
			SceneObjectIterator<SceneType> it1(m_scene), it2(m_scene);
			for (it1.GoToBegin() ; !it1.IsAtEnd() ; ++it1) {
				for (it2 = ++it1 ; !it2.IsAtEnd() ; ++it2) {
					if ( !TestBoundingBoxesIntersection_NoCheck(it1.GetObject()->GetPhysicalBoundingBox(), it2.GetObject()->GetPhysicalBoundingBox()) ) continue;
					//if the bboxes intersect, then do the exact test.
					InteractionDataType interactionData;
					ComputePairWiseObjectInteractionData(it1.GetObject(), it2.GetObject(), interactionData);
					if ( !interactionData.interactionCostFlag ) continue;
					//Register the interaction, bilaterally: store the information in both objects
					it1.GetObjectInScene()->interactionData[it2.GetID()] = interactionData;
					it2.GetObjectInScene()->interactionData[it1.GetID()] = interactionData;
				}
			}			
		}
	}
	
	
protected:	
	ObjectInteractionManager() {
		m_scene = 0;
		m_costFunction = static_cast<PairWiseIntersectionCostFunction*>(PairWiseOverlapDiceCoefficient::New().GetPointer()); //default is the dice coefficient.
		m_monitorInteractionsOnTheFly = true;
	};
	virtual ~ObjectInteractionManager() {};	

	typename SceneType::Pointer m_scene;
	PairWiseIntersectionCostFunction::Pointer m_costFunction;
	InteractionDataType m_emptyInteraction;
	bool m_monitorInteractionsOnTheFly;

private:
	ObjectInteractionManager(const Self&);	//purposely not implemented
	const Self & operator=( const Self & );	//purposely not implemented
};


} // namespace psciob

#endif /* OBJECTINTERACTIONMANAGER_H_ */