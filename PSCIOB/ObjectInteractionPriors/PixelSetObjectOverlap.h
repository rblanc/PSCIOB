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
* \file PixelSetObjectOverlap.h
* \author Rémi Blanc 
* \date 13. March 2011
*/

#ifndef __PIXELSETOBJECTOVERLAP_H_
#define __PIXELSETOBJECTOVERLAP_H_

#include "ObjectInteractionManager.h"
#include "ObjectPixelSet.h"

namespace psciob {


/**\class PixelSetObjectOverlap
* \brief PixelSetObjectOverlap: detect objects intersection based on their pixelset representations
* requires the scene to use a DataContainer that enables the storage of PixelSet (e.g. CostsAndPixelSetContainer)
*                           and an Interaction container authorizing also PixelSet (e.g. PixelIntersectionContainer)
*
* \warning: this class is becoming obsolete, prefer the use of LabelObjectOverlap, which ought to be faster
*/

//CONCRETE CLASS
template<class TScene>
class PixelSetObjectOverlap : public ObjectInteractionManager<TScene> {
public:
	/** Standard class typedefs. */
	typedef PixelSetObjectOverlap          Self;
	typedef ObjectInteractionManager       Superclass;
	typedef itk::SmartPointer<Self>        Pointer;
	typedef itk::SmartPointer<const Self>  ConstPointer;
	/** Run-time type information (and related methods). */
	itkTypeMacro(PixelSetObjectOverlap, ObjectInteractionManager);
	itkNewMacro(PixelSetObjectOverlap);

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
		if (!m_tmpMapFlag) { m_tmpMap->SetRegions( m_scene->GetSceneImageRegion() ); m_tmpMap->SetSpacing( m_scene->GetSceneSpacing() ); m_tmpMap->SetOrigin( m_scene->GetSceneOrigin() ); m_tmpMapFlag=true; }
		if (!InsertSingleObjectLabelMapIntoAnother<DeformableObjectType::LabelMapType, LabelMapType>(object1->GetObjectAsLabelMap(), m_tmpMap, 1)) { throw DeformableModelException("LabelImageObjectOverlap::ComputePairWiseObjectInteractionData -- problem inserting the object in the map... SHOULD NEVER HAPPEN!!"); }
		if (!InsertSingleObjectLabelMapIntoAnother<DeformableObjectType::LabelMapType, LabelMapType>(object2->GetObjectAsLabelMap(), m_tmpMap, 2)) { throw DeformableModelException("LabelImageObjectOverlap::ComputePairWiseObjectInteractionData -- problem inserting the object in the map... SHOULD NEVER HAPPEN!!"); }
		//2: get the pixel sets...
		ObjectPixelSet::SetType set1, set2;
		ConvertSingleObjectLabelMapToPixelList<LabelMapType>( m_tmpMap, set1, 1 );
		ConvertSingleObjectLabelMapToPixelList<LabelMapType>( m_tmpMap, set2, 2 );
		//3: compute the intersection
		std::set_intersection(set1.begin(), set1.end(), set2.begin(), set2.end(), std::inserter(interactionData.intersectionPixels, interactionData.intersectionPixels.end()) );
		if (interactionData.intersectionPixels.size()>0) {
			interactionData.nbOverlappingPixels = interactionData.intersectionPixels.size();
			interactionData.interactionCostFlag = true;
			interactionData.interactionCost = m_costFunction->Evaluate(set1.size(), set2.size(), interactionData.nbOverlappingPixels); 
		}
		else interactionData.interactionCostFlag = false;

		m_tmpMap->ClearLabels();
	}

	/** given two objects from the scene, compute what their interaction cost would be 
	* provided in case some information needs to be cached. the default version is an alias for ComputePairWiseObjectInteractionData(DeformableObjectType *object1, DeformableObjectType *object2)
	*/
	void ComputePairWiseObjectInteractionData(ObjectInScene *object1, ObjectInScene *object2, InteractionDataType &interactionData) {
		if (!object1->objectData.offContextPixelSetFlag) {
			if (!ConvertSingleObjectLabelMapToPixelList<LabelMapType>( m_scene->GetSceneAsLabelMap(), object1->objectData.offContextPixelSet, object1->id )) { throw DeformableModelException("LabelImageScene::DrawObjectInScene -- problem computing the pixel set... SHOULD NEVER HAPPEN!!"); }
			object1->objectData.offContextPixelSetFlag = true;
		}
		if (!object2->objectData.offContextPixelSetFlag) {
			if (!ConvertSingleObjectLabelMapToPixelList<LabelMapType>( m_scene->GetSceneAsLabelMap(), object2->objectData.offContextPixelSet, object2->id )) { throw DeformableModelException("LabelImageScene::DrawObjectInScene -- problem computing the pixel set... SHOULD NEVER HAPPEN!!"); }
			object2->objectData.offContextPixelSetFlag = true;
		}
		std::set_intersection(object1->objectData.offContextPixelSet.begin(), object1->objectData.offContextPixelSet.end(), object2->objectData.offContextPixelSet.begin(), object2->objectData.offContextPixelSet.end(), std::inserter(interactionData.intersectionPixels, interactionData.intersectionPixels.end()) );
		if (interactionData.intersectionPixels.size()>0) {
			interactionData.nbOverlappingPixels = interactionData.intersectionPixels.size();
			interactionData.interactionCostFlag = true;
			interactionData.interactionCost = m_costFunction->Evaluate(object1->objectData.offContextPixelSet.size(), object2->objectData.offContextPixelSet.size(), interactionData.nbOverlappingPixels); 
		}
		else {
			interactionData.interactionCostFlag = false;
		}
	}


protected:	
	PixelSetObjectOverlap() : ObjectInteractionManager() {
		m_tmpMap = LabelMapType::New(); m_tmpMapFlag = false;
	};
	virtual ~PixelSetObjectOverlap() {};	

	typename LabelMapType::Pointer m_tmpMap; bool m_tmpMapFlag;
private:
	PixelSetObjectOverlap(const Self&);	//purposely not implemented
	const Self & operator=( const Self & );	//purposely not implemented
};


} // namespace psciob

#endif /* __PIXELSETOBJECTOVERLAP_H_ */


