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
* \file SceneEnergy.txx
* \author Rémi Blanc 
* \date 25. October 2011
*/

#ifndef SCENEENERGY_TXX_
#define SCENEENERGY_TXX_

#include "SceneEnergy.h"

namespace psciob {


template<class TScene, class TReferenceImage>
SceneEnergy<TScene, TReferenceImage>::SceneEnergy() {
	m_UseObjectPrior = true;		m_weightObjectPrior = 1;
	m_UseInteractionPrior = true;	m_weightInteractionPrior = 1;
	m_UseGlobalPrior = true;		m_weightGlobalPrior = 1;
	m_UseSingleObjectImagePrior = false; m_weightSingleObjectImageFitPrior = 1;
	m_UseImagePrior = false;		m_weightImageFitPrior = 1;
	m_useTypeSpecificMetric = false; //by default, use the same metric for all object types.

	m_sceneToImageMetric = 0;
	m_singleObjectSceneToImageMetric = 0;
}

//
//
//
template<class TScene, class TReferenceImage>	
void 
SceneEnergy<TScene, TReferenceImage>
::SetTypeSpecificObjectToImageCostFunction(unsigned int i, SingleObjectSceneToImageCostFunctionType *metric) {
	m_useTypeSpecificMetric = true;

	std::pair<unsigned int, SingleObjectSceneToImageCostFunctionType::Pointer> tmp_pair;
	tmp_pair.first = i;	tmp_pair.second = metric;
	//WARNING: if the key is already present, the metric is NOT modified!
	m_listObjectSpecificMetrics.insert( m_listObjectSpecificMetrics.end(), tmp_pair );
}

//
//
//
template<class TScene, class TReferenceImage>
double
SceneEnergy<TScene, TReferenceImage>
::GetSceneGlobalCost() {	
	if (m_UseGlobalPrior) { return  (m_scene->GetGlobalPriorCost()); }
	else return 0;
}

//
//
//
template<class TScene, class TReferenceImage>
double
SceneEnergy<TScene, TReferenceImage>
::GetAverageObjectPriorCost() {
	if (m_UseObjectPrior) {
		if (m_scene->GetNumberOfObjects()==0) return 0;
		double sumObjectPrior = 0; 
		SceneObjectIterator<SceneType> it(m_scene);
		for (it.GoToBegin() ; !it.IsAtEnd() ; ++it) {
			sumObjectPrior += m_scene->GetObjectPriorCost( it.GetID() );
		}
		//for (SceneType::ObjectSetIteratorType it = m_scene->GetObjectSet()->begin() ; it!=m_scene->GetObjectSet()->end() ; it++) {
		//	
		//	//if (it->second->priorCostFlag) {
		//	//	sumObjectPrior += it->second->priorCost;
		//	//	//CLEAN: temporary sanity check
		//	//	//if (it->second->priorCost + m_objectTypesLibrary->GetObjectPDF( it->second->objectTypeId, PDF_OBJECTLIKELIHOOD )->GetLogLikelihood(it->second->object->GetParameters()) > TINY) std::cout<<"WRONGLY ASSUMING THE OBJECT PRIOR IS UP TO DATE : cache = "<<it->second->priorCost<<" vs loglikelihood = "<<-m_objectTypesLibrary->GetObjectPDF( it->second->objectTypeId, PDF_OBJECTLIKELIHOOD )->GetLogLikelihood(it->second->object->GetParameters())<<std::endl; 
		//	//}
		//	//else {//update the value if necessary ;  use ( - loglikelihood ) so that high value means bad object... i.e. generate a COST function
		//	//	double value = - m_objectTypesLibrary->GetObjectPDF( it->second->objectTypeId, PDF_OBJECTLIKELIHOOD )->GetLogLikelihood(it->second->object->GetParameters());
		//	//	it->second->priorCost = value;	it->second->priorCostFlag = true;
		//	//	sumObjectPrior += value;
		//	//}
		//}
		return sumObjectPrior / static_cast<double>( m_scene->GetNumberOfObjects() );
	}
	else return 0;
}

//
//
//
template<class TScene, class TReferenceImage>
double
SceneEnergy<TScene, TReferenceImage>
::GetSumObjectPriorCost() {
	if (m_UseObjectPrior) {
		if (m_scene->GetNumberOfObjects()==0) return 0;
		double sumObjectPrior = 0; 
		SceneObjectIterator<SceneType> it(m_scene);
		for (it.GoToBegin() ; !it.IsAtEnd() ; ++it) {
			sumObjectPrior += m_scene->GetObjectPriorCost( it.GetID() );
		}
		return sumObjectPrior;
	}
	else return 0;
}

//
//
//
template<class TScene, class TReferenceImage>
double
SceneEnergy<TScene, TReferenceImage>
::GetAverageObjectInteractionsCost() {
	if (m_UseInteractionPrior) {
		if (m_scene->GetNumberOfObjects()==0) return 0;
		double sumObjectInteractions = 0, sumCurrentObject=0; //iterate over all objects of the scene
		SceneObjectIterator<SceneType> it(m_scene);
		for (it.GoToBegin() ; !it.IsAtEnd() ; ++it) {
			sumObjectInteractions += m_scene->GetObjectTotalInteractionCost( it.GetID() );
		}
		//for (SceneType::ObjectSetIteratorType it = m_scene->GetObjectSet()->begin() ; it!=m_scene->GetObjectSet()->end() ; it++) {
		//	sumObjectInteractions += m_scene->GetObjectTotalInteractionCost(it);
		//	//if ( it->second->interactions.size()>0 ) {
		//	//	sumCurrentObject=0;		//check the list of interacting objects...
		//	//	for (SceneType::ObjectInteractionMapType::iterator mapit = it->second->interactions.begin() ; mapit!=it->second->interactions.end() ; mapit++) {
		//	//		sumCurrentObject += m_interactionManager->GetPairWiseInteractionValue(it, m_scene->GetIteratorFromLabel(mapit->first));
		//	//	}
		//	//	sumObjectInteractions+=sumCurrentObject;
		//	//}
		//}
		return sumObjectInteractions / static_cast<double>( m_scene->GetNumberOfObjects() );
	}
	else return 0;
}

//
//
//
template<class TScene, class TReferenceImage>
double
SceneEnergy<TScene, TReferenceImage>
::GetSumObjectInteractionsCost() {
	if (m_UseInteractionPrior) {
		if (m_scene->GetNumberOfObjects()==0) return 0;
		double sumObjectInteractions = 0, sumCurrentObject=0; //iterate over all objects of the scene
		SceneObjectIterator<SceneType> it(m_scene);
		for (it.GoToBegin() ; !it.IsAtEnd() ; ++it) {
			sumObjectInteractions += m_scene->GetObjectTotalInteractionCost( it.GetID() );
		}
		return sumObjectInteractions;
	}
	else return 0;
}


//
//
//
template<class TScene, class TReferenceImage>
double
SceneEnergy<TScene, TReferenceImage>
::GetAverageSingleObjectSceneToImageCost() {
	if (m_UseSingleObjectImagePrior) { 
		if (m_scene->GetNumberOfObjects()==0) return 0;
		double sumObjectCosts = 0, costCurrentObject=0; //iterate over all objects of the scene			
		SceneObjectIterator<SceneType> it(m_scene);
		if ( m_useTypeSpecificMetric ) { //browse all objects, look at their types, compute the object specific metric and average the cost of every individual object.
			std::cout<<"SceneEnergy::GetSceneToImageMetricValue : the use of type-specific object-to-image cost functions has not been fully tested yet..."<<std::endl;
			for (it.GoToBegin() ; !it.IsAtEnd() ; ++it) {
				TypeSpecificObjectToImageCostFunctionListType::iterator tmpIt = m_listObjectSpecificMetrics.find( it.GetObject()->objectTypeId );
				if (tmpIt == m_listObjectSpecificMetrics.end()) {
					throw DeformableModelException("SceneEnergy::GetSceneToImageMetricValue() - requesting type specific object-to-image cost functions, but not all object types have an associated metric !!");
				}
				else {
					tmpIt->second->SelectObject(it.GetID());
					costCurrentObject = tmpIt->second->GetValue();
				}
				sumObjectCosts+=costCurrentObject;
			}
		}//
		else { 
			for (it.GoToBegin() ; !it.IsAtEnd() ; ++it) {
				m_singleObjectSceneToImageMetric->SelectObject(it.GetID());
				costCurrentObject = m_singleObjectSceneToImageMetric->GetValue();
				sumObjectCosts+=costCurrentObject;
			}
		}
		return sumObjectCosts/static_cast<double>(m_scene->GetNumberOfObjects());
	}
	else return 0;
}

//
//
//
template<class TScene, class TReferenceImage>
double
SceneEnergy<TScene, TReferenceImage>
::GetSumSingleObjectSceneToImageCost() {
	if (m_UseSingleObjectImagePrior) { 
		if (m_scene->GetNumberOfObjects()==0) return 0;
		double sumObjectCosts = 0, costCurrentObject=0; //iterate over all objects of the scene			
		SceneObjectIterator<SceneType> it(m_scene);
		if ( m_useTypeSpecificMetric ) { //browse all objects, look at their types, compute the object specific metric and average the cost of every individual object.
			std::cout<<"SceneEnergy::GetSceneToImageMetricValue : the use of type-specific object-to-image cost functions has not been fully tested yet..."<<std::endl;
			for (it.GoToBegin() ; !it.IsAtEnd() ; ++it) {
				TypeSpecificObjectToImageCostFunctionListType::iterator tmpIt = m_listObjectSpecificMetrics.find( it.GetObject()->objectTypeId );
				if (tmpIt == m_listObjectSpecificMetrics.end()) {
					throw DeformableModelException("SceneEnergy::GetSceneToImageMetricValue() - requesting type specific object-to-image cost functions, but not all object types have an associated metric !!");
				}
				else {
					tmpIt->second->SelectObject(it.GetID());
					costCurrentObject = tmpIt->second->GetValue();
				}
				sumObjectCosts+=costCurrentObject;
			}
		}//
		else { 
			for (it.GoToBegin() ; !it.IsAtEnd() ; ++it) {
				m_singleObjectSceneToImageMetric->SelectObject(it.GetID());
				costCurrentObject = m_singleObjectSceneToImageMetric->GetValue();
				sumObjectCosts+=costCurrentObject;
			}
		}
		return sumObjectCosts;
	}
	else return 0;
}
//
//
//
template<class TScene, class TReferenceImage>
double
SceneEnergy<TScene, TReferenceImage>
::GetSceneToImageCost() {
	if (m_UseImagePrior) { 
		return m_sceneToImageMetric->GetValue(); 
	}
	else return 0;
}


} // namespace psciob

#endif /* SCENEENERGY_TXX_ */
