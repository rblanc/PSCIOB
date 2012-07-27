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
* \file SingleObjectEnergy.h
* \author Rémi Blanc 
* \date 20. December 2011
*/

#ifndef SINGLEOBJECTENERGY_H_
#define SINGLEOBJECTENERGY_H_


#include "BaseScene.h"
#include "SingleObjectSceneToImageCostFunction.h"

namespace psciob {


/** \brief SingleObjectEnergy: Class for associating an energy, or potential, to a single object in the scene
 * the idea is to be able to switch on/off a number of energy terms, including:
 * - object prior							<- use the prior PDF from the scene object library "ObjectTypesLibrary"
 * - object interactions					<- using the "ObjectInteractionManager" which is associated to the scene (this enables caching the pairs of interacting objects during scene modifications)
 * - goodness of fit to a target image		<- "SingleObjectSceneToImageCostFunction" needs to be provided by the user
 * there is the possibilitiy to use different image metrics for different object types...
 * and each provided SingleObjectSceneToImageCostFunction can be tuned to InContext or OffContext behavior
 *
 * The object Energy is computed as a weighted sum of the different terms (weights can be 0)
*/


template<class TScene, class TReferenceImage = typename TScene::BinaryImageType>
class SingleObjectEnergy : public SceneCostFunction_Base<TScene> {
public:
	/** Standard class typedefs. */
	typedef SingleObjectEnergy				Self;
	typedef SceneCostFunction_Base<TScene>	Superclass;
	typedef itk::SmartPointer<Self>			Pointer;
	/** Run-time type information (and related methods). */
	itkTypeMacro(SingleObjectEnergy,SceneCostFunction_Base);
	itkNewMacro(Self);

	static const unsigned int SceneDimension =	TScene::Dimension;
	typedef TScene								SceneType;
	typedef typename SceneType::IDType			IDType;
	typedef TReferenceImage						ReferenceImageType;

	typedef typename SceneType::ScenePriorType					SceneGlobalPriorType;
	typedef	typename SceneType::ObjectTypesLibraryType			ObjectTypesLibraryType;
	typedef typename SceneType::ObjectInteractionManagerType	ObjectInteractionManagerType;

	typedef SingleObjectSceneToImageCostFunction<TScene, TReferenceImage>			SingleObjectSceneToImageCostFunctionType;
	typedef SceneToImageCostFunction<TScene, TReferenceImage>						SceneToImageCostFunctionType;
	typedef std::map<unsigned int, typename SingleObjectSceneToImageCostFunctionType::Pointer>	TypeSpecificObjectToImageCostFunctionListType;

	/** Create a clone (= an exact, independant copy) of the current cost function */
	BaseClassPointer CreateClone() {
		Pointer clonePtr = Self::New();
		if (!m_transformFunction) {} else {clonePtr->SetNormalizationFunction(m_transformFunction);}
		if (!m_scene) {} else {clonePtr->SetScene(m_scene);}
		clonePtr->SelectObject(m_requestedLabel);

		if (!m_singleObjectSceneToImageMetric) {} else {
			clonePtr->SetSingleObjectSceneToImageCostFunction( static_cast<SingleObjectSceneToImageCostFunctionType*>(m_singleObjectSceneToImageMetric->CreateClone().GetPointer()), m_weightSingleObjectImageFitPrior );
		}
		for (TypeSpecificObjectToImageCostFunctionListType::iterator it=m_listObjectSpecificMetrics.begin() ; it!=m_listObjectSpecificMetrics.end() ; ++it) {
			clonePtr->SetTypeSpecificObjectToImageCostFunction( it->first, static_cast<SingleObjectSceneToImageCostFunctionType*>(it->second->CreateClone().GetPointer()) );
		}
		clonePtr->m_UseObjectPrior = m_UseObjectPrior;           clonePtr->m_weightObjectPrior = m_weightObjectPrior;
		clonePtr->m_UseInteractionPrior = m_UseInteractionPrior; clonePtr->m_weightInteractionPrior = m_weightInteractionPrior;
		clonePtr->m_UseSingleObjectImagePrior = m_UseSingleObjectImagePrior; clonePtr->m_weightSingleObjectImageFitPrior = m_weightSingleObjectImageFitPrior;
		clonePtr->m_useTypeSpecificMetric = m_useTypeSpecificMetric;

		return static_cast<BaseClass*>( clonePtr );
	}

	/** Set the scene on which to make the measurements 
	* and propagates to any other members that may also be working on this scene
	* maybe overloaded by child classes
	*/
	virtual void SetScene(SceneType *scene) {
		Superclass::SetScene(scene);

		//make sure the other functions work on the same scene.
		if (!m_singleObjectSceneToImageMetric) {} else {m_singleObjectSceneToImageMetric->SetScene(scene);}
		for (unsigned i=0 ; i<m_listObjectSpecificMetrics.size() ; i++) m_listObjectSpecificMetrics[i]->SetScene(scene);
	}

	/** Associate a similarity measure to the scene ; calling it a second time will invalidate all object datacosts 
	 * (assuming the newly set cost function is different from the previous one) 
	 * It invalidates any type-specific metrics if any were set - data costs of all objects get invalidated in that case.
	 */
	void SetSingleObjectSceneToImageCostFunction(SingleObjectSceneToImageCostFunctionType *metric, double weight = 1)	{ 
		//if a metric was already existing, assume the current is different and disable all cached data cost value		
		//if (m_UseSingleObjectImagePrior) m_scene->InvalidateObjectDataCosts();
		m_singleObjectSceneToImageMetric = metric; m_weightSingleObjectImageFitPrior = weight;	m_UseSingleObjectImagePrior = true;
		if (m_useTypeSpecificMetric) ResetObjectSpecificCostFunctions();
		//make sure the scene is the same as that of the metric...
		this->SetScene( metric->GetScene() );		
	}

	/** Clear any type-specific single object to image cost function - invalidates all cached object datacosts */
	void ResetObjectSpecificCostFunctions()	{ 
		m_listObjectSpecificMetrics.clear(); m_useTypeSpecificMetric = false; 
		//disable all cached data cost value
		//m_scene->InvalidateObjectDataCosts();
	}
	/** Start / continue defining type-specific single object to image cost functions
	 * e.g. use a different cost function for the goodness-of-fit of a disk, than for the goodness-of-fit of a square
	 */
	void SetTypeSpecificObjectToImageCostFunction(unsigned int i, SingleObjectSceneToImageCostFunctionType *metric);


	/** Switch On/Off the usage of the different cost terms */
	void UseObjectPrior(bool b=true)				{m_UseObjectPrior=b;}
	void UseObjectInteractionPrior(bool b=true)		{m_UseInteractionPrior=b;}
	void UseSingleObjectSceneToImageCostFunction(bool b=true)	{m_UseSingleObjectImagePrior=b;}

	/** Set the relative weights of the different terms composing the energy 
	 * the output value is a weighted sum of the individual terms.
	 */
	void SetObjectPriorWeight(double weight)				{ m_weightObjectPrior = weight; }
	void SetInteractionPriorWeight(double weight)			{ m_weightInteractionPrior = weight; }
	void SetSingleObjectSceneToImageCostFunctionWeight(double weight)	{ m_weightSingleObjectImageFitPrior = weight; }


	/** Compute the energy for the requested object */
	double GetRawValue();

	/** Get the prior cost for a specified object - independently whether it is used or not in the energy computation */
	inline double GetPriorCostForObject(IDType label)	{ return m_scene->GetObjectPriorCost(label); }

	/** Get the interaction cost for a specified object - independently whether it is used or not in the energy computation */
	inline double GetInteractionCostForObject(IDType label)	{ return m_scene->GetObjectTotalInteractionCost(label); }

	/** Get the data cost for a specified object - independently whether it is used or not in the energy computation
	* \todo is it really worth checking that parameters have been correctly set?
	*/
	double GetSingleObjectCostForObject(IDType label)	{ 
		if (m_useTypeSpecificMetric) {
			TypeSpecificObjectToImageCostFunctionListType::iterator costFunctionIt = m_listObjectSpecificMetrics.find( m_scene->GetObject(label)->objectTypeId );
			if (costFunctionIt == m_listObjectSpecificMetrics.end()) { throw DeformableModelException("SceneEnergy::GetSingleObjectCostForObject() - requesting type specific object-to-image cost functions, but not all object types have an associated metric !!"); }
			else {
				costFunctionIt->second->SelectObject( label );
				return costFunctionIt->second->GetValue();
			}
		}
		else {
			if (!m_singleObjectSceneToImageMetric) throw DeformableModelException("SceneEnergy::GetSingleObjectCostForObject() - a single object cost function must be provided by the user! ");
			m_singleObjectSceneToImageMetric->SelectObject( label );
			return m_singleObjectSceneToImageMetric->GetValue();
		}	
	}

	void PrintDetailsForObject(IDType label) { 
		std::cout<<"energy details for object with label "<<label<<std::endl;
		if (m_UseObjectPrior) std::cout<<"prior cost  : value = "<<GetPriorCostForObject(label)<<"\t, weight = "<<m_weightObjectPrior<<"\t, used? "<<m_UseObjectPrior<<std::endl;
		if (m_UseInteractionPrior) std::cout<<"interaction : value = "<<GetInteractionCostForObject(label)<<"\t, weight = "<<m_weightInteractionPrior<<"\t, used? "<<m_UseInteractionPrior<<std::endl;
		if (m_UseSingleObjectImagePrior) std::cout<<"image fit   : value = "<<GetSingleObjectCostForObject(label)<<"\t, weight = "<<m_weightSingleObjectImageFitPrior<<"\t, used? "<<m_UseSingleObjectImagePrior<<std::endl;
	}


protected:
	SingleObjectEnergy();
	virtual ~SingleObjectEnergy() {};	


	typename SingleObjectSceneToImageCostFunctionType::Pointer	m_singleObjectSceneToImageMetric;
	TypeSpecificObjectToImageCostFunctionListType				m_listObjectSpecificMetrics;

	bool m_UseObjectPrior, m_UseInteractionPrior, m_UseSingleObjectImagePrior, m_useTypeSpecificMetric;
	double m_weightObjectPrior, m_weightInteractionPrior, m_weightSingleObjectImageFitPrior;

private:
	SingleObjectEnergy(const Self&);         //purposely not implemented
	const Self & operator=( const Self & );	 //purposely not implemented
};

} // namespace psciob

#include "SingleObjectEnergy.txx"

#endif /* SINGLEOBJECTENERGY_H_ */
