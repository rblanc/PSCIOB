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
* \file SceneEnergy.h
* \author Rémi Blanc 
* \date 25. October 2011
*/

#ifndef SCENEENERGY_H_
#define SCENEENERGY_H_

#include "SceneCostFunction_Base.h"

//#include "CommonTypes.h"
#include "GeneralUtils.h"
#include "ITKUtils.h"
#include "ScalarFunctions.h"

#include "SceneToImageCostFunction.h"
#include "SingleObjectSceneToImageCostFunction.h"

namespace psciob {

/** \brief SceneEnergy: Class for associating an energy, or potential, to a scene
 * the idea is to be able to switch on/off a number of energy terms, including:
 * - object priors									<- use the prior PDF from the scene object library "ObjectTypesLibrary"
 * - global priors on the statistics of the scene	<- provide a class to do that ~> "ScenePriorManager"
 * - object interactions							<- using the "ObjectInteractionManager" which is associated to the scene (this enables caching the pairs of interacting objects during scene modifications)
 * - goodness of fit of each object to the target image <- "SingleObjectSceneToImageCostFunction"
 * - goodness of fit to a target image				<- "SceneToImageCostFunction", typically an "AverageObjectToImageCostFunction", but could as well be a direct similarity between both images
 *
*/

template<class TScene, class TReferenceImage = typename TScene::BinaryImageType>
class SceneEnergy : public SceneCostFunction_Base<TScene> {
public:
	/** Standard class typedefs. */
	typedef SceneEnergy						Self;
	typedef SceneCostFunction_Base<TScene>	Superclass;
	typedef itk::SmartPointer<Self>			Pointer;
	/** Run-time type information (and related methods). */
	itkTypeMacro(SceneEnergy,SceneCostFunction_Base);
	itkNewMacro(Self);

	static const unsigned int SceneDimension =  TScene::Dimension;
	typedef TScene                              SceneType;
	typedef typename SceneType::IDType          IDType;
	typedef TReferenceImage                     ReferenceImageType;

	typedef typename SceneType::ScenePriorType               SceneGlobalPriorType;
	typedef	typename SceneType::ObjectTypesLibraryType       ObjectTypesLibraryType;
	typedef typename SceneType::ObjectInteractionManagerType ObjectInteractionManagerType;

	typedef SingleObjectSceneToImageCostFunction<TScene, TReferenceImage> SingleObjectSceneToImageCostFunctionType;
	typedef SceneToImageCostFunction<TScene, TReferenceImage>             SceneToImageCostFunctionType;
	typedef std::map<unsigned int, typename SingleObjectSceneToImageCostFunctionType::Pointer> TypeSpecificObjectToImageCostFunctionListType;

	/** Create a clone (= an exact, independant copy) of the current cost function */
	BaseClassPointer CreateClone() {
		Pointer clonePtr = Self::New();
		if (!m_transformFunction) {} else {clonePtr->SetNormalizationFunction(m_transformFunction);}
		if (!m_scene) {} else {clonePtr->SetScene(m_scene);}
		clonePtr->SelectObject(m_requestedLabel);

		//make sure the other functions work on the same scene.
		if (!m_sceneToImageMetric) {} else {
			clonePtr->SetSceneToImageCostFunction( static_cast<SceneToImageCostFunctionType*>(m_sceneToImageMetric->CreateClone().GetPointer()), m_weightImageFitPrior );
		}
		if (!m_singleObjectSceneToImageMetric) {} else {
			clonePtr->SetSingleObjectSceneToImageCostFunction( static_cast<SingleObjectSceneToImageCostFunctionType*>(m_singleObjectSceneToImageMetric->CreateClone().GetPointer()), m_weightSingleObjectImageFitPrior );
		}
		for (TypeSpecificObjectToImageCostFunctionListType::iterator it=m_listObjectSpecificMetrics.begin() ; it!=m_listObjectSpecificMetrics.end() ; ++it) {
			clonePtr->SetTypeSpecificObjectToImageCostFunction( it->first, static_cast<SingleObjectSceneToImageCostFunctionType*>(it->second->CreateClone().GetPointer()) );
		}
		clonePtr->m_UseGlobalPrior = m_UseGlobalPrior;           clonePtr->m_weightGlobalPrior = m_weightGlobalPrior;
		clonePtr->m_UseObjectPrior = m_UseObjectPrior;           clonePtr->m_weightObjectPrior = m_weightObjectPrior;
		clonePtr->m_UseInteractionPrior = m_UseInteractionPrior; clonePtr->m_weightInteractionPrior = m_weightInteractionPrior;
		clonePtr->m_UseSingleObjectImagePrior = m_UseSingleObjectImagePrior; clonePtr->m_weightSingleObjectImageFitPrior = m_weightSingleObjectImageFitPrior;
		clonePtr->m_UseImagePrior = m_UseImagePrior;			clonePtr->m_weightImageFitPrior = m_weightImageFitPrior;
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
		if (!m_sceneToImageMetric) {} else {m_sceneToImageMetric->SetScene(scene);}
		if (!m_singleObjectSceneToImageMetric) {} else {m_singleObjectSceneToImageMetric->SetScene(scene);}		
		for (TypeSpecificObjectToImageCostFunctionListType::iterator it=m_listObjectSpecificMetrics.begin() ; it!=m_listObjectSpecificMetrics.end() ; ++it) {
			it->second->SetScene(scene);
		}
	}


	/** Associate a global similarity measure to the scene 
	 */
	void SetSceneToImageCostFunction(SceneToImageCostFunctionType *metric, double weight = 1) { 
		m_sceneToImageMetric = metric; m_weightImageFitPrior = weight;	m_UseImagePrior = true;
		//make sure the scene is the same as that of the metric...
		this->SetScene( metric->GetScene() );		
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

	/** Set the relative weights of the different terms composing the energy 
	 * the output value is a weighted sum of the individual terms.
	 */
	void SetSceneGlobalPriorWeight(double weight)           { m_weightGlobalPrior = weight; }
	void SetObjectPriorWeight(double weight)                { m_weightObjectPrior = weight; }
	void SetInteractionPriorWeight(double weight)           { m_weightInteractionPrior = weight; }
	void SetSingleObjectSceneToImageCostFunctionWeight(double weight) { m_weightSingleObjectImageFitPrior = weight; }
	void SetSceneToImageCostFunctionWeight(double weight)   { m_weightImageFitPrior = weight; }

	/** Compute the energy */
	double GetRawValue() { //OPTIMIZATION: do GetAverageObjectPriors and GetAverageObjectInteractions in a single pass if both are requested...
//std::cout<<"SceneEnery -- get raw value..., m_scene = "<<m_scene.GetPointer()<<std::endl;
		double outputValue = 0;
		if (m_UseGlobalPrior) outputValue += m_weightGlobalPrior * GetSceneGlobalCost();			//std::cout<<"------------------------ global prior: outputValue = "<<outputValue<<std::endl;
		if (m_UseObjectPrior) outputValue += m_weightObjectPrior * GetSumObjectPriorCost();		//std::cout<<"------------------------ +object prior: outputValue = "<<outputValue<<std::endl;
		if (m_UseInteractionPrior) outputValue += m_weightInteractionPrior * GetSumObjectInteractionsCost();	//std::cout<<"------------------------ +inter prior: outputValue = "<<outputValue<<std::endl;
		if (m_UseSingleObjectImagePrior) outputValue += m_weightSingleObjectImageFitPrior * GetSumSingleObjectSceneToImageCost();		//std::cout<<"------------------------ +single object prior: outputValue = "<<outputValue<<std::endl;
		if (m_UseImagePrior) outputValue += m_weightImageFitPrior * GetSceneToImageCost();		//std::cout<<"------------------------ +image prior: outputValue = "<<outputValue<<std::endl;
		return ( outputValue );
	}

	//double GetValueExcludingObject(IDType id) { //OPTIMIZATION: do GetAverageObjectPriors and GetAverageObjectInteractions in a single pass if both are requested...
	//	double outputValue = 0;
	//	if (m_UseGlobalPrior) outputValue += m_weightGlobalPrior * GetSceneGlobalCost();			//std::cout<<"------------------------ global prior: outputValue = "<<outputValue<<std::endl;
	//	if (m_UseObjectPrior) outputValue += m_weightObjectPrior * GetAverageObjectPriorCost();		//std::cout<<"------------------------ +object prior: outputValue = "<<outputValue<<std::endl;
	//	if (m_UseInteractionPrior) outputValue += m_weightInteractionPrior * GetAverageObjectInteractionsCost();	//std::cout<<"------------------------ +inter prior: outputValue = "<<outputValue<<std::endl;
	//	if (m_UseSingleObjectImagePrior) outputValue += m_weightSingleObjectImageFitPrior * GetAverageSingleObjectSceneToImageCost();		//std::cout<<"------------------------ +single object prior: outputValue = "<<outputValue<<std::endl;
	//	if (m_UseImagePrior) outputValue += m_weightImageFitPrior * GetSceneToImageCost();		//std::cout<<"------------------------ +image prior: outputValue = "<<outputValue<<std::endl;
	//	return ( outputValue );
	//}


	double GetValueForObject(IDType id) {
		double outputValue = 0;
//		if (m_UseGlobalPrior) outputValue = m_weightGlobalPrior * GetSceneGlobalCost(); //added so that the individual cost can be compared with the global cost...
////std::cout<<"object "<<it->first<<", global: "<<GetSceneGlobalEnergy()<<std::endl;
		if (m_UseObjectPrior) outputValue+= m_weightObjectPrior * GetPriorCostForObject(id);
//std::cout<<"object "<<it->first<<", prior: "<<GetPriorCostForObject(it)<<std::endl;
		if (m_UseInteractionPrior) outputValue+= m_weightInteractionPrior * GetInteractionCostForObject(id);
//std::cout<<"object "<<it->first<<", interaction: "<<GetInteractionCostForObject(it)<<std::endl;
		if (m_UseSingleObjectImagePrior) outputValue+= m_weightInteractionPrior * GetSingleObjectCostForObject(id);
//std::cout<<"object "<<it->first<<", single object cost: "<<GetSingleObjectCostForObject(id)<<std::endl;
//		if (m_UseImagePrior) outputValue+= m_weightImageFitPrior * GetSceneToImageCost();
////std::cout<<"object "<<it->first<<", data: "<<GetImageFitCostForObject(it)<<std::endl;
		return (  outputValue );
	}

	/** Get the prior cost for a specified object - independently whether it is used or not in the energy computation */
	inline double GetPriorCostForObject(IDType id)	{ return m_scene->GetObjectPriorCost(id); }

	/** Get the interaction cost for a specified object - independently whether it is used or not in the energy computation */
	inline double GetInteractionCostForObject(IDType id)	{ return m_scene->GetObjectTotalInteractionCost(id); }

	/** Get the data cost for a specified object - independently whether it is used or not in the energy computation*/
	double GetSingleObjectCostForObject(IDType id)	{ 
		if (m_useTypeSpecificMetric) {
			TypeSpecificObjectToImageCostFunctionListType::iterator costFunctionIt = m_listObjectSpecificMetrics.find(m_scene->GetObject(id)->objectTypeId);
			if (costFunctionIt == m_listObjectSpecificMetrics.end()) { throw DeformableModelException("SceneEnergy::GetSingleObjectCostForObject() - requesting type specific object-to-image cost functions, but not all object types have an associated metric !!"); }
			else {
				costFunctionIt->second->SelectObject(id);
				return costFunctionIt->second->GetValue();
			}
		}
		else {
			if (!m_singleObjectSceneToImageMetric) throw DeformableModelException("SceneEnergy::GetSingleObjectCostForObject() - a single object cost function must be provided by the user! ");
			m_singleObjectSceneToImageMetric->SelectObject(id);
			return m_singleObjectSceneToImageMetric->GetValue();
		}	
	}




	void PrintDetails() {
		if (!m_scene) throw DeformableModelException("SceneEnergy::PrintDetails() - the scene is undefined!! ");
		std::cout<<"number of objects: "<<m_scene->GetNumberOfObjects()<<std::endl;
		std::cout<<"energy relative to the global prior: value = "<<GetSceneGlobalCost()<<", weight = "<<m_weightGlobalPrior<<", used? "<<m_UseGlobalPrior<<std::endl;
		std::cout<<"average energy relative to object priors: value = "<<GetAverageObjectPriorCost()<<", weight = "<<m_weightObjectPrior<<", used? "<<m_UseObjectPrior<<std::endl;
		std::cout<<"energy related to pairwise interactions : value = "<<GetAverageObjectInteractionsCost()<<", weight = "<<m_weightInteractionPrior<<", used? "<<m_UseInteractionPrior<<std::endl;
		std::cout<<"energy related to object goodness of fit : value = "<<GetAverageSingleObjectSceneToImageCost()<<", weight = "<<m_weightSingleObjectImageFitPrior<<", used? "<<m_UseSingleObjectImagePrior<<std::endl;
		std::cout<<"energy related to image fit   : value = "<<GetSceneToImageCost()<<", weight = "<<m_weightImageFitPrior<<", used? "<<m_UseImagePrior<<std::endl;
	}

	void PrintDetailsForObject(IDType id) { 
		std::cout<<"energy details for object with id "<<id<<std::endl;
		if (m_UseObjectPrior) std::cout<<"prior cost  : value = "<<GetPriorCostForObject(id)<<"\t, weight = "<<m_weightObjectPrior<<"\t, used? "<<m_UseObjectPrior<<std::endl;
		if (m_UseInteractionPrior) std::cout<<"interaction : value = "<<GetInteractionCostForObject(id)<<"\t, weight = "<<m_weightInteractionPrior<<"\t, used? "<<m_UseInteractionPrior<<std::endl;
		if (m_UseSingleObjectImagePrior) std::cout<<"image fit   : value = "<<GetSingleObjectCostForObject(id)<<"\t, weight = "<<m_weightSingleObjectImageFitPrior<<"\t, used? "<<m_UseSingleObjectImagePrior<<std::endl;
	}

	double GetEnergyForObject(IDType id) { 
		double value=0;
		if (m_UseObjectPrior) value+=m_weightObjectPrior*GetPriorCostForObject(id);
		if (m_UseInteractionPrior) value+=m_weightInteractionPrior*GetInteractionCostForObject(id);
		if (m_UseSingleObjectImagePrior) value+=GetSingleObjectCostForObject(id);
		return value;
	}

	double GetSceneGlobalCost();
	double GetAverageObjectPriorCost();
	double GetAverageObjectInteractionsCost();
	double GetAverageSingleObjectSceneToImageCost();

	double GetSumObjectPriorCost();
	double GetSumObjectInteractionsCost();
	double GetSumSingleObjectSceneToImageCost();

	double GetSceneToImageCost();

	void UseGlobalScenePrior(bool b=true)			{m_UseGlobalPrior=b;}
	void UseObjectPrior(bool b=true)				{m_UseObjectPrior=b;}
	void UseObjectInteractionPrior(bool b=true)		{m_UseInteractionPrior=b;}
	void UseSingleObjectSceneToImageCostFunction(bool b=true)	{m_UseSingleObjectImagePrior=b;}
	void UseSceneToImageCostFunction(bool b=true)	{m_UseImagePrior=b;}

protected:
	SceneEnergy();
	virtual ~SceneEnergy() {};	

	typename SceneToImageCostFunctionType::Pointer				m_sceneToImageMetric;	
	typename SingleObjectSceneToImageCostFunctionType::Pointer	m_singleObjectSceneToImageMetric;
	TypeSpecificObjectToImageCostFunctionListType				m_listObjectSpecificMetrics;

	bool m_UseGlobalPrior;				double m_weightGlobalPrior;
	bool m_UseObjectPrior;				double m_weightObjectPrior;
	bool m_UseInteractionPrior;			double m_weightInteractionPrior;
	bool m_UseSingleObjectImagePrior;	double m_weightSingleObjectImageFitPrior;
	bool m_UseImagePrior;				double m_weightImageFitPrior;
	bool m_useTypeSpecificMetric;

private:
	SceneEnergy(const Self&);					//purposely not implemented
	const Self & operator=( const Self & );		//purposely not implemented
};

} // namespace psciob

#include "SceneEnergy.txx"

#endif /* SCENEENERGY_H_ */
