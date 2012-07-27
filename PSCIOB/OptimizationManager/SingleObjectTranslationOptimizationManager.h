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
 * \file SingleObjectTranslationOptimizationManager.h
 * \author Rémi Blanc 
 * \date 21. September 2011
*/


#ifndef SINGLEOBJECTTRANSLATIONOPTIMIZATIONMANAGER_H_
#define SINGLEOBJECTTRANSLATIONOPTIMIZATIONMANAGER_H_

#include "SingleSceneObjectOptimizationManager.h"

namespace psciob {

/** \class SingleObjectTranslationOptimizationManager
 * \brief SingleObjectTranslationOptimizationManager: optimize a selected translation parameter for a single object.
*/


template<class TScene>
class SingleObjectTranslationOptimizationManager : public SingleSceneObjectOptimizationManager<TScene> {
public:
	/** Standard class typedefs. */
	typedef SingleObjectTranslationOptimizationManager	Self;
	typedef SingleSceneObjectOptimizationManager		Superclass;
	typedef itk::SmartPointer<Self>					Pointer;
	/** Run-time type information (and related methods). */
	itkTypeMacro(SingleObjectTranslationOptimizationManager,SingleSceneObjectOptimizationManager);
	itkNewMacro(Self);


	/** 0 : translate along x, 1: y, 2: z ; 
	* returns false if the axis is invalid
	*/
	bool SetTranslateAlongAxis(unsigned int axis) {
		if (axis>SceneType::Dimension) return false;
		m_axis = axis; return true;
	}

	inline unsigned int GetNumberOfParameters()			{ return 1; }

	//inline vnl_vector<double> GetParameters() { 
	//	vnl_vector<double> params = m_scene->GetParametersOfObject(m_requestedLabel);
	//	return params.extract(1, m_axis); 
	//}
	//bool SetParameters(vnl_vector<double> p)			{ 
	//	vnl_vector<double> params = m_scene->GetParametersOfObject(m_requestedLabel);
	//	params(m_axis) = p(0);
	//	return m_scene->ModifyObjectParameters(m_requestedLabel, params); 
	//}	
	inline vnl_vector<double> GetParameters() { return m_currentParameters.extract(1, m_axis); }
	inline bool SetParameters(vnl_vector<double> p) { 
		double oldparam = m_currentParameters(m_axis);
		m_currentParameters(m_axis) = p(0);
		if (m_scene->ModifyObjectParameters(m_requestedLabel, m_currentParameters)) return true;
		else { m_currentParameters(m_axis) = oldparam; return false; }
	}	


	/** get pdf for the parameters of one specific object
	// \todo It seems difficult to provide the prior ; however, the manager could provide with random samples more easily;
	// => change that, for all optimization manager & optimizers: optimization manager provides with requested random values
	// => also, should there be a way to manually set the distributions, instead of relying always on scene characteristics... ?
	*/
	MultivariatePDF* GetParameterPriorPDF() {
		std::cout<<"INVALID FOR NOW, SEE CODE FOR DETAILS..."<<std::endl;
		unsigned int typeIndex = m_scene->GetObject(m_requestedLabel)->objectTypeId;
		return m_scene->GetObjectTypesLibrary()->GetObjectPDF(typeIndex , PDF_OBJECTGENERATIONPRIOR);
	}

	MultivariatePDF* GetParameterProposalMovePDF() {
		std::cout<<"INVALID FOR NOW, SEE CODE FOR DETAILS..."<<std::endl;
		unsigned int typeIndex = m_scene->GetObject(m_requestedLabel)->objectTypeId;
		return m_scene->GetObjectTypesLibrary()->GetObjectPDF(typeIndex , PDF_RANDOMWALK);
	}

protected:
	SingleObjectTranslationOptimizationManager() : SingleSceneObjectOptimizationManager() { m_axis = 0; };
	~SingleObjectTranslationOptimizationManager() {};

	unsigned int m_axis;

private:
	SingleObjectTranslationOptimizationManager(const Self&);	//purposely not implemented
	const Self & operator=( const Self & );						//purposely not implemented
};



} // namespace psciob

#endif /* SINGLEOBJECTTRANSLATIONOPTIMIZATIONMANAGER_H_ */
