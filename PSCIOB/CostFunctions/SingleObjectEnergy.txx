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
* \file SingleObjectEnergy.txx
* \author Rémi Blanc 
* \date 20. December 2011
*/

#ifndef SINGLEOBJECTENERGY_TXX_
#define SINGLEOBJECTENERGY_TXX_

#include "SingleObjectEnergy.h"

namespace psciob {


template<class TScene, class TReferenceImage>
SingleObjectEnergy<TScene, TReferenceImage>::SingleObjectEnergy() {
	m_UseObjectPrior = true;		m_weightObjectPrior = 1;
	m_UseInteractionPrior = true;	m_weightInteractionPrior = 1;
	m_UseSingleObjectImagePrior = false; m_weightSingleObjectImageFitPrior = 1;
	m_useTypeSpecificMetric = false; //by default, use the same metric for all object types.

	m_singleObjectSceneToImageMetric = 0;
}

//
//
//
template<class TScene, class TReferenceImage>	
void 
SingleObjectEnergy<TScene, TReferenceImage>
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
SingleObjectEnergy<TScene, TReferenceImage>
::GetRawValue() { 
	double outputValue = 0;
	if (m_UseObjectPrior) outputValue += m_weightObjectPrior * GetPriorCostForObject(m_requestedLabel);		//std::cout<<"------------------------ +object prior: outputValue = "<<outputValue<<std::endl;
	if (m_UseInteractionPrior) outputValue += m_weightInteractionPrior * GetInteractionCostForObject(m_requestedLabel);	//std::cout<<"------------------------ +inter prior: outputValue = "<<outputValue<<std::endl;
	if (m_UseSingleObjectImagePrior) outputValue += m_weightSingleObjectImageFitPrior * GetSingleObjectCostForObject(m_requestedLabel);		//std::cout<<"------------------------ +single object prior: outputValue = "<<outputValue<<std::endl;
	return ( outputValue );
}

} // namespace psciob

#endif /* SINGLEOBJECTENERGY_TXX_ */
