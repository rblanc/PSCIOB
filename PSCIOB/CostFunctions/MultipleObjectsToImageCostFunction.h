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
 * \file MultipleObjectsToImageCostFunction.h
 * \author Rémi Blanc 
 * \date 15. December 2011
 */

#ifndef MULTIPLEOBJECTSTOIMAGECOSTFUNCTION_H_
#define MULTIPLEOBJECTSTOIMAGECOSTFUNCTION_H_


#include <SceneToImageCostFunction.h>
#include <SingleObjectSceneToImageCostFunction.h>

namespace psciob {

/** \brief MultipleObjectsToImageCostFunction
 * base class for computing a cost combining the metric of individual objects of the scene
 * The individual object - metric must be completely defined
 * 
 * IDEA: another possibility would be to assign different metrics for different object types...
 * check maybe with the SceneEnergy what is preferable
*/
template<class TScene, class TReferenceImage>
class MultipleObjectsToImageCostFunction : public SceneToImageCostFunction<TScene, TReferenceImage> {
public:
	/** Standard class typedefs. */
	typedef MultipleObjectsToImageCostFunction	Self;
	typedef SceneToImageCostFunction			Superclass;
	typedef itk::SmartPointer<Self>				Pointer;
	/** Run-time type information (and related methods). */
	itkTypeMacro(MultipleObjectsToImageCostFunction, SceneToImageCostFunction);

	typedef SingleObjectSceneToImageCostFunction<TScene, TReferenceImage> SOOCostFunctionType;

	/** Create a clone (= an exact, independant copy) of the current cost function 	*/
	virtual BaseClassPointer CreateClone() {
		Pointer clonePtr = static_cast<Self*>(this->CreateAnother().GetPointer());
		if (!m_transformFunction) {} else {clonePtr->SetNormalizationFunction(m_transformFunction);}
		if (!m_scene) {} else {clonePtr->SetScene(m_scene);}
		clonePtr->SelectObject(m_requestedLabel);

		if (!m_referenceImage) {} else {clonePtr->SetReferenceImage(m_referenceImage);}

		if (!m_sooMetric) {} else {clonePtr->SetSingleObjectMetric( static_cast<SOOCostFunctionType *>(m_sooMetric->CreateClone().GetPointer()) );} 

		return static_cast<BaseClass*>( clonePtr );
	}

	/** Set the base function */
	void SetSingleObjectMetric(SOOCostFunctionType *metric) { m_sooMetric = metric; m_scene = m_sooMetric->GetScene(); }

protected:
	MultipleObjectsToImageCostFunction() : SceneToImageCostFunction() {
		m_sooMetric = 0;
	}
	~MultipleObjectsToImageCostFunction() {};	

	typename SOOCostFunctionType::Pointer m_sooMetric;

	/** nothing to do: it is up to the single object metric to do this */
	void InterpolateReferenceImage() { } 
	

private:
	MultipleObjectsToImageCostFunction(const Self&);	//purposely not implemented
	const Self & operator=( const Self & );				//purposely not implemented
};

} // namespace psciob

#endif /* MULTIPLEOBJECTSTOIMAGECOSTFUNCTION_H_ */
