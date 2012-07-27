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
 * \file AverageObjectToImageCostFunction.h
 * \author Rémi Blanc 
 * \date 3. October 2011
 */

#ifndef AVERAGEOBJECTTOIMAGECOSTFUNCTION_H_
#define AVERAGEOBJECTTOIMAGECOSTFUNCTION_H_


#include <MultipleObjectsToImageCostFunction.h>

namespace psciob {

/** \brief AverageObjectToImageCostFunction
 * computes the average of individual objects costs
*/
template<class TScene, class TReferenceImage>
class AverageObjectToImageCostFunction : public MultipleObjectsToImageCostFunction<TScene, TReferenceImage> {
public:
	/** Standard class typedefs. */
	typedef AverageObjectToImageCostFunction	Self;
	typedef MultipleObjectsToImageCostFunction	Superclass;
	typedef itk::SmartPointer<Self>				Pointer;
	/** Run-time type information (and related methods). */
	itkTypeMacro(AverageObjectToImageCostFunction, MultipleObjectsToImageCostFunction);
	itkNewMacro(Self);

	/** average value of individual costs */
	double GetRawValue() {
		if (!m_sooMetric) throw DeformableModelException("AverageSingleObjectsCostFunction::GetValue : the single object metric must be set!");
		double sum=0;
		if (m_scene->GetNumberOfObjects()==0) return 0;
		SceneObjectIterator<SceneType> it(m_scene);
		for (it.GoToBegin() ; !it.IsAtEnd() ; ++it) {
			m_sooMetric->SelectObject(it.GetID());
			sum += m_sooMetric->GetValue();
		}
		return sum / static_cast<double>(m_scene->GetNumberOfObjects());		
	}

protected:
	AverageObjectToImageCostFunction() : MultipleObjectsToImageCostFunction() {}
	~AverageObjectToImageCostFunction() {};	

private:
	AverageObjectToImageCostFunction(const Self&);	//purposely not implemented
	const Self & operator=( const Self & );			//purposely not implemented
};

} // namespace psciob

#endif /* AVERAGEOBJECTTOIMAGECOSTFUNCTION_H_ */
