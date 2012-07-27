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
 * \file SingleObjectSceneToImageCostFunction.h
 * \author Rémi Blanc 
 * \date 2. November 2011
 */

#ifndef SINGLEOBJECTSCENETOIMAGECOSTFUNCTION_H_
#define SINGLEOBJECTSCENETOIMAGECOSTFUNCTION_H_

#include <SceneToImageCostFunction.h>
#include <itkImageMaskSpatialObject.h>
#include <itkBinaryThresholdImageFilter.h>
#include "itkRegionOfInterestImageFilter.h"

#include <itkImageSpatialObject.h>

namespace psciob {

/**\class SingleObjectSceneToImageCostFunction
 * \brief SingleObjectSceneToImageCostFunction which purpose is to implement a layer from which 
 *		  all single object metrics will inherit from... 
 * Implements concepts of in and off context measures, which are cached in the ObjectInSceneContainer, and treat objects
 * either naturally from the scene, with the potential overlaps, partial visiblility on the sensor, shadowing, etc...
 * or off-context: as if it were alone in the scene
 */

template<class TScene, class TReferenceImage>
class SingleObjectSceneToImageCostFunction : public SceneToImageCostFunction<TScene, TReferenceImage> {
public:
	/** Standard class typedefs. */
	typedef SingleObjectSceneToImageCostFunction	Self;
	typedef SceneToImageCostFunction				Superclass;
	typedef itk::SmartPointer<Self>					Pointer;
	/** Run-time type information (and related methods). */
	itkTypeMacro(SingleObjectSceneToImageCostFunction, SceneToImageCostFunction);


	/** Create a clone (= an exact, independant copy) of the current cost function 
	* transfers the pointer to the reference image.
	*/
	virtual BaseClassPointer CreateClone() {
		Pointer clonePtr = static_cast<Self*>(this->CreateAnother().GetPointer());
		if (!m_transformFunction) {} else { clonePtr->SetNormalizationFunction(m_transformFunction); }
		if (!m_scene) {} else { clonePtr->SetScene(m_scene); }
		clonePtr->SelectObject(m_requestedLabel);

		if (!m_referenceImage) {} else { clonePtr->SetReferenceImage(m_referenceImage); }
		clonePtr->SetInterpolator( static_cast<InterpolatorType *>(m_interpolator->CreateAnother().GetPointer()) );

		clonePtr->m_inContextMetricFlag = m_inContextMetricFlag;
		return static_cast<BaseClass*>( clonePtr );
	}


	/** InContext: object in the scene, accounting for overlaps, foreground / background => get the data from sensor->GetOutput() */
	void SelectInContextComputation()	{ m_inContextMetricFlag = true; }
	//void SelectInContextComputation()	{ if (!m_inContextMetricFlag) InvalidateDataCostCache(); m_inContextMetricFlag = true; }
	/** OffContext: object as if alone in the scene => get the data from sensor->GetOffContextImageOfObject( m_requestedObject ) */
	void SelectOffContextComputation()	{ m_inContextMetricFlag = false;}
	//void SelectOffContextComputation()	{ if ( m_inContextMetricFlag ) InvalidateDataCostCache(); m_inContextMetricFlag = false;}

	/** Look in the cache if the requested cost (in/off context) is already available, otherwise compute it */
	virtual double GetRawValue() {
		if (!m_scene) throw DeformableModelException("SingleObjectSceneToImageCostFunction::GetRawValue() -- no scene has been attached to the cost function !");
		if (m_requestedLabel==0) return 0;

		double output;
		if (m_inContextMetricFlag) { //if the cached data is valid, just output it immediatly
			//if (m_scene->GetObject(m_requestedLabel)->objectData.dataCostFlag) return m_scene->GetObject(m_requestedLabel)->objectData.dataCost;
			//else output = ComputeInContextValue();
			////update the cache
			//m_scene->GetObject(m_requestedLabel)->objectData.dataCost = output; 
			//m_scene->GetObject(m_requestedLabel)->objectData.dataCostFlag = true;
			output = ComputeInContextValue();
		}
		else { //if the cached data is valid, just output it immediatly
			if (m_scene->GetObject(m_requestedLabel)->objectData.offContextDataCostFlag) return m_scene->GetObject(m_requestedLabel)->objectData.offContextDataCost;
			else output = ComputeOffContextValue();
			//update the cache
			m_scene->GetObject(m_requestedLabel)->objectData.offContextDataCost = output; 
			m_scene->GetObject(m_requestedLabel)->objectData.offContextDataCostFlag = true;
		}		
		return output;
	}

	/** not really intended for the standard usage, but kept public for convenience, just in case... maybe make it protected later on */
	virtual double ComputeInContextValue() = 0;
	virtual double ComputeOffContextValue() = 0;

protected:
	SingleObjectSceneToImageCostFunction() : SceneToImageCostFunction() { m_inContextMetricFlag = true; };
	~SingleObjectSceneToImageCostFunction() {};	

	bool m_inContextMetricFlag;

	//void InvalidateDataCostCache() { 
	//	for (ObjectIteratorType it = m_scene->GetObjectSet()->begin() ; it!=m_scene->GetObjectSet()->end() ; it++) {
	//		it->second->dataCostFlag = false; 
	//	}
	//}

private:
	SingleObjectSceneToImageCostFunction(const Self&);	//purposely not implemented
	const Self & operator=( const Self & );		//purposely not implemented
};

} // namespace psciob

#endif /* SINGLEOBJECTSCENETOIMAGECOSTFUNCTION_H_ */
