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

/*
 * \file SceneToImageCostFunction.h
 * \author Rémi Blanc 
 * \date 15. December 2011
*/

#ifndef SCENETOIMAGECOSTFUNCTION_H_
#define SCENETOIMAGECOSTFUNCTION_H_

#include "SceneCostFunction_Base.h"
#include "BoundingBoxesUtils.h"

//#include "CommonTypes.h"
#include "GeneralUtils.h"
#include "ITKUtils.h"
#include "ScalarFunctions.h"
#include "ImageSensor_Base.h"
#include <itkImageToImageMetric.h>
#include <itkIdentityTransform.h>	
#include <itkNearestNeighborInterpolateImageFunction.h>

#include <itkInterpolateImageFunction.h>

#include "LineIntegralImageAndSquaredImageFilter.h"

namespace psciob {


/** 
 * \class SceneToImageCostFunction
 * \brief base class for computing a (dis)similarity value (low value = similar images) between a reference image and a scene
 * the reference image is interpolated as required to match the grid of the image it is compared with - default is a nearest neighbor interpolation
 *
 * ADD: what about the gradient of the metric ?
 */

template<class TScene, class TReferenceImage>
class SceneToImageCostFunction : public SceneCostFunction_Base<TScene> {
public:
	/** Standard class typedefs. */
	typedef SceneToImageCostFunction		Self;
	typedef SceneCostFunction_Base			Superclass;
	typedef itk::SmartPointer<Self>			Pointer;
	/** Run-time type information (and related methods). */
	itkTypeMacro(SceneToImageCostFunction, SceneCostFunction_Base);

	static const unsigned int ImageDimension = TReferenceImage::ImageDimension;

	typedef TReferenceImage                    ReferenceImageType;

	typedef itk::ResampleImageFilter<TReferenceImage, TReferenceImage >        ResampleImageFilterType;
	typedef typename itk::IdentityTransform< double, ImageDimension >          TransformType;		//this is useful to restrict the computation region for singleobject metrics	
	typedef typename itk::InterpolateImageFunction< TReferenceImage, double >  InterpolatorType;
	
	//some metric may cache integral image to speed-up the evaluations
	typedef itk::Image<double, ImageDimension> IntegralImageType;
	typedef typename LineIntegralImageAndSquaredImageFilter< TReferenceImage, IntegralImageType> ReferenceImageIntegrateFilterType;
	
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

		return static_cast<BaseClass*>( clonePtr );
	}

	/** Set the reference image, and computes its extreme values */
	void SetReferenceImage(ReferenceImageType *img) {
		m_refImageInterpolatedFlag = false;
		m_referenceImage=img; m_interpolator->SetInputImage(m_referenceImage); 
		
		typedef itk::ImageRegionConstIterator< ReferenceImageType > IteratorType; 
		IteratorType it(m_referenceImage, m_referenceImage->GetLargestPossibleRegion());
		double val;
		it.GoToBegin(); m_refImgMinValue=it.Get(), m_refImgMaxValue=it.Get(); ++it;
		while(!it.IsAtEnd()) { 
			val = it.Get();
			m_refImgMinValue = std::min<double>(m_refImgMinValue, val);
			m_refImgMaxValue = std::max<double>(m_refImgMaxValue, val);
			++it;
		}		
	}

	/** Set the interpolator ; default is a nearest neighbor interpolator */
	void SetInterpolator(InterpolatorType *interp)	{ 
		m_interpolator = interp; 
		if (!m_referenceImage) {} else {m_interpolator->SetInputImage(m_referenceImage);}
		m_refImageInterpolatedFlag = false;
	}

	/** Set the interpolator ; default is a nearest neighbor interpolator */
	TReferenceImage* GetReferenceImage() {
		if (!m_referenceImage)	throw DeformableModelException("SensedSceneToImageCostFunction::InterpolateReferenceImage : the reference image must be set!");
		return m_referenceImage.GetPointer(); 
	}

	/** Get the reference image - interpolated so that its grid matches that of the sensor image */
	inline ReferenceImageType *GetInterpolatedReferenceImage() {
		if (!m_refImageInterpolatedFlag) {
			InterpolateReferenceImage();
			m_refImageInterpolatedFlag = true;
		}		
		return m_interpolatedReferenceImage.GetPointer();
	}

protected:
	SceneToImageCostFunction() : SceneCostFunction_Base() {
		m_interpolator = itk::NearestNeighborInterpolateImageFunction<TReferenceImage, double>::New();

		m_transform = TransformType::New();
		m_transform->SetIdentity();
		m_nulltransformparams.set_size(m_transform->GetNumberOfParameters());
		m_nulltransformparams.fill(0);
		
		m_filter = ResampleImageFilterType::New();
		m_integralFilter = ReferenceImageIntegrateFilterType::New();

		m_refImageInterpolatedFlag = 0;
		m_referenceImage = 0;
	}
	virtual ~SceneToImageCostFunction() {};	


	typename ReferenceImageType::Pointer m_referenceImage, m_interpolatedReferenceImage;
	bool m_refImageInterpolatedFlag;

	double m_refImgMinValue, m_refImgMaxValue;

	typename ResampleImageFilterType::Pointer	m_filter;
	typename InterpolatorType::Pointer			m_interpolator;
	typename TransformType::Pointer				m_transform;
	typename TransformType::ParametersType		m_nulltransformparams;
	typename ReferenceImageIntegrateFilterType::Pointer m_integralFilter;

	/** DEVELOPER: do not forget to set m_interpolatedReferenceImage 
	* kept virtual at this point, because the interpolation may be different depending whether a sensor is used, or directly the scene... ??? not sure this is really relevant...
	* \warning: if the sensor is modified (e.g. its resolution is modified), the metric is not informed automatically...
	* may be overloaded by child classes
	*/
	virtual void InterpolateReferenceImage() = 0;

private:
	SceneToImageCostFunction(const Self&);	//purposely not implemented
	const Self & operator=( const Self & );		//purposely not implemented
};

} // namespace psciob

#endif /* SCENETOIMAGECOSTFUNCTION_H_ */
