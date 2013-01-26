/*
 * This file is part of the psciob library.
 *
 * Copyright (c) 2012, Remi Blanc
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
* \file ImageFeaturePointDetector_2DCannyEdges_Orientation.h
* \author Rémi Blanc 
* \date 15. October 2012
*/


#ifndef __IMAGEFEATUREPOINTDETECTOR_2DCANNYEDGES_ORIENTATION_H_
#define __IMAGEFEATUREPOINTDETECTOR_2DCANNYEDGES_ORIENTATION_H_


#include "ImageFeaturePointDetector_Base.h"
#include <itkCastImageFilter.h>
#include <itkCannyEdgeDetectionImageFilter.h>
//#include <itkDiscreteGaussianDerivativeImageFilter.h> //check how to make this filter work...
#include <itkGradientRecursiveGaussianImageFilter.h>

namespace psciob {

/** \brief ImageFeaturePointDetector_2DCannyEdges_Orientation
* Performs a 2D Canny Edge Detection step, and get the gradient orientation at these points, using a similar gradient computation ; check APPLI - segmentation2D_Test
* uses the itk::CannyEdgeDetectionImageFilter class (check for meaning of parameters)

*/


template<class InputImagePixelType>
class ImageFeaturePointDetector_2DCannyEdges_Orientation : public ImageFeaturePointDetector_Base<InputImagePixelType, 2> {
public:
	static const unsigned m_featureVectorDimension = 1;
  /** Standard class typedefs. */
  typedef ImageFeaturePointDetector_2DCannyEdges_Orientation    Self;
  typedef itk::LightObject        Superclass;
  typedef itk::SmartPointer<Self> Pointer;
  /** Run-time type information (and related methods). */
  itkTypeMacro(ImageFeaturePointDetector_2DCannyEdges_Orientation, ImageFeaturePointDetector_Base);
  itkNewMacro(Self);

	typedef itk::Image<float, 2>                                               FloatImageType;
	typedef itk::CastImageFilter<InputImageType, FloatImageType>               CastImageFilterType;
	typedef itk::CannyEdgeDetectionImageFilter<FloatImageType, FloatImageType> CannyEdgeDetectionImageFilterType;
	
	typedef itk::CovariantVector< float, 2 >   GradientPixelType;
	typedef itk::Image< GradientPixelType, 2 > GradientImageType;
	//typedef itk::DiscreteGaussianDerivativeImageFilter<FloatImageType, FloatImageType> DerivativeFilterType;
	typedef itk::GradientRecursiveGaussianImageFilter<InputImageType, GradientImageType> GradientFilterType;
	
	inline unsigned GetFeatureVectorDimension() const { return m_featureVectorDimension; }

  /** Create an independant copy of the object, with the same parameters
  * but the data is left blank.
  */
  virtual BaseClassPointer CreateCopy() {
    Pointer clonePtr = static_cast<Self*>(this->CreateAnother().GetPointer());
    clonePtr->SetCannyLowerThreshold(m_lowerThreshold); 
    clonePtr->SetCannyUpperThreshold(m_upperThreshold); 
    clonePtr->SetCannyVariance(m_variance); 

		return static_cast<BaseClass*>( clonePtr );
  }


	/** Set the lower threshold value for the canny edge detector */
	void SetCannyLowerThreshold(double t = 2) {m_lowerThreshold=t;}
	/** Set the upper threshold value for the canny edge detector */
	void SetCannyUpperThreshold(double t = 8) {m_upperThreshold=t;}
	/** Set the upper threshold value for the canny edge detector */
	void SetCannyVariance(double v = 4) {m_variance=v;}
	
	
	/** Force the update of the detector 
	* Compute the gradient 
	*/
	void Update() {
		if (!m_inputImage) throw DeformableModelException("ImageFeaturePointDetector_2DCannyEdges_Orientation::Update() -- NO INPUT IMAGE!!");
		//ITK doesn't seem to support re-use of a filter... so create them anew at each call...
		m_castImageFilter = CastImageFilterType::New();
		m_cannyFilter = CannyEdgeDetectionImageFilterType::New();
		m_gradientFilter = GradientFilterType::New();

		m_castImageFilter->SetInput(m_inputImage);
		
		m_cannyFilter->SetInput(m_castImageFilter->GetOutput());

		m_cannyFilter->SetLowerThreshold(m_lowerThreshold);
		m_cannyFilter->SetUpperThreshold(m_upperThreshold);
		m_cannyFilter->SetVariance(m_variance);

		m_cannyFilter->Update();

		m_gradientFilter->SetInput(m_inputImage);
		m_gradientFilter->SetSigma(sqrt(m_variance)); m_gradientFilter->Update();
		
		//m_orientationImage -> allocate, and fill from the x&y derivatives...
		m_orientationImage = FloatImageType::New();
		m_orientationImage->SetSpacing(m_inputImage->GetSpacing());
		m_orientationImage->SetOrigin(m_inputImage->GetOrigin());
		m_orientationImage->SetRegions(m_inputImage->GetLargestPossibleRegion());
		m_orientationImage->Allocate();

		typedef itk::ImageRegionIterator<FloatImageType>		FloatImageIteratorType;
		FloatImageIteratorType itOut(m_orientationImage, m_inputImage->GetLargestPossibleRegion());
		typedef itk::ImageRegionIteratorWithIndex<FloatImageType>		FloatImageIteratorWitIndexType;
		FloatImageIteratorWitIndexType itFP(m_cannyFilter->GetOutput(), m_inputImage->GetLargestPossibleRegion());

		typedef itk::ImageRegionIterator<GradientImageType>		GradientImageIteratorType;
		GradientImageIteratorType itG(m_gradientFilter->GetOutput(), m_gradientFilter->GetOutput()->GetLargestPossibleRegion());

		//cache the orientation image, and the list of detected feature points
		for (itOut.GoToBegin(), itG.GoToBegin(), itFP.GoToBegin() ; !itOut.IsAtEnd() ; ++itOut, ++itG, ++itFP) {
			itOut.Set( std::atan2(itG.Get()[1],itG.Get()[0]) );
			if (itFP.Get()!=0) {
				m_featureVector(0) = itOut.Get();
				m_featurePointList.push_back( FeaturePointType(itFP.GetIndex(), m_featureVector) );
			}
		}
		
		//finally, release the memory (gradient image, etc...)...
		m_uptodate = true;
	}
	
	/** Compute the feature vector at the requested location 
	* \warning does not perform any check (e.g. uptodate status) for efficiency reasons.
	*/
	inline const FeatureVectorType &  GetFeatureVectorAtLocation(const ImageIndexType &index) {	
		m_featureVector(0) = m_orientationImage->GetPixel(index);
		return m_featureVector;
	}

	/* Get the image of gradient orientation */
	typename FloatImageType* GetOrientationImage() const {return m_orientationImage;}

	/* Get the image of gradient orientation */
	typename FloatImageType* GetFeaturePointImage() const {return m_cannyFilter->GetOutput();}

protected:
  ImageFeaturePointDetector_2DCannyEdges_Orientation() : ImageFeaturePointDetector_Base(), m_featureVector(m_featureVectorDimension) {
    m_castImageFilter = CastImageFilterType::New();
    m_cannyFilter = CannyEdgeDetectionImageFilterType::New();
    m_gradientFilter = GradientFilterType::New();

    //deafult parameter values
    m_lowerThreshold = 2;
    m_upperThreshold = 8;
    m_variance = 4;

    m_orientationImage = NULL;
  };
  ~ImageFeaturePointDetector_2DCannyEdges_Orientation() {};

  double m_variance, m_lowerThreshold, m_upperThreshold;

	typename CastImageFilterType::Pointer m_castImageFilter;
	typename CannyEdgeDetectionImageFilterType::Pointer m_cannyFilter;
	typename GradientFilterType::Pointer m_gradientFilter;
	
	typename FloatImageType::Pointer m_orientationImage;	
	FeatureVectorType m_featureVector;
	
private:
    ImageFeaturePointDetector_2DCannyEdges_Orientation(const Self&); //purposely not implemented
    const Self & operator=( const Self & ); //purposely not implemented
};


} // namespace psciob

#endif /* __IMAGEFEATUREPOINTDETECTOR_2DCANNYEDGES_ORIENTATION_H_ */
