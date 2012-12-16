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
* \file ImageFeaturePointDetector_Base.h
* \author Rémi Blanc 
* \date 15. October 2012
*/


#ifndef __IMAGEFEATUREPOINTDETECTOR_BASE_H_
#define __IMAGEFEATUREPOINTDETECTOR_BASE_H_


#include "ITKUtils.h"

namespace psciob {

/** \brief ImageFeaturePointDetector_Base
*
* This is the base class for implementing feature point detectors, useful, e.g. for the Hough Transform
* The prototype behavior is, from an input image, to generate a list of pixel indices indicating the pixels of interest, according to some specific criterion (implemented by child classes)
* The second half of the job is to be able to associate a feature vector to those pixels.
* (another view on the task would be to compute a feature vector at each point, and use a classifier to select 'on' pixels)
*/


template<class InputImagePixelType, unsigned int InputImageDimension>
class ImageFeaturePointDetector_Base : public itk::LightObject {
public:
    /** Standard class typedefs. */
    typedef ImageFeaturePointDetector_Base Self;
    typedef itk::LightObject               Superclass;
    typedef itk::SmartPointer<Self>        Pointer;
    /** Run-time type information (and related methods). */
    itkTypeMacro(ImageFeaturePointDetector_Base, itk::LightObject);

	/** type of the image on which to perform the detection */
	typedef itk::Image<InputImagePixelType, InputImageDimension>  InputImageType;
	/** Type of pixel indices in the input image */
	typedef typename InputImageType::IndexType                    ImageIndexType;
	/** type defined to represent a feature vector */
	typedef vnl_vector<double>                                    FeatureVectorType;
	// ordering criterion for feature vectors, not intended for public use, but can be useful e.g. for the Hough Tranform
	typedef less_vnlvector< FeatureVectorType >                   FeatureVectorLessType;
	/** Type defined to represent feature points */
	typedef typename std::pair<ImageIndexType, FeatureVectorType> FeaturePointType;
	/** Type of pixel indices in the input image */
	typedef typename std::vector<FeaturePointType>                FeaturePointListType;
	/** Type of pixel indices in the input image */
	typedef typename FeaturePointListType::const_iterator         FeaturePointListConstIterator;

	/** gives the dimension of the feature vector */
	virtual unsigned GetFeatureVectorDimension() const = 0;
	
	/** Set the input image on which feature points need to be detected */
	void SetInputImage(typename InputImageType *image) { m_inputImage = image; m_uptodate=false; m_featurePointList.clear(); }

	/** Set the input image on which feature points need to be detected */
	InputImageType * GetInputImage() { return m_inputImage.GetPointer(); }

	/** Get the list of feature points */
	const typename FeaturePointListType & GetFeaturePointList() {
		if (!m_uptodate) Update();
		return m_featurePointList;
	}
	
	/** Update the detector */
	virtual void Update() = 0;
	
	/** Compute the feature vector at the requested location */
	virtual const FeatureVectorType & GetFeatureVectorAtLocation(const ImageIndexType &index) = 0;
	
protected:
	ImageFeaturePointDetector_Base() {
		m_uptodate = false;
		m_inputImage = NULL;
    };
    ~ImageFeaturePointDetector_Base() {};

	typename InputImageType::Pointer m_inputImage;
	
	bool m_uptodate;
	FeaturePointListType m_featurePointList;
	
private:
    ImageFeaturePointDetector_Base(const Self&); //purposely not implemented
    const Self & operator=( const Self & ); //purposely not implemented
};


} // namespace psciob

#endif /* __IMAGEFEATUREPOINTDETECTOR_BASE_H_ */
