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
* \file ImageFeaturePointDetector_GradientAngles.h
* \author R�mi Blanc 
* \date 15. October 2012
*/


#ifndef __IMAGEFEATUREPOINTDETECTOR_GRADIENTANGLES_H_
#define __IMAGEFEATUREPOINTDETECTOR_GRADIENTANGLES_H_


namespace psciob {

/** \brief ImageFeaturePointDetector_GradientAngles
* Selects points based on their gradient magnitude, and use the angle(s) to describe the gradient orientation as a feature vector
*/

#include "ITKUtils.h"


template<class InputImagePixelType, unsigned int InputImageDimension>
class ImageFeaturePointDetector_GradientAngles : public itk::LightObject {
public:
    /** Standard class typedefs. */
    typedef ImageFeaturePointDetector_GradientAngles    Self;
    typedef itk::LightObject        Superclass;
    typedef itk::SmartPointer<Self> Pointer;
    /** Run-time type information (and related methods). */
    itkTypeMacro(ImageFeaturePointDetector_GradientAngles, itk::LightObject);
    itkNewMacro(Self);

	
	/** Update the detector 
	* Compute the gradient 
	*/
	void Update() {
	}
	
	/** Compute the feature vector at the requested location */
	vnl_vector<double> GetFeatureVectorAtLocation(const ImageIndexType &index) {
	}
	
protected:
	ImageFeaturePointDetector_GradientAngles() : {
		m_uptodate = false;
		m_inputImage = NULL;
    };
    ~ImageFeaturePointDetector_GradientAngles() {};

	typename InputImageType::Pointer m_inputImage;
	
	bool m_uptodate;
	FeaturePointListType m_featurePointList;
	
private:
    ImageFeaturePointDetector_GradientAngles(const Self&); //purposely not implemented
    const Self & operator=( const Self & ); //purposely not implemented
};


} // namespace psciob

#endif /* __IMAGEFEATUREPOINTDETECTOR_GRADIENTANGLES_H_ */
