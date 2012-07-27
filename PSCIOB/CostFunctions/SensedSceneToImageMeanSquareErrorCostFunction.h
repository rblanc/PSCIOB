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
 * \file SensedSceneToImageMeanSquareErrorCostFunction.h
 * \author Rémi Blanc
 * \date 9. September 2011
 *
 */

#ifndef SENSEDSCENETOIMAGEMEANSQUAREERRORCOSTFUNCTION_H_
#define SENSEDSCENETOIMAGEMEANSQUAREERRORCOSTFUNCTION_H_


#include <SensedSceneToImageCostFunction.h>
#include <itkImageMaskSpatialObject.h>

namespace psciob {

/** 
 * \class SensedSceneToImageMeanSquareErrorCostFunction
 * \brief SensedSceneToImageMeanSquareErrorCostFunction 
 *
 * similarity between the sensed scene and the reference image
 * using the mean square difference between each pixel/voxel
 */



template<class TScene, class TReferenceImage, class TSensedImage>
class SensedSceneToImageMeanSquareErrorCostFunction : public SensedSceneToImageCostFunction<TScene, TReferenceImage, TSensedImage> {
public:
	/** Standard class typedefs. */
	typedef SensedSceneToImageMeanSquareErrorCostFunction	Self;
	typedef SensedSceneToImageCostFunction					Superclass;
	typedef itk::SmartPointer<Self>			Pointer;
	/** Run-time type information (and related methods). */
	itkTypeMacro(SensedSceneToImageMeanSquareErrorCostFunction, SensedSceneToImageCostFunction);
	itkNewMacro(Self);

	/** Evaluate the mean square differences between the sensed image and the reference image 
	 *  interpolating the reference image automatically if necessary
	 */
	double GetRawValue() {
		double output = 0, tmp;

		SensedImageType* sensedImage    = m_sensor->GetOutput();
		ReferenceImageType* referenceImage = this->GetInterpolatedReferenceImage();
		
		itk::ImageRegionIterator< ReferenceImageType > referenceIt( referenceImage, referenceImage->GetLargestPossibleRegion() );
		itk::ImageRegionIterator< SensedImageType >    sensorIt( sensedImage, sensedImage->GetLargestPossibleRegion() );

		//sum of square differences
		for ( referenceIt.GoToBegin(), sensorIt.GoToBegin(); !sensorIt.IsAtEnd(); ++referenceIt, ++sensorIt) {
			tmp = referenceIt.Get() - sensorIt.Get();
			output += tmp*tmp;
		}

		return output / (double)referenceImage->GetLargestPossibleRegion().GetNumberOfPixels( );
	}


protected:
	SensedSceneToImageMeanSquareErrorCostFunction() : SensedSceneToImageCostFunction() {};
	~SensedSceneToImageMeanSquareErrorCostFunction() {};	

private:
	SensedSceneToImageMeanSquareErrorCostFunction(const Self&);	//purposely not implemented
	const Self & operator=( const Self & );		//purposely not implemented
};


} // namespace psciob

#endif /* SENSEDSCENETOIMAGEMEANSQUAREERRORCOSTFUNCTION_H_ */