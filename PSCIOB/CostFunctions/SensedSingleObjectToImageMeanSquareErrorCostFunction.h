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
 * \file SensedSingleObjectToImageMeanSquareErrorCostFunction.h
 * \author Rémi Blanc 
 * \date 3. October 2011
*/

#ifndef SENSEDSINGLEOBJECTTOIMAGEMEANSQUAREERRORCOSTFUNCTION_H_
#define SENSEDSINGLEOBJECTTOIMAGEMEANSQUAREERRORCOSTFUNCTION_H_

#include <SensedSingleObjectToImageCostFunction.h>
#include <itkImageMaskSpatialObject.h>
#include <itkBinaryThresholdImageFilter.h>
#include "itkRegionOfInterestImageFilter.h"

#include <itkImageSpatialObject.h>

namespace psciob {


/**\class SensedSingleObjectToImageMeanSquareErrorCostFunction 
 * \brief SensedSingleObjectToImageMeanSquareErrorCostFunction
 * similarity between the visible pixels of the sensed object and the reference image
 * using the mean square difference 
*/


template<class TScene, class TReferenceImage, class TSensedImage>
class SensedSingleObjectToImageMeanSquareErrorCostFunction : public SensedSingleObjectToImageCostFunction<TScene, TReferenceImage, TSensedImage> {
public:
	/** Standard class typedefs. */
	typedef SensedSingleObjectToImageMeanSquareErrorCostFunction	Self;
	typedef SensedSingleObjectToImageCostFunction					Superclass;
	typedef itk::SmartPointer<Self>									Pointer;
	/** Run-time type information (and related methods). */
	itkTypeMacro(SensedSingleObjectToImageMeanSquareErrorCostFunction, SensedSingleObjectToImageCostFunction);
	itkNewMacro(Self);

	typedef typename SensorType::OutputImageType			SensedImageType;
	typedef typename SensorType::OutputLabelImageType		LabelImageType;
	typedef typename LabelImageType::PixelType				LabelPixelType;	//make sure the scene is a scene of 'labels'
	typedef itk::Image<LabelPixelType, LabelImageType::ImageDimension>	BinaryImageType;

	typedef itk::ImageRegionConstIterator< SensedImageType >	ConstSensedImageIteratorType;	
	typedef itk::ImageRegionConstIterator< LabelImageType >		ConstLabelImageIteratorType;	
	typedef itk::ImageRegionConstIterator< ReferenceImageType > ConstReferenceImageIteratorType;	

	/** 
	 * Off Context: same value whether other objects alter the visibility of the current object 
	 * uses the OffContext method provided by the sensor
	*/
	double ComputeOffContextValue() {
		//pointers to the images of interest
		SensedImageType *sensedImage = m_sensor->GetOutput();
		ReferenceImageType *referenceImage = this->GetInterpolatedReferenceImage();

		//PixelSet of interest
		SensedLabelMapType::Pointer labelMap = SensedLabelMapType::New();
		m_sensor->GetOffContextObjectLabelMap(  m_scene->GetObject( m_requestedLabel ), labelMap );
		
		if (labelMap->GetNumberOfLabelObjects()==0) return 1e10;
		SensedLabelObjectType* labelObject = labelMap->GetNthLabelObject(0);

		//Do the real job!
		double value = 0, tmp;
		unsigned n = 0;

		for( unsigned lit = 0; lit < labelObject->GetNumberOfLines(); lit++ ) {			
			const SensedLabelMapType::IndexType & firstIdx = labelObject->GetLine(lit).GetIndex();
			const SensedLabelMapType::LengthType & length = labelObject->GetLine(lit).GetLength();
			n += length;
			long endIdx0 = firstIdx[0] + length;
			for( SensedLabelMapType::IndexType idx = firstIdx; idx[0]<endIdx0; idx[0]++) {
				tmp = (referenceImage->GetPixel(idx) - sensedImage->GetPixel(idx)) * (1.0/255.0);
				value += tmp*tmp;
			}
		}
		if (n<1) return 1e10;		
		value /= static_cast<double>(n); 
		return value;
	}

	/** 
	* In Context: get the in-context value : the object may be partially obscured by other objects, altering the metric 
	*/
	double ComputeInContextValue() {
		//pointers to the images of interest
		SensedImageType *sensedImage = m_sensor->GetOutput();
		ReferenceImageType *referenceImage = this->GetInterpolatedReferenceImage();

		//PixelSet of interest
		SensedLabelMapType* labelMap = m_sensor->GetLabelOutput();
		SensedLabelObjectType* labelObject = labelMap->GetLabelObject( m_requestedLabel );

		//Do the real job!
		double value = 0, tmp;
		unsigned n = 0;

		for( unsigned lit = 0; lit < labelObject->GetNumberOfLines(); lit++ ) {			
			const SensedLabelMapType::IndexType & firstIdx = labelObject->GetLine(lit).GetIndex();
			const SensedLabelMapType::LengthType & length = labelObject->GetLine(lit).GetLength();
			n += length;
			long endIdx0 = firstIdx[0] + length;
			for( SensedLabelMapType::IndexType idx = firstIdx; idx[0]<endIdx0; idx[0]++) {
				tmp = (referenceImage->GetPixel(idx) - sensedImage->GetPixel(idx)) * (1.0/255.0);
				value += tmp*tmp;
			}
		}
		if (n<1) return 1e10;		
		value /= static_cast<double>(n); 
		return value;
	}


protected:
	SensedSingleObjectToImageMeanSquareErrorCostFunction() : SensedSingleObjectToImageCostFunction() {};
	~SensedSingleObjectToImageMeanSquareErrorCostFunction() {};	

private:
	SensedSingleObjectToImageMeanSquareErrorCostFunction(const Self&);	//purposely not implemented
	const Self & operator=( const Self & );		//purposely not implemented
};

} // namespace psciob

#endif /* SENSEDSINGLEOBJECTTOIMAGEMEANSQUAREERRORCOSTFUNCTION_H_ */

