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
 * \file SensedSingleObjectInOutWeighted_SAD.h
 * \author Rémi Blanc 
 * \date 3. October 2012
 */

#ifndef __SENSEDSINGLEOBJECTINOUTWEIGHTED_SAD_H_
#define __SENSEDSINGLEOBJECTINOUTWEIGHTED_SAD_H_


#include <SensedObjectWithSurroundingsToImageCostFunction.h>

namespace psciob {

/**
 * \class SensedSingleObjectInOutWeighted_SAD
 * \brief SensedSingleObjectInOutWeighted_SAD
 * SAD stands for sum of absolute differences
 * weight the sum of average SAD inside and average SAD outside the object
 * 0 means good ; while increasing values indicate a bad match
*/

template<class TScene, class TReferenceImage, class TSensedImage>
class SensedSingleObjectInOutWeighted_SAD : public SensedObjectWithSurroundingsToImageCostFunction<TScene, TReferenceImage, TSensedImage> {
public:
	/** Standard class typedefs. */
	typedef SensedSingleObjectInOutWeighted_SAD             Self;
	typedef SensedObjectWithSurroundingsToImageCostFunction	Superclass;
	typedef itk::SmartPointer<Self>				            Pointer;
	/** Run-time type information (and related methods). */
	itkTypeMacro(SensedSingleObjectInOutWeighted_SAD, SensedObjectWithSurroundingsToImageCostFunction);
	itkNewMacro(Self);

	/** Set the relative weight w to be applied to the inside pixels
	* the weight attributed to the outside pixels is (1-w)
	* by default, this is 0.5
	*/
	void SetRelativeWeightInsidePixel(double w=0.5) {
		if ((w<0) || (w>1)) {
			std::cout<<"Warning SensedSingleObjectInOutWeighted_SAD::SetRelativeWeightInsidePixel : the input parameter should be in [0,1] while it was set to "<<w<<" ; it is automatically reset to 0.5"<<std::endl;
			w=0.5;
		}
		m_win = w;
	}

	/** 
	* In Context: get the in-context value : the object may be partially obscured by other objects, altering the metric 
	*/
	double ComputeInContextValue() {
		//pointers to the images of interest
		ReferenceImageType *referenceImage = this->GetInterpolatedReferenceImage();
		SensorType::OutputImageType *sensorImage = m_sensor->GetOutputImage(); // = in context image

		if (! m_sensor->GetInContextObjectAndSurroundingsLabelMap( m_scene->GetObject(m_requestedLabel), m_tmpLabelMap) ) {
			//object is not visible!!
			return 1e10;
		}
		SensedLabelMapType::Iterator objectIt =  SensedLabelMapType::Iterator(m_tmpLabelMap);
		objectIt.GoToBegin(); SensedLabelObjectType *insideObject = objectIt.GetLabelObject();
		++objectIt;           SensedLabelObjectType *outsideObject = objectIt.GetLabelObject();


		//Do the real job!
		unsigned n_in = 0, n_out = 0;
		double sad_in=0, sad_out=0, value=0;

		//inside pixels
		for( unsigned lit = 0; lit < insideObject->GetNumberOfLines(); lit++ ) {
			const SensedLabelMapType::IndexType & firstIdx = insideObject->GetLine(lit).GetIndex();
			const SensedLabelMapType::LengthType & length = insideObject->GetLine(lit).GetLength();
			n_in += length;
			for( SensedLabelMapType::IndexType idx = firstIdx; idx[0]<firstIdx[0] + length; idx[0]++) { 
				sad_in += fabs( referenceImage->GetPixel(idx) - sensorImage->GetPixel(idx) ); 
			}
		}
		//surrounding pixels
		for( unsigned lit = 0; lit < outsideObject->GetNumberOfLines(); lit++ ) {
			const SensedLabelMapType::IndexType & firstIdx = outsideObject->GetLine(lit).GetIndex();
			const SensedLabelMapType::LengthType & length = outsideObject->GetLine(lit).GetLength();
			n_out += length;
			for( SensedLabelMapType::IndexType idx = firstIdx; idx[0]<firstIdx[0] + length; idx[0]++) { 
				sad_out += fabs( referenceImage->GetPixel(idx) - sensorImage->GetPixel(idx) ); 
			}
		}

		if (n_in<1) { value = 1e10; } //object is too small for this metric ; throw exception / print warning?
		else {
			sad_in /=static_cast<double>(n_in); 
			sad_out/=static_cast<double>(n_out); 

			value = m_win*sad_in + (1.0-m_win)*sad_out; 
		}
		return value;
	}

	/** 
	* Off Context: same value whether other objects alter the visibility of the current object 
	* uses the OffContext method provided by the sensor
	*/
	double ComputeOffContextValue() {
		ReferenceImageType *referenceImage = this->GetInterpolatedReferenceImage();

		//warning, the offContextObjectImage is given with respect to the smallest necessary bbox
		//the pixel indices do not match directly with those provided by GetOffContextObjectAndSurroundingsLabelMap
		if (!m_sensor->GetOffContextObjectImage( m_scene->GetObject(m_requestedLabel), m_offContextObjectImage)) {
			throw DeformableModelException("SensedSingleObjectInOutWeighted_SAD::ComputeOffContextValue : error getting the small offcontext image of the object ; should never happen...");
		}
		m_sensor->GetOffContextObjectAndSurroundingsLabelMap( m_scene->GetObject(m_requestedLabel), m_tmpLabelMap);
		//compute the offset to apply to thumbnail indices
		const SensorType::OutputImageType::PointType &imageOrigin = m_tmpLabelMap->GetOrigin();
		const SensorType::OutputImageType::PointType &thumbnailOrigin = m_offContextObjectImage->GetOrigin();
		const SensorType::OutputImageType::SpacingType &spacing = m_offContextObjectImage->GetSpacing();
		SensorType::OutputImageType::OffsetType offset; //this ought to be negative...
		for (unsigned i=0 ; i<SensorType::OutputDimension ; i++) offset[i] = round((imageOrigin[i] - thumbnailOrigin[i])/spacing[i]);

		SensedLabelMapType::Iterator objectIt =  SensedLabelMapType::Iterator(m_tmpLabelMap);
		objectIt.GoToBegin(); SensedLabelObjectType *insideObject = objectIt.GetLabelObject();
		++objectIt;           SensedLabelObjectType *outsideObject = objectIt.GetLabelObject();

		//Do the real job!
		unsigned n_in = 0, n_out = 0;
		double sad_in=0, sad_out=0, value=0;

		//inside pixels
		for( unsigned lit = 0; lit < insideObject->GetNumberOfLines(); lit++ ) {
			const SensedLabelMapType::IndexType & firstIdx = insideObject->GetLine(lit).GetIndex();
			const SensedLabelMapType::LengthType & length = insideObject->GetLine(lit).GetLength();
			n_in += length;
			for( SensedLabelMapType::IndexType idx = firstIdx; idx[0]<firstIdx[0] + length; idx[0]++) { 
				sad_in += fabs( referenceImage->GetPixel(idx) - m_offContextObjectImage->GetPixel(idx+offset)); 
			}
		}
		//surrounding pixels => there are necessarily 0 outside
		//and some of them are anyway outside the bounds of the thumbnail, so don't try to get there to check
		for( unsigned lit = 0; lit < outsideObject->GetNumberOfLines(); lit++ ) {
			const SensedLabelMapType::IndexType & firstIdx = outsideObject->GetLine(lit).GetIndex();
			const SensedLabelMapType::LengthType & length = outsideObject->GetLine(lit).GetLength();
			n_out += length;
			for( SensedLabelMapType::IndexType idx = firstIdx; idx[0]<firstIdx[0] + length; idx[0]++) { 
				sad_out += fabs( referenceImage->GetPixel(idx) - 0.0 ); //necessarily 0 outside...
			}
		}

		if (n_in<1) { value = 1e10; } //object is too small for this metric ; throw exception / print warning?
		else {
			sad_in /=static_cast<double>(n_in); 
			sad_out/=static_cast<double>(n_out); 

			value = m_win*sad_in + (1.0-m_win)*sad_out; 
		}
		return value;
	}


protected:
	SensedSingleObjectInOutWeighted_SAD() : SensedObjectWithSurroundingsToImageCostFunction() { 
		m_offContextObjectImage = SensorType::OutputImageType::New();
		m_win = 0.5;
	};
	~SensedSingleObjectInOutWeighted_SAD() {};	

	typename SensorType::OutputImageType::Pointer m_offContextObjectImage;
	double m_win;
private:
	SensedSingleObjectInOutWeighted_SAD(const Self&);	//purposely not implemented
	const Self & operator=( const Self & );		//purposely not implemented
};


} // namespace psciob

#endif /* __SENSEDSINGLEOBJECTINOUTWEIGHTED_SAD_H_ */
