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
 * \file SensedSingleObjectInOutContrast.h
 * \author Rémi Blanc 
 * \date 24. November 2011
 */

#ifndef SENSEDSINGLEOBJECTINOUTCONTRAST_H_
#define SENSEDSINGLEOBJECTINOUTCONTRAST_H_


#include <SensedObjectWithSurroundingsToImageCostFunction.h>

namespace psciob {

/**
 * \class SensedSingleObjectInOutContrast
 * \brief SensedSingleObjectInOutContrast
 * coherence between the inside and outside of an object with respect to a reference image
 * sensitive about overlaps: if an object is not visible on the sensor, because another one gets in front of it => return a huge cost
 * as proposed in {pami09b_lafarge}
*/

template<class TScene, class TReferenceImage, class TSensedImage>
class SensedSingleObjectInOutContrast : public SensedObjectWithSurroundingsToImageCostFunction<TScene, TReferenceImage, TSensedImage> {
public:
	/** Standard class typedefs. */
	typedef SensedSingleObjectInOutContrast			Self;
	typedef SensedObjectWithSurroundingsToImageCostFunction	Superclass;
	typedef itk::SmartPointer<Self>				Pointer;
	/** Run-time type information (and related methods). */
	itkTypeMacro(SensedSingleObjectInOutContrast, SensedObjectWithSurroundingsToImageCostFunction);
	itkNewMacro(Self);


	/** 
	* In Context: get the in-context value : the object may be partially obscured by other objects, altering the metric 
	*/
	double ComputeInContextValue() {
		//pointers to the images of interest
		ReferenceImageType *referenceImage = this->GetInterpolatedReferenceImage();

		if (! m_sensor->GetInContextObjectAndSurroundingsLabelMap( m_scene->GetObject(m_requestedLabel), m_tmpLabelMap) ) {
			//object is not visible!!
			return 1e10;
		}
		SensedLabelMapType::Iterator objectIt =  SensedLabelMapType::Iterator(m_tmpLabelMap);
		objectIt.GoToBegin(); SensedLabelObjectType::Pointer insideObject = objectIt.GetLabelObject();
		++objectIt;           SensedLabelObjectType::Pointer outsideObject = objectIt.GetLabelObject();


		//Do the real job!
		unsigned n_in = 0, n_out = 0;
		double m_in = 0, m_out = 0, value=0;

		//inside pixels
		for( unsigned lit = 0; lit < insideObject->GetNumberOfLines(); lit++ ) {
			const SensedLabelMapType::IndexType & firstIdx = insideObject->GetLine(lit).GetIndex();
			const SensedLabelMapType::LengthType & length = insideObject->GetLine(lit).GetLength();
			n_in += length;
			for( SensedLabelMapType::IndexType idx = firstIdx; idx[0]<firstIdx[0] + length; idx[0]++) { m_in += referenceImage->GetPixel(idx); }
		}
		//surrounding pixels
		for( unsigned lit = 0; lit < outsideObject->GetNumberOfLines(); lit++ ) {
			const SensedLabelMapType::IndexType & firstIdx = outsideObject->GetLine(lit).GetIndex();
			const SensedLabelMapType::LengthType & length = outsideObject->GetLine(lit).GetLength();
			n_out += length;
			for( SensedLabelMapType::IndexType idx = firstIdx; idx[0]<firstIdx[0] + length; idx[0]++) { m_out += referenceImage->GetPixel(idx); }
		}

		if (n_in<=1) { value = 1e10; } //object is too small for this metric ; throw exception / print warning?
		else {
			m_in /=static_cast<double>(n_in); 
			m_out/=static_cast<double>(n_out); 

			double diff = (m_in-m_out)/(m_refImgMaxValue-m_refImgMinValue);
			if (m_in>m_out) { value = 1-diff; }
			else			{ value = exp(-5.0*diff); }
		}
		return value;
	}

	/** 
	* Off Context: same value whether other objects alter the visibility of the current object 
	* uses the OffContext method provided by the sensor
	*/
	double ComputeOffContextValue() {
		ReferenceImageType *referenceImage = this->GetInterpolatedReferenceImage();

		////PixelSet of interest
		//SensedLabelMapType::Pointer insideMap = SensedLabelMapType::New();
		//m_sensor->GetOffContextObjectLabelMap(  m_scene->GetObject( m_requestedLabel ), insideMap );
		//SensedLabelObjectType::Pointer insideObject = insideMap->GetNthLabelObject(0);

		//SensedLabelMapType::Pointer outsideMap = SensedLabelMapType::New();
		//outsideMap->SetSpacing(insideMap->GetSpacing()); outsideMap->SetOrigin(insideMap->GetOrigin()); outsideMap->SetRegions(insideMap->GetLargestPossibleRegion());

		//GetSurroundingPixelSet(insideMap, outsideMap);
		//SensedLabelObjectType::Pointer outsideObject = outsideMap->GetNthLabelObject(0);
		m_sensor->GetOffContextObjectAndSurroundingsLabelMap( m_scene->GetObject(m_requestedLabel), m_tmpLabelMap);

		SensedLabelMapType::Iterator objectIt =  SensedLabelMapType::Iterator(m_tmpLabelMap);
		objectIt.GoToBegin(); SensedLabelObjectType::Pointer insideObject = objectIt.GetLabelObject();
		++objectIt;           SensedLabelObjectType::Pointer outsideObject = objectIt.GetLabelObject();

		//Do the real job!
		unsigned n_in = 0, n_out = 0;
		double m_in = 0, m_out = 0, value=0;

		//inside pixels
		for( unsigned lit = 0; lit < insideObject->GetNumberOfLines(); lit++ ) {
			const SensedLabelMapType::IndexType & firstIdx = insideObject->GetLine(lit).GetIndex();
			const SensedLabelMapType::LengthType & length = insideObject->GetLine(lit).GetLength();
			n_in += length;
			for( SensedLabelMapType::IndexType idx = firstIdx; idx[0]<firstIdx[0] + length; idx[0]++) { m_in += referenceImage->GetPixel(idx); }
		}
		//surrounding pixels
		for( unsigned lit = 0; lit < outsideObject->GetNumberOfLines(); lit++ ) {
			const SensedLabelMapType::IndexType & firstIdx = outsideObject->GetLine(lit).GetIndex();
			const SensedLabelMapType::LengthType & length = outsideObject->GetLine(lit).GetLength();
			n_out += length;
			for( SensedLabelMapType::IndexType idx = firstIdx; idx[0]<firstIdx[0] + length; idx[0]++) { m_out += referenceImage->GetPixel(idx); }
		}

		if (n_in<=1) { value = 1e10; } //object is too small for this metric ; throw exception / print warning?
		else {
			m_in /=static_cast<double>(n_in); 
			m_out/=static_cast<double>(n_out); 

			double diff = (m_in-m_out)/(m_refImgMaxValue-m_refImgMinValue);
			if (m_in>m_out) { value = 1.0-diff; }
			else			{ value = exp(-5.0*diff); }
		}
		return value;
	}


protected:
	SensedSingleObjectInOutContrast() : SensedObjectWithSurroundingsToImageCostFunction() { 
	};
	~SensedSingleObjectInOutContrast() {};	

private:
	SensedSingleObjectInOutContrast(const Self&);	//purposely not implemented
	const Self & operator=( const Self & );		//purposely not implemented
};


} // namespace psciob

#endif /* SENSEDSINGLEOBJECTINOUTCONTRAST_H_ */
