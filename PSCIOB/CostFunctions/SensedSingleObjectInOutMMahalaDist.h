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
* \file SensedSingleObjectInOutMMahalaDist.h
* \author Rémi Blanc 
* \date 2. November 2011
*/

#ifndef __SENSEDSINGLEOBJECTINOUTMMAHALADIST_H_
#define __SENSEDSINGLEOBJECTINOUTMMAHALADIST_H_


#include <SensedObjectWithSurroundingsToImageCostFunction.h>
#include <LineIntegralImageAndSquaredImageFilter.h>

namespace psciob {

/**
* \class SensedSingleObjectInOutMMahalaDist
* \brief SensedSingleObjectInOutMMahalaDist
* modified Mahalanobis distance between the inside and outside of an object with respect to a reference image
* as proposed in  {Geometric Feature Extraction by a multiMarked Point Process, Lafarge, Gimel'farb, Descombes}
*
* I removed the strange factor related to the object's surface
*
* except that the 'theta' parameter should be handled by wrapping the object into a rescaling function, e.g.
* GPF_LinearRescaling<double, double>::Pointer normalizationFunction = GPF_LinearRescaling<double, double>::New();
* vnl_vector<double> normFParams(2); normFParams(0) = -theta; normFParams(1)=1; normalizationFunction->SetParameters(normFParams);
* MMahalanobisCostFunction->SetNormalizationFunction(normalizationFunction);
*
* The class expects a scalar image (single band...)
*/

template<class TScene, class TReferenceImage, class TSensedImage>
class SensedSingleObjectInOutMMahalaDist : public SensedObjectWithSurroundingsToImageCostFunction<TScene, TReferenceImage, TSensedImage> {
public:
	/** Standard class typedefs. */
	typedef SensedSingleObjectInOutMMahalaDist		Self;
	typedef SensedObjectWithSurroundingsToImageCostFunction	Superclass;
	typedef itk::SmartPointer<Self>				Pointer;
	/** Run-time type information (and related methods). */
	itkTypeMacro(SensedSingleObjectInOutMMahalaDist, SensedObjectWithSurroundingsToImageCostFunction);
	itkNewMacro(Self);

	/** Create a clone (= an exact, independant copy) of the current cost function 
	* TODO: (dev opt) call the superclass method, and copy the additional data, instead of duplicating the code.
	*/
	virtual BaseClassPointer CreateClone() {
		Pointer clonePtr = static_cast<Self*>(this->CreateAnother().GetPointer());
		if (!m_transformFunction) {} else { clonePtr->SetNormalizationFunction(m_transformFunction); }
		if (!m_scene) {} else { clonePtr->SetScene(m_scene); }
		clonePtr->SelectObject(m_requestedLabel);
		if (!m_referenceImage) {} else { clonePtr->SetReferenceImage(m_referenceImage); }
		clonePtr->SetInterpolator( static_cast<InterpolatorType *>(m_interpolator->CreateAnother().GetPointer()) );
		if (!m_sensor)         {} else { clonePtr->SetSensor( m_sensor->CreateClone() );}
		clonePtr->m_inContextMetricFlag = m_inContextMetricFlag;
		clonePtr->m_surroundingRadius = m_surroundingRadius;

		clonePtr->m_bright = m_bright;

		return static_cast<BaseClass*>( clonePtr );
	}


	/** Configure the function to detect objects that are brighter (true ; default) or darker than their surroundings pixels */
	void DetectBrightObjects(bool b = true) { 
		if (b) m_bright=1;
		else m_bright=-1;
	}

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
		double m_in = 0, s_in = 0, m_out = 0, s_out = 0, value=0;

		//pixels inside the object
		SensedLabelMapType::IndexType firstIdx, endIdx;
		for( unsigned lit = 0; lit < insideObject->GetNumberOfLines(); lit++ ) {
			firstIdx = insideObject->GetLine(lit).GetIndex();
			const SensedLabelMapType::LengthType & length = insideObject->GetLine(lit).GetLength();
			n_in += length;
			endIdx = firstIdx; endIdx[0] = firstIdx[0] + length - 1; //last index visited -> -1
			if (firstIdx[0]==0) {
				m_in += m_partialIntegralImage->GetPixel( endIdx );
				s_in += ( m_partialIntegralSquaredImage->GetPixel( endIdx ) );
			}
			else {
				firstIdx[0]--;
				m_in += ( m_partialIntegralImage->GetPixel( endIdx ) - m_partialIntegralImage->GetPixel( firstIdx ) );
				s_in += ( m_partialIntegralSquaredImage->GetPixel( endIdx ) - m_partialIntegralSquaredImage->GetPixel( firstIdx ) );
			}

		}

		//pixels outside the object
		for( unsigned lit = 0; lit < outsideObject->GetNumberOfLines(); lit++ ) {
			firstIdx = outsideObject->GetLine(lit).GetIndex();
			const SensedLabelMapType::LengthType & length = outsideObject->GetLine(lit).GetLength();
			n_out += length;
			endIdx = firstIdx; endIdx[0] = firstIdx[0] + length - 1; //last index visited -> -1
			if (firstIdx[0]==0) {
				m_out += m_partialIntegralImage->GetPixel( endIdx );
				s_out += ( m_partialIntegralSquaredImage->GetPixel( endIdx ) );
			}
			else {
				firstIdx[0]--;
				m_out += ( m_partialIntegralImage->GetPixel( endIdx ) - m_partialIntegralImage->GetPixel( firstIdx ) );
				s_out += ( m_partialIntegralSquaredImage->GetPixel( endIdx ) - m_partialIntegralSquaredImage->GetPixel( firstIdx ) );
			}
		}

		//metric computation
		if (n_in<=1) { value = 1e10; } //object is too small for this metric ; throw exception / print warning?
		else {
			s_in = ( s_in -  m_in*m_in /static_cast<double>(n_in) ) / static_cast<double>(n_in -1);	m_in /=static_cast<double>(n_in); 
			s_out= ( s_out- m_out*m_out/static_cast<double>(n_out)) / static_cast<double>(n_out-1);	m_out/=static_cast<double>(n_out); 

			if ( m_bright*(m_in-m_out)>0 ) value = sqrt( (s_in+s_out+TINY) / ( (m_in-m_out)*(m_in-m_out)) ); 
			else value = 1e10;
		}
		return value;
	}

	/** 
	 * Off Context: same value whether other objects alter the visibility of the current object 
	 * uses the OffContext method provided by the sensor
	*/
	double ComputeOffContextValue() {
		//ReferenceImageType *referenceImage = this->GetInterpolatedReferenceImage();
		this->GetInterpolatedReferenceImage(); //make sure the reference image, and partial integral images are uptodate.

		m_sensor->GetOffContextObjectAndSurroundingsLabelMap( m_scene->GetObject(m_requestedLabel), m_tmpLabelMap);

		SensedLabelMapType::Iterator objectIt =  SensedLabelMapType::Iterator(m_tmpLabelMap);
		objectIt.GoToBegin(); SensedLabelObjectType::Pointer insideObject = objectIt.GetLabelObject();
		++objectIt;           SensedLabelObjectType::Pointer outsideObject = objectIt.GetLabelObject();


		//Do the real job!
		unsigned n_in = 0, n_out = 0;
		double m_in = 0, s_in = 0, m_out = 0, s_out = 0, value=0;

		//pixels inside the object
		SensedLabelMapType::IndexType firstIdx, endIdx;
		for( unsigned lit = 0; lit < insideObject->GetNumberOfLines(); lit++ ) {
			firstIdx = insideObject->GetLine(lit).GetIndex();
			const SensedLabelMapType::LengthType & length = insideObject->GetLine(lit).GetLength();
			n_in += length;
			endIdx = firstIdx; endIdx[0] = firstIdx[0] + length - 1; //last index visited -> -1
			if (firstIdx[0]==0) {
				m_in += m_partialIntegralImage->GetPixel( endIdx );
				s_in += ( m_partialIntegralSquaredImage->GetPixel( endIdx ) );
			}
			else {
				firstIdx[0]--;
				m_in += ( m_partialIntegralImage->GetPixel( endIdx ) - m_partialIntegralImage->GetPixel( firstIdx ) );
				s_in += ( m_partialIntegralSquaredImage->GetPixel( endIdx ) - m_partialIntegralSquaredImage->GetPixel( firstIdx ) );
			}

		}

		//pixels outside the object
		for( unsigned lit = 0; lit < outsideObject->GetNumberOfLines(); lit++ ) {
			firstIdx = outsideObject->GetLine(lit).GetIndex();
			const SensedLabelMapType::LengthType & length = outsideObject->GetLine(lit).GetLength();
			n_out += length;
			endIdx = firstIdx; endIdx[0] = firstIdx[0] + length - 1; //last index visited -> -1
			if (firstIdx[0]==0) {
				m_out += m_partialIntegralImage->GetPixel( endIdx );
				s_out += ( m_partialIntegralSquaredImage->GetPixel( endIdx ) );
			}
			else {
				firstIdx[0]--;
				m_out += ( m_partialIntegralImage->GetPixel( endIdx ) - m_partialIntegralImage->GetPixel( firstIdx ) );
				s_out += ( m_partialIntegralSquaredImage->GetPixel( endIdx ) - m_partialIntegralSquaredImage->GetPixel( firstIdx ) );
			}
		}

		//metric computation
		if (n_in<=1) { value = 1e10; } //object is too small for this metric ; throw exception / print warning?
		else {
			s_in = ( s_in -  m_in*m_in /static_cast<double>(n_in) ) / static_cast<double>(n_in -1);	m_in /=static_cast<double>(n_in); 
			s_out= ( s_out- m_out*m_out/static_cast<double>(n_out)) / static_cast<double>(n_out-1);	m_out/=static_cast<double>(n_out); 

			if ( m_bright*(m_in-m_out)>0 ) value = sqrt( (s_in+s_out+TINY) / ( (m_in-m_out)*(m_in-m_out)) ); 
			else value = 1e10;
		}
		return value;
	}

	void PrintDetails() {
		if (m_inContextMetricFlag) {
			//copy-paste of the ComputeInContextValue()
			ReferenceImageType *referenceImage = this->GetInterpolatedReferenceImage();
			m_sensor->GetInContextObjectAndSurroundingsLabelMap( m_scene->GetObject(m_requestedLabel), m_tmpLabelMap);
			SensedLabelMapType::Iterator objectIt =  SensedLabelMapType::Iterator(m_tmpLabelMap);
			objectIt.GoToBegin(); SensedLabelObjectType::Pointer insideObject = objectIt.GetLabelObject();
			++objectIt;           SensedLabelObjectType::Pointer outsideObject = objectIt.GetLabelObject();
			unsigned n_in = 0, n_out = 0;
			double m_in = 0, s_in = 0, m_out = 0, s_out = 0, value=0;

			SensedLabelMapType::IndexType firstIdx, endIdx;
			for( unsigned lit = 0; lit < insideObject->GetNumberOfLines(); lit++ ) {
				firstIdx = insideObject->GetLine(lit).GetIndex();
				const SensedLabelMapType::LengthType & length = insideObject->GetLine(lit).GetLength();
				n_in += length;
				endIdx = firstIdx; endIdx[0] = firstIdx[0] + length - 1;
				if (firstIdx[0]==0) {
					m_in += m_partialIntegralImage->GetPixel( endIdx );
					s_in += ( m_partialIntegralSquaredImage->GetPixel( endIdx ) );
				}
				else {
					firstIdx[0]--;
					m_in += ( m_partialIntegralImage->GetPixel( endIdx ) - m_partialIntegralImage->GetPixel( firstIdx ) );
					s_in += ( m_partialIntegralSquaredImage->GetPixel( endIdx ) - m_partialIntegralSquaredImage->GetPixel( firstIdx ) );
				}
			}
			for( unsigned lit = 0; lit < outsideObject->GetNumberOfLines(); lit++ ) {
				firstIdx = outsideObject->GetLine(lit).GetIndex();
				const SensedLabelMapType::LengthType & length = outsideObject->GetLine(lit).GetLength();
				n_out += length;
				endIdx = firstIdx; endIdx[0] = firstIdx[0] + length - 1;
				if (firstIdx[0]==0) {
					m_out += m_partialIntegralImage->GetPixel( endIdx );
					s_out += ( m_partialIntegralSquaredImage->GetPixel( endIdx ) );
				}
				else {
					firstIdx[0]--;
					m_out += ( m_partialIntegralImage->GetPixel( endIdx ) - m_partialIntegralImage->GetPixel( firstIdx ) );
					s_out += ( m_partialIntegralSquaredImage->GetPixel( endIdx ) - m_partialIntegralSquaredImage->GetPixel( firstIdx ) );
				}
			}
			if (n_in<=1) { value = 1e10; } //object is too small for this metric ; throw exception / print warning?
			else {
				s_in = ( s_in -  m_in*m_in /static_cast<double>(n_in) ) / static_cast<double>(n_in -1);	m_in /=static_cast<double>(n_in); 
				s_out= ( s_out- m_out*m_out/static_cast<double>(n_out)) / static_cast<double>(n_out-1);	m_out/=static_cast<double>(n_out); 

				if ( m_bright*(m_in-m_out)>0 ) value = sqrt( (s_in+s_out+TINY) / ( (m_in-m_out)*(m_in-m_out)) ); 				else value = 1e10;
			}
			std::cout<<"   object "<<m_requestedLabel<<" INContext ; m_in = "<<m_in<<", s_in = "<<s_in<<", n_in = "<<n_in<<", m_out = "<<m_out<<", s_out = "<<s_out<<", n_out = "<<n_out<<" ; value = "<<value<<std::endl;
		}
		else {
			ReferenceImageType *referenceImage = this->GetInterpolatedReferenceImage();
			m_sensor->GetOffContextObjectAndSurroundingsLabelMap( m_scene->GetObject(m_requestedLabel), m_tmpLabelMap);
			SensedLabelMapType::Iterator objectIt =  SensedLabelMapType::Iterator(m_tmpLabelMap);
			objectIt.GoToBegin(); SensedLabelObjectType::Pointer insideObject = objectIt.GetLabelObject(); 
			++objectIt;           SensedLabelObjectType::Pointer outsideObject = objectIt.GetLabelObject();
			unsigned n_in = 0, n_out = 0;
			double m_in = 0, s_in = 0, m_out = 0, s_out = 0, value=0;

			SensedLabelMapType::IndexType firstIdx, endIdx;
			for( unsigned lit = 0; lit < insideObject->GetNumberOfLines(); lit++ ) {
				firstIdx = insideObject->GetLine(lit).GetIndex();
				const SensedLabelMapType::LengthType & length = insideObject->GetLine(lit).GetLength();
				n_in += length;
				endIdx = firstIdx; endIdx[0] = firstIdx[0] + length - 1;
				if (firstIdx[0]==0) {
					m_in += m_partialIntegralImage->GetPixel( endIdx );
					s_in += ( m_partialIntegralSquaredImage->GetPixel( endIdx ) );
				}
				else {
					firstIdx[0]--;
					m_in += ( m_partialIntegralImage->GetPixel( endIdx ) - m_partialIntegralImage->GetPixel( firstIdx ) );
					s_in += ( m_partialIntegralSquaredImage->GetPixel( endIdx ) - m_partialIntegralSquaredImage->GetPixel( firstIdx ) );
				}
			
			}
			for( unsigned lit = 0; lit < outsideObject->GetNumberOfLines(); lit++ ) {
				firstIdx = outsideObject->GetLine(lit).GetIndex();
				const SensedLabelMapType::LengthType & length = outsideObject->GetLine(lit).GetLength();
				n_out += length;
				endIdx = firstIdx; endIdx[0] = firstIdx[0] + length - 1;
				if (firstIdx[0]==0) {
					m_out += m_partialIntegralImage->GetPixel( endIdx );
					s_out += ( m_partialIntegralSquaredImage->GetPixel( endIdx ) );
				}
				else {
					firstIdx[0]--;
					m_out += ( m_partialIntegralImage->GetPixel( endIdx ) - m_partialIntegralImage->GetPixel( firstIdx ) );
					s_out += ( m_partialIntegralSquaredImage->GetPixel( endIdx ) - m_partialIntegralSquaredImage->GetPixel( firstIdx ) );
				}
			}
			if (n_in<=1) { value = 1e10; } //object is too small for this metric ; throw exception / print warning?
			else {
				s_in = ( s_in -  m_in*m_in /static_cast<double>(n_in) ) / static_cast<double>(n_in -1);	m_in /=static_cast<double>(n_in); 
				s_out= ( s_out- m_out*m_out/static_cast<double>(n_out)) / static_cast<double>(n_out-1);	m_out/=static_cast<double>(n_out); 

				if ( m_bright*(m_in-m_out)>0 ) value = sqrt( (s_in+s_out+TINY) / ( (m_in-m_out)*(m_in-m_out)) ); 
				else value = 1e10;
			}
			std::cout<<"   object "<<m_requestedLabel<<" OFFContext ; m_in = "<<m_in<<", s_in = "<<s_in<<", n_in = "<<n_in<<", m_out = "<<m_out<<", s_out = "<<s_out<<", n_out = "<<n_out<<" ; value = "<<value<<std::endl;
		}
	}

	/** Get the partial integral image used to speed-up the metric evaluation - this is essentially for testing purposes */
	IntegralImageType* GetPartialIntegralImage() {
		GetInterpolatedReferenceImage(); //make sure it is up to date...
		return m_partialIntegralImage.GetPointer();
	}

	/** Get the partial integral squared image used to speed-up the metric evaluation - this is essentially for testing purposes */
	IntegralImageType* GetPartialIntegralSquaredImage() {
		GetInterpolatedReferenceImage(); //make sure it is up to date...
		return m_partialIntegralSquaredImage.GetPointer();
	}

protected:
	SensedSingleObjectInOutMMahalaDist() : SensedObjectWithSurroundingsToImageCostFunction() {
		m_partialIntegralImage = IntegralImageType::New();
		m_partialIntegralSquaredImage = IntegralImageType::New();
		m_bright = 1;
	};
	~SensedSingleObjectInOutMMahalaDist() {};	

	double m_bright;
	typename IntegralImageType::Pointer m_partialIntegralImage, m_partialIntegralSquaredImage;

	/** DEVELOPER: do not forget to set m_interpolatedReferenceImage 
	* kept virtual at this point, because the interpolation may be different depending whether a sensor is used, or directly the scene... ??? not sure this is really relevant...
	* \warning: if the sensor is modified (e.g. its resolution is modified), the metric is not informed automatically...
	* 
	* In addition to the base method, a partial integral image is computed (along the rows), for the image and squared image
	* in order to speed-up the evaluation of the metric.
	*/
	void InterpolateReferenceImage() {
		if (!m_sensor)			throw DeformableModelException("SensedSceneToImageCostFunction::InterpolateReferenceImage : the sensor must be set!");
		if (!m_referenceImage)	throw DeformableModelException("SensedSceneToImageCostFunction::InterpolateReferenceImage : the reference image must be set!");

		m_filter->SetInput(m_referenceImage);		
		m_filter->SetTransform(m_transform);
		m_filter->SetInterpolator(m_interpolator);
		m_filter->SetDefaultPixelValue( 0 );

		SensedImageType::Pointer sensedImage = m_sensor->GetOutput();

		m_filter->SetOutputSpacing( sensedImage->GetSpacing() );
		m_filter->SetOutputOrigin( sensedImage->GetOrigin() );
		m_filter->SetSize( sensedImage->GetLargestPossibleRegion().GetSize() );

		try { m_filter->Update(); }
		catch( itk::ExceptionObject &e) { std::cerr << "SensedSceneToImageCostFunction: Exception raised when resampling the reference image... : "<< std::endl; std::cerr << e << std::endl; }

		m_interpolatedReferenceImage = m_filter->GetOutput();

		// ADD THE CODE TO compute m_partialIntegralImage and m_partialIntegralSquaredImage 
		m_integralFilter->SetInput(m_interpolatedReferenceImage);
		m_integralFilter->Update();
		m_partialIntegralImage = m_integralFilter->GetOutput1();
		m_partialIntegralSquaredImage = m_integralFilter->GetOutput2();
		//lbip::LinearIntegrationImageAndSquaredImageFilter<ReferenceImageType, IntegralImageType>(m_interpolatedReferenceImage, m_partialIntegralImage, m_partialIntegralSquaredImage);

		m_refImageInterpolatedFlag = true;
	}

private:
	SensedSingleObjectInOutMMahalaDist(const Self&);	//purposely not implemented
	const Self & operator=( const Self & );		//purposely not implemented
};


} // namespace psciob

#endif /* __SENSEDSINGLEOBJECTINOUTMMAHALADIST_H_ */
