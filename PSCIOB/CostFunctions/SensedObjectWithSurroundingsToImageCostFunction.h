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
* \file SensedObjectWithSurroundingsToImageCostFunction.h
* \author Rémi Blanc 
* \date 18. January 2011
*/

#ifndef SENSEDOBJECTWITHSURROUNDINGSTOIMAGECOSTFUNCTION_H_
#define SENSEDOBJECTWITHSURROUNDINGSTOIMAGECOSTFUNCTION_H_

#include <SensedSingleObjectToImageCostFunction.h>

namespace psciob {


/**
* \class SensedObjectWithSurroundingsToImageCostFunction
* \brief SensedObjectWithSurroundingsToImageCostFunction
* base class for metrics considering an object and its surrounding area
* comparing it with a reference image to compute a goodness of fit
* 
* by default, the sensor is reset to consider the surrounding radius to be at least 2.
*/

template<class TScene, class TReferenceImage, class TSensedImage>
class SensedObjectWithSurroundingsToImageCostFunction : public SensedSingleObjectToImageCostFunction<TScene, TReferenceImage, TSensedImage> {
public:
	/** Standard class typedefs. */
	typedef SensedObjectWithSurroundingsToImageCostFunction Self;
	typedef SensedSingleObjectToImageCostFunction           Superclass;
	typedef itk::SmartPointer<Self>                         Pointer;
	/** Run-time type information (and related methods). */
	itkTypeMacro(SensedObjectWithSurroundingsToImageCostFunction, SensedSingleObjectToImageCostFunction);


	/** Create a clone (= an exact, independant copy) of the current cost function */
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

		return static_cast<BaseClass*>( clonePtr );
	}

	/** overload the method to inform the sensor about the requested radius (dilating the region of interest to get the object context) 
	* the sensor is automatically configured to consider a context-radius of at least 2 pixels
	* \todo it has NOT been IMPLEMENTED YET, BUT THIS WOULD BE USEFUL FOR INVALIDATING IN-CONTEXT COSTS (to detect when an object is being modified near other objects, so that these objects are signalled as out-of-date...
	*/
	void SetSensor(SensorType *sensor) { 
		m_sensor = sensor; 
		this->SetScene( m_sensor->GetScene() );
		m_sensor->SetObjectContextRadius( m_surroundingRadius );
	}

	/** defines the radius defining the context of the object : area obtained by dilation of the object */
	void SetSurroundingRadius(unsigned int radius) { 
		m_surroundingRadius = radius;
		m_sensor->SetObjectContextRadius( m_surroundingRadius );
	}	

protected:
	SensedObjectWithSurroundingsToImageCostFunction() : SensedSingleObjectToImageCostFunction() { m_surroundingRadius=2; };
	~SensedObjectWithSurroundingsToImageCostFunction() {};	

	int m_surroundingRadius;

private:
	SensedObjectWithSurroundingsToImageCostFunction(const Self&);	//purposely not implemented
	const Self & operator=( const Self & );		//purposely not implemented
};

} // namespace psciob

#endif /* SENSEDOBJECTWITHSURROUNDINGSTOIMAGECOSTFUNCTION_H_ */
