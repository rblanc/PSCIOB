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
 * \file SensedSceneToImageCostFunction.h
 * \author Rémi Blanc 
 * \date 9. September 2011
*/

#ifndef SENSEDSCENETOIMAGECOSTFUNCTION_H_
#define SENSEDSCENETOIMAGECOSTFUNCTION_H_

//#include "CommonTypes.h"
#include "GeneralUtils.h"
#include "ITKUtils.h"
#include "ScalarFunctions.h"
#include "ImageSensor_Base.h"
#include <itkImageToImageMetric.h>
#include <itkIdentityTransform.h>	
#include <itkNearestNeighborInterpolateImageFunction.h>

#include <itkInterpolateImageFunction.h>

#include "SceneToImageCostFunction.h"

namespace psciob {


/** 
 * \class SensedSceneToImageCostFunction
 * \brief base class for computing a (dis)similarity value (low = similar) between a reference image and a 'sensed scene' = synthetic image produced from the scene
 *
 * The reference image is interpolated to match the grid of the Sensed Image
 * ADD: what about the gradient of the metric ?
 *
 * TODO: check how to work with the potentially different pixel types between the 'reference' and the 'moving' image
 * a priori, I expect the synthetic image and fixed image to have the same resolution...
 *
 */



template<class TScene, class TReferenceImage, class TSensedImage>
class SensedSceneToImageCostFunction : public SceneToImageCostFunction<TScene, TReferenceImage> {
public:
	//template<class TScene, class TSensedImage, class TReferenceImage> friend class SceneImageOptimizationManager_Base;

	/** Standard class typedefs. */
	typedef SensedSceneToImageCostFunction	Self;
	typedef SceneToImageCostFunction		Superclass;
	typedef itk::SmartPointer<Self>			Pointer;
	/** Run-time type information (and related methods). */
	itkTypeMacro(SensedSceneToImageCostFunction,SceneToImageCostFunction);

	typedef TSensedImage			SensedImageType;
	typedef ImageSensor_Base<SceneType, TSensedImage>	SensorType;
	typedef typename SensorType::OutputLabelImageType	SensedLabelImageType;
	typedef typename SensorType::OutputLabelMapType     SensedLabelMapType;

	typedef itk::Image<unsigned char, SensorType::OutputDimension> BinaryImageType;

	/** Create a clone (= an exact, independant copy) of the current cost function 
	* transfers the pointer to the reference image.
	* \warning: does not clone the interpolator ; keeps the default nearest neighbor interpolator
	*/
	virtual BaseClassPointer CreateClone() {
		Pointer clonePtr = static_cast<Self*>(this->CreateAnother().GetPointer());
		if (!m_transformFunction) {} else { clonePtr->SetNormalizationFunction(m_transformFunction);}
		if (!m_scene) {} else {clonePtr->SetScene(m_scene);}
		clonePtr->SelectObject(m_requestedLabel);

		if (!m_referenceImage) {} else { clonePtr->SetReferenceImage(m_referenceImage); }
		if (!m_sensor)         {} else { clonePtr->SetSensor( m_sensor->CreateClone() );}

		return static_cast<BaseClass*>( clonePtr );
	}

	/** Set the scene on which to make the measurements 
	* and propagates to any other members that may also be working on this scene
	* maybe overloaded by child classes
	*/
	virtual void SetScene(SceneType *scene) {
		Superclass::SetScene(scene);

		//make sure the other functions work on the same scene.
		if (!m_sensor) {} else {m_sensor->SetScene(scene);}
		m_refImageInterpolatedFlag = false;
	}

	/** Set the sensor to be used for comparison with the reference image ; automatically (re)sets the associated scene */
	void SetSensor(SensorType *sensor) {
		m_sensor=sensor; 
		this->SetScene( m_sensor->GetScene() );
		m_refImageInterpolatedFlag = false;
	}

	/** recover the pointer to the sensor */
	SensorType* GetSensor() {
		if (!m_sensor) throw DeformableModelException("SensedSceneToImageCostFunction::InterpolateReferenceImage : the sensor must be set!");
		return m_sensor.GetPointer(); 
	}

protected:
	SensedSceneToImageCostFunction() : SceneToImageCostFunction() {
		m_sensor = 0;
	}
	virtual ~SensedSceneToImageCostFunction() {};	

	typename SensorType::Pointer		 m_sensor;

	/** DEVELOPER: do not forget to set m_interpolatedReferenceImage 
	* kept virtual at this point, because the interpolation may be different depending whether a sensor is used, or directly the scene... ??? not sure this is really relevant...
	* \warning: if the sensor is modified (e.g. its resolution is modified), the metric is not informed automatically...
	* may be overloaded by child classes
	*/
	virtual void InterpolateReferenceImage() {
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

		m_refImageInterpolatedFlag = true;
	}

private:
	SensedSceneToImageCostFunction(const Self&);	//purposely not implemented
	const Self & operator=( const Self & );		//purposely not implemented
};

} // namespace psciob

#endif /* SENSEDSCENETOIMAGECOSTFUNCTION_H_ */