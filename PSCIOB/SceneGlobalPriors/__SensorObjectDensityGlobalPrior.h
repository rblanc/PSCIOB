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
 * \file SensorObjectDensityGlobalPrior.h
 * \author Rémi Blanc 
 * \date 25. October 2011
*/

#ifndef SENSOROBJECTDENSITYGLOBALPRIOR_H_
#define SENSOROBJECTDENSITYGLOBALPRIOR_H_

#include "SceneGlobalPrior_Base.h"
#include "UnivariatePDF.h"

namespace psciob {

/**\class SensorObjectDensityGlobalPrior
 * \brief SensorObjectDensityGlobalPrior
 * associate a likelihood to the percentage of voxels of the sensor that are occupied by objects
 * -> non zero value on the corresponding label image
*/


//CONCRETE CLASS
template<class TScene, class TSensor>
class SensorObjectDensityGlobalPrior : public SceneGlobalPrior_Base<TScene> {
public:
	/** Standard class typedefs. */
	typedef SensorObjectDensityGlobalPrior	Self;
	typedef SceneGlobalPrior_Base<TScene>	Superclass;
	typedef itk::SmartPointer<Self>			Pointer;
	typedef itk::SmartPointer<const Self>	ConstPointer;
	/** Run-time type information (and related methods). */
	itkTypeMacro(SensorObjectDensityGlobalPrior,SceneGlobalPrior_Base);
	itkNewMacro(Self);

	typedef TSensor										SensorType;
	typedef typename SensorType::OutputLabelImageType	ImageType;

	/** Set the sensor */
	void SetSensor(TSensor *sensor) { m_sensor = sensor; m_globalPriorFlag=false; }

	/** Set the target density: mean and stdev of a gaussian distribution */
	void SetTargetDensity(double mean, double std) { 
		m_globalPriorFlag=false;
		m_normalPDF->SetParameters(mean, std*std);
	}

	/** this prior is difficult to update... just invalidate the value and recompute it when requested
	*-- to update it, it would be necessary to compute how many pixels are observed on the sensor ; and whether objects behind (partially) disappear
	* This function should only be called by the scene - use friendship instead of leaving the method public */
	inline void Update_AddingObject(ObjectType *object) { m_globalPriorFlag=false; }

	/** this prior is difficult to update... just invalidate the value and recompute it when requested
	*-- to update it, it would be necessary to see how many pixels are removed / added ; and how the visibility of other objects is affected...
	* This function should only be called by the scene - use friendship instead of leaving the method public */
	inline void Update_ModifyingObject(ObjectType *initialObject, ObjectType *newObject) { m_globalPriorFlag=false; }

	/** this prior is difficult to update... just invalidate the value and recompute it when requested
	*-- to update it, it would be necessary to compute how many pixels are removed from the sensor ; and whether objects behind may become apparent
	* This function should only be called by the scene - use friendship instead of leaving the method public 
	*/
	inline void Update_RemovingObject(ObjectType *object) { m_globalPriorFlag=false; }


protected:	
	SensorObjectDensityGlobalPrior() : SceneGlobalPrior_Base() {
		m_normalPDF = NormalPDF::New();
		m_normalPDF->SetParameters(0.5, 0.01*0.01);
	}

	virtual ~SensorObjectDensityGlobalPrior() {};	

	NormalPDF::Pointer m_normalPDF;
	typename SensorType::Pointer m_sensor;

	typedef itk::ImageRegionConstIterator<ImageType> IteratorType;
	double ComputeGlobalPrior_Internal() {
		//first, compute the density
		ImageType::Pointer img = m_sensor->GetOutputImageLabelMap();
		IteratorType it(img, img->GetLargestPossibleRegion());
		unsigned int n_on=0, n_total = img->GetLargestPossibleRegion().GetNumberOfPixels();
		for ( it.GoToBegin() ; !it.IsAtEnd() ; ++it ) { if (it.Get()!=0) n_on++; }
		double density = static_cast<double>(n_on)/static_cast<double>(n_total);
		return - m_normalPDF->GetLogLikelihood(density); 
	}


private:
	SensorObjectDensityGlobalPrior(const Self&);	//purposely not implemented
	const Self & operator=( const Self & );			//purposely not implemented
};



} // namespace psciob

#endif /* SENSOROBJECTDENSITYGLOBALPRIOR_H_ */
