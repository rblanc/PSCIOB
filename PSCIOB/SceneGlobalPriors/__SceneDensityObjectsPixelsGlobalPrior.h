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
 * \file SceneDensityObjectsPixelsGlobalPrior.h
 * \author Rémi Blanc 
 * \date 30. January 2012
*/

#ifndef SCENEDENSITYOBJECTSPIXELSGLOBALPRIOR_H_
#define SCENEDENSITYOBJECTSPIXELSGLOBALPRIOR_H_

#include "SceneGlobalPrior_Base.h"
#include "UnivariatePDF.h"

namespace psciob {

/**\class SceneDensityObjectsPixelsGlobalPrior 
 * \brief SceneDensityObjectsPixelsGlobalPrior
 * Counts how many pixels each object would potentially contribute (ignore overlaps) and compute the corresponding density ( which thus, can be >1 )
 * Assign a weight proportional to a normal distribution with given mean and standard deviation, with respect to such density.
 * default target density: 0.5 +/- 0.01
*/


//CONCRETE CLASS
template<class TScene>
class SceneDensityObjectsPixelsGlobalPrior : public SceneGlobalPrior_Base<TScene> {
public:
	/** Standard class typedefs. */
	typedef SceneDensityObjectsPixelsGlobalPrior			Self;
	typedef SceneGlobalPrior_Base<TScene>	Superclass;
	typedef itk::SmartPointer<Self>			Pointer;
	typedef itk::SmartPointer<const Self>	ConstPointer;
	/** Run-time type information (and related methods). */
	itkTypeMacro(SceneDensityObjectsPixelsGlobalPrior, SceneGlobalPrior_Base);
	itkNewMacro(Self);

	/** Set the reference scene	*/
	void SetScene(SceneType* scene) {
		m_scene = scene;
		m_nbScenePixels = m_scene->GetSceneImageRegion().GetNumberOfPixels();
		m_globalPriorFlag=false;
	}


	void SetTargetDensity(double mean, double std) { 
		m_globalPriorFlag=false;
		m_normalPDF->SetParameters(mean, std*std);
	}

	/** Updates the cost */
	inline void Update_AddingObject(ObjectType *object) { 
		//m_globalPriorFlag = false;
		if (m_globalPriorFlag) { 
			m_sumObjectPixels += object->GetObjectAsLabelMap()->GetNumberOfPixels();
			m_globalPrior = - m_normalPDF->GetLogLikelihood(static_cast<double>(m_sumObjectPixels) / static_cast<double>(m_nbScenePixels));
		}
	}

	/** Updates the cost automatically */
	inline void Update_ModifyingObject(ObjectType *initialObject, ObjectType *newObject) { 
		//m_globalPriorFlag = false;
		if (m_globalPriorFlag) {
			m_sumObjectPixels = m_sumObjectPixels + newObject->GetObjectAsLabelMap()->GetNumberOfPixels() - initialObject->GetObjectAsLabelMap()->GetNumberOfPixels();
			m_globalPrior = - m_normalPDF->GetLogLikelihood(static_cast<double>(m_sumObjectPixels) / static_cast<double>(m_nbScenePixels));
		}
	}

	/** Updates the cost automatically */
	inline void Update_RemovingObject(ObjectType *object) { 
		//m_globalPriorFlag = false;
		if (m_globalPriorFlag) {
			m_sumObjectPixels -= object->GetObjectAsLabelMap()->GetNumberOfPixels();
			m_globalPrior = - m_normalPDF->GetLogLikelihood(static_cast<double>(m_sumObjectPixels) / static_cast<double>(m_nbScenePixels));
		}
	}

protected:	
	SceneDensityObjectsPixelsGlobalPrior() : SceneGlobalPrior_Base() {
		m_sumObjectPixels = 0;
		m_normalPDF = NormalPDF::New();
		m_normalPDF->SetParameters(0.5, 0.01*0.01);
	}

	virtual ~SceneDensityObjectsPixelsGlobalPrior() {};	

	unsigned long m_nbScenePixels, m_sumObjectPixels;
	NormalPDF::Pointer m_normalPDF;

	inline double ComputeGlobalPrior_Internal() { 
		m_sumObjectPixels = 0;
		for (unsigned i=0 ; i<m_scene->GetNumberOfObjects() ; i++) {
			m_sumObjectPixels += m_scene->GetObjectByIndex(i)->nbPixels;
		}
		return - m_normalPDF->GetLogLikelihood(static_cast<double>(m_sumObjectPixels) / static_cast<double>(m_nbScenePixels));  
	}

private:
	SceneDensityObjectsPixelsGlobalPrior(const Self&);		//purposely not implemented
	const Self & operator=( const Self & );		//purposely not implemented
};



} // namespace psciob

#endif /* SCENEDENSITYOBJECTSPIXELSGLOBALPRIOR_H_ */
