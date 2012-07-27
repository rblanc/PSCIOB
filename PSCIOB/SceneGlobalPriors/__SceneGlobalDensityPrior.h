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
 * \file SceneGlobalDensityPrior.h
 * \author Rémi Blanc 
 * \date 25. October 2011
*/

#ifndef SCENEGLOBADENSITYPRIOR_H_
#define SCENEGLOBADENSITYPRIOR_H_

#include "SceneGlobalPrior_Base.h"
#include "UnivariatePDF.h"

namespace psciob {

/**\class SceneGlobalDensityPrior 
 * \brief SceneGlobalDensityPrior
 * in terms of number of objects... just the likelihood of the current number of objects with respect to a Poisson distribution with the requested density
*/


//CONCRETE CLASS
template<class TScene>
class SceneGlobalDensityPrior : public SceneGlobalPrior_Base<TScene> {
public:
	/** Standard class typedefs. */
	typedef SceneGlobalDensityPrior			Self;
	typedef SceneGlobalPrior_Base<TScene>	Superclass;
	typedef itk::SmartPointer<Self>			Pointer;
	typedef itk::SmartPointer<const Self>	ConstPointer;
	/** Run-time type information (and related methods). */
	itkTypeMacro(SceneGlobalDensityPrior,SceneGlobalPrior_Base);
	itkNewMacro(Self);

	void SetTargetDensity(double d) {
		m_globalPriorFlag=false;
		vnl_vector<double> bbox = m_scene->GetSceneBoundingBox();
		double vol=1; for (unsigned i=0 ; i<bbox.size()/2 ; i++) vol*= ( bbox(2*i+1) - bbox(2*i) );
		m_targetNbObjects = d * vol;	m_poissonPDF->SetLambda(m_targetNbObjects);
	}

	void SetTargetNumberOfObjects(unsigned int n)	{ m_globalPriorFlag=false; m_targetNbObjects = n; m_poissonPDF->SetLambda(m_targetNbObjects);	}

	/** this prior is costless to compute, just invalidate the current value, and recompute the value when requested
	* This function should only be called by the scene - use friendship instead of leaving the method public 
	*/
	void Update_AddingObject(ObjectType *object) { m_globalPriorFlag=false; }

	/** this prior is costless to compute, just invalidate the current value, and recompute the value when requested
	* This function should only be called by the scene - use friendship instead of leaving the method public 
	*/
	void Update_ModifyingObject(ObjectType *initialObject, ObjectType *newObject) { m_globalPriorFlag=false; }

	/** this prior is costless to compute, just invalidate the current value, and recompute the value when requested
	* This function should only be called by the scene - use friendship instead of leaving the method public 
	*/
	void Update_RemovingObject(ObjectType *object) { m_globalPriorFlag=false; }

protected:	
	SceneGlobalDensityPrior() : SceneGlobalPrior_Base() {
		m_targetNbObjects=10;
		m_poissonPDF = PoissonPDF::New();
		m_poissonPDF->SetLambda(m_targetNbObjects);
	}

	virtual ~SceneGlobalDensityPrior() {};	

	double m_targetNbObjects;
	PoissonPDF::Pointer m_poissonPDF;

	double ComputeGlobalPrior_Internal() { return - m_poissonPDF->GetLogLikelihood(m_scene->GetNumberOfObjects()); }

private:
	SceneGlobalDensityPrior(const Self&);		//purposely not implemented
	const Self & operator=( const Self & );		//purposely not implemented
};



} // namespace psciob

#endif /* SCENEGLOBADENSITYPRIOR_H_ */
