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
 * \file CostFunction_Base.h
 * \author Rémi Blanc 
 * \date 26. October 2011
*/

#ifndef COSTFUNCTION_BASE_H_
#define COSTFUNCTION_BASE_H_




#include "itkLightObject.h"
#include "GenericParametricFunctions.h"

namespace psciob {

/** 
 * \class CostFunction_Base
 * \brief Base class for cost functions ; define an abstract GetRawValue()
 * It is possible to define a normalization function that transforms the raw output.
 * The method GetValue() returns either the raw value or the normalized value as requested
 * The normalization function is a parametric function of type GenericParametricFunctions<double, double>
 *
 * ADD? what about the gradient of the metric ?
*/

class CostFunction_Base : public itk::LightObject {
public:
	/** Standard class typedefs. */
	typedef CostFunction_Base       Self;
	typedef itk::LightObject        Superclass;
	typedef itk::SmartPointer<Self> Pointer;
	/** Base Types to which all inherited classes can refer */
	typedef Self                    BaseClass;
	typedef Pointer                 BaseClassPointer;
	/** Run-time type information (and related methods). */
	itkTypeMacro(SceneCostFunction_Base,itk::LightObject);

	/** Create a clone (= an exact, independant copy) of the current cost function */
	virtual BaseClassPointer CreateClone() {
		BaseClassPointer clonePtr = static_cast<Self*>(this->CreateAnother().GetPointer());
		if (!m_transformFunction) {} else {clonePtr->SetNormalizationFunction(m_transformFunction);}
		return clonePtr;
	}

	/** the normalization function type ... */
	typedef GenericParametricFunctions<double, double> NormalizationFunctionType;
	
	/** Evaluate the goal function */
	inline double GetValue() {
		if (m_useTransform) return m_transformFunction->Evaluate( GetRawValue() );
		else                return GetRawValue();
	}

	/** Get the raw cost function */
	virtual double GetRawValue() = 0;

	/** Set the normalization function */
	void SetNormalizationFunction(NormalizationFunctionType *f) { m_transformFunction = f; m_useTransform = true; }

	/** Set the parameters of the normalization function */
	bool SetNormalizationFunctionParameters(vnl_vector<double> p) { 
		if (!m_transformFunction) throw DeformableModelException("CostFunction_Base::SetTransformFunctionParameters - no function has been defined");
		return m_transformFunction->SetParameters(p); 
	}

	/** select the output: raw or normalized value - no effect if the normalization function is undefined */
	void UseNormalizedOutput(bool b) {
		m_useTransform = b;
		if (!m_transformFunction) m_useTransform = false;
	}

protected:
	CostFunction_Base() {
		m_useTransform = false;
		m_transformFunction = NULL;
	};
	virtual ~CostFunction_Base() {};	

	bool m_useTransform;
	NormalizationFunctionType::Pointer m_transformFunction;

private:
	CostFunction_Base(const Self&);				//purposely not implemented
	const Self & operator=( const Self & );		//purposely not implemented
};

} // namespace psciob

#endif /* COSTFUNCTION_BASE_H_ */
