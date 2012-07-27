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
 * \file ScalarFunctions.h
 * \author Rémi Blanc 
 * \date 29. August 2011
*/

#ifndef SCALARFUNCTIONS_H_
#define SCALARFUNCTIONS_H_

#include <math.h> 

namespace psciob {

/** \class ScalarFunction_Base
* \brief ScalarFunction_Base base class for univariate functions
*/
class ScalarFunction_Base : public itk::LightObject {
public:
	typedef ScalarFunction_Base		Self;
	typedef itk::LightObject		Superclass;
	typedef itk::SmartPointer<Self>	Pointer;
	/** Run-time type information (and related methods). */
	itkTypeMacro(ScalarFunction_Base, itk::LightObject);
	
	//TODO: add access to the parameters controlling the function's behavior
	virtual double Evaluate(double input) = 0;
};


/** \class ScalarFunction_Geometric
* \brief ScalarFunction_Geometric geometric function
* f(x) = a*x ; where a is an input factor ; by default a = 1
*/
class ScalarFunction_Geometric : public ScalarFunction_Base {
public:
	typedef ScalarFunction_Geometric		Self;
	typedef ScalarFunction_Base				Superclass;
	typedef itk::SmartPointer<Self>			Pointer;
	/** Run-time type information (and related methods). */
	itkTypeMacro(ScalarFunction_Geometric, ScalarFunction_Base);
	itkNewMacro(Self);

	void SetFactor(double s=1) {m_factor = s;}
	inline double Evaluate(double input) { return m_factor * input; }
protected:
	ScalarFunction_Geometric()		   {m_factor = 1.0;}
	~ScalarFunction_Geometric() {};

	double m_factor;
};


/** \class ScalarFunction_Exp
* \brief ScalarFunction_Exp exponential function
* f(x) = exp(x) 
*/
class ScalarFunction_Exp : public ScalarFunction_Base {
public:
	typedef ScalarFunction_Exp		Self;
	typedef ScalarFunction_Base		Superclass;
	typedef itk::SmartPointer<Self>	Pointer;
	/** Run-time type information (and related methods). */
	itkTypeMacro(ScalarFunction_Exp, ScalarFunction_Base);
	itkNewMacro(Self);

	inline double Evaluate(double input) { return exp(input); }
protected:
	ScalarFunction_Exp() {}
	~ScalarFunction_Exp() {};
};


/** \class ScalarFunction_ScaledExpDecr
* \brief ScalarFunction_ScaledExpDecr exponential function
* f(x) = a*exp(-x) ; by default, a=1;
*/
class ScalarFunction_ScaledExpDecr : public ScalarFunction_Base {
public:
	typedef ScalarFunction_ScaledExpDecr	Self;
	typedef ScalarFunction_Base				Superclass;
	typedef itk::SmartPointer<Self>			Pointer;
	/** Run-time type information (and related methods). */
	itkTypeMacro(ScalarFunction_Exp, ScalarFunction_Base);
	itkNewMacro(Self);
	
	void SetScale(double s) {m_scale = s;}
	inline double Evaluate(double input) { return m_scale * exp( - input ); }
protected:
	ScalarFunction_ScaledExpDecr()		   {m_scale = 1.0;}
	~ScalarFunction_ScaledExpDecr() {};

	double m_scale;
};

} // namespace psciob


#endif //SCALARFUNCTIONS_H_