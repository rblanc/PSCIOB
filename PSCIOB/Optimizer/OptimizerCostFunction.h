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
* \file OptimizerCostFunction.h
* \author Rémi Blanc 
* \date 22. September 2011
*/


#ifndef OPTIMIZERCOSTFUNCTION_H_
#define OPTIMIZERCOSTFUNCTION_H_


#include <vnl/vnl_cost_function.h>
#include "OptimizationManager_Base.h"

namespace psciob {

/**\class OptimizerCostFunction 
 * \brief OptimizerCostFunction
 * wrapping vnl_cost_function, for use with optimizers taken from the vnl library
*/

/** */
class OptimizerCostFunction: public vnl_cost_function {
public:
	OptimizerCostFunction(int n) : vnl_cost_function(n)	{ m_outputForbiddenParameters = 1e50; }

	/** */
	void SetCaller(OptimizationManager_Base *c) {
		m_caller=c;	this->set_number_of_unknowns(m_caller->GetNumberOfParameters());
	}

	/** */
	inline double GetValue(const vnl_vector<double>& x) {return f(x);}

	/** */
	inline double f(const vnl_vector<double>& x) {
		if (m_caller->SetParameters(x))	{ return m_caller->GetValue(); }
		else							{ return m_outputForbiddenParameters; }	//return a very large energy value if the parameters are invalid
	}

	/** */
	void SetCostForForbiddenParameters(double v)	{ m_outputForbiddenParameters = v; }

	/** */
	double GetCostForForbiddenParameters(double v)	{ return m_outputForbiddenParameters; }

protected:
	OptimizationManager_Base::Pointer m_caller;
	double m_outputForbiddenParameters;
};


/** Same but using scaling of the values, to handle parameters with different dynamics / sensitivity */
class OptimizerScaledCostFunction: public OptimizerCostFunction {
public:
	OptimizerScaledCostFunction(int n) : OptimizerCostFunction(n) { m_scales.set_size(n); m_scales.fill(1); }

	/** */
	void SetCaller(OptimizationManager_Base *c) {
		m_caller=c;	this->set_number_of_unknowns(m_caller->GetNumberOfParameters());
		m_scales.set_size(m_caller->GetNumberOfParameters()); m_scales.fill(1); 
	}

	/** */
	inline double f(const vnl_vector<double>& x) {
		if (m_caller->SetParameters( element_product(x, m_scales) ))	{ return m_caller->GetValue(); }
		else	{ return m_outputForbiddenParameters; }	//return a very large energy value if the parameters are invalid
	}

	/** */
	void SetScales(vnl_vector<double> sc)	{ 
		if (sc.size() != m_scales.size()) throw DeformableModelException("OptimizerScaledCostFunction::SetScales : unexpected vector dimension");
		m_scales = sc;
	}

protected:
	vnl_vector<double> m_scales;
};



} // namespace psciob

#endif /* OPTIMIZERCOSTFUNCTION_H_ */
