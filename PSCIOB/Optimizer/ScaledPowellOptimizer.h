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
 * \file ScaledPowellOptimizer.h
 * \author Rémi Blanc 
 * \date 20. December 2011
 */


#ifndef SCALEDPOWELLOPTIMIZER_H_
#define SCALEDPOWELLOPTIMIZER_H_

#include "Optimizer_Base.h"
#include "OptimizerCostFunction.h"
#include <vnl/algo/vnl_powell.h>

namespace psciob {

/**\class ScaledPowellOptimizer
 * \brief wrapper to the powell optimization class: vnl_powell
 * adding a layer to rescale the variables, if they have very different dynamics
 * the parameters actually optimizer are element_quotient( true_params, scales)
 * -> a large scale value implies that the dynamic will be larger for this parameter
*/


//TODO: implement all handles on the internal parameters of the optimizer

class ScaledPowellOptimizer : public Optimizer_Base {
public:
	/** Standard class typedefs. */
	typedef ScaledPowellOptimizer			Self;
	typedef Optimizer_Base					Superclass;
	typedef itk::SmartPointer<Self>			Pointer;
	/** Run-time type information (and related methods). */
	itkTypeMacro(ScaledPowellOptimizer,Optimizer_Base);
	itkNewMacro(ScaledPowellOptimizer);

	void SetInitialStepSize(double s = 0.1)						{m_initialStep=s;}
	void SetMetricTolerance(double t = 1e-5)					{m_f_tol=t;}
	void SetMaximumNumberOfIterations(unsigned int n = 2000)	{m_maxNbIterations=n;}

	void SetScales(vnl_vector<double> sc) { m_scales = sc; }

	double Optimize(OptimizationManager_Base *manager) {
		m_nbParameters=manager->GetNumberOfParameters();
		if (m_scales.size() != m_nbParameters) throw DeformableModelException("ScaledPowellOptimizer::Optimize : scale vector doesn't have the expected dimension");
		//IDEA: is it possible to store the object as a member of the class ; instead of creating a new one at each call
		OptimizerScaledCostFunction fct(m_nbParameters); fct.SetCaller(manager); vnl_powell powell(&fct);	
		fct.SetScales(m_scales);
		
		powell.set_initial_step(m_initialStep);
		powell.set_f_tolerance(m_f_tol);
		powell.set_max_function_evals(m_maxNbIterations);

		vnl_vector<double> x(m_nbParameters); x = element_quotient( manager->GetParameters(), m_scales );

		powell.minimize(x);
		//std::cout<<"nb of evaluations in powell minimization: "<<powell.get_num_evaluations()<<std::endl;
		manager->SetParameters( element_product(x, m_scales) );
		return manager->GetValue();
	}

protected:
	ScaledPowellOptimizer() {
		m_maxNbIterations = 2000;
		m_f_tol = 1e-5;
		m_initialStep = 0.1;
	};
	~ScaledPowellOptimizer() {};

	double m_initialStep, m_f_tol;
	unsigned int m_maxNbIterations;
	vnl_vector<double> m_scales;

private:
	ScaledPowellOptimizer(const Self&);				//purposely not implemented
	const Self & operator=( const Self & );		//purposely not implemented
};




} // namespace psciob

#endif /* SCALEDPOWELLOPTIMIZER_H_ */
