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
 * \file PowellOptimizer.h
 * \author Rémi Blanc 
 * \date 22. September 2011
 */


#ifndef POWELLOPTIMIZER_H_
#define POWELLOPTIMIZER_H_


#include "Optimizer_Base.h"
#include "OptimizerCostFunction.h"
#include <vnl/algo/vnl_powell.h>

namespace psciob {

/**\class PowellOptimizer
 * \brief wrapper to the powell optimization class: vnl_powell
 * \sa ScaledPowellOptimizer
*/


class PowellOptimizer : public Optimizer_Base {
public:
	/** Standard class typedefs. */
	typedef PowellOptimizer					Self;
	typedef Optimizer_Base					Superclass;
	typedef itk::SmartPointer<Self>			Pointer;
	/** Run-time type information (and related methods). */
	itkTypeMacro(PowellOptimizer,Optimizer_Base);
	itkNewMacro(PowellOptimizer);

	/** initial step size for exploring the parameter space 
	 *  for more control on heterogeneous parameters ranges, see ScaledPowellOptimizer
	 */
	void SetInitialStepSize(double s = 0.1)						{m_initialStep=s;}
	/** Stop criterion based on the metric: if the evolution is less than this threshold, convergence is assumed */
	void SetMetricTolerance(double t = 1e-5)					{m_f_tol=t;}
	/** Maximum number of iterations before forcing the exit */
	void SetMaximumNumberOfIterations(unsigned int n = 2000)	{m_maxNbIterations=n;}
	/** Stop criterion: if the parameter do not significantly evolve, convergence is assumed */
	void SetParameterTolerance(double t = 1e-5)					{m_x_tol=t;}

	/** Apply Powell optimization - getting and setting parameters, and evaluating the cost function according to the provided manager */
	double Optimize(OptimizationManager_Base *manager) {
		//std::cout<<"Start optimizing with powell..."<<std::endl;
		m_nbParameters=manager->GetNumberOfParameters();
		//IDEA: is it possible to store the object as a member of the class ; instead of creating a new one at each call
		OptimizerCostFunction fct(m_nbParameters); fct.SetCaller(manager); vnl_powell powell(&fct);	
		
		powell.set_initial_step(m_initialStep);
		powell.set_f_tolerance(m_f_tol);
		powell.set_max_function_evals(m_maxNbIterations);
		powell.set_x_tolerance(m_x_tol);

		vnl_vector<double> x(m_nbParameters); x = manager->GetParameters();
		//std::cout<<"initial parameters: "<<x<<", initial cost: "<<manager->GetValue()<<std::endl;
		powell.minimize(x);
		//std::cout<<"nb of evaluations in powell minimization: "<<powell.get_num_evaluations()<<std::endl;
		manager->SetParameters(x);
		return manager->GetValue();
	}

protected:
	PowellOptimizer() {
		m_maxNbIterations = 2000;
		m_f_tol = 1e-5;
		m_initialStep = 0.1;
		m_x_tol = 1e-5;	//tolerance on the variations of the parameters
	};
	~PowellOptimizer() {};

	double m_initialStep, m_f_tol, m_x_tol;
	unsigned int m_maxNbIterations;

private:
	PowellOptimizer(const Self&);				//purposely not implemented
	const Self & operator=( const Self & );		//purposely not implemented
};




} // namespace psciob

#endif /* POWELLOPTIMIZER_H_ */
