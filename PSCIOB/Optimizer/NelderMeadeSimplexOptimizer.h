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
 * \file NelderMeadeSimplexOptimizer.h
 * \author Rémi Blanc 
 * \date 04. November 2011
*/


#ifndef NELDERMEADESIMPLEXOPTIMIZER_H_
#define NELDERMEADESIMPLEXOPTIMIZER_H_

#include "Optimizer_Base.h"
#include "OptimizerCostFunction.h"
#include <vnl/algo/vnl_amoeba.h>

//TODO: implement all handles on the internal parameters of the optimizer

namespace psciob {

/** \class NelderMeadeSimplexOptimizer
 * \brief wrapper to the vnl Nelder-Meade Simplex optimizer : vnl_amoeba 
 */


class NelderMeadeSimplexOptimizer : public Optimizer_Base {
public:
	/** Standard class typedefs. */
	typedef NelderMeadeSimplexOptimizer		Self;
	typedef Optimizer_Base					Superclass;
	typedef itk::SmartPointer<Self>			Pointer;
	/** Run-time type information (and related methods). */
	itkTypeMacro(NelderMeadeSimplexOptimizer,Optimizer_Base);
	itkNewMacro(Self);

	//in practice, it is multiplied by the number of unknowns
	void SetMaximumNumberOfIterations(unsigned int n = 200) {m_maxNbIterations=n;}
	void SetMetricTolerance(double t = 1e-5)				{m_f_tol=t;}
	//isotropic factor deciding the initial size of the simplex, its vertices are obtained by adding this value to the corresponding coordinate of the initial point
	void SetInitialSimplexSize(double s = 0.1)				{m_isoSimplexSize=s;m_useSimplexSize=false;}
	//factor deciding the initial size of the simplex, its vertices are obtained by adding the corresponding value to the corresponding coordinate of the initial point
	void SetInitialSimplexSize(vnl_vector<double> s)		{m_SimplexSize=s;m_useSimplexSize=true;}

	double Optimize(OptimizationManager_Base *manager) {
		m_nbParameters=manager->GetNumberOfParameters();
		//IDEA: is it possible to store the object as a member of the class ; instead of creating a new one at each call
		OptimizerCostFunction fct(m_nbParameters); fct.SetCaller(manager); 
		vnl_amoeba optimizer(fct);	
		
		optimizer.set_f_tolerance(m_f_tol);
		optimizer.set_max_iterations(m_maxNbIterations * m_nbParameters);

		vnl_vector<double> x(m_nbParameters); x = manager->GetParameters();

		//use the provided simplex size ; unless it has wrong dimensionality...
		if ( (!m_useSimplexSize) || (m_SimplexSize.size() != m_nbParameters) ) {
			m_SimplexSize.set_size(m_nbParameters);
			m_SimplexSize.fill(m_isoSimplexSize);
		}

		optimizer.minimize(x, m_SimplexSize);

		manager->SetParameters(x);
		return manager->GetValue();
	}

protected:
	NelderMeadeSimplexOptimizer() {
		m_useSimplexSize = false;
		m_maxNbIterations = 200;
		m_f_tol = 1e-5;
		m_isoSimplexSize = 0.1;
	};
	~NelderMeadeSimplexOptimizer() {};

	double m_f_tol, m_isoSimplexSize; 
	vnl_vector<double> m_SimplexSize; bool m_useSimplexSize;
	unsigned int m_maxNbIterations;

private:
	NelderMeadeSimplexOptimizer(const Self&);				//purposely not implemented
	const Self & operator=( const Self & );		//purposely not implemented
};




} // namespace psciob

#endif /* NELDERMEADESIMPLEXOPTIMIZER_H_ */
