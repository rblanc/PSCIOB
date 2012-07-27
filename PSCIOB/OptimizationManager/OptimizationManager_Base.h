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
* \file OptimizationManager_Base.h
* \author Rémi Blanc 
* \date 9. September 2011
*/

#ifndef OPTIMIZATIONMANAGER_BASE_H_
#define OPTIMIZATIONMANAGER_BASE_H_

//#include "CommonTypes.h"
#include "GeneralUtils.h"

#include "CostFunction_Base.h"
#include "Optimizer_Base.h"

namespace psciob {

/** \brief OptimizationManager_Base
 * Base Classe for Managing Scene Optimization
 * implements the functionalities for communication between these classes and the scene
 * abstract methods to select which parameters needs to be optimized, how to get and to set them
*/


class OptimizationManager_Base : public itk::LightObject {
public:
	/** Standard class typedefs. */
	typedef OptimizationManager_Base		Self;
	typedef itk::LightObject				Superclass;
	typedef itk::SmartPointer<Self>			Pointer;
	/** Run-time type information (and related methods). */
	itkTypeMacro(OptimizationManager_Base,itk::LightObject);

	virtual unsigned int GetNumberOfParameters()		= 0;	//problems to set this to const, see the comment in SingleObjectOptimizationManager

	virtual vnl_vector<double> GetParameters()			= 0;
	virtual bool SetParameters(vnl_vector<double> p)	= 0;	//returns false if there was problems with these parameters


	//some optimizer may also request densities to be associated with the parameters - the manager should be able to provide them with these
	//think about it more thoroughly -- 
	//potential uses: get likelihood for some parameters ; propose new parameters ; propose modifications of the parameters ; ...
	virtual MultivariatePDF* GetParameterPriorPDF()			{return NULL;}	//or throw an exception?
	virtual MultivariatePDF* GetParameterProposalMovePDF()	{return NULL;}	//or throw an exception?

	/** sets the cost function to be optimized 
	* may be overloaded by child classes
	*/
	virtual void SetCostFunction( CostFunction_Base *costFunction )	{ m_costFunction = costFunction; }
	CostFunction_Base* GetCostFunction()					{ return m_costFunction.GetPointer(); }
	/** defines the actual optimizer, i.e. the algorithm that modifies the parameters in order to minimize the given cost function */
	void SetOptimizer(Optimizer_Base *optimizer)			{ m_optimizer = optimizer; }
	
	/** evaluates the cost function */
	inline double GetValue()								{ return m_costFunction->GetValue(); }

	/** Performs the optimization 
	 * The initial state is save as backup ; depending on the optimizer, if a best state is encountered, it is saved in place
	 * The method ensures that it returns the scene in the best state that has been encountered during optimization
	 */
	double Optimize(bool verbose = false) {
		if ( (!m_costFunction) && (!m_optimizer) ) throw DeformableModelException("OptimizationManager: the metric and optimizer must be set before anything can be optimized...");
	
		//std::cout<<"entering Optimize... saving initial state"<<std::endl;
		this->SaveCurrentStateAsBest();
		//std::cout<<"initial cost: "<<m_bestCost<<std::endl;
		double finalCost = m_optimizer->Optimize(this);
		if (finalCost > m_bestCost + TINY ) { 
			if (verbose) std::cout<<"  optimizer final state is not the best encountered state ... falling back there - final: "<<finalCost<<" vs best: "<<m_bestCost<<" ; diff = "<<finalCost-m_bestCost<<std::endl;
			finalCost = this->ReturnToBestKnownState();
			if (verbose) if (finalCost != m_bestCost) std::cout<<"!!! could not return to the best state!! should never happen !! reached only "<<finalCost<<std::endl;
		}
		return finalCost;	
	}


	/**virtual, because the state may take different forms depending on what is optimized
	 * some optimization concern only fixed parameters ; while some other may modify the SPACE (add new objects, etc...)
	 * \todo access shouldn't be public, but restricted to optimizers... => use friendship
	 * since many child classes share the same implementation, based only a a vector of parameters, this basic implementation could be made here, with standard virtual method...
	 */
	virtual void SaveCurrentStateAsBest() = 0;
protected:
	OptimizationManager_Base() { m_costFunction = NULL; m_optimizer = NULL; };
	virtual ~OptimizationManager_Base() {};	

	CostFunction_Base::Pointer		m_costFunction;
	Optimizer_Base::Pointer			m_optimizer;

	double m_bestCost;
	virtual double ReturnToBestKnownState() = 0;

private:
	OptimizationManager_Base(const Self&);		//purposely not implemented
	const Self & operator=( const Self & );		//purposely not implemented
};



} // namespace psciob

#endif /* OPTIMIZATIONMANAGER_BASE_H_ */
