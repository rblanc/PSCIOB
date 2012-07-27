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
 * \file SimulatedAnnealingOptimizer.h
 * \author Rémi Blanc 
 * \date 28. September 2011
*/


#ifndef SIMULATEDANNEALINGOPTIMIZER_H_
#define SIMULATEDANNEALINGOPTIMIZER_H_


#include "GeneralUtils.h"
#include "Optimizer_Base.h"
#include "OptimizerCostFunction.h"

#include "ScalarFunctions.h"
#include "MultivariatePDF.h"


namespace psciob {

/** \brief Base Class for Simulated Annealing
 * implements the basic mechanism of proposing random moves, accept or reject it with some probability
 * depending on the temperature T
 *  + decrease of the temperature over time ; maximum number of iterations ; starting temperature
 *
 * can be overridden e.g. with Random Jump
 * in this one, we just perform random walk on the available parameters 
 * it requests a (multivariate) probability distribution for proposing random moves
 * implements the base mechanisms for temperature setting and decrease...
 *
 * implementation inspired from http://www.mathworks.de/matlabcentral/fileexchange/10548
*/ 

class SimulatedAnnealingOptimizer : public Optimizer_Base {
public:
	/** Standard class typedefs. */
	typedef SimulatedAnnealingOptimizer		Self;
	typedef Optimizer_Base					Superclass;
	typedef itk::SmartPointer<Self>			Pointer;
	/** Run-time type information (and related methods). */
	itkTypeMacro(SimulatedAnnealingOptimizer,Optimizer_Base);
	itkNewMacro(SimulatedAnnealingOptimizer);

	/** Set the initial temperature */
	void SetInitialTemperature(double t = 1)			{ if (t<=0) throw DeformableModelException("Exception in Simulated Annealing: temperatures should be >0"); m_initialTemperature=t; }

	/** Stop optimization when this temperature is reached */
	void SetStopTemperature(double t = 1e-6)			{ if (t<=0)	throw DeformableModelException("Exception in Simulated Annealing: temperatures should be >0"); m_stopTemperature=t; }
	
	/** Stop optimization when the metric becomes lower than this value */
	void SetStopMetric(double v = -1e10)				{ m_stopValue=v; }

	/** Sets the cooling parameter function
	* default is a ScalarFunction_Geometric with geometric factor = 0.995 
	*/
	void SetCoolingFunction(ScalarFunction_Base *fct)                   { m_coolingFunction = fct; }   //this should be a decreasing function...
	
	/** Stops optimization when there are that many successive valid propositions have been rejected (no way to improve anymore <=> reached a local minima, and the temperature is too low to get out ...) */
	void SetMaximumNumberOfConsecutiveFailures(unsigned int n = 5000)   { m_maxSuccessiveFailures=n; }

	/** Maximum number of propositions at a given temperature => important schedule parameter to cool down the system */
	void SetMaximumNumberOfAttemptsAtT(unsigned int n = 300)            { m_maxTriesAtT=n; }

	/** Maximum number of consecutive successes at T, before cooling: if all moves are accepted, than likely the temperature is too high */
	void SetMaximumNumberOfConsecutiveSuccessesAtT(unsigned int n = 20) { m_maxSuccessesAtT=n; }

	/** number of parameters that are simultaneously modified - default is 1 */
	void SetNumberOfSimultaneousMoves(unsigned int n)	{ m_nbSimultaneousMoves = n; } //this should be < nb of parameters

	/** Get Pointer to the cooling function */
	ScalarFunction_Base * GetCoolingFunction() {return m_coolingFunction.GetPointer();}
	//
	void GetProposalDistributionFromManager(OptimizationManager_Base *manager) {
		m_proposalDistribution = manager->GetParameterProposalMovePDF();
		if (!m_proposalDistribution) m_proposalPDFIsSet=false;  //check if this went well (not a NULL pointer...) does this work properly?
		else m_proposalPDFIsSet=true; 
	} 

	//
	virtual double Optimize(OptimizationManager_Base *manager) {
		m_nbParameters = manager->GetNumberOfParameters();
		GetProposalDistributionFromManager(manager);
		if (!m_proposalPDFIsSet) {//in case the manager cannot provide a pdf, use a standard MV-Normal distribution...
			MVN_PDF::Pointer tmp = MVN_PDF::New(); tmp->SetNumberOfDimensions( m_nbParameters );
			vnl_vector<double> m(m_nbParameters); m.fill(0);
			vnl_matrix<double> c(m_nbParameters,m_nbParameters); c.set_identity(); c=c*0.1;
			m_proposalDistribution = static_cast<MultivariatePDF::Pointer>(tmp);	
		}
		if (m_proposalDistribution->GetNumberOfDimensions() != m_nbParameters) throw DeformableModelException("Exception in Simulated Annealing: proposal distribution of wrong dimension: ");

		//TODO: make other consistency checks...

		//OptimizerCostFunction cost_function(m_nbParameters); cost_function.SetCaller(manager);

		vnl_vector<double>	currentParameters, proposalParameters;
		double				bestEnergy     = manager->GetValue(),	   currentEnergy,	  proposalEnergy;
		unsigned int		nbTotalIterations=0, nbSuccessiveFailures=0, nbSuccessesAtT=0, nbIterationsAtT=0;
		double				currentT = m_initialTemperature;

		bool converged = false, validMove;

		while (!converged){
			nbIterationsAtT++;
//std::cout<<"iteration at T: "<<nbIterationsAtT<<", T = "<<currentT<<std::endl;
			//save state at the beginning of the iteration
			currentParameters = manager->GetParameters(); currentEnergy = manager->GetValue();

			// check if we need to stop or to decrease the temperature
			if ( (currentT<m_stopTemperature) || (nbSuccessiveFailures>=m_maxSuccessiveFailures) ) {
				converged = true; nbTotalIterations+=nbIterationsAtT;
				break;
			}
			if ( (nbIterationsAtT>=m_maxTriesAtT) || (nbSuccessesAtT>=m_maxSuccessesAtT) ) {
				currentT = m_coolingFunction->Evaluate(currentT); 
				nbTotalIterations+=nbIterationsAtT; 
				nbSuccessesAtT = 0;
				nbIterationsAtT = 0;
				//nbSuccessiveFailures = static_cast<unsigned int>(max<int>(0, nbSuccessiveFailures/2)); //OPTIMIZATION?? //ensure that a minimum number of attempts are made after decreasing the temperature ??
			}

			//propose a (valid) move
			validMove = false;
			while (!validMove) {
				proposalParameters = ProposeRandomMove(currentParameters, currentT);
				//std::cout<<"current params: "<<currentParameters<<"\npropose params: "<<proposalParameters<<std::endl;
				validMove = manager->SetParameters(proposalParameters);
				//std::cout<<"set parameters: "<<manager->GetParameters()<<" -- valid move? "<<validMove<<std::endl;
			}
			//std::cout<<"get value..."<<std::endl;
			proposalEnergy = manager->GetValue(); 

			//now check it for acceptance
			if (proposalEnergy < m_stopValue) { //if the new energy is lower than the requested minimum => accept and exit
				manager->SaveCurrentStateAsBest(); bestEnergy = proposalEnergy;
				converged = true;
//std::cout<<"SA final energy: "<<bestEnergy<<", parameters: "<<proposalParameters<<std::endl;
				break;
			}

			if ( currentEnergy - proposalEnergy > TINY ) { // if the energy decreased: ACCEPT
				nbSuccessesAtT++; nbSuccessiveFailures = 0;
				if (proposalEnergy < bestEnergy) { manager->SaveCurrentStateAsBest(); bestEnergy = proposalEnergy; }	//update the best state if necessary
//std::cout<<"SA better: "<<proposalEnergy<<" vs "<<currentEnergy<<std::endl;
			}
			else { //accept with some probability
				if ( m_rndgen->GetUniformVariate() < exp( (currentEnergy - proposalEnergy)/(currentT) ) ) { //... how sensitive is it to the dynamic of the metric...
					nbSuccessesAtT++;
					//nbSuccessiveFailures = 0; // questionable...
//std::cout<<"SA accept: "<<proposalEnergy<<" vs "<<currentEnergy<<std::endl;
				}
				else { // reject the move
					manager->SetParameters(currentParameters); //undo the proposed move
					nbSuccessiveFailures++;
//std::cout<<"SA keep  : "<<currentEnergy<<" vs "<<proposalEnergy<<std::endl;
				}
			}

		}
		
		return manager->GetValue();//the manager is in charge of returning to the best visited state, in case the end position is not this one...
	}

protected:
	SimulatedAnnealingOptimizer() : Optimizer_Base() {
		m_initialTemperature = 1; 
		m_stopTemperature = 1e-5; //since the amplitude of the moves decreases with the temperature, it is not very useful to iterate at very low T
		m_stopValue = -1e10;	  //when the energy reaches this value -> exit
		m_maxSuccessiveFailures = 5000; m_maxTriesAtT = 300; m_maxSuccessesAtT = 20;
		m_nbSimultaneousMoves = 1;//how many parameters are modified at each move
		//
		ScalarFunction_Geometric::Pointer tmp = ScalarFunction_Geometric::New(); tmp->SetFactor(0.995); //at this cooling rate with the default parameters, there are about 1375 temperature steps
		m_coolingFunction = static_cast<ScalarFunction_Base::Pointer>(tmp);
		m_proposalPDFIsSet=false;
		//
		m_rndgen = RandomVariableGenerator::New();
	};
	~SimulatedAnnealingOptimizer() {};

	double m_initialTemperature, m_stopTemperature;
	double m_stopValue; //when reaching this
	unsigned int m_maxTriesAtT, m_maxSuccessesAtT, m_maxSuccessiveFailures;
	unsigned int m_nbSimultaneousMoves;				 //keep only a couple of input values -> IDEA: sample only the requested variable ? ~> need to extract marginal distribs.... not so easy

	bool m_proposalPDFIsSet;
	MultivariatePDF::Pointer m_proposalDistribution; //draw a sample //WARNING: the distribution should have zero mean ?!?!?!!
	ScalarFunction_Base::Pointer m_coolingFunction;
	RandomVariableGenerator::Pointer m_rndgen;

//IDEA: monitor the evolution of the process... std::vector<unsigned> m_nbObjectObserver; std::vector<double> m_energyObserver;

	//OPTIMIZATION: another possibility would be to directly modify the parameters... instead of copying the whole vector, etc...
	inline vnl_vector<double> ProposeRandomMove(vnl_vector<double> initial_value, double T) const {
		vnl_vector<double> tmp_sample = m_proposalDistribution->DrawSample();
		vnl_vector<double> output; output = initial_value;
		unsigned int rnd_n;
		//select the elements to be modified, and modify them...
		if (m_nbSimultaneousMoves==1) {//select one parameter randomly
			rnd_n = m_rndgen->GetIntegerVariate(m_nbParameters-1);
//std::cout<<"modifiying parameters with index: "<<rnd_n<<", old value = "<<output( rnd_n )<<", new value = "<<output( rnd_n )+tmp_sample( rnd_n ) * T<<" -- with T = "<<T<<std::endl;
			output(rnd_n) += tmp_sample(rnd_n) * T; //amplitude of the move decreases with the temperature (/time)
		}
		else {	//perform a random permutation on [1:nb_params] , and select the first elements of the results as the indices of the parameters to modify
			vnl_vector<unsigned int> order(m_nbParameters);
			unsigned int tmp, rnd;
			//TODO: move this to a class ~> generic random permutation...
			for (unsigned i=0 ; i<m_nbParameters ; i++) {order(i)=i;}
			for (unsigned i=0 ; i<m_nbParameters-1 ; i++) {//swap it with a random candidate : algo from http://c-faq.com/lib/shuffle.html
				rnd_n = m_rndgen->GetIntegerVariate(m_nbParameters-1-i);
				tmp = order(i); order(i) = order(i+rnd_n); order(i+rnd_n) = tmp;
			}
			for (unsigned i=0 ; i<m_nbSimultaneousMoves ; i++) { output( order(i) ) += tmp_sample( order(i) ) * T; }
		}
		return output;
	}

private:
	SimulatedAnnealingOptimizer(const Self&);	//purposely not implemented
	const Self & operator=( const Self & );		//purposely not implemented
};






} // namespace psciob

#endif /* SIMULATEDANNEALINGOPTIMIZER_H_ */
