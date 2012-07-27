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

/*
 * \file RandomJumpSAOptimizer.h
 * \author Rémi Blanc 
 * \date 28. September 2011
*/


#ifndef RANDOMJUMPSAOPTIMIZER_H_
#define RANDOMJUMPSAOPTIMIZER_H_


//IDEA IS TO PROPOSE RANDOM JUMPS (BIRTH&DEATH ; OBJECT TYPE CHANGES ; ...) at each temperature change of the standard SA algorithm
//IDEA: is it possible to implement a number of "jump types", and to register them as requested
//type 1: birth & death
//type 2: type change
//type 3: ....? specific optimizer for a single object, ...


#include <SimulatedAnnealingOptimizer.h>
#include <SceneOptimizationManager_Base.h>
#include <ReversibleSceneModifierKernel_Base.h>

//THINK: this optimizer actually may not really require any manager at all... 
//at least not the get/set parameters ; however, the get/set scene state is required to jump back to the initial / best configuration - 
//it needs a cost function to know how good the proposed move are
//its kernels should know how to work by themselves (proposing new moves, getting/setting object parameters, etc...)


namespace psciob {
 
 /** \class RandomJumpSAOptimizer
 * \brief RandomJumpSAOptimizer
 * Optimizer which works with a set of kernels (ReversibleSceneModifierKernels)
 * At each iteration, a kernel is chosen randomly, proposes a move, which is accepter or rejected
 * using a Simulated Annealing type of behavior
*/

//expected to work on the objects of a scene <-> templated against the scene, and using a scene-specific optimization manager
template<class TScene>
class RandomJumpSAOptimizer : public SimulatedAnnealingOptimizer {
public:
	/** Standard class typedefs. */
	typedef RandomJumpSAOptimizer			Self;
	typedef SimulatedAnnealingOptimizer		Superclass;
	typedef itk::SmartPointer<Self>			Pointer;
	/** Run-time type information (and related methods). */
	itkTypeMacro(RandomJumpSAOptimizer,SimulatedAnnealingOptimizer);
	itkNewMacro(RandomJumpSAOptimizer);

	typedef TScene                                        SceneType;
	typedef SceneOptimizationManager_Base<SceneType>      OptimizationManagerType;
	typedef ReversibleSceneModifierKernel_Base<SceneType> KernelType;
	typedef typename KernelType::Pointer                  KernelPointerType;


	/** Add proposition kernels, with associated relative weight.
	 * The weight is related to the probability with which this particular kernel is selected
	 * A larger weight means the kernel will be selected more often (comparatively to kernels with a lower weight...)
	 */
	void AddKernel(KernelType *kernel, double weight = 1) {	
		//if (!m_scene) throw DeformableModelException("Exception in RandomJumpSAOptimizer: the scene must be set before kernels are added");
		//kernel->SetScene( m_scene );
		m_listKernels.push_back(kernel);
		m_listKernelWeights.push_back(weight);
		m_totalKernelWeight+=weight;
	}

	/** Removes all the kernels ; this is the only way to remove any kernel, at least for the moment. */
	void RemoveAllKernels() {m_listKernels.clear();m_totalKernelWeight=0;}


	double Optimize(OptimizationManager_Base *manager) {
		if (m_totalKernelWeight==0) throw DeformableModelException("Exception in RandomJumpSAOptimizer: no kernels have been proposed!");
		
		//
		m_manager = dynamic_cast<SceneOptimizationManager_Base<SceneType>*>(manager);
		if (!m_manager) throw DeformableModelException("Exception in RandomJumpSAOptimizer: the optimization manager should derive from SceneOptimizationManager_Base");
		m_scene = m_manager->GetScene();
		if (!m_scene) throw DeformableModelException("Exception in RandomJumpSAOptimizer: the given optimization manager is not related to any scene!");
		for (unsigned i=0 ; i<m_listKernels.size() ; i++) {
			m_listKernels[i]->SetScene(m_scene);
			m_listKernels[i]->CheckKernelValidity();
		}


		double			bestEnergy = manager->GetValue(), currentEnergy, proposalEnergy;
		unsigned int	nbTotalIterations=0, nbSuccessiveFailures=0, nbSuccessesAtT=0, nbIterationsAtT=0;
		double			currentT = m_initialTemperature;


clock_t t0=clock();
		unsigned int kernelIndex;
		double kernelOutput;
		bool converged = false, validMove;
		while (!converged){
			nbIterationsAtT++;
//std::cout<<"T = "<<currentT<<", iter: "<<nbIterationsAtT<<std::endl;
			currentEnergy = manager->GetValue();
			
			// check if we need to stop or to decrease the temperature
			if ( (currentT<m_stopTemperature) || (nbSuccessiveFailures>=m_maxSuccessiveFailures) ) {
				if (nbSuccessiveFailures>=m_maxSuccessiveFailures) std::cout<<"too many successive failures... exiting"<<std::endl;
				converged = true; nbTotalIterations+=nbIterationsAtT;
				break;
			}
			if ( (nbIterationsAtT>=m_maxTriesAtT) || (nbSuccessesAtT>=m_maxSuccessesAtT) ) {
				currentT = m_coolingFunction->Evaluate(currentT); 
				nbTotalIterations+=nbIterationsAtT; 
				nbSuccessesAtT = 0;
				nbIterationsAtT = 0;
//nbSuccessiveFailures = static_cast<unsigned int>(max<int>(0, nbSuccessiveFailures/2)); //OPTIMIZATION?? //ensure that a minimum number of attempts are made after decreasing the temperature ??
std::cout<<"nb iterations: "<<nbTotalIterations<<", Temperature: "<<currentT<<", current Energy: "<<currentEnergy<<", (best="<<bestEnergy<<"), current nb objects: "<<m_scene->GetNumberOfObjects()<<" ; time: "<<(clock()-t0)/((double)CLOCKS_PER_SEC)<<" s."<<std::endl;
			}

			//HERE: check which action to perform
//std::cout<<"energy before: "<<currentEnergy<<std::endl;
			validMove = false;
			while(!validMove) {
				kernelIndex = RandomKernelSelection();
//std::cout<<"applying kernel "<<kernelIndex;
				kernelOutput = m_listKernels[kernelIndex]->Apply(currentT/m_initialTemperature);
//std::cout<<" -- application: "<<kernelOutput<<std::endl;
				if (kernelOutput>0) { validMove=true; break; }
				if (kernelOutput==0) { nbSuccessiveFailures++; continue; } //the kernel proposed an invalid move
				//the kernel can also output the code -1, which means the kernel cannot be applied at all (e.g. killing an object if the scene is empty...)
			}
//std::cout<<"!!kernel applied..."<<kernelIndex<<", valid move? "<<validMove<<std::endl;
			//}
			proposalEnergy = manager->GetValue();
//std::cout<<"!!proposal energy: "<<proposalEnergy<<std::endl;
			//end of the application of the kernel

			if (proposalEnergy < m_stopValue) { //if the new energy is lower than the requested minimum => accept and exit
				converged = true; 
				//std::cout<<"saving state as best"<<std::endl;
				manager->SaveCurrentStateAsBest();	bestEnergy = proposalEnergy;
				//std::cout<<"saving done"<<std::endl;
				break;
			}

			if ( currentEnergy - proposalEnergy > TINY ) { // if the energy decreased: ACCEPT
				nbSuccessesAtT++;
				nbSuccessiveFailures = 0;
				if (proposalEnergy < bestEnergy) { 
					//std::cout<<"saving state as best"<<std::endl;
					manager->SaveCurrentStateAsBest(); bestEnergy = proposalEnergy; 
					//std::cout<<"saving done"<<std::endl;
				}
			}
			else { //accept with some probability
				if ( m_rndgen->GetUniformVariate() < exp( (currentEnergy - proposalEnergy)/(currentT) ) ) { //... how sensitive is it to the dynamic of the metric...
					nbSuccessesAtT++;
					//nbSuccessiveFailures = 0; // questionable...
//		std::cout<<"accepting the new state..."<<std::endl;
					//stay in this state: nothing to do...
				}
				else { //reject the move
//		std::cout<<"undoing move"<<std::endl;
					m_listKernels[kernelIndex]->Undo();		//ask the kernel to undo the move
					nbSuccessiveFailures++;
//		std::cout<<"undone"<<std::endl;
				}
			}

		}

		return manager->GetValue();	//the manager will return to the best visited state in case this is not the case
	}

protected:
	RandomJumpSAOptimizer() : SimulatedAnnealingOptimizer() {
		m_totalKernelWeight = 0;
		m_manager = NULL;
		m_scene=NULL;
	};
	~RandomJumpSAOptimizer() {};

	//bool m_spaceChanged;				//flag indicating whether the parameter space changed (to due e.g. the birth or death of an object)
	double m_totalKernelWeight;
	std::vector<KernelPointerType> m_listKernels;	//list of action kernels (e.g. birth kernel, death kernel, etc...)
	std::vector<double> m_listKernelWeights;		//weight of the kernels

	typename SceneOptimizationManager_Base<SceneType>::Pointer m_manager;
	typename SceneType::Pointer m_scene;

	unsigned int RandomKernelSelection() {
		double tmp = m_rndgen->GetUniformVariate(0, m_totalKernelWeight);
		unsigned i=0;
		while (tmp>0) {
			if (tmp<m_listKernelWeights[i]) return i;
			tmp-=m_listKernelWeights[i]; i++;
		}
		return i; //should never happen... but prevents a compiler warning...
	}
	

private:
	RandomJumpSAOptimizer(const Self&);			//purposely not implemented
	const Self & operator=( const Self & );		//purposely not implemented
};






} // namespace psciob

#endif /* RANDOMJUMPSAOPTIMIZER_H_ */
