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
* \file SingleSceneObjectOptimizationManager.h
* \author Rémi Blanc 
* \date 21. September 2011
*/


#ifndef SINGLESCENEOBJECTOPTIMIZATIONMANAGER_H_
#define SINGLESCENEOBJECTOPTIMIZATIONMANAGER_H_

#include "SceneOptimizationManager_Base.h"

namespace psciob {

/** \brief SingleSceneObjectOptimizationManager
 * Single Object (from a scene) Optimization
 * generic class for the optimization of the parameters of a single object from a scene
*/


template<class TScene>
class SingleSceneObjectOptimizationManager : public SceneOptimizationManager_Base<TScene> {
public:
	/** Standard class typedefs. */
	typedef SingleSceneObjectOptimizationManager	Self;
	typedef SceneOptimizationManager_Base			Superclass;
	typedef itk::SmartPointer<Self>					Pointer;
	/** Run-time type information (and related methods). */
	itkTypeMacro(SingleSceneObjectOptimizationManager,SceneOptimizationManager_Base);
	itkNewMacro(Self);

	/** overloads the base method to initialize the requested label */
	void SetScene(SceneType* scene)		{ 
		Superclass::SetScene(scene);
		m_requestedLabel = m_scene->FirstID();
	}

	/** sets the cost function to be optimized 
	* overloads the base method, it checks whether the cost function is deriving from SceneCostFunction_Base
	* so that it may tune it to compute the cost associated with the current object.
	*/
	void SetCostFunction( CostFunction_Base *costFunction )	{ 
		Superclass::SetCostFunction(costFunction);
		m_convertedCostFunction = dynamic_cast<SceneCostFunction_Base<SceneType>*>(m_costFunction.GetPointer());
		if (!m_convertedCostFunction) {} else {m_convertedCostFunction->SelectObject( m_requestedLabel );}
	}

	/** Indicates the number of parameter of the object selected for optimization */
	inline unsigned int GetNumberOfParameters()			{ return m_scene->GetObject(m_requestedLabel)->obj->GetNumberOfParameters(); }

	/** get the parameters of the selected object */
	virtual inline vnl_vector<double> GetParameters()       { return m_currentParameters; }

	/** modifies the parameters of the selected object, updating the scene accordingly */
	virtual inline bool SetParameters(vnl_vector<double> p)	{ 
		if (m_scene->ModifyObjectParameters(m_requestedLabel, p)) { m_currentParameters = p; return true; }
		else return false;
	}

	/** Select the object to optimize */
	//-- DEV WARNING: this should NEVER be called by an optimizer 
	// there could be issues with the 'best cost' mechanisms, and the Optimize() method could mess things up, trying to modify the parameters of the wrong object
	//check whether this can be handled by resseting the 'best cost' and 'best parametes' when calling this function.
	void SelectObject( IDType label ) { 
		m_requestedLabel = label; 
		//if the cost function is capable of working on single object, tell it to do so.
		if (!m_convertedCostFunction) {} else {m_convertedCostFunction->SelectObject( m_requestedLabel );}
		m_currentParameters = m_scene->GetParametersOfObject( m_requestedLabel );
	}

	/** get pdf for the parameters of one specific object */
	MultivariatePDF* GetParameterPriorPDF() {
		return m_scene->GetObjectTypesLibrary()->GetObjectPDF( m_scene->GetObject( m_requestedLabel )->objectTypeId , PDF_OBJECTGENERATIONPRIOR);
	}

	/** get pdf for the parameters of one specific object */
	MultivariatePDF* GetParameterProposalMovePDF() {
		return m_scene->GetObjectTypesLibrary()->GetObjectPDF( m_scene->GetObject( m_requestedLabel )->objectTypeId , PDF_RANDOMWALK);
	}

	/** optimize the parameters of each object of the scene, in the same order they are stored in the scene (through their index)
	 * returns the final cost value
	 */
	double OptimizeEachObjectSequentially(bool verbose = false) {
		clock_t t0;
		//vnl_vector<double> iniparams, iniparams_;
		double val; SceneObjectIterator<SceneType> it(m_scene);
		for (it.GoToBegin() ; !it.IsAtEnd() ; ++it) {
			SelectObject(it.GetID());
			//iniparams = GetParameters(); iniparams_ = m_scene->GetParametersOfObject(it.GetID());
			t0=clock();  double ini = this->GetValue();
			val = this->Optimize();
			if (verbose) {
				std::cout<<"  optimized object with label: "<< it.GetID() <<" / "<<m_scene->GetNumberOfObjects()<<" ; final value: "<<val<<" vs initial: "<<ini<<" -- optimization time: "<<(clock()-t0)/((double)CLOCKS_PER_SEC)<<" s."<<std::endl;
				if (val>1e8) std::cout<<"    object params: "<<m_scene->GetParametersOfObject(it.GetID())<<std::endl;
				//std::cout<<"     initial parameters: "<<iniparams<<", object params: "<<iniparams_<<std::endl;
				//std::cout<<"     final   parameters: "<<GetParameters()<<", object params: "<<m_scene->GetParametersOfObject(it.GetID())<<std::endl;
			}
		}
		return this->GetValue();
	}

	//imagine other ways... take the worst object, optimize it ; if it is still the worse, stop with it and continue with the next... until all objects are processed.
	double OptimizeWorstObjectUntilConvergence(bool verbose = false) {
		clock_t t0;

		double worstCost=1e100, previousWorstCost, cost;
		IDType worstObject=0, previousWorstObject;
		unsigned int nbObjects = m_scene->GetNumberOfObjects(); //indices of the objects
		bool converged=false;

		while (!converged) {
			//find the worst object
			previousWorstCost=worstCost;previousWorstObject=worstObject;
			for ( IDType id = FirstID() ; id!=EndID() ; id=NextID() ) { 
				SelectObject( id );
				cost = this->GetValue();
				if (cost>worstCost) worstObject=id;
			}
			if ( (previousWorstObject == worstObject) && (previousWorstCost<=worstCost) ) converged=true;
			else {
				this->SelectObject( worstObject );
				//and optimize it...
				this->Optimize();
				if (verbose) std::cout<<"  optimized worst object with label: "<< worstObject <<" ; final value: "<<GetValue()<<" (init: "<<worstCost<<") ; optimization time: "<<(clock()-t0)/((double)CLOCKS_PER_SEC)<<" s."<<std::endl;
			}
		}

		return this->GetValue();
	}


	//CLEAN: access should be restricted to some optimizers ... use friendship?
	void SaveCurrentStateAsBest()		{ m_bestCost = this->GetValue(); m_bestKnownParameters = this->GetParameters();	}
protected:
	SingleSceneObjectOptimizationManager() : SceneOptimizationManager_Base() {m_convertedCostFunction=NULL;}
	~SingleSceneObjectOptimizationManager() {};

	typename SceneType::IDType m_requestedLabel;

	vnl_vector<double> m_bestKnownParameters, m_currentParameters;
	double ReturnToBestKnownState()		{ this->SetParameters(m_bestKnownParameters); return this->GetValue(); }

	typename SceneCostFunction_Base<SceneType>::Pointer m_convertedCostFunction; //this is used only to select the right object.

private:
	SingleSceneObjectOptimizationManager(const Self&);	//purposely not implemented
	const Self & operator=( const Self & );				//purposely not implemented
};



} // namespace psciob

#endif /* SINGLEOBJECTOPTIMIZATIONMANAGER_H_ */
