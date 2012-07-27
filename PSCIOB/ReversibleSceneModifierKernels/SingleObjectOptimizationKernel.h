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
* \file SingleObjectOptimizationKernel.h
* \author Rémi Blanc 
* \date 10. October 2011
*
* SingleObjectOptimizationKernel: 
* some specialized kernels for single object optimization
* - SingleObjectOptimizationKernel
* - WorstObjectOptimizationKernel
*/



#ifndef _SINGLEOBJECTOPTIMIZATIONKERNEL_H_
#define _SINGLEOBJECTOPTIMIZATIONKERNEL_H_

#include <ReversibleSceneModifierKernel_Base.h>
#include "SingleSceneObjectOptimizationManager.h"


namespace psciob {



/**\class SingleObjectOptimizationKernel
* \brief SingleObjectOptimizationKernel: propose to apply a given optimizer to a randomly selected object
* \sa SingleSceneObjectOptimizationManager
*/
template<class TScene>
class SingleObjectOptimizationKernel : public ReversibleSceneModifierKernel_Base<TScene> {
public:
	/** Standard class typedefs. */
	typedef SingleObjectOptimizationKernel     Self;
	typedef ReversibleSceneModifierKernel_Base Superclass;
	typedef itk::SmartPointer<Self>            Pointer;
	/** Run-time type information (and related methods). */
	itkTypeMacro(SingleObjectOptimizationKernel, ReversibleSceneModifierKernel_Base);
	itkNewMacro(Self);

	typedef SingleSceneObjectOptimizationManager<TScene>          OptimizationManagerType;


	/** Checks that all the parameters of the kernel have been set properly (e.g. the scene...) */
	virtual bool CheckKernelValidity() {
		Superclass::CheckKernelValidity();
		if (!m_manager){ throw DeformableModelException("SingleObjectOptimizationKernel is not valid: no associated SingleObjectOptimizationManager"); return false; }
		return true;
	}

	/** Set the optimization manager (deriving from SingleSceneObjectOptimizationManager) used for optimization
	* The cost function and actual optimizer used must be defined.
	*/
	void SetOptimizationManager(OptimizationManagerType *manager) { m_manager = manager; }


	inline IDType SelectRandomObject() { //select an object randomly...
		unsigned int requestedIndex = m_rndgen->GetIntegerVariate(m_scene->GetNumberOfObjects()-1);
		SceneObjectIterator<SceneType> objectIt(m_scene); objectIt.GoToBegin(); objectIt.Advance(requestedIndex);
		return objectIt.GetID();		
	}

	//temperature is given as an argument ; is it usefull in this case??? not so sure...
	double Apply(double T) {
		if (m_scene->GetNumberOfObjects()==0) return -1;
		m_label = SelectRandomObject();
		//if (!m_label) return -1;
		m_manager->SelectObject(m_label);
		//remember the previous parameters
		m_params = m_manager->GetParameters();
		//do the modification
//clock_t t0=clock();

		m_manager->Optimize();
//std::cout<<"optimizing object with label: "<<m_label<<", initial params: "<<m_params<<", final params: "<<m_manager->GetParameters()<<", time to optimize: "<<(clock()-t0)/((double)CLOCKS_PER_SEC)<<" s."<<std::endl;
		return 1;	//this is always OK? or maybe return false if no improvement happened
	}

	//move the scene to the state it used to have the last time we called Apply
	void Undo() { //return the requested object to its original parameters.
		m_manager->SelectObject(m_label);	//shouldn't be necessary ...
		if (!m_manager->SetParameters(m_params)) throw DeformableModelException("SingleObjectOptimizationKernel::Undo : unable to undo the move properly -- SHOULD NEVER HAPPEN");
	}

protected:
	SingleObjectOptimizationKernel() : ReversibleSceneModifierKernel_Base() { m_manager = 0; };
	~SingleObjectOptimizationKernel() {};

	typename OptimizationManagerType::Pointer m_manager;
	IDType m_label;
	vnl_vector<double> m_params;

private:
	SingleObjectOptimizationKernel(const Self&);		//purposely not implemented
	const Self & operator=( const Self & );				//purposely not implemented
};


/**\class WorstObjectOptimizationKernel
* \brief WorstObjectOptimizationKernel: propose to apply a given optimizer to the worst object from the scene
* \sa SingleSceneObjectOptimizationManager
*/
template<class TScene>
class WorstObjectOptimizationKernel : public ReversibleSceneModifierKernel_Base<TScene> {
public:
	/** Standard class typedefs. */
	typedef WorstObjectOptimizationKernel  Self;
	typedef ReversibleSceneModifierKernel_Base                     Superclass;
	typedef itk::SmartPointer<Self>           Pointer;
	/** Run-time type information (and related methods). */
	itkTypeMacro(WorstObjectOptimizationKernel, ReversibleSceneModifierKernel_Base);
	itkNewMacro(Self);

	typedef SingleSceneObjectOptimizationManager<TScene>          OptimizationManagerType;

	/** Checks that all the parameters of the kernel have been set properly (e.g. the scene...) */
	bool CheckKernelValidity() {
		if (!m_scene)  { throw DeformableModelException("WorstObjectOptimizationKernel is not valid: no associated scene"); return false; }
		if (!m_manager){ throw DeformableModelException("WorstObjectOptimizationKernel is not valid: no associated SingleObjectOptimizationManager"); return false; }
		return true;
	}


	/** Set the optimization manager (deriving from SingleSceneObjectOptimizationManager) used for optimization
	* The cost function and actual optimizer used must be defined.
	*/
	void SetOptimizationManager(OptimizationManagerType *manager) { m_manager = manager; }

	/** Selects the worst object ; returns false if this failed (e.g. scene is empty), and true otherwise */
	inline bool SelectWorstObject() { //select the worst object with respect to the cost function employed by the local optimization manager		
		double worstCost, tmp;
		SceneObjectIterator<SceneType> it(m_scene);
		it.GoToBegin();
		m_label = it.GetID(); m_manager->SelectObject( it.GetID() ); worstCost = m_manager->GetValue();
		++it;
		while (!it.IsAtEnd()) {
			m_manager->SelectObject( it.GetID() );
			tmp = m_manager->GetValue();
			if (tmp>worstCost) {
				worstCost = tmp;
				m_label = it.GetID();
			}
			++it;
		}
		m_manager->SelectObject(m_label);
		return true;
	}

	//temperature is given as an argument ; is it usefull in this case??? not so sure...
	double Apply(double T) {
		if (!m_manager) throw DeformableModelException("SingleObjectOptimizationKernel::Apply : Must define a SingleObjectOptimizationManager first! ");
		if ( m_scene->GetNumberOfObjects()==0 ) return -1;
		if (!SelectWorstObject()) return -1;
		//remember the previous parameters
		m_params = m_manager->GetParameters();
		//do the modification
//		clock_t t0=clock();

		m_manager->Optimize();
//std::cout<<"optimizing worst object, label: "<<m_label<<", initial params: "<<m_params<<", final params: "<<m_manager->GetParameters()<<", time to optimize: "<<(clock()-t0)/((double)CLOCKS_PER_SEC)<<" s."<<std::endl;
		return 1;	//this is always OK? or maybe return false if no improvement happened
	}

	//move the scene to the state it used to have the last time we called Apply
	void Undo() { //return the requested object to its original parameters.
		m_manager->SelectObject(m_label);	//shouldn't be necessary ...
		if (!m_manager->SetParameters(m_params)) throw DeformableModelException("SingleObjectOptimizationKernel::Undo : unable to undo the move properly -- SHOULD NEVER HAPPEN");
	}

protected:
	WorstObjectOptimizationKernel() : ReversibleSceneModifierKernel_Base() { m_manager = 0; };
	~WorstObjectOptimizationKernel() {};

	typename OptimizationManagerType::Pointer m_manager;
	IDType m_label;
	vnl_vector<double> m_params;

private:
	WorstObjectOptimizationKernel(const Self&);		//purposely not implemented
	const Self & operator=( const Self & );				//purposely not implemented
};


} // namespace psciob

#endif /* _SINGLEOBJECTOPTIMIZATIONKERNEL_H_ */
