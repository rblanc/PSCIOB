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
* \file BirthAndOptimizeKernel.h
* \author Rémi Blanc 
* \date 20. June 2012
*/



#ifndef __BIRTHANDOPTIMIZEKERNEL_H_
#define __BIRTHANDOPTIMIZEKERNEL_H_

#include <ReversibleSceneModifierKernel_Base.h>
#include "SingleSceneObjectOptimizationManager.h"


namespace psciob {

/**\class BirthAndOptimizeKernel
* \brief BirthAndOptimizeKernel
* This kernel works in 2 steps:
* 1/ Birth: A random number of new objects is added to the scene, according to the GenerationPrior 
* 2/ Optimize: the parameters of this objects are optimized locally, using the specified optimization manager.
* \sa SingleSceneObjectOptimizationManager
*/
template<class TScene>
class BirthAndOptimizeKernel : public ReversibleSceneModifierKernel_Base<TScene> {
public:
	/** Standard class typedefs. */
	typedef BirthAndOptimizeKernel  Self;
	typedef ReversibleSceneModifierKernel_Base                     Superclass;
	typedef itk::SmartPointer<Self>           Pointer;
	/** Run-time type information (and related methods). */
	itkTypeMacro(BirthAndOptimizeKernel, ReversibleSceneModifierKernel_Base);
	itkNewMacro(Self);

	typedef SingleSceneObjectOptimizationManager<TScene>          OptimizationManagerType;

	/** Checks that all the parameters of the kernel have been set properly (e.g. the scene...) */
	virtual bool CheckKernelValidity() {
		Superclass::CheckKernelValidity();
		if (m_library->GetNumberOfEntries()==0) { throw DeformableModelException("BirthAndOptimizeKernel is not valid: no object type defined in the scene library"); return false; }
		for (unsigned i=0 ; i<m_library->GetNumberOfEntries() ; i++) {
			m_library->GetObjectPDF( i, PDF_OBJECTGENERATIONPRIOR ); //this function throws an exception if the PDF is not set... TODO: improve the message...
		}
		if (!m_manager){ throw DeformableModelException("BirthAndOptimizeKernel is not valid: no associated SingleObjectOptimizationManager"); return false; }
		return true;
	}


	/** Set the optimization manager (deriving from SingleSceneObjectOptimizationManager) used for optimization
	* The cost function and actual optimizer used must be defined.
	*/
	void SetOptimizationManager(OptimizationManagerType *manager) { m_manager = manager; }

	/** Select the type of object to generate */
	inline void SelectType() { 
		m_requestedType = 0;
		if (m_library->GetNumberOfEntries()==1) return;
		double tmp = m_rndgen->GetUniformVariate(0, m_library->GetSumTypeWeights());
		while (tmp>0) {
			if ( tmp < m_library->GetObjectEntry(m_requestedType)->weight ) break;
			tmp-=m_library->GetObjectEntry(m_requestedType)->weight; m_requestedType++;
		}
	}


	//temperature is given as an argument ; in case this is useful for deciding the amplitude of the "move"...
	double Apply(double T) {
		//1: decide which type of object to generate - look at the scene registered types
		SelectType();
		//2: generate a object of the requested type, with random parameters from the given distribution
		ObjectType::Pointer obj = m_library->GenerateNewRandomObject(m_requestedType);

		//3: add it object to the scene
		m_labelToUndo = m_scene->AddObject(obj);	// what about specific object insertion policies??	//should child classes of Scene_Base force specific policies (e.g. label map ; or binary map <-> boolean model ; or ADD mode <-> shot noise, ...)
		if (m_labelToUndo==0) { return -1; } //for some reason, the insertion failed ; this should probably never happen, but...
		//4: optimize the parameters of this object wrt the provided optimizer
		m_manager->SelectObject(m_labelToUndo);
		m_manager->Optimize();
		return 1;
	}

	//move the scene to the state it used to have the last time we called Apply
	void Undo() {
		m_scene->RemoveObject(m_labelToUndo);
	}



protected:
	BirthAndOptimizeKernel() : ReversibleSceneModifierKernel_Base() { m_manager = 0; };
	~BirthAndOptimizeKernel() {};

	typename OptimizationManagerType::Pointer m_manager;
	unsigned int m_requestedType;
	IDType m_labelToUndo;

private:
	BirthAndOptimizeKernel(const Self&);		//purposely not implemented
	const Self & operator=( const Self & );		//purposely not implemented
};




} // namespace psciob

#endif /* __BIRTHANDOPTIMIZEKERNEL_H_ */




