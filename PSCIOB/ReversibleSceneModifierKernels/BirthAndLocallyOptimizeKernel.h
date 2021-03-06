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
* \file BirthAndLocallyOptimizeKernel.h
* \author R�mi Blanc 
* \date 20. June 2012
*/



#ifndef __BIRTHANDOPTIMIZEOFFCONTEXTKERNEL_H_
#define __BIRTHANDOPTIMIZEOFFCONTEXTKERNEL_H_

#include <KernelWithLocalSingleObjectOptimizer_Base.h>
#include "SingleSceneObjectOptimizationManager.h"


namespace psciob {

/**\class BirthAndLocallyOptimizeKernel
* \brief BirthAndLocallyOptimizeKernel
* This kernel works in 2 steps:
* 1/ Birth: A random number of new objects is added to the scene, according to the GenerationPrior 
* 2/ Optimize: the parameters of this objects are optimized locally, without caring about any other objects in the scene, using a given optimization strategy.
* \sa SingleSceneObjectOptimizationManager
*/

/* ideas: (code efficiency) in case interactions are not taken into account during this optimization, it would be good to avoid the interaction computation overhead generated by the object manipulation within the scene.
* there are 2 options for this:
*     1: create a temporary scene in which to perform the optimization (but check how this would work with sensors, cost functions, etc... ???) 
*     2: switch interaction computation off, then on ... 
*/
template<class TScene>
class BirthAndLocallyOptimizeKernel : public KernelWithLocalSingleObjectOptimizer_Base<TScene> {
public:
	/** Standard class typedefs. */
	typedef BirthAndLocallyOptimizeKernel             Self;
	typedef KernelWithLocalSingleObjectOptimizer_Base Superclass;
	typedef itk::SmartPointer<Self>                   Pointer;
	/** Run-time type information (and related methods). */
	itkTypeMacro(BirthAndLocallyOptimizeKernel, KernelWithLocalSingleObjectOptimizer_Base);
	itkNewMacro(Self);

	/** Checks that all the parameters of the kernel have been set properly (e.g. the scene...) */
	virtual bool CheckKernelValidity() {
		Superclass::CheckKernelValidity();
		if (m_library->GetNumberOfEntries()==0) { throw DeformableModelException("BirthAndLocallyOptimizeKernel is not valid: no object type present in the scene library"); return false; }
		for (unsigned i=0 ; i<m_library->GetNumberOfEntries() ; i++) {
			m_library->GetObjectPDF( i, PDF_OBJECTGENERATIONPRIOR ); //this function throws an exception if the PDF is not set... TODO: improve the message...
		}
		return true;
	}

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
//		ObjectType::Pointer obj = m_library->GenerateNewRandomObject(m_requestedType);

		//3: add it object to the local scene
		IDType tmpLabel = m_localScene->AddObject(  m_library->GenerateNewRandomObject(m_requestedType) );	// what about specific object insertion policies??	//should child classes of Scene_Base force specific policies (e.g. label map ; or binary map <-> boolean model ; or ADD mode <-> shot noise, ...)

		if (tmpLabel==0) { std::cout<<"FAILED TO INSERT THE OBJECT IN AN EMPTY SCENE... THIS SHOULD NEVER HAPPEN!!"<<std::endl; return -1; } //this should NEVER happen, but...
		//4: optimize the parameters of this object wrt the provided optimizer
		m_localOptimizationManager->SelectObject(tmpLabel);
		m_localOptimizationManager->Optimize();

		//add the optimized object into the real scene, and clean the local scene.
		//obj->SetParameters(m_localScene->GetParametersOfObject(tmpLabel));
		m_labelToUndo = m_scene->AddObject( m_localScene->GetObject(tmpLabel)->obj );
		m_localScene->RemoveObject(tmpLabel);

		if (!m_labelToUndo) {return -1; } //for some reason, the insertion failed ; this should probably never happen, but...

		return 1;
	}

	//move the scene to the state it used to have the last time we called Apply
	void Undo() {
		m_scene->RemoveObject(m_labelToUndo);
	}



protected:
	BirthAndLocallyOptimizeKernel() : KernelWithLocalSingleObjectOptimizer_Base() {};
	~BirthAndLocallyOptimizeKernel() {};

	unsigned int m_requestedType;
	IDType m_labelToUndo;

private:
	BirthAndLocallyOptimizeKernel(const Self&);		//purposely not implemented
	const Self & operator=( const Self & );		//purposely not implemented
};




} // namespace psciob

#endif /* __BIRTHANDOPTIMIZEOFFCONTEXTKERNEL_H_ */




