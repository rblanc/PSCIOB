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
* \file KernelWithLocalSingleObjectOptimizer_Base.h
* \author Rémi Blanc 
* \date 20. June 2012
*/



#ifndef __KERNELWITHLOCALSINGLEOBJECTOPTIMIZER_BASE_H_
#define __KERNELWITHLOCALSINGLEOBJECTOPTIMIZER_BASE_H_

#include <ReversibleSceneModifierKernel_Base.h>
#include "SingleSceneObjectOptimizationManager.h"

namespace psciob {

/**\class KernelWithLocalSingleObjectOptimizer_Base
* \brief KernelWithLocalSingleObjectOptimizer_Base: this kernel defines a local scene on which it will do some off-context computation (without caring about objects in the real scene
* requests a local optimizer which will be set to work on it
* \warning: the cost functions attached with this local optimizer will be modified to work on the local scene
*           be very careful to set functions that are not used elsewhere...
*       IDEA: perhaps I can internally construct a clone of this cost function instead of forcing the user to care about this aspect...
*/
template<class TScene>
class KernelWithLocalSingleObjectOptimizer_Base : public ReversibleSceneModifierKernel_Base<TScene> {
public:
	/** Standard class typedefs. */
	typedef KernelWithLocalSingleObjectOptimizer_Base  Self;
	typedef ReversibleSceneModifierKernel_Base         Superclass;
	typedef itk::SmartPointer<Self>                    Pointer;
	/** Run-time type information (and related methods). */
	itkTypeMacro(KernelWithLocalSingleObjectOptimizer_Base, ReversibleSceneModifierKernel_Base);

	typedef SingleSceneObjectOptimizationManager<TScene>          LocalOptimizationManagerType;

	/** Set the scene on which the kernel works	
	* creates another scene of the same type to perform internal, off context optimization without computing any interaction.
	*/
	virtual void SetScene(SceneType *scene) {	
		Superclass::SetScene(scene);

		//generate a local scene with the same characteristics, which will be used to perform offcontext optimization without caring about interaction computations...
		m_localScene = SceneType::New(); //TODO: check that they are configured in the same way (insertion policy, etc...)
		m_localScene->SetPhysicalDimensions( scene->GetSceneBoundingBox(), scene->GetSceneSpacing().GetVnlVector() );
		m_localScene->FuseObjectTypesLibrary( scene->GetObjectTypesLibrary() );
		m_localScene->SetGlobalPrior( scene->GetGlobalPrior()->CreateClone() ); //TODO: I should make sure they are properly configured. (if there are parameters to set, ...)
		m_localScene->SetObjectPriorNormalizationFunction( scene->GetObjectPriorNormalizationFunction() );
		//m_localScene->SetTrackChanges( scene->GetTrackChangesStatus() );
		for (SceneType::SensorInterfaceListType::iterator it = m_scene->GetListOfConnectedSensors().begin() ; 
			it != m_scene->GetListOfConnectedSensors().end() ; ++it) {
				m_localScene->ConnectSensor(*it);
		}

		//tell the manager to work with this scene (the manager will propagate this information to the cost function, which in turn will send the message to the sensor, etc...)
		if (!m_localOptimizationManager) {} else { m_localOptimizationManager->SetScene(m_localScene); }
	}	

	/** Set the optimization manager (deriving from SingleSceneObjectOptimizationManager) used for optimization
	* It will be reconfigured to work with the local scene.
	*/
	void SetLocalOptimizationManager(LocalOptimizationManagerType *manager) { 
		m_localOptimizationManager = manager; 

		//make sure the manager works on the right scene, if it has been defined already (not supposed to happen, but keep it here in case...)
		if (!m_localScene) {} else {m_localOptimizationManager->SetScene(m_localScene);}		
	}



	/** Checks that all the parameters of the kernel have been set properly (e.g. the scene...) */
	virtual bool CheckKernelValidity() {
		Superclass::CheckKernelValidity();
		if (!m_localOptimizationManager){ throw DeformableModelException("KernelWithLocalSingleObjectOptimizer_Base is not valid: no associated SingleObjectOptimizationManager"); return false; }
		return true;
	}

protected:
	KernelWithLocalSingleObjectOptimizer_Base() : ReversibleSceneModifierKernel_Base() { m_localOptimizationManager = NULL; };
	~KernelWithLocalSingleObjectOptimizer_Base() {};

	typename LocalOptimizationManagerType::Pointer m_localOptimizationManager;
	typename SceneType::Pointer m_localScene; //local scene to perform off-context optimization, without caring about interaction estimation.

private:
	KernelWithLocalSingleObjectOptimizer_Base(const Self&);		//purposely not implemented
	const Self & operator=( const Self & );		//purposely not implemented
};




} // namespace psciob

#endif /* __KERNELWITHLOCALSINGLEOBJECTOPTIMIZER_BASE_H_ */
