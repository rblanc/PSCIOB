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
* \file SceneOptimizationManager_Base.h
* \author Rémi Blanc 
* \date 9. September 2011
*/

#ifndef SCENEOPTIMIZATIONMANAGER_BASE_H_
#define SCENEOPTIMIZATIONMANAGER_BASE_H_

#include "OptimizationManager_Base.h"

namespace psciob {

/** \brief SceneOptimizationManager_Base
 * Base Classe for Managing Scene objects during Optimization - implements the functionalities for communication between these classes and the scene
 * abstract methods to select what needs to be optimized, to formulate the cost function for the real optimizer, etc...
*/

//ABSTRACT
template<class TScene>
class SceneOptimizationManager_Base : public OptimizationManager_Base {
public:

	/** Standard class typedefs. */
	typedef SceneOptimizationManager_Base	Self;
	typedef OptimizationManager_Base		Superclass;
	typedef itk::SmartPointer<Self>			Pointer;
	/** Run-time type information (and related methods). */
	itkTypeMacro(SceneOptimizationManager_Base,OptimizationManager_Base);

	typedef TScene                                SceneType;
	typedef typename TScene::DeformableObjectType ObjectType;
	typedef typename TScene::IDType               IDType;

	/** Set the scene to optimize */
	virtual void SetScene(SceneType* scene) { 
		m_scene = scene; 
		if (!m_costFunction) {} else {
			//try to reset the cost function to make it work on the same scene, if it derives from SceneCostFunction_Base
			SceneCostFunction_Base<SceneType>* tmp = dynamic_cast<SceneCostFunction_Base<SceneType>*>(m_costFunction.GetPointer());
			if (!tmp) {} else {tmp->SetScene( m_scene );}
		}
	}

	/** Set the cost function to optimize */
	void SetCostFunction( CostFunction_Base *costFunction )	{ 
		Superclass::SetCostFunction(costFunction);
		//reset the cache for data cost terms for all objects in the scene
		//m_scene->InvalidateObjectDataCosts();

		SceneCostFunction_Base<SceneType>* tmp = dynamic_cast<SceneCostFunction_Base<SceneType>*>(m_costFunction.GetPointer());
		if (!tmp) {} else {m_scene = tmp->GetScene();}
	}

	/** Get the scene */
	SceneType* GetScene()				{ return m_scene.GetPointer(); }

protected:
	SceneOptimizationManager_Base() {};
	virtual ~SceneOptimizationManager_Base() {};	

	typename SceneType::Pointer m_scene;

private:
	SceneOptimizationManager_Base(const Self&);	//purposely not implemented
	const Self & operator=( const Self & );		//purposely not implemented
};



} // namespace psciob

#endif /* SCENEOPTIMIZATIONMANAGER_BASE_H_ */
