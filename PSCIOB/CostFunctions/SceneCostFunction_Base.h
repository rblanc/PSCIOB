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
 * \file SceneCostFunction_Base.h
 * \author Rémi Blanc 
 * \date 25. October 2011
*/

#ifndef SceneCostFunction_Base_H_
#define SceneCostFunction_Base_H_


#include "CostFunction_Base.h"
#include "BaseScene.h"


namespace psciob {


/** 
 * \class SceneCostFunction_Base
 * \brief Base class for associating a cost / energy / potential ... to a scene
 */

template<class TScene>
class SceneCostFunction_Base : public CostFunction_Base {
public:
	/** Standard class typedefs. */
	typedef SceneCostFunction_Base			Self;
	typedef CostFunction_Base				Superclass;
	typedef itk::SmartPointer<Self>			Pointer;
	/** Run-time type information (and related methods). */
	itkTypeMacro(SceneCostFunction_Base, CostFunction_Base);

	static const unsigned int SceneDimension       = TScene::Dimension;
	typedef TScene                                   SceneType;
	typedef typename SceneType::IDType               IDType;
	typedef typename SceneType::DeformableObjectType ObjectType;
	typedef typename SceneType::ObjectInScene        ObjectInScene;

	/** Create a clone (= an exact, independant copy) of the current cost function */
	virtual BaseClassPointer CreateClone() {
		Pointer clonePtr = static_cast<Self*>(this->CreateAnother().GetPointer());
		if (!m_transformFunction) {} else {clonePtr->SetNormalizationFunction(m_transformFunction);}
		if (!m_scene) {} else {clonePtr->SetScene(m_scene);}
		clonePtr->SelectObject(m_requestedLabel);
		return static_cast<BaseClass*>( clonePtr );
	}

	/** Set the scene on which to make the measurements 
	* maybe overloaded by child classes
	*/
	virtual void SetScene(SceneType *scene) {
		m_scene = scene; 
		m_requestedLabel = m_scene->FirstID();
	}
	SceneType *GetScene()			{ 
		if (!m_scene) throw DeformableModelException("Exception in SceneCostFunction_Base : no scene has been attached to the cost function !");
		return m_scene.GetPointer(); 
	}

	inline void SelectObject( IDType label ) { m_requestedLabel = label; }

protected:
	SceneCostFunction_Base() : CostFunction_Base() { m_scene = 0; };
	virtual ~SceneCostFunction_Base() {};	

	typename SceneType::Pointer m_scene;
	IDType m_requestedLabel;

private:
	SceneCostFunction_Base(const Self&);		//purposely not implemented
	const Self & operator=( const Self & );		//purposely not implemented
};

} // namespace psciob

#endif /* SCENEENERGY_H_ */
