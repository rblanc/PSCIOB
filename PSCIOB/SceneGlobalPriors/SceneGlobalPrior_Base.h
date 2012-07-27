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
 * \file SceneGlobalPrior_Base.h
 * \author Rémi Blanc 
 * \date 25. October 2011
 */

#ifndef SCENEGLOBAPRIOR_BASE_H_
#define SCENEGLOBAPRIOR_BASE_H_


//#include <CommonTypes.h>
#include <stack>

namespace psciob {

/**\class SceneGlobalPrior_Base 
 * \brief SceneGlobalPrior_Base
 * base class for associating a global prior to a scene
 * just computes a scalar out of a scene
 * 
 * By convention, a high value returned by ComputeGlobalPrior() should be penalized, because the optimizers are minimizers...
 * 
 * IDEA: specialized versions could also make use of the scene library of objects types
 * to give e.g. priors concerning specific object types (twice as more objects of type 1 then of type 2, ...)
 *
 * In general, for undesired values, make a very steep cost function, it is important to avoid threshold effects, so that a direction can be given (better / worse)
 * to know if more, or less objects are required when starting outside the requested density range...
 *
*/


//ABSTRACT CLASS
template<class TScene>
class SceneGlobalPrior_Base : public itk::LightObject {
public:
	/** Standard class typedefs. */
	typedef SceneGlobalPrior_Base			Self;
	typedef itk::LightObject				Superclass;
	typedef itk::SmartPointer<Self>			Pointer;
	typedef itk::SmartPointer<const Self>	ConstPointer;
	/** Base Types to which all inherited classes can refer */
	typedef Self                    BaseClass;
	typedef Pointer                 BaseClassPointer;
	/** Run-time type information (and related methods). */
	itkTypeMacro(SceneGlobalPrior_Base,itk::LightObject);

	typedef TScene        SceneType;

	typedef typename SceneType::DeformableObjectType DeformableObjectType;
	typedef typename SceneType::IDType               IDType;
	typedef typename SceneType::ObjectInScene        ObjectInScene;


	/** Create a clone (= an exact, independant copy) of the current object */
	virtual BaseClassPointer CreateClone() {
		BaseClassPointer clonePtr = static_cast<Self*>(this->CreateAnother().GetPointer());
		if (!m_scene) {} else { clonePtr->SetScene(m_scene); }
		clonePtr->m_globalPrior = m_globalPrior;
		clonePtr->m_globalPriorFlag = m_globalPriorFlag;
		return clonePtr;
	}

	/** Set the reference scene	*/
	void SetScene(SceneType* scene) {
		m_scene = scene;
		m_globalPriorFlag=false;
	}

	///** if true, the output value is updated each time the scene is modified 
	//* otherwise, it has to recomputed from scratch in case of modifications
	//*/
	//void KeepUpToDate(bool b = true) {m_keepUpdated = b;}

	inline double GetValue() {
		if (!m_globalPriorFlag) { m_globalPrior = ComputeGlobalPrior_Internal(); m_globalPriorFlag=true; }
		return m_globalPrior;
	}

	/** Clears the prior */
	void Clear() {
		//if (m_keepUpdated) GetValue();
		//else m_globalPriorFlag=false;
		m_globalPriorFlag=false;
	}

	/** This function should only be called by the scene - use friendship instead of leaving the method public*/
	virtual void AddObject(ObjectInScene *objectPtr) = 0;

	/** This function should only be called by the scene - use friendship instead of leaving the method public*/
	virtual void RemoveObject(ObjectInScene *objectPtr) = 0;

	/** This function should only be called by the scene - use friendship instead of leaving the method public*/
	virtual void ModifyObjectParameters(ObjectInScene *objectPtr, ObjectInScene *newObject) = 0;

protected:	
	SceneGlobalPrior_Base() : m_globalPriorFlag(false) { }
	virtual ~SceneGlobalPrior_Base() {};	

	virtual double ComputeGlobalPrior_Internal() = 0;

	typename SceneType::Pointer m_scene;
	double m_globalPrior;
	bool m_globalPriorFlag;

private:
	SceneGlobalPrior_Base(const Self&);		//purposely not implemented
	const Self & operator=( const Self & );	//purposely not implemented
};



} // namespace psciob

#endif /* SCENEGLOBAPRIOR_BASE_H_ */
