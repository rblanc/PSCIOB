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
* \file FBMovementManager.h
* \author Rémi Blanc 
* \date 31. July 2012
*/


#ifndef __FBMOVEMENTMANAGER_H_
#define __FBMOVEMENTMANAGER_H_

#include "BaseScene.h"
#include "ForceBiasedAlgorithm.h"

namespace psciob {

/** \brief FBMovementManager
*
* This is the default movement manager for the ForceBiasedAlgorithm, which only proposes translations
*/

//DEV: Child classes should implement a system similar to the ObjectTypesLibrary, to associate 

template<class TScene>
class FBMovementManager : public itk::LightObject {
public:
	/** Standard class typedefs. */
	typedef FBMovementManager       Self;
	typedef itk::LightObject        Superclass;
	typedef itk::SmartPointer<Self> Pointer;
	/** Run-time type information (and related methods). */
	itkTypeMacro(FBMovementManager, itk::LightObject);
	itkNewMacro(Self);

	typedef TScene                                     SceneType;
	typedef typename SceneType::IDType                 IDType;
	typedef typename SceneType::ObjectTypesLibraryType ObjectTypesLibraryType;
	typedef typename SceneType::DeformableObjectType   ObjectType;

	/** Attach a scene to the algorithm */
	void SetScene(SceneType* scene) { m_scene = scene; }

	/** Set the maximum number of iterations for the method IterateUntilConvergence - default is 1e10 */
	void SetMaximumNumberOfIterations(unsigned n) {m_maxNbIteration=n;}

	/** Get the maximum number of iterations for the method IterateUntilConvergence - default is 1e10 */
	unsigned GetMaximumNumberOfIterations() { return m_maxNbIteration;}
	
	/** Set the scaling coefficient, to be applied at the end of each iteration 
	* Values should be in ]0,1] ; if the input is outside this range, value 1 is set instead
	* If the value is 1 (default behavior), no scaling is applied
	* Otherwise, it should be specified, for each object type, which is the coefficient that performs scaling... this is done through the object movement manager...
	*/
	void SetScalingAmount(double s) {
		if (s<=0) {m_scaleFactor=1;return;}
		if (s>=1) {m_scaleFactor=1;return;}
		m_scaleFactor = s;
	}
	
	/** Get the value of the scaling parameter */
	double GetScalingAmount() {return m_scaleFactor;}


	/** Applies a single iteration of the algorithm */
	bool ApplyOneIteration() {
		
	
	}
	
	/** Iterate until convergence 
	* returns -1 if it exited after reaching the maximum number of allowed iterations
	* and 1 otherwise.
	*/
	int IterateUntilConvergence() {
		if (!m_scene) throw DeformableModelException("FBMovementManager::IterateUntilConvergence -- the scene must be set first.");
		unsigned nbIter=0;
		int converged = 0;
		while (converged==0) {
			nbIter++;
			if (nbIter>=m_maxNbIteration) { converged = -1; break; }			
			if (!this->ApplyOneIteration()) converged = 1;			
		}
		return converged;
	}


protected:
	FBMovementManager() : m_scaleFactor(1), m_translationFactor(1) {
		m_scene = 0;
	};
	~FBMovementManager() {};

	typename SceneType::Pointer m_scene;
	double m_scaleFactor;
	double m_translationFactor;

private:
	FBMovementManager(const Self&);      //purposely not implemented
	const Self & operator=( const Self & ); //purposely not implemented
};





} // namespace psciob

#endif /* __FBMOVEMENTMANAGER_H_ */
