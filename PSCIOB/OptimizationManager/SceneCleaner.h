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
 * \file SceneCleaner.h
 * \author Rémi Blanc 
 * \date 21. September 2011
 */


#ifndef SCENECLEANER_H_
#define SCENECLEANER_H_

#include "SceneStateOptimizationManager.h"

namespace psciob {

/** \brief SceneCleaner
 * browses objects sequentially, and kill it if this improves the associated cost function
*/


template<class TScene>
class SceneCleaner : public SceneStateOptimizationManager<TScene> {
public:
	/** Standard class typedefs. */
	typedef SceneCleaner						Self;
	typedef SceneStateOptimizationManager		Superclass;
	typedef itk::SmartPointer<Self>				Pointer;
	/** Run-time type information (and related methods). */
	itkTypeMacro(SceneCleaner,SceneStateOptimizationManager);
	itkNewMacro(Self);


	//overloads the base optimize method... 
	double Optimize() { KillUnnecessaryObjects(); return GetValue(); }

	void KillUnnecessaryObjects() {
		SCENEDRAWMODE backup_insertionMode;	
		unsigned int backup_label;
		unsigned int backup_indexObjectType;
		vnl_vector<double> backup_params;
		double backup_value;

		std::vector<SceneType::LabelType> listLabels;

		for (unsigned i=0 ; i<m_scene->GetNumberOfObjects() ; i++) { listLabels.push_back( m_scene->GetLabelFromIndex(i) ); }
		//for (it = m_scene->GetObjectSet()->begin() ; it!=m_scene->GetObjectSet()->end() ; it++) { listLabels.push_back(it->first); }

		//OPTIMIZATION: could hold on the object before deciding to destroy it, instead of backing-up its parameters and recreating it
		backup_value = GetValue();
		for (unsigned i=0 ; i<listLabels.size() ; i++) {
			if (m_scene->GetNumberOfObjects() <= 1) break;
			//it = m_scene->GetIteratorFromLabel(listLabels[i]);
			//backup the object
			backup_indexObjectType = m_scene->GetObjectByLabel(listLabels[i])->indexObjectType;	
			backup_label = m_scene->GetObjectByLabel(listLabels[i])->label;
			backup_insertionMode = m_scene->GetObjectByLabel(listLabels[i])->insertionMode;
			backup_params = m_scene->GetParametersOfObject(listLabels[i]);
			//kill it
			m_scene->RemoveObject(backup_label);

			if ( GetValue() > backup_value + TINY ) {//if necessary, re-insert this object in the scene
				ObjectType::Pointer obj = m_scene->GetObjectTypesLibrary()->GenerateNewObjectDefault(backup_indexObjectType);
				obj->SetParameters(backup_params); 
				if ( m_scene->AddObject(obj, backup_label) != backup_label ) throw DeformableModelException("SceneCleaner::KillUnnecessaryObjects : failed to reinsert correctly the object --- SHOULD NEVER HAPPEN");
			}	
		}

	}

protected:
	SceneCleaner() : SceneStateOptimizationManager() {};
	~SceneCleaner() {};

private:
	SceneCleaner(const Self&);					//purposely not implemented
	const Self & operator=( const Self & );		//purposely not implemented
};



} // namespace psciob

#endif /* SCENECLEANER_H_ */
