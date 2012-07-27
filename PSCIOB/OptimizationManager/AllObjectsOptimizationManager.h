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
 * \file AllObjectsOptimizationManager.h
 * \author Rémi Blanc 
 * \date 21. September 2011
 */


#ifndef ALLOBJECTSOPTIMIZATIONMANAGER_H_
#define ALLOBJECTSOPTIMIZATIONMANAGER_H_


#include "SceneOptimizationManager_Base.h"

namespace psciob {

/** \brief AllObjectsOptimizationManager
 * consider the optimization of all parameters, of all objects at once.
 * however, the objects are permanent, cannot appear or disappear, nor change type.
*/

template<class TScene>
class AllObjectsOptimizationManager : public SceneOptimizationManager_Base<TScene> {
public:
	/** Standard class typedefs. */
	typedef AllObjectsOptimizationManager		Self;
	typedef SceneOptimizationManager_Base		Superclass;
	typedef itk::SmartPointer<Self>				Pointer;
	/** Run-time type information (and related methods). */
	itkTypeMacro(AllObjectsOptimizationManager,SceneOptimizationManager_Base);
	itkNewMacro(Self);


	inline unsigned int GetNumberOfParameters()		{ return m_scene->GetNumberOfParameters(); }

	//OPTIMIZATION? this piece of code seems much inefficient... however, the number of parameters may change between different calls
	inline vnl_vector<double> GetParameters() { //concatenate the parameters of all the objects of the scene
		vnl_vector<double> params(this->GetNumberOfParameters()), tmp; unsigned n=0;
		SceneObjectIterator<SceneType> it(m_scene);
		for (it.GoToBegin() ; !it.IsAtEnd() ; ++it) {
			tmp = m_scene->GetParametersOfObject( it.GetID() );
			for (unsigned j=0 ; j<tmp.size() ; j++)	{ params(n) = tmp(j); n++;}
		}
		return params; 
	}

	//
	bool SetParameters(vnl_vector<double> p) { //
		if (p.size()!=GetNumberOfParameters()) return false;
		SceneType::IDType label;
		unsigned pos = 0, i;
		std::vector< vnl_vector<double> > oldParams;
		vnl_vector<double> objectParams;
		SceneObjectIterator<SceneType> objectIt(m_scene);
		for (objectIt.GoToBegin(), i=0 ; !objectIt.IsAtEnd() ; ++objectIt, ++i) {
			objectParams = m_scene->GetParametersOfObject( objectIt.GetID() );
			oldParams.push_back( objectParams ); //backit up in case it is necessary to rewind...
			
			//replace the object params by the new ones.
			objectParams = p.extract( objectParams.size(), pos );
			pos+=objectParams.size();

			if ( !m_scene->ModifyObjectParameters( objectIt.GetID(), objectParams ) ) {
				//if something goes wrong, rewind all modifications... and return false
				std::cout<<"invalid parameters - rewinding all modifications..."<<std::endl;
				oldParams.pop_back(); --objectIt;
				for (std::vector< vnl_vector<double> >::reverse_iterator rit = oldParams.rbegin() ; rit!=oldParams.rend() ; ++rit, --objectIt) {
					if ( !m_scene->ModifyObjectParameters( objectIt.GetID(), *rit) ) throw DeformableModelException("SceneStateOptimizationManager::SetParameters : unable to rewing the modifications ; SHOULD NEVER HAPPEN");
				}
				return false;
			}
		}
		return true;
	}

	/** get a multivariate pdf for all parameters of all objects - same as the parameter list, cannot be realistically maintained by the scene - too much overhead only for this class.
	*however, it should be called only once within a single optimization run! => the optimizer should keep it stored.
	*/
	MultivariatePDF* GetParameterPriorPDF() const {
		IndependentPDFs::Pointer pdf = IndependentPDFs::New();			
		SceneObjectIterator<SceneType> objectIt(m_scene);
		for (objectIt.GoToBegin() ; !objectIt.IsAtEnd() ; ++objectIt) {
			pdf->AddMultivariatePDF( m_scene->GetObjectTypesLibrary()->GetObjectPDF( objectIt.GetObject()->objectTypeId , PDF_OBJECTGENERATIONPRIOR) );
		}
		return m_priorPDF.GetPointer();
	}

	/** get a multivariate pdf for all parameters of all objects - same as the parameter list, cannot be realistically maintained by the scene - too much overhead only for this class.
	* however, it should be called only once within a single optimization run! => the optimizer should keep it stored.
	*/
	MultivariatePDF* GetParameterProposalMovePDF() {
		IndependentPDFs::Pointer pdf = IndependentPDFs::New();
		SceneObjectIterator<SceneType> objectIt(m_scene);
		for (objectIt.GoToBegin() ; !objectIt.IsAtEnd() ; ++objectIt) {
			pdf->AddMultivariatePDF( m_scene->GetObjectTypesLibrary()->GetObjectPDF( objectIt.GetObject()->objectTypeId , PDF_RANDOMWALK) );
		}
		m_priorPDF = static_cast<MultivariatePDF::Pointer>(pdf);
		return m_priorPDF.GetPointer();
	}

	//CLEAN: access should be restricted to some optimizers ... use friendship?
	void SaveCurrentStateAsBest()		{ m_bestCost = this->GetValue(); m_bestKnownParameters = this->GetParameters(); }
protected:
	AllObjectsOptimizationManager() : SceneOptimizationManager_Base() {};
	~AllObjectsOptimizationManager() {};

	MultivariatePDF::Pointer m_priorPDF, m_proposalPDF;

	vnl_vector<double> m_bestKnownParameters;
	double ReturnToBestKnownState()		{ this->SetParameters(m_bestKnownParameters); return this->GetValue(); }

private:
	AllObjectsOptimizationManager(const Self&);	//purposely not implemented
	const Self & operator=( const Self & );		//purposely not implemented

};



} // namespace psciob

#endif /* ALLOBJECTSOPTIMIZATIONMANAGER_H_ */
