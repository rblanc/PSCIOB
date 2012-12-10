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
 * \file SceneStateOptimizationManager.h
 * \author Rémi Blanc 
 * \date 21. September 2011
*/


#ifndef SCENESTATEOPTIMIZATIONMANAGER_H_
#define SCENESTATEOPTIMIZATIONMANAGER_H_

#include "SceneOptimizationManager_Base.h"

namespace psciob {

/** \brief SceneStateOptimizationManager
 * consider the optimization of the scene as a whole
 * the number of objects may change, their type also, etc...
*/ 

template<class TScene>
class SceneStateOptimizationManager : public SceneOptimizationManager_Base<TScene> {
public:
	/** Standard class typedefs. */
	typedef SceneStateOptimizationManager		Self;
	typedef SceneOptimizationManager_Base		Superclass;
	typedef itk::SmartPointer<Self>				Pointer;
	/** Run-time type information (and related methods). */
	itkTypeMacro(SceneStateOptimizationManager, SceneOptimizationManager_Base);
	itkNewMacro(Self);

	//void SetScene(SceneType* scene)	{ 
	//	m_scene = scene; 
	//}

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
				oldParams.pop_back(); --objectIt;
				for (std::vector< vnl_vector<double> >::reverse_iterator rit = oldParams.rbegin() ; rit!=oldParams.rend() ; ++rit, --objectIt) {
					if ( !m_scene->ModifyObjectParameters( objectIt.GetID(), *rit) ) throw DeformableModelException("SceneStateOptimizationManager::SetParameters : unable to rewing the modifications ; SHOULD NEVER HAPPEN");
				}
				return false;
			}
		}
		return true;
	}

	////OPTIMIZATION? this piece of code seems much inefficient... however, the number of parameters may change between different calls
	//inline vnl_vector<double> GetParameters() { //concatenate the parameters of all the objects of the scene
	//	vnl_vector<double> params(this->GetNumberOfParameters()), tmp; unsigned n=0;
	//	for (unsigned i=0 ; i<m_scene->GetNumberOfObjects() ; i++) {
	//		tmp = m_scene->GetParametersOfObject( m_scene->GetLabelFromIndex(i) );
	//		for (unsigned j=0 ; j<tmp.size() ; j++)	{ params(n) = tmp(j); n++;}
	//	}
	//	return params; 
	//}

	////
	//bool SetParameters(vnl_vector<double> p) {
	//	SceneType::LabelType label;
	//	unsigned nbParams, pos = 0;
	//	vnl_vector<double> oldParams(this->GetNumberOfParameters()), tmpParams;
	//	for (unsigned i=0 ; i<m_scene->GetNumberOfObjects() ; i++) {
	//		label = m_scene->GetLabelFromIndex(i);
	//		tmpParams = m_scene->GetParametersOfObject( label );
	//		for (unsigned j=0 ; j<tmpParams.size() ; j++)	{ oldParams(pos) = tmpParams(j); pos++;}
	//		if ( !m_scene->ModifyObjectParameters( label, tmpParams ) ) {
	//			//if something goes wrong, rewing the modifications... and return false
	//			std::cout<<"invalid parameters - rewinding all modifications..."<<std::endl;
	//			pos-=tmpParams.size();
	//			for (int ii=static_cast<int>(i)-1 ; ii>=0 ; ii--) {
	//				label = m_scene->GetLabelFromIndex(ii);
	//				tmpParams = m_scene->GetParametersOfObject( label );
	//				for (int j=tmpParams.size()-1 ; j>=0 ; j--) { pos--; tmpParams(j) = oldParams(pos); }
	//				if ( !m_scene->ModifyObjectParameters( label, tmpParams ) ) throw DeformableModelException("SceneStateOptimizationManager::SetParameters : unable to rewing the modifications ; SHOULD NEVER HAPPEN");
	//			}
	//			return false;
	//		}
	//	}
	//	return true;
	//}

	////OPTIMIZATION: identify all objects which have different parameters and remove/add only these!
	////try to use m_scene->ModifyParameters	instead of this...
	//bool SetParameters(vnl_vector<double> p) { 	
	//	SceneType::ObjectSetType *old_objectSet = new SceneType::ObjectSetType(); m_scene->GetObjectSetCopy(old_objectSet);
	//	SceneType::ObjectSetType *new_objectSet = new SceneType::ObjectSetType(); m_scene->GetObjectSetCopy(new_objectSet);

	//	//insert the new objects one by one...
	//	m_scene->RemoveAllObjects();
	//	vnl_vector<double> new_params;
	//	unsigned n=0, nb, i=0;
	//	for (SceneType::ObjectSetIteratorType it = new_objectSet->begin() ; it!=new_objectSet->end() ; it++, i++) {
	//		nb = it->second->object->GetNumberOfParameters();
	//		new_params = p.extract( nb, n );

	//		it->second->object->SetParameters(new_params); 
	//		if ( m_scene->AddObject( it->second->object ) == 0 ) {// if something goes wrong with one of the object, fall back to the initial state and exit
	//			m_scene->RemoveAllObjects();
	//			m_scene->SetObjectSet(old_objectSet);
	//			return false;
	//		}
	//		n+=nb;
	//	}
	//	delete old_objectSet;
	//	delete new_objectSet;		
	//	return true;
	//}


	/** get a multivariate pdf for all parameters of all objects - same as the parameter list, cannot be realistically maintained by the scene - too much overhead only for this class.
	*however, it should be called only once within a single optimization run! => the optimizer should keep it stored.
	*/
	MultivariatePDF* GetParameterPriorPDF() const {
		IndependentPDFs::Pointer pdf = IndependentPDFs::New();			
		SceneObjectIterator<SceneType> objectIt(m_scene);
		for (objectIt.GoToBegin() ; !objectIt.IsAtEnd() ; ++objectIt) {
			pdf->AddMultivariatePDF( m_scene->GetObjectTypesLibrary()->GetObjectPDF( objectIt.GetObjectInScene()->objectTypeId , PDF_OBJECTGENERATIONPRIOR) );
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
			pdf->AddMultivariatePDF( m_scene->GetObjectTypesLibrary()->GetObjectPDF( objectIt.GetObjectInScene()->objectTypeId , PDF_RANDOMWALK) );
		}
		m_priorPDF = static_cast<MultivariatePDF::Pointer>(pdf);
		return m_priorPDF.GetPointer();
	}

	////get a multivariate pdf for all parameters of all objects - same as the parameter list, cannot be realistically maintained by the scene - too much overhead only for this class.
	////however, it should be called only once within a single optimization run! => the optimizer should keep it stored.
	//MultivariatePDF* GetParameterPriorPDF() const {
	//	IndependentPDFs::Pointer pdf = IndependentPDFs::New();			
	//	for (unsigned i=0 ; i<m_scene->GetNumberOfObjects() ; i++) {
	//		pdf->AddMultivariatePDF( m_scene->GetObjectTypesLibrary()->GetObjectPDF( m_scene->GetObjectByIndex(i)->indexObjectType , PDF_OBJECTGENERATIONPRIOR) );
	//	}
	//	m_priorPDF = static_cast<MultivariatePDF::Pointer>(pdf);
	//	return m_priorPDF.GetPointer();
	//}

	//MultivariatePDF* GetParameterProposalMovePDF() {
	//	IndependentPDFs::Pointer pdf = IndependentPDFs::New();			
	//	for (unsigned i=0 ; i<m_scene->GetNumberOfObjects() ; i++) {
	//		pdf->AddMultivariatePDF( m_scene->GetObjectTypesLibrary()->GetObjectPDF( m_scene->GetObjectByIndex(i)->indexObjectType , PDF_RANDOMWALK) );
	//	}
	//	m_priorPDF = static_cast<MultivariatePDF::Pointer>(pdf);
	//	return m_priorPDF.GetPointer();
	//}

	//CLEAN: access should be restricted to some optimizers ... use friendship?
	void SaveCurrentStateAsBest() { 
		m_bestCost = this->GetValue();
		m_bestKnownState.clear();
		m_nbObjectAtBestState = m_scene->GetNumberOfObjects();
		m_scene->GetObjectSetCopy(m_bestKnownState); 
	}
protected:
	SceneStateOptimizationManager() : SceneOptimizationManager_Base() {	};
	~SceneStateOptimizationManager() {
		//delete m_bestKnownState;
	};

	MultivariatePDF::Pointer m_priorPDF, m_proposalPDF;

	unsigned m_nbObjectAtBestState;
	typename SceneType::ObjectSetType m_bestKnownState;
	
	double ReturnToBestKnownState() { 
		m_scene->SetObjectSet(m_bestKnownState); 
		return this->GetValue(); 
	}

private:
	SceneStateOptimizationManager(const Self&);	//purposely not implemented
	const Self & operator=( const Self & );		//purposely not implemented

};



} // namespace psciob

#endif /* SCENESTATEOPTIMIZATIONMANAGER_H_ */
