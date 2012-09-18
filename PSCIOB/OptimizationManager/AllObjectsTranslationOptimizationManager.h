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
 * \file AllObjectsTranslationOptimizationManager.h
 * \author Rémi Blanc 
 * \date 21. September 2011
*/


#ifndef ALLOBJECTSTRANSLATIONOPTIMIZATIONMANAGER_H_
#define ALLOBJECTSTRANSLATIONOPTIMIZATIONMANAGER_H_

#include "SceneOptimizationManager_Base.h"

namespace psciob {

/** \class AllObjectsTranslationOptimizationManager
 * \brief AllObjectsTranslationOptimizationManager optimize only the selected translation parameters of the objects
 * by default, select all axes
*/


template<class TScene>
class AllObjectsTranslationOptimizationManager : public SceneOptimizationManager_Base<TScene> {
public:
	/** Standard class typedefs. */
	typedef AllObjectsTranslationOptimizationManager Self;
	typedef SceneOptimizationManager_Base            Superclass;
	typedef itk::SmartPointer<Self>                  Pointer;
	/** Run-time type information (and related methods). */
	itkTypeMacro(AllObjectsTranslationOptimizationManager, SceneOptimizationManager_Base);
	itkNewMacro(Self);

	
	inline unsigned int GetNumberOfParameters() { return m_axis.size() * m_scene->GetNumberOfObjects(); }


	/** select an axis for translation ; returns false if the axis is invalid
	* 0: translate along x ; 1: along y, etc... 
	*/
	bool SelectTranslationAxis(unsigned int axis) {
		if (axis>SceneType::Dimension) return false;
		m_axis.insert(axis); return true;
	}

	/** Select all axes */
	void SelectAllAxes() { m_axis.clear(); for (unsigned i=0 ; i<SceneType::Dimension ; i++) m_axis.insert(i); return true; }

	/** Clears all the axes from the selection */
	void ResetAxesSelection() { m_axis.clear(); }

	/** */
	inline vnl_vector<double> GetParameters() {
		vnl_vector<double> translationParams(GetNumberOfParameters()), tmpParams;
		if (m_axis.size()==0) return translationParams;
		SceneObjectIterator<SceneType> objectIt(m_scene);
		unsigned pos=0, j; std::set<unsigned int>::iterator axisIt;
		for (objectIt.GoToBegin() ; !objectIt.IsAtEnd() ; ++objectIt ) {
			for (axisIt=m_axis.begin() ; axisIt!=m_axis.end() ; axisIt++ ) {
				//postfix increment: pos is incremented after -> first fill the value at 0, and move to 1 after that.
				translationParams(pos++) = m_scene->GetParametersOfObject(objectIt.GetID())[*axisIt];
			}
		}
		return translationParams; 
	}

	inline bool SetParameters(vnl_vector<double> p) { 
		if (p.size()!=GetNumberOfParameters()) return false;
		if (m_axis.size()==0) return true;

		unsigned pos = 0, i; std::set<unsigned int>::iterator axisIt;
		std::vector< vnl_vector<double> > oldParams;
		vnl_vector<double> objectParams;
		SceneObjectIterator<SceneType> objectIt(m_scene);
		for (objectIt.GoToBegin(), i=0 ; !objectIt.IsAtEnd() ; ++objectIt, ++i) {
			objectParams = m_scene->GetParametersOfObject( objectIt.GetID() );
			oldParams.push_back( objectParams ); //backit up in case it is necessary to rewind...
			for (axisIt=m_axis.begin() ; axisIt!=m_axis.end() ; axisIt++ ) { objectParams(*axisIt) = p(pos++); }

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
	*\todo FOR THIS, IT WILL BE NECESSARY TO IMPOSE THAT THE PDF ARE ORGANIZED IN SUCH A WAY THAT THE TRANSLATION CAN BE RECOVERED FROM THE MULTIVARIATE PDF WITHOUT TOO MUCH TROUBLE...
	*    -> allow the IndependentPDFs to give access to the pdf it is made of
	*	 -> THE PDF SHOULD BE A SET OF THREE INDEPENDENT UNIVARIATE PDFS, so that they can be accessed independently..!!!
	*	 -> get it for all objects, and append everything together...
	* -> furthermore, let an option to decide if the pdf should be taken from the scene, or whether it could be set manually
	*/
	MultivariatePDF* GetParameterPriorPDF() const {
		std::cout<<"INVALID FOR NOW, SEE CODE FOR DETAILS..."<<std::endl;
		IndependentPDFs::Pointer pdf = IndependentPDFs::New();
		SceneObjectIterator<SceneType> objectIt(m_scene);
		for (objectIt.GoToBegin() ; !objectIt.IsAtEnd() ; ++objectIt) {
			pdf->AddMultivariatePDF( m_scene->GetObjectTypesLibrary()->GetObjectPDF( objectIt.GetObjectInScene()->objectTypeId , PDF_OBJECTGENERATIONPRIOR) );
		}
		m_priorPDF = static_cast<MultivariatePDF::Pointer>(pdf);
		return m_priorPDF.GetPointer();
	}

	/** get a multivariate pdf for all parameters of all objects - same as the parameter list, cannot be realistically maintained by the scene - too much overhead only for this class.
	* however, it should be called only once within a single optimization run! => the optimizer should keep it stored.
	*/
	MultivariatePDF* GetParameterProposalMovePDF() {
		std::cout<<"INVALID FOR NOW, SEE CODE FOR DETAILS..."<<std::endl;
		IndependentPDFs::Pointer pdf = IndependentPDFs::New();			
		SceneObjectIterator<SceneType> objectIt(m_scene);
		for (objectIt.GoToBegin() ; !objectIt.IsAtEnd() ; ++objectIt) {
			pdf->AddMultivariatePDF( m_scene->GetObjectTypesLibrary()->GetObjectPDF( objectIt.GetObjectInScene()->objectTypeId , PDF_RANDOMWALK) );
		}
		m_priorPDF = static_cast<MultivariatePDF::Pointer>(pdf);
		return m_priorPDF.GetPointer();
	}

	void SaveCurrentStateAsBest()		{ m_bestCost = this->GetValue(); m_bestKnownParameters = this->GetParameters(); }
protected:
	AllObjectsTranslationOptimizationManager() : SceneOptimizationManager_Base() { 
		for (unsigned i=0 ; i<SceneType::Dimension ; i++) m_axis.insert(i);
	};
	~AllObjectsTranslationOptimizationManager() {};

	MultivariatePDF::Pointer m_priorPDF, m_proposalPDF;

	std::set<unsigned int> m_axis;
	
	vnl_vector<double> m_bestKnownParameters;
	double ReturnToBestKnownState()		{ this->SetParameters(m_bestKnownParameters); return this->GetValue(); }

private:
	AllObjectsTranslationOptimizationManager(const Self&);	//purposely not implemented
	const Self & operator=( const Self & );						//purposely not implemented
};



} // namespace psciob

#endif /* ALLOBJECTSTRANSLATIONOPTIMIZATIONMANAGER_H_ */
