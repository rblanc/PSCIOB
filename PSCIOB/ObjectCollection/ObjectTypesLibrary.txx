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
 * \file ObjectTypesLibrary.txx
 * \author Rémi Blanc 
 * \date 11. October 2011
*/


#include "ObjectTypesLibrary.h"

namespace psciob {


template<class TObject>
inline
unsigned int ObjectTypesLibrary<TObject>::RegisterObjectType(ObjectType *object, double weight = 1) {
	//check whether the type is new
	unsigned typeIndex=0;
	for (ObjectTypesCollectionType::iterator it = m_library.begin() ; it!=m_library.end() ; it++) {
		if ( it->objectGenerator->CheckTypes( object ) ) return typeIndex;
		typeIndex++;
	}
	//Register the new type, and return the corresponding typeIndex
	struct LibraryEntryType objType; 
	objType.objectGenerator = DeformableObjectGenerator<ObjectType>::New();   
	objType.objectGenerator->SetBaseObject(object);
	objType.objectDimensionality = object->GetNumberOfParameters();
	objType.weight = weight;
	m_sumWeights+=weight;

	//ADD: define some default pdfs => dirac on the default shape parameters... ...
	//IndependentPDFs::Pointer pdf = IndependentPDFs::New(); vnl_vector<double> defaultParams = object->GetParameters();
	//for (unsigned i = 0 ; i<defaultParams.size() ; i++) { DiracPDF::Pointer univ = DiracPDF::New(); univ->SetParameters(defaultParams(i)); pdf->AddUnivariatePDF(univ); }; objType.pdf = pdf;
	////objType.pdf = NULL; //no default...

	m_library.push_back(objType);
	return typeIndex;
}

template<class TObject>
unsigned int ObjectTypesLibrary<TObject>::RegisterEntry(LibraryEntryType *entry) { //copies a complete entry in the library
	//check whether the type is new
	unsigned typeIndex=0;
	ObjectType *object = entry->objectGenerator->GetNewObject();
	for (ObjectTypesCollectionType::iterator it = m_library.begin() ; it!=m_library.end() ; it++) {
		if ( it->objectGenerator->CheckTypes( object ) ) return typeIndex;
		typeIndex++;
	}
	//Add this new entry to the list
	m_sumWeights+=entry->weight;
	m_library.push_back(*entry);	//VERIFY that all objects are correctly transfered (generator, pdfs, ...)
	return typeIndex;
}

template<class TObject>
void ObjectTypesLibrary<TObject>::SetObjectWeight(unsigned int index, double weight) {
	if (index>=m_library.size) throw DeformableModelException("ObjectTypesLibrary::SetObjectWeight : wrong index");
	m_sumWeights += weight - it->weight;
	m_library[index].weight = weight;	
}


template<class TObject>
void ObjectTypesLibrary<TObject>::SetObjectPDF(unsigned int index, ObjectPDFTypes pdfType, MultivariatePDF *pdf) {
	if (index>=m_library.size()) throw DeformableModelException("ObjectTypesLibrary::SetObjectPDF : wrong index");
	m_library[index].pdf[pdfType] = pdf;
}


//default parameters except for the translation -> uniform over the scene bbox
template<class TObject>
void ObjectTypesLibrary<TObject>::SetDefaultScenePositionPDF(unsigned int index, vnl_vector<double> sceneBBox) {
	if (sceneBBox.size() != 2*VDimension) throw DeformableModelException("ObjectTypesLibrary::SetDefaultScenePositionPDF : unexpected dimension of the bounding box...");
	if (index>=m_library.size) throw DeformableModelException("ObjectTypesLibrary::SetDefaultScenePositionPDF : wrong index");

	ParametricObject<VDimension>::Pointer object = m_library[index].objectGenerator->GetNewObject();
	IndependentPDFs::Pointer pdf = IndependentPDFs::New(); vnl_vector<double> defaultParams = object->GetParameters();
	// set translations to uniform over the sceneBBox
	for (unsigned i=0 ; i<VDimension ; i++) { UniformPDF::Pointer univ = UniformPDF::New(); univ->SetParameters(sceneBBox(2*i),sceneBBox(2*i+1)); pdf->AddUnivariatePDF(univ); }
	//set the other parameters to stay on default
	for (unsigned i = VDimension ; i<defaultParams.size() ; i++) { DiracPDF::Pointer univ = DiracPDF::New(); univ->SetParameters(defaultParams(i)); pdf->AddUnivariatePDF(univ); }

	m_library[index].pdf[PDF_SCENEUNIFORMTRANSLATION] = pdf;
}


//default parameters except for the translation -> uniform in {-spacing, 0, +spacing}
template<class TObject>
void ObjectTypesLibrary<TObject>::SetIntegerTranslationPDF(unsigned int index, vnl_vector<double> sceneSpacing) { 
	if (sceneSpacing.size() != VDimension) throw DeformableModelException("ObjectTypesLibrary::SetIntegerTranslationPDF : unexpected dimension of the spacing...");
	if (index>=m_library.size) throw DeformableModelException("ObjectTypesLibrary::SetDefaultScenePositionPDF : wrong index");

	ParametricObject<VDimension>::Pointer object = m_library[index].objectGenerator->GetNewObject();
	IndependentPDFs::Pointer pdf = IndependentPDFs::New(); vnl_vector<double> defaultParams = object->GetParameters();
	// set translations to uniform over the sceneBBox
	for (unsigned i=0 ; i<VDimension ; i++) { 
		UnivariateMixturePDF::Pointer univ = UnivariateMixturePDF::New();
		DiracPDF::Pointer tmp1 = DiracPDF::New(); tmp1->SetParameters(-sceneSpacing(i)); univ->AddPDF(tmp1); 
		DiracPDF::Pointer tmp2 = DiracPDF::New(); tmp2->SetParameters(        0       ); univ->AddPDF(tmp2); 
		DiracPDF::Pointer tmp3 = DiracPDF::New(); tmp3->SetParameters(+sceneSpacing(i)); univ->AddPDF(tmp3); 
		pdf->AddUnivariatePDF(univ);
	}
	//set the other parameters to stay on default
	for (unsigned i = VDimension ; i<defaultParams.size() ; i++) { DiracPDF::Pointer univ = DiracPDF::New(); univ->SetParameters(defaultParams(i)); pdf->AddUnivariatePDF(univ); }

	m_library[index].pdf[PDF_INTEGERTRANSLATION] = pdf;
}

} // namespace psciob

