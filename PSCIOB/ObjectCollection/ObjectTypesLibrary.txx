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


/** Print info into a stream */
template<class TObject>
void 
ObjectTypesLibrary<TObject>::PrintInfo(std::ostream & os, itk::Indent indent, bool printPDF) const {
	os << indent << "Number of entries: " << m_library.size() << std::endl;
	if (m_library.size()==0) return;
	
	for (unsigned i=0 ; i<m_library.size() ; i++) {
		os << indent << "entry " << i <<" ; weight = "<<m_library[i].weight<< std::endl;
		os << indent.GetNextIndent() << "object type: "; m_library[i].objectSample->PrintInfo(os);
		if (printPDF) {
			for (std::map<ObjectPDFTypes, MultivariatePDF::Pointer>::const_iterator pdfIt = m_library[i].pdf.begin() ; pdfIt != m_library[i].pdf.end() ; ++pdfIt) {
				os << indent.GetNextIndent() << "pdf code " << pdfIt->first << ": ";
				pdfIt->second->PrintInfo(os);
			}
		}
	}
}


template<class TObject>
inline
unsigned int ObjectTypesLibrary<TObject>::RegisterObjectType(ObjectType *object, double weight = 1) {
	//check whether the type is new
	unsigned typeIndex=0;
	for (unsigned i = 0 ; i<m_library.size() ; i++) {
		if ( TestSameTypeAsEntry(object, i) ) return typeIndex;
		typeIndex++;
	}
	//Register the new type, and return the corresponding typeIndex
	struct LibraryEntryType objType; 
	objType.objectSample = object->CreateCopy();   
	objType.objectDimensionality = object->GetNumberOfParameters();
	objType.weight = weight;
	m_sumWeights+=weight;

	//WARNING: Remember to add other ambiguous types here
	PoseTransformedBinaryShapeType* tmp = dynamic_cast<PoseTransformedBinaryShapeType*>(object);
	if (!tmp) { objType.isAmbiguous = false; } 
	else      { objType.isAmbiguous = true;  }

	m_library.push_back(objType);
	return typeIndex;
}

template<class TObject>
unsigned int ObjectTypesLibrary<TObject>::RegisterEntry(LibraryEntryType *entry) { //copies a complete entry in the library
	return RegisterObjectType(entry->objectSample);
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


////default parameters except for the translation -> uniform over the scene bbox
//template<class TObject>
//void ObjectTypesLibrary<TObject>::SetDefaultScenePositionPDF(unsigned int index, vnl_vector<double> sceneBBox) {
//	if (sceneBBox.size() != 2*VDimension) throw DeformableModelException("ObjectTypesLibrary::SetDefaultScenePositionPDF : unexpected dimension of the bounding box...");
//	if (index>=m_library.size) throw DeformableModelException("ObjectTypesLibrary::SetDefaultScenePositionPDF : wrong index");
//
//	ParametricObject<VDimension>::Pointer object = m_library[index].objectGenerator->GetNewObject();
//	IndependentPDFs::Pointer pdf = IndependentPDFs::New(); vnl_vector<double> defaultParams = object->GetParameters();
//	// set translations to uniform over the sceneBBox
//	for (unsigned i=0 ; i<VDimension ; i++) { UniformPDF::Pointer univ = UniformPDF::New(); univ->SetParameters(sceneBBox(2*i),sceneBBox(2*i+1)); pdf->AddUnivariatePDF(univ); }
//	//set the other parameters to stay on default
//	for (unsigned i = VDimension ; i<defaultParams.size() ; i++) { DiracPDF::Pointer univ = DiracPDF::New(); univ->SetParameters(defaultParams(i)); pdf->AddUnivariatePDF(univ); }
//
//	m_library[index].pdf[PDF_SCENEUNIFORMTRANSLATION] = pdf;
//}
//
//
////default parameters except for the translation -> uniform in {-spacing, 0, +spacing}
//template<class TObject>
//void ObjectTypesLibrary<TObject>::SetIntegerTranslationPDF(unsigned int index, vnl_vector<double> sceneSpacing) { 
//	if (sceneSpacing.size() != VDimension) throw DeformableModelException("ObjectTypesLibrary::SetIntegerTranslationPDF : unexpected dimension of the spacing...");
//	if (index>=m_library.size) throw DeformableModelException("ObjectTypesLibrary::SetDefaultScenePositionPDF : wrong index");
//
//	ParametricObject<VDimension>::Pointer object = m_library[index].objectGenerator->GetNewObject();
//	IndependentPDFs::Pointer pdf = IndependentPDFs::New(); vnl_vector<double> defaultParams = object->GetParameters();
//	// set translations to uniform over the sceneBBox
//	for (unsigned i=0 ; i<VDimension ; i++) { 
//		UnivariateMixturePDF::Pointer univ = UnivariateMixturePDF::New();
//		DiracPDF::Pointer tmp1 = DiracPDF::New(); tmp1->SetParameters(-sceneSpacing(i)); univ->AddPDF(tmp1); 
//		DiracPDF::Pointer tmp2 = DiracPDF::New(); tmp2->SetParameters(        0       ); univ->AddPDF(tmp2); 
//		DiracPDF::Pointer tmp3 = DiracPDF::New(); tmp3->SetParameters(+sceneSpacing(i)); univ->AddPDF(tmp3); 
//		pdf->AddUnivariatePDF(univ);
//	}
//	//set the other parameters to stay on default
//	for (unsigned i = VDimension ; i<defaultParams.size() ; i++) { DiracPDF::Pointer univ = DiracPDF::New(); univ->SetParameters(defaultParams(i)); pdf->AddUnivariatePDF(univ); }
//
//	m_library[index].pdf[PDF_INTEGERTRANSLATION] = pdf;
//}

} // namespace psciob

