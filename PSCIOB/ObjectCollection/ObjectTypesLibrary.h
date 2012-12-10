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
 * \file ObjectTypesLibrary.h
 * \author Rémi Blanc 
 * \date 11. October 2011
*/

#ifndef OBJECTTYPESLIBRARY_H_
#define OBJECTTYPESLIBRARY_H_

#include "ParametricObject.h"
#include "MultivariatePDF.h"
#include <typeinfo>
#include "PoseTransformedBinaryShape.h" 
//remember to add here all object types that cannot be further resolved, but can still show variety... (so we can differentiate between e.g. translation+sphere and similarity+cylinder...)
//and to modify the method RegisterObjectType

namespace psciob {


/**\class ObjectTypesLibrary
 * \brief ObjectTypesLibrary
 * maintain a list of the types of object that are available / registered with the scene
 * as well as various pdfs so that new objects can be generated / random parameters can be drawn with respect to object-dependent laws
 *
 * the idea is to register the pdfs here, so they can be set &/ accessed easily, for all objects types that are of interest.
 * these pdfs can be used e.g. to generate new objects, propose random modifications, or compute an associated likelihood
 * these distribution do not need to have the same dimensionality as the object ; it is up to the user to use these distributions carefully...
*/
// add any name here that could be useful at some point...
typedef enum {
	PDF_OBJECTGENERATIONPRIOR,		//intended to be a global prior for the object <-> typically related to the prior in the DeformableObjectGenerator
	PDF_SCENEUNIFORMTRANSLATION,	//intended to be a uniform distribution of the translation parameters, over a given bounding box (-> scene)
	PDF_INTEGERTRANSLATION,			//intended to be a Dirac distribution {-1;0;1} over the translation parameters
	PDF_RANDOMWALK,					//intended as a basic random walk distribution (move all parameters at once by a small amount) (e.g. multivariate normal distribution)
	PDF_OBJECTLIKELIHOOD			//intended as a prior over the object parameters (e.g. don't care about pose parameters...)
} ObjectPDFTypes ;

//TODO: enrich the interface...
template<class TObject>
class ObjectTypesLibrary : public itk::LightObject {
public:
	/** Standard class typedefs. */
	typedef ObjectTypesLibrary				Self;
	typedef itk::LightObject				Superclass;
	typedef itk::SmartPointer<Self>			Pointer;
	/** Run-time type information (and related methods). */
	itkTypeMacro(ObjectTypesLibrary, itk::LightObject);
	itkNewMacro(Self);
	
	static const unsigned int Dimension = TObject::Dimension;
	typedef TObject						  ObjectType;
	typedef typename ObjectType::TexturedPixelType	AppearanceType;
	
	//add here all 'composite types', which wraps several subtypes under a same child type
	typedef PoseTransformedBinaryShape<Dimension> PoseTransformedBinaryShapeType;

	struct LibraryEntryType {
		typename ObjectType::Pointer objectSample;
		unsigned int objectDimensionality;
		std::map<ObjectPDFTypes, MultivariatePDF::Pointer> pdf;
		double weight;
		bool isAmbiguous;
	};

	typedef std::vector<LibraryEntryType>				ObjectTypesCollectionType;	


	void Clear() {m_sumWeights = 0;m_library.clear();}
	void Reset() {m_sumWeights = 0;m_library.clear();}

	/** Print info into a stream */
	void PrintInfo(std::ostream & os, itk::Indent indent = 0, bool printPDF = false) const;

	/** Get properties */
	unsigned int GetNumberOfEntries()       const   { return m_library.size(); }
	ObjectTypesCollectionType* GetLibrary() const   { return const_cast<ObjectTypesCollectionType*>(&m_library); }
	
	/** */
	LibraryEntryType* GetObjectEntry(unsigned int i) const { 
		if (i>=m_library.size()) throw DeformableModelException("ObjectTypesLibrary::GetObject : unregistered type...");
		return const_cast<LibraryEntryType*>(&m_library[i]);  
	}
	/** */
	MultivariatePDF *GetObjectPDF(unsigned int i, ObjectPDFTypes pdfType) {
		if (i>=m_library.size()) throw DeformableModelException("ObjectTypesLibrary::GetObjectPDF : unregistered type...");
		if ( m_library[i].pdf.find(pdfType) == m_library[i].pdf.end() ) {
			throw DeformableModelException("ObjectTypesLibrary::GetObjectPDF : requested pdf has not been set ; code = " + stringify(pdfType));
		}
		return m_library[i].pdf[pdfType].GetPointer();
	}

	/** */
	double GetSumTypeWeights() const				{return m_sumWeights;}
	
	/** New object from the requested type, with default parameters */
	typename ObjectType* GenerateNewObjectDefault(unsigned int i) {
		m_library[i].objectSample->SetDefaultParameters();
		return m_library[i].objectSample.GetPointer();
	}

	/** New object with random parameters from the chosen distribution */
	typename ObjectType* GenerateNewRandomObject(unsigned int i, ObjectPDFTypes pdfType = PDF_OBJECTGENERATIONPRIOR) {
		while (!m_library[i].objectSample->SetParameters( m_library[i].pdf[pdfType]->DrawSample() )) {};
		return m_library[i].objectSample.GetPointer();
	}

	/** Looks into the library for the index entry of the object type
	 * if this type of object is not yet registered, a new entry is generated associated with the given weight...
	 * if not, the weight is ignored
	 */
	unsigned int RegisterObjectType(ObjectType *object, double weight = 1);	//outputs the entry index
	unsigned int RegisterEntry(LibraryEntryType *entry);					//copies a complete entry in the library

	void SetObjectWeight(unsigned int entryIndex, double weight);
	void SetObjectPDF(unsigned int entryIndex, ObjectPDFTypes pdfType, MultivariatePDF *pdf);

	////some convenience methods for setting default pdfs for specific types
	//void SetDefaultScenePositionPDF(unsigned int entryIndex, vnl_vector<double> sceneBBox);
	//void SetIntegerTranslationPDF(unsigned int entryIndex, vnl_vector<double> sceneSpacing);
	//
	////set default pdfs for the whole library!
	//void SetDefaultScenePositionPDF(vnl_vector<double> sceneBBox)  { for (unsigned i=0 ; i<m_library.size() ; i++) {SetDefaultScenePositionPDF(m_library[i].shapeName, m_library[i].transformName, sceneBBox);} }
	//void SetIntegerTranslationPDF(vnl_vector<double> sceneSpacing) { for (unsigned i=0 ; i<m_library.size() ; i++) {SetIntegerTranslationPDF(m_library[i].shapeName, m_library[i].transformName, sceneSpacing);}}


	/** checks if typeid of the input is the same as that of the reference object (2nd) */
	inline bool TestSameTypeAsEntry(ObjectType *object, unsigned entryIndex) {	
		if ( typeid(*object)==typeid( *m_library[entryIndex].objectSample.GetPointer() ) ) {
			if (!m_library[entryIndex].isAmbiguous) return true; //if this is not an ambiguous type, than both objects are really of the same type.
			
			//else, need to resolve the ambiguity...
			PoseTransformedBinaryShapeType* tmp1 = dynamic_cast<PoseTransformedBinaryShapeType*>(object);
			if (!tmp1) {return true;} 
			else {
				PoseTransformedBinaryShapeType* tmp2 = dynamic_cast<PoseTransformedBinaryShapeType*>(m_library[entryIndex].objectSample.GetPointer());
				if (!tmp2) {throw DeformableModelException("ObjectTypesLibrary::CompareTypes : types are 'identical', but both objects cannot be casted to the same type ; SHOULD NEVER HAPPEN!!"); }
				if ( typeid(*tmp1->GetShape())!=typeid(*tmp2->GetShape()) ) return false;
				if ( typeid(*tmp1->GetTransform())!=typeid(*tmp2->GetTransform()) ) return false;
				return true;
			}
			
		}
		else return false;
	}


protected:
	ObjectTypesLibrary() : m_sumWeights(0) {}
	virtual ~ObjectTypesLibrary() {}

	ObjectTypesCollectionType m_library;
	double m_sumWeights;
private:
	ObjectTypesLibrary(const Self&);			//purposely not implemented
	const Self & operator=( const Self & );		//purposely not implemented
};


} // namespace psciob

#include "ObjectTypesLibrary.txx"

#endif /* OBJECTTYPESLIBRARY_H_ */

