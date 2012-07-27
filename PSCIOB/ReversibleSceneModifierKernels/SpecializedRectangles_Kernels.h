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

/*
* \file RJKernel_SpecializedRectangles.h
* \author Rémi Blanc 
* \date 10. October 2011
*
* \brief RJKernel_SpacializedRectangles: 
* some specialized kernels for rectangles
* including split and merge kernels
* The classes in there have been developed with FastRectangles in mind, but are probably valid for other 2D objects such as ellipses
* they assume the following parameters for the object:
* 2 coordinates for the center ; orientation ; long axis ; elongation (b/a) between 0 and 1
*
* \TODO: THESE CLASSES ARE VERY LIKELY BROKEN, and would need to be updated !!
*/




#ifndef RANDOMJUMPKERNEL_SPECIALIZEDRECTANGLES_H_
#define RANDOMJUMPKERNEL_SPECIALIZEDRECTANGLES_H_

#include <ReversibleSceneModifierKernel_Base.h>
#include "PixelSetUtils.h"


namespace psciob {


/** \class RJKernel_SplitRectangle
 * \brief RJKernel_SplitRectangle: splits a rectangle into 2 parts, cutting it in half perpendicular to the main axis
 */
template<class TSceneType, class TCostFunction>
class RJKernel_SplitRectangle : public ReversibleSceneModifierKernel_Base<TSceneType> {
public:
	/** Standard class typedefs. */
	typedef RJKernel_SplitRectangle					Self;
	typedef ReversibleSceneModifierKernel_Base					Superclass;
	typedef itk::SmartPointer<Self>			Pointer;
	/** Run-time type information (and related methods). */
	itkTypeMacro(RJKernel_SplitRectangle,ReversibleSceneModifierKernel_Base);
	itkNewMacro(Self);

	typedef TCostFunction CostFunctionType; //even more, it should be of type SingleObjectSceneToImageCostFunction -- but the ReferenceImageType is not important...

	/** Set the associated cost function - which should be castable to a SingleObjectSceneToImageCostFunction ... */
	void SetCostFunction(CostFunctionType *costFunction) { m_costFunction = costFunction; }


	/** Selects bad objects in priority, and split them into 2 along the main axis 
	 * Temperature affects the selected object -> takes the worst fitting object at low T
	 */
	bool Apply(double T) {
		//1: decide which object to split (fills m_labelModifiedObject)
		SelectObject(T);
		//2: generate a object of the requested type, with random parameters from the given distribution
		m_undoParameters = m_scene->GetParametersOfObject( m_labelModifiedObject );
		ObjectType::Pointer obj = m_library->GenerateNewObjectDefault( m_scene->GetObjectByLabel(m_labelModifiedObject)->indexObjectType );
		//generate the parameters for both objects - initialize them with the previous params
		vnl_vector<double> newParams(m_undoParameters), newParams2(m_undoParameters);

		double halflength = m_undoParameters(3)/2.0, width = m_undoParameters(3) * m_undoParameters(4);
		double ct = cos(m_undoParameters(2)), st = sin(m_undoParameters(2));

		newParams(0) += halflength*ct/2.0; newParams2(0) -= halflength*ct/2.0;
		newParams(1) += halflength*st/2.0; newParams2(1) -= halflength*st/2.0; // + or - ?

		if (width <= halflength) { //the orientation of the rectangles is kept the same
			newParams(3) = halflength; newParams(4) = width / halflength;
			newParams2(3)= halflength; newParams2(4)= width / halflength;
		}
		else { // the rectangles are oriented at PI/2
			newParams(3) = width; newParams(4) = halflength / width;
			newParams2(3)= width; newParams2(4)= halflength / width;
			//try to keep the rotation parameter in range...
			if ( m_undoParameters(2)>PI/2.0 ) { newParams(2) = m_undoParameters(2)-PI/2.0; newParams2(2)= m_undoParameters(2)-PI/2.0; }
			else { newParams(2) = m_undoParameters(2)+PI/2.0; newParams2(2)= m_undoParameters(2)+PI/2.0; }
		}

		//3: modify the scene
		if (!m_scene->ModifyObjectParameters(m_labelModifiedObject, newParams)) return false; //SHOULD NEVER HAPPEN
		obj->SetParameters(newParams2);
		m_labelToUndo = m_scene->AddObject(obj);	// what should be done in this case?? currently, just return to the original state and return false.
		if (m_labelToUndo==0) {m_scene->ModifyObjectParameters(m_labelModifiedObject, m_undoParameters); return false;}
		return true;
	}

	//move the scene to the state it used to have the last time we called Apply
	void Undo() {
		m_scene->RemoveObject(m_labelToUndo);
		if (!m_scene->ModifyObjectParameters(m_labelModifiedObject, m_undoParameters)) throw DeformableModelException("RJKernel_SplitRectangle: unable to undo properly (modifying the modified object)");
	}

protected:
	RJKernel_SplitRectangle() : ReversibleSceneModifierKernel_Base() {};
	~RJKernel_SplitRectangle() {};

	typename CostFunctionType::Pointer m_costFunction;
	//
	//At high temperatures, any object may be selected, with a proba decreasing with its rank and cost
	//at low temperature, only the most costly object will ever be selected.
	void SelectObject(double T) { 
		unsigned i, nbobj = m_scene->GetNumberOfObjects();
		double *dataCosts = (double *)malloc(nbobj * sizeof(double));
		unsigned int *indices = (unsigned int *)malloc(nbobj * sizeof(unsigned int));

		double sumCosts = 0, tmpcumul = 0;
		
		for (i=0 ; i<nbobj ; i++) {
			m_costFunction->SelectObjectByIndex( i );
			dataCosts[i] = m_costFunction->GetValue();
			sumCosts += dataCosts[i];
			indices[i] = i;
		}

		std::sort(indices, indices+nbobj, index_more<double*>(dataCosts)); //sorted in descending order

		//random threshold for object selection 
		double threshold = m_rndgen->GetUniformVariate(0, sumCosts * T); 
		i=0; tmpcumul = dataCosts[indices[i]];
		while (tmpcumul<threshold) {
			tmpcumul += dataCosts[indices[++i]];
			if (i==nbobj) { i--; break; }
		}

		m_labelModifiedObject = m_scene->GetLabelFromIndex( i );

		free(dataCosts);
		free(indices);
	}

	//
	LabelType m_labelToUndo, m_labelModifiedObject; //the action of this kernel to modify the selected object (m_labelModifiedObject) and to create a second one (m_labelToUndo)
	vnl_vector<double> m_undoParameters; //initial parameters of the modified object
private:
	RJKernel_SplitRectangle(const Self&);				//purposely not implemented
	const Self & operator=( const Self & );		//purposely not implemented
};



/**\class RJKernel_MergeRectangles
 * \brief RJKernel_MergeRectangles: merge two overlapping rectangles into a single one
 */
template<class TSceneType>
class RJKernel_MergeRectangles : public ReversibleSceneModifierKernel_Base<TSceneType> {
public:
	/** Standard class typedefs. */
	typedef RJKernel_MergeRectangles					Self;
	typedef ReversibleSceneModifierKernel_Base					Superclass;
	typedef itk::SmartPointer<Self>			Pointer;
	/** Run-time type information (and related methods). */
	itkTypeMacro(RJKernel_MergeRectangles, ReversibleSceneModifierKernel_Base);
	itkNewMacro(Self);


	//temperature is given as an argument ; in case this is useful for deciding the amplitude of the "move"...
	bool Apply(double T) {
		//1: decide which object to split (fills m_labelModifiedObject)
		SelectObjects(T);
		if (m_labelModifiedObject==0) return false; //if no object overlap, exit.

		//BACKUP the old objects
		m_deletedParameters = m_scene->GetParametersOfObject( m_labelDeletedObject );
		m_indexObjectType = m_scene->GetObjectByLabel(m_labelDeletedObject)->indexObjectType;
		m_undoParameters = m_scene->GetParametersOfObject( m_labelModifiedObject );

		//propose parameters for the merged object... 
		ObjectPixelSet::Pointer unionSet = ObjectPixelSet::New();
		PixelSetUnion( m_scene->GetObjectByLabel(m_labelDeletedObject)->object->GetObjectAsLabelMap(), m_scene->GetObjectByLabel(m_labelModifiedObject)->object->GetObjectAsLabelMap(), unionSet );
		
		double cx, cy, a,b,th;
		PixelSetParametersBestFittingRectangle( unionSet, cx, cy, a, b, th );
		vnl_vector<double> newParams( m_deletedParameters.size() );
		newParams(0) = cx; newParams(1) = cy;
		newParams(2) = -th;
		newParams(3) = a; newParams(4) = b/a;

		m_scene->RemoveObject( m_labelDeletedObject );
		if (!m_scene->ModifyObjectParameters(m_labelModifiedObject, newParams)) {
			ObjectType::Pointer obj = m_library->GenerateNewObjectDefault( m_indexObjectType );
			obj->SetParameters(m_deletedParameters);
			m_scene->AddObject(obj, m_labelDeletedObject);
			return false;
		}

		return true;
	}

	//move the scene to the state it used to have the last time we called Apply
	void Undo() {
		ObjectType::Pointer obj = m_library->GenerateNewObjectDefault( m_indexObjectType );
		obj->SetParameters(m_deletedParameters);
		if (m_scene->AddObject(obj, m_labelDeletedObject) != m_labelDeletedObject) throw DeformableModelException("RJKernel_MergeRectangles: unable to undo properly (reviving the deleted object)");
		if (!m_scene->ModifyObjectParameters(m_labelModifiedObject, m_undoParameters)) throw DeformableModelException("RJKernel_MergeRectangles: unable to undo properly (modifying the modified object)");
	}

protected:
	RJKernel_MergeRectangles() : ReversibleSceneModifierKernel_Base() {};
	~RJKernel_MergeRectangles() {};

	void SelectObjects(double T) { 
		//find a pair of overlapping objects...  first identify all the objects which present overlaps
		//IDEA: maybe I could try to use slightly looser criteria => e.g. their surroundings interesect ... !!!
		unsigned i, nbobj = m_scene->GetNumberOfObjects(), nbobjwithinter=0;
		SceneType::ObjectInteractionMapType::iterator interationIt;

		unsigned int *indices = (unsigned int *)malloc(nbobj * sizeof(unsigned int));
		for (i=0 ; i<nbobj ; i++) {
			for (interationIt = m_scene->GetObjectByIndex(i)->interactions.begin() ; interationIt != m_scene->GetObjectByIndex(i)->interactions.end() ; ++interationIt) {
				if (interationIt->second.nbOverlappingPixels != 0) {
					indices[nbobjwithinter] = i;
					nbobjwithinter++;
					break;
				}
			}
		}
		
		//if no object overlap, exit.
		if (nbobjwithinter<2) { m_labelModifiedObject = 0; m_labelDeletedObject = 0; return; }

		m_labelModifiedObject = m_scene->GetLabelFromIndex( indices[m_rndgen->GetIntegerVariate(nbobjwithinter-1)] ); //select a random object that has interactions
		interationIt = m_scene->GetObjectByLabel(m_labelModifiedObject)->interactions.begin();

		if ( m_scene->GetObjectByLabel(m_labelModifiedObject)->interactions.size() > 1 ) {
			unsigned nbsteps = m_rndgen->GetIntegerVariate( m_scene->GetObjectByLabel(m_labelModifiedObject)->interactions.size()-1 );
			if (nbsteps>0) std::advance(interationIt, nbsteps);//select a object randomly among those which intersection this one
		}
		m_labelDeletedObject = interationIt->first;

		free(indices);
	}

	LabelType m_labelDeletedObject, m_labelModifiedObject; //the action of this kernel to modify the selected object (m_labelModifiedObject) and to delete a second one (m_labelDeletedObject)
	vnl_vector<double> m_deletedParameters, m_undoParameters; //initial parameters of the modified object, and of the deleted object
	unsigned int m_indexObjectType; // type of the deleted object
private:
	RJKernel_MergeRectangles(const Self&);				//purposely not implemented
	const Self & operator=( const Self & );		//purposely not implemented
};


} // namespace psciob

#endif /* RANDOMJUMPKERNEL_SPECIALIZEDRECTANGLES_H_ */
