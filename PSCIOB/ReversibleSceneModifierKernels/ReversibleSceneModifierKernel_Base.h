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
* \file ReversibleSceneModifierKernel_Base.h
* \author Rémi Blanc 
* \date 10. October 2011
*/

// TODO: write separate files for the different kernels


#ifndef _REVERSIBLESCENEMODIFIERKERNEL_BASE_H_
#define _REVERSIBLESCENEMODIFIERKERNEL_BASE_H_

//IDEA IS TO PROPOSE RANDOM JUMPS (BIRTH&DEATH ; OBJECT TYPE CHANGES ; ...) at each temperature change of the standard SA algorithm
//IDEA: is it possible to implement a number of "jump types", and to register them as requested
//type 1: birth & death
//type 2: type change
//type 3: ....? specific optimizer for a single object, ...
//4: random parameter changes according to some pdf (e.g. rotate one object by 90° around a random axis)
//5: split & merge
//6: combine multiple kernels => rotate an objcet by 90°, then apply a given optimizer to the result...


//implemented kernels: 
//BirthKernel
//DeathKernel
//SingleObjectOptimizationKernel
//RandomWalkOneRandomObjectKernel
//BirthAndOptimizeKernel

//RJKernel_ShrinkBadEllipse

//RJKernel_KillBadObject
//RJKernel_KillBadEllipse
//RJKernel_KillUnlikelyObject

#include "GeneralUtils.h"
#include "MultivariatePDF.h"
#include <Optimizer_Base.h>
#include <SceneOptimizationManager_Base.h>
#include <BaseScene.h>
#include "SingleSceneObjectOptimizationManager.h"

namespace psciob {

/** \class ReversibleSceneModifierKernel_Base
*\brief ReversibleSceneModifierKernel_Base: Base class for 'kernel moves'
* 
* A 'kernel move' is a proposition for modifying something in the scene ; a move may be undone if the optimizer rejects the proposition.
* move can be modification of some parameters, birth, death, split or merge of objects, ...
* mulitple birth and / or multiple death
* or even calling a deterministic optimizer on a object...
* It is typically called by a stochastic optimizer such as RJSA
*
* The method Apply attempts to modify the scene. If this modification is valid, the method returns a >=0 value, which should represent the likelihood score for acceptance
* If the method returns -1, no modifications occured to the scene.
*
* By convention, the Temperature, given as parameter to Apply is between 0 and 1.
* Typically, Simulated annealing starts at high temperature (~1) implying more randomness.
* As the temperature decrease, the propositions usually tend to become more and more deterministic
* however, not all kernels are affected by temperature.
*/
template<class TSceneType>
class ReversibleSceneModifierKernel_Base : public itk::LightObject {
public:
	/** Standard class typedefs. */
	typedef ReversibleSceneModifierKernel_Base					Self;
	typedef itk::LightObject				Superclass;
	typedef itk::SmartPointer<Self>			Pointer;
	/** Run-time type information (and related methods). */
	itkTypeMacro(ReversibleSceneModifierKernel_Base,itk::LightObject);

	typedef TSceneType                                 SceneType;
	typedef typename SceneType::IDType                 IDType;
	typedef typename SceneType::ObjectTypesLibraryType ObjectTypesLibraryType;
	typedef typename SceneType::DeformableObjectType   ObjectType;

	/** Set the scene on which the kernel works	*/
	virtual void SetScene(SceneType *scene) {	
		m_scene = scene;
		m_library = m_scene->GetObjectTypesLibrary();
	}	

	SceneType* GetScene()			{ return m_scene.GetPointer(); }

	/** Checks that all the parameters of the kernel have been set properly (e.g. the scene...) */
	virtual bool CheckKernelValidity() {
		if (!m_scene) { throw DeformableModelException("Kernel is invalid: the scene is not set"); return false; }
		return true;
	}

	/** Proposes a move from the current configuration, and returns the 'acceptance ratio'
	* The Temperature is given as an argument, which can be useful for controlling the amplitude of the move
	* output 0 means the move cannot be accepted ; -1 means is was not even possible to perform it...
	* It is up to the child class to implement precisely this output...
	*/
	virtual double Apply(double T) = 0;	//temperature is given as an argument ; in case this is useful for deciding the amplitude of the "move"...
	virtual void Undo() = 0;			//move the scene to the state it used to have the last time we called Apply

	/** re-initialize the internal random number generator 
	* use the code -1 to initialize it based on the clock.
	* by default, they are all initialized with seed=0
	*/
	void InitializeInternalRandomGenerator(int seed=0) {
		if (seed==-1) m_rndgen->Initialize();
		else m_rndgen->Initialize(seed);
	}

protected:
	ReversibleSceneModifierKernel_Base() { m_scene = NULL; m_rndgen = RandomVariableGenerator::New(); };
	virtual ~ReversibleSceneModifierKernel_Base() {};

	typename SceneType::Pointer					m_scene;
	typename ObjectTypesLibraryType::Pointer	m_library;
	RandomVariableGenerator::Pointer			m_rndgen;

private:
	ReversibleSceneModifierKernel_Base(const Self&);					//purposely not implemented
	const Self & operator=( const Self & );		//purposely not implemented
};



//TODO: host these specific kernel in different files...
//TODO: enable to use a specific pdf from the library by default , but also enable to use a custom pdf if the user requests it.
//do that for all kernels that work with random distributions.

/** \class BirthKernel
* \brief BirthKernel: propose to sprout a new object with random parameters
* The type of objects is randomly drawn within the types listed in the library associated to the scene
* The parameters, including the location of the object, are sampled from the corresponding PDF_OBJECTGENERATIONPRIOR entry in this library
* \sa ObjectTypesLibrary
*/
template<class TSceneType>
class BirthKernel : public ReversibleSceneModifierKernel_Base<TSceneType> {
public:
	/** Standard class typedefs. */
	typedef BirthKernel          Self;
	typedef ReversibleSceneModifierKernel_Base           Superclass;
	typedef itk::SmartPointer<Self> Pointer;
	/** Run-time type information (and related methods). */
	itkTypeMacro(BirthKernel, ReversibleSceneModifierKernel_Base);
	itkNewMacro(Self);

	/** Checks that all the parameters of the kernel have been set properly (e.g. the scene...) */
	bool CheckKernelValidity() {
		Superclass::CheckKernelValidity();
		if (m_library->GetNumberOfEntries()==0) { throw DeformableModelException("BirthKernel is not valid: no object type defined in the scene library"); return false; }
		for (unsigned i=0 ; i<m_library->GetNumberOfEntries() ; i++) {
			m_library->GetObjectPDF( i, PDF_OBJECTGENERATIONPRIOR ); //this function throws an exception if the PDF is not set... TODO: improve the message...
		}
		//TODO: check also that the PDFs that may be called when Applying really exist.
		return true;
	}

	/** Select the type of object to generate */
	inline void SelectType() { 
		m_requestedType = 0;
		if (m_library->GetNumberOfEntries()==1) return;
		double tmp = m_rndgen->GetUniformVariate(0, m_library->GetSumTypeWeights());
		while (tmp>0) {
			if ( tmp < m_library->GetObjectEntry(m_requestedType)->weight ) break;
			tmp-=m_library->GetObjectEntry(m_requestedType)->weight; m_requestedType++;
		}
	}

	//temperature is given as an argument ; in case this is useful for deciding the amplitude of the "move"...
	double Apply(double T) {
		//1: decide which type of object to generate - look at the scene registered types
		SelectType();
		//2: generate a object of the requested type, with random parameters from the given distribution
		ObjectType::Pointer obj = m_library->GenerateNewRandomObject(m_requestedType);
		//3: add it object to the scene
		m_labelToUndo = m_scene->AddObject(obj);	// what about specific object insertion policies??	//should child classes of Scene_Base force specific policies (e.g. label map ; or binary map <-> boolean model ; or ADD mode <-> shot noise, ...)

		if (m_labelToUndo==0) { return -1; }
		return 1;
	}

	//move the scene to the state it used to have the last time we called Apply
	void Undo() {
//std::cout<<"undoing birth of object with label "<<m_labelToUndo<<", params: "<<m_scene->GetParametersOfObject(m_labelToUndo)<<std::endl;
		if (m_labelToUndo!=0) m_scene->RemoveObject(m_labelToUndo);
	}

protected:
	BirthKernel() : ReversibleSceneModifierKernel_Base() {m_labelToUndo=0;};
	~BirthKernel() {};

	unsigned int m_requestedType;
	IDType m_labelToUndo;
private:
	BirthKernel(const Self&);				//purposely not implemented
	const Self & operator=( const Self & );		//purposely not implemented
};



//
/** \class DeathKernel
* \brief DeathKernel: propose to kill one random object
*/
template<class TSceneType>
class DeathKernel : public ReversibleSceneModifierKernel_Base<TSceneType> {
public:
	/** Standard class typedefs. */
	typedef DeathKernel					Self;
	typedef ReversibleSceneModifierKernel_Base					Superclass;
	typedef itk::SmartPointer<Self>			Pointer;
	/** Run-time type information (and related methods). */
	itkTypeMacro(DeathKernel,ReversibleSceneModifierKernel_Base);
	itkNewMacro(Self);

	/** Checks that all the parameters of the kernel have been set properly (e.g. the scene...) */
	bool CheckKernelValidity() {
		Superclass::CheckKernelValidity();
		return true;
	}

	//temperature is given as an argument ; in this case, it is not really useful...
	double Apply(double T) {
		//1: decide which object to kill
		if (m_scene->GetNumberOfObjects()<=1) { m_label=0; return -1; }	//if there is only 1 object remaining, don't kill it!!
		unsigned int requestedIndex = m_rndgen->GetIntegerVariate(m_scene->GetNumberOfObjects()-1);
		SceneObjectIterator<SceneType> objectIt(m_scene); objectIt.GoToBegin(); objectIt.Advance(requestedIndex);
		m_label = objectIt.GetID();
		if (m_label==0) return -1;

		//2: remember what this object is ... OPTIMIZATION: simplify things with the ObjectInScene structure (copy the associated data, and re-insert it properly in the undo method...)
		m_objectTypeId = m_scene->GetObject(m_label)->objectTypeId;
		m_params = m_scene->GetParametersOfObject(m_label);
		//3: kill the object
		m_scene->RemoveObject(m_label);
//std::cout<<"killing object with label: "<<m_label<<", and params: "<<m_params<<std::endl;
		return 1;
	}

	//move the scene to the state it used to have the last time we called Apply
	void Undo() { //resurect the dead object...
//std::cout<<"reviving object with label: "<<m_label<<", and params: "<<m_params<<std::endl;
		
		//1: create an object of the right type, and give it the old parameters...
		ObjectType::Pointer obj = m_library->GenerateNewObjectDefault(m_objectTypeId);
		if (!obj->SetParameters(m_params)) throw DeformableModelException("DeathKernel::Undo : unable to reassign the object parameters -- SHOULD NEVER HAPPEN");

		//2: re-add it to the scene - this should always succeed ... otherwise, there is trouble somewhere
		if (m_label!=m_scene->AddObject(obj, m_label)) throw DeformableModelException("DeathKernel::Undo : unable to undo the move properly -- SHOULD NEVER HAPPEN");
		//std::cout<<"undoing death of object with label: "<<m_label<<" ; assigned label: "<<tmp_label<<std::endl;
	}

protected:
	DeathKernel() : ReversibleSceneModifierKernel_Base() {};
	~DeathKernel() {};

	IDType m_label;
	unsigned int m_objectTypeId;
	vnl_vector<double> m_params;
private:
	DeathKernel(const Self&);				//purposely not implemented
	const Self & operator=( const Self & );		//purposely not implemented
};



/** \class RandomWalkOneRandomObjectKernel
* \brief RandomWalkOneRandomObjectKernel: propose a random modification of one (or more) random parameter(s) of one random object
* the modification follows the PDF indicated in the PDF_RANDOMWALK entry of the library for the selected object type.
* \sa ObjectTypesLibrary
* The Temperature parameter is initially used to dilate the PDF, and to shrink it by the end of the process
* the current implementation multiplies the perturbation vector by (0.5+T)
*/
template<class TSceneType>
class RandomWalkOneRandomObjectKernel : public ReversibleSceneModifierKernel_Base<TSceneType> {
public:
	/** Standard class typedefs. */
	typedef RandomWalkOneRandomObjectKernel     Self;
	typedef ReversibleSceneModifierKernel_Base           Superclass;
	typedef itk::SmartPointer<Self> Pointer;
	/** Run-time type information (and related methods). */
	itkTypeMacro(RandomWalkOneRandomObjectKernel, ReversibleSceneModifierKernel_Base);
	itkNewMacro(Self);

	/** Checks that all the parameters of the kernel have been set properly (e.g. the scene...) */
	virtual bool CheckKernelValidity() {
		Superclass::CheckKernelValidity();
		for (unsigned i=0 ; i<m_library->GetNumberOfEntries() ; i++) {
			m_library->GetObjectPDF( i, PDF_RANDOMWALK ); //this function throws an exception if the PDF is not set... TODO: improve the message...
		}
		return true;
	}

	/** Sets how many parameters are simultaneously modified by the kernel */
	void SetNumberOfSimultaneousMove(unsigned int n)  { m_nbSimultaneousMoves = n; }

	//temperature is given as an argument ; in case this is useful for deciding the amplitude of the "move"...
	double Apply(double T) {
		if ( m_scene->GetNumberOfObjects()==0 ) return -1;

		//1: select a random object to modify
		unsigned int requestedIndex = m_rndgen->GetIntegerVariate(m_scene->GetNumberOfObjects()-1);
		SceneObjectIterator<SceneType> objectIt(m_scene); objectIt.GoToBegin(); objectIt.Advance(requestedIndex);
		m_label = objectIt.GetID();
		if (m_label==0) return -1;

		//select the element(s) to be modified, and modify them...
		m_oldParams = m_scene->GetParametersOfObject(m_label);
		vnl_vector<double> tmp_sample = m_library->GetObjectPDF( m_scene->GetObject(m_label)->objectTypeId, PDF_RANDOMWALK )->DrawSample();

		vnl_vector<double> newParams = m_oldParams;

		unsigned int rnd_n;
		if (m_nbSimultaneousMoves==1) {//select one parameter randomly
			rnd_n = m_rndgen->GetIntegerVariate(m_oldParams.size()-1);
			newParams(rnd_n) += tmp_sample(rnd_n) * (T+0.5); //amplitude of the move decreases with the temperature (/time)
		}
		else {	//perform a random permutation on [1:nb_params] , and select the first elements of the results as the indices of the parameters to modify
			vnl_vector<unsigned int> order(m_oldParams.size());
			unsigned int tmp, rnd;
			//TODO: move this to a class ~> generic random permutation...
			for (unsigned i=0 ; i<m_oldParams.size() ; i++) {order(i)=i;}
			for (unsigned i=0 ; i<m_oldParams.size()-1 ; i++) {//swap it with a random candidate : algo from http://c-faq.com/lib/shuffle.html
				rnd_n = m_rndgen->GetIntegerVariate(m_oldParams.size()-1-i);
				tmp = order(i); order(i) = order(i+rnd_n); order(i+rnd_n) = tmp;
			}
			for (unsigned i=0 ; i<m_nbSimultaneousMoves ; i++) { newParams( order(i) ) += tmp_sample( order(i) ) * T; }
		}
		if (!m_scene->ModifyObjectParameters(m_label, newParams)) {return -1;} //the proposed parameters are not allowed, this is not really considered as a pure failure...
		return 1;
	}

	//move the scene to the state it used to have the last time we called Apply
	void Undo() { 
		if (!m_scene->ModifyObjectParameters(m_label, m_oldParams)) throw DeformableModelException("RandomWalkOneRandomObjectKernel::Undo : unable to undo the move properly -- SHOULD NEVER HAPPEN"); 
	} 

protected:
	RandomWalkOneRandomObjectKernel() : ReversibleSceneModifierKernel_Base() {m_nbSimultaneousMoves=1;};
	~RandomWalkOneRandomObjectKernel() {};

	vnl_vector<double> m_oldParams;
	unsigned int m_nbSimultaneousMoves;
	IDType m_label;

private:
	RandomWalkOneRandomObjectKernel(const Self&);			//purposely not implemented
	const Self & operator=( const Self & );		//purposely not implemented
};




//
//
////
////


//	CLARIFY WHAT SHALL BE DONE HERE
////
////RJKernel_ShrinkBadEllipse: this particular kernel assumes all objects in the scene are ellipses with rigid transform, it just reduces the short axis by 10% <=> 4th parameter of the object
////to select the object, they are sorted by decreasing value of interactionCost, a random number is drawn between 0 and sum(all interaction costs) ; ...
//template<class TScene>
//class RJKernel_ShrinkBadEllipse : public ReversibleSceneModifierKernel_Base<TScene> {
//public:
//	/** Standard class typedefs. */
//	typedef RJKernel_ShrinkBadEllipse		Self;
//	typedef ReversibleSceneModifierKernel_Base					Superclass;
//	typedef itk::SmartPointer<Self>			Pointer;
//	/** Run-time type information (and related methods). */
//	itkTypeMacro(RJKernel_ShrinkBadEllipse,ReversibleSceneModifierKernel_Base);
//	itkNewMacro(Self);
//
//
//	IDType SelectBadObject() { //this may be quite slow and inefficient --- what about caching these interaction cost terms?
//		std::vector<double> objectCosts; 
//		std::vector<IDType> labels;
//		double cost, sumcosts = 0;
//		for (unsigned i=0 ; i<m_scene->GetNumberOfObjects() ; i++) {
//			labels.push_back(m_scene->GetLabelFromIndex(i));
//			objectCosts.push_back( m_scene->GetObjectTotalInteractionCost(labels.back()) );
//			sumcosts+=cost;
//		}
//		for (SceneType::ObjectSetIteratorType it = m_scene->GetObjectSet()->begin() ; it!=m_scene->GetObjectSet()->end() ; it++) {
//			cost = 0;
//			//check if this object is in interaction with others, and add the costs
//			for (SceneType::ObjectInteractionMapType::iterator mapit = it->second->interactions.begin() ; mapit!=it->second->interactions.end() ; mapit++) {
//				cost += m_scene->GetInteractionManager()->GetPairWiseInteractionValue(it, m_scene->GetIteratorFromLabel(mapit->first));
//			}
//			objectCosts.push_back(cost);
//			labels.push_back(it->first);
//			sumcosts+=cost;
//		}
//
//		unsigned int nobj = objectCosts.size();
//		std::vector<unsigned int> indices;		for (unsigned i = 0 ; i < nobj ; i++) { indices.push_back(i); }
//		std::sort(indices.begin(), indices.end(), index_less<std::vector<double>&>(objectCosts));
//		//sorted in ascending order
//		double threshold = m_rndgen->GetUniformVariate(0, sumcosts);
//		double cumsum = 0;
//		for (unsigned i=0 ; i<nobj; i++) {
//			//look at the object, sorted in descending order ; stop on the first object for which the cumulated cost is larger than the threshold...
//			//if there is one object with a huge cost compared to all the others ; it will be selected with very high probability
//			cumsum+=objectCosts[ indices[nobj-1-i] ];
//			if (cumsum>=threshold) return labels[ indices[nobj-1-i] ];
//		}
//		return labels[indices[nobj-1]];
//	}
//
//	//temperature is given as an argument ; in case this is useful for deciding the amplitude of the "move"...
//	double Apply(double T) {
//		//1: decide which object should be shrunk
//		m_labelToUndo = SelectBadObject();
//		//2: backup the previous value
//		m_oldParams = m_scene->GetParametersOfObject(m_labelToUndo);
//		//3: shrink it by 10%
//		vnl_vector<double> newParams = m_oldParams;	newParams(3) = 0.9 * m_oldParams(3);
//		//4: modify the parameters of the object.
//std::cout<<"shrinking the radius of ellipse with label: "<<m_oldParams<<", old value: "<<m_oldParams(3)<<", new value: "<<newParams(3)<<std::endl;
//		return m_scene->ModifyObjectParameters(m_labelToUndo, newParams);
//	}
//
//	//move the scene to the state it used to have the last time we called Apply
//	void Undo() {
//		m_scene->ModifyObjectParameters(m_labelToUndo, m_oldParams);
//	}
//
//protected:
//	RJKernel_ShrinkBadEllipse() : ReversibleSceneModifierKernel_Base() {};
//	~RJKernel_ShrinkBadEllipse() {};
//
//	IDType m_labelToUndo; 
//	vnl_vector<double> m_oldParams;
//
//private:
//	RJKernel_ShrinkBadEllipse(const Self&);		//purposely not implemented
//	const Self & operator=( const Self & );		//purposely not implemented
//
//};
//
//
///**
// * RJKernel_KillBadObject: kill an object that have a bad ( interaction cost + likelihood )
// * assumes the temperature is between 0 and 1 !!! at low temperature, it will select the object with the highest cost
// * ... CHECK THE IMPLEMENTATION -- it seems to be mixing things between dataCost and PriorCost...
//*/
//template<class TScene>
//class RJKernel_KillBadObject : public ReversibleSceneModifierKernel_Base<TScene> {
//public:
//	/** Standard class typedefs. */
//	typedef RJKernel_KillBadObject		Self;
//	typedef ReversibleSceneModifierKernel_Base				Superclass;
//	typedef itk::SmartPointer<Self>		Pointer;
//	/** Run-time type information (and related methods). */
//	itkTypeMacro(RJKernel_KillBadObject,ReversibleSceneModifierKernel_Base);
//	itkNewMacro(Self);
//
//
//	typename SceneType::IDType SelectBadObject(double T) {
//		std::vector<double> objectCosts; 
//		std::vector<unsigned int> labels;
//		SceneType::ObjectSetType* objectSet = m_scene->GetObjectSet();
//		if (objectSet->size()<=1) {return 0;}	//if there is only 1 object remaining, don't kill it!!
//		double cost, sumcosts = 0;
//		for (SceneType::ObjectSetIteratorType it = m_scene->GetObjectSet()->begin() ; it!=m_scene->GetObjectSet()->end() ; it++) {
//			cost = 0;
//			//look at the object likelihood
//			if (it->second->dataCostFlag) { cost = it->second->dataCost; }
//			else {//update the value if necessary ;  use ( - loglikelihood ) so that high value means bad object... i.e. generate a COST function
//				cost = - m_library->GetObjectPDF( it->second->objectTypeId, PDF_OBJECTLIKELIHOOD )->GetLogLikelihood(it->second->object->GetParameters());
//				it->second->priorCost = cost;	it->second->priorCostFlag = true;
//			}
//			//check if this object is in interaction with others, and add the costs
//			for (SceneType::ObjectInteractionMapType::iterator mapit = it->second->interactions.begin() ; mapit!=it->second->interactions.end() ; mapit++) {
//				cost += m_scene->GetInteractionManager()->GetPairWiseInteractionValue(it, m_scene->GetIteratorFromLabel(mapit->first));
//			}
//			objectCosts.push_back(cost);
//			labels.push_back(it->first);
//			sumcosts+=cost;
//		}
//
//		unsigned int nobj = objectCosts.size();
//		std::vector<unsigned int> indices;		for (unsigned i = 0 ; i < nobj ; i++) { indices.push_back(i); }
//		std::sort(indices.begin(), indices.end(), index_less<std::vector<double>&>(objectCosts));
//		//sorted in ascending order
//		double threshold = m_rndgen->GetUniformVariate(0, sumcosts) * T;
//		double cumsum = 0;
//		for (unsigned i=0 ; i<nobj; i++) {
//			//look at the object, sorted in descending order ; stop on the first object for which the cumulated cost is larger than the threshold...
//			//if there is one object with a huge cost compared to all the others ; it will be selected with very high probability
//			cumsum+=objectCosts[ indices[nobj-1-i] ];
//			if (cumsum>=threshold) return labels[ indices[nobj-1-i] ];
//		}
//		return labels[indices[nobj-1]];
//	}
//
//	//temperature is given as an argument ; in case this is useful for deciding the amplitude of the "move"...
//	double Apply(double T) {
//		//1: decide which object should be shrunk
//		m_labelToUndo = SelectBadObject(T);
//		if (m_labelToUndo==0) return false;
//		//2: remember what this object is
//		SceneType::ObjectSetType::iterator it = m_scene->GetIteratorFromLabel(m_labelToUndo);
//		m_objectTypeId = it->second->objectTypeId;
//		m_insertionMode = it->second->insertionMode;
//		m_oldParams = it->second->object->GetParameters();
//		//3: kill the object
//		m_scene->RemoveObject(m_labelToUndo);
//		return true;
//	}
//
//	//move the scene to the state it used to have the last time we called Apply
//	void Undo() { //resurect the dead object...
//		//1: create an object of the right type, and give it the old parameters...
//		ObjectType::Pointer obj = m_library->GenerateNewObjectDefault(m_objectTypeId);
//		obj->SetParameters(m_oldParams); 
//		//2: re-add it to the scene - this should always succeed ... otherwise, there is trouble somewhere
//		if ( m_labelToUndo!=m_scene->AddObject(obj, m_labelToUndo) ) throw DeformableModelException("RJKernel_KillBadObject::Undo : unable to undo the move properly -- SHOULD NEVER HAPPEN");
//	}
//
//protected:
//	RJKernel_KillBadObject() : ReversibleSceneModifierKernel_Base() {};
//	~RJKernel_KillBadObject() {};
//
//	typename SceneType::IDType m_labelToUndo; vnl_vector<double> m_oldParams;
//	SCENEDRAWMODE m_insertionMode;	
//	unsigned int m_objectTypeId;
//
//private:
//	RJKernel_KillBadObject(const Self&);		//purposely not implemented
//	const Self & operator=( const Self & );		//purposely not implemented
//};
//
//
////
////RJKernel_KillBadlyFittingObject: kill an object that have a bad data cost
////assumes the temperature is between 0 and 1 !!! at low temperature, it will select the object with the highest cost
//template<class TScene>
//class RJKernel_KillBadlyFittingObject : public ReversibleSceneModifierKernel_Base<TScene> {
//public:
//	/** Standard class typedefs. */
//	typedef RJKernel_KillBadlyFittingObject	Self;
//	typedef ReversibleSceneModifierKernel_Base					Superclass;
//	typedef itk::SmartPointer<Self>			Pointer;
//	/** Run-time type information (and related methods). */
//	itkTypeMacro(RJKernel_KillBadObject,ReversibleSceneModifierKernel_Base);
//	itkNewMacro(Self);
//
//	typedef SingleSceneObjectOptimizationManager<TScene>	OptimizationManagerType;
//	void SetOptimizationManager(OptimizationManagerType *manager) {m_manager = manager;}
//
//	unsigned int SelectBadObject(double T) {
//		std::vector<double> objectCosts; 
//		std::vector<unsigned int> labels;
//		SceneType::ObjectSetType* objectSet = m_scene->GetObjectSet();
//		if (objectSet->size()<=1) {return 0;}	//if there is only 1 object remaining, don't kill it!!
//		double cost, sumcosts = 0;
//		for (SceneType::ObjectSetIteratorType it = m_scene->GetObjectSet()->begin() ; it!=m_scene->GetObjectSet()->end() ; it++) {
//			cost = 0;
//			if ( it->second->dataCostFlag ) cost = it->second->dataCost;
//			else {
//				m_manager->SelectObjectToOptimize(it);
//				cost = m_manager->GetValue();
//			}			
//
//			objectCosts.push_back(cost);
//			labels.push_back(it->first);
//			sumcosts+=cost;
//		}
//
//		unsigned int nobj = objectCosts.size();
//		std::vector<unsigned int> indices;		for (unsigned i = 0 ; i < nobj ; i++) { indices.push_back(i); }
//		std::sort(indices.begin(), indices.end(), index_less<std::vector<double>&>(objectCosts));
//		//sorted in ascending order
//		double threshold = m_rndgen->GetUniformVariate(0, sumcosts) * T;
//		double cumsum = 0;
//		for (unsigned i=0 ; i<nobj; i++) {
//			//look at the object, sorted in descending order ; stop on the first object for which the cumulated cost is larger than the threshold...
//			//if there is one object with a huge cost compared to all the others ; it will be selected with very high probability
//			cumsum+=objectCosts[ indices[nobj-1-i] ];
//			if (cumsum>=threshold) return labels[ indices[nobj-1-i] ];
//		}
//		return labels[indices[nobj-1]];
//	}
//
//	//temperature is given as an argument ; in case this is useful for deciding the amplitude of the "move"...
//	double Apply(double T) {
//		//1: decide which object should be shrunk
//		m_labelToUndo = SelectBadObject(T);
//		if (m_labelToUndo==0) return false;
//		//2: remember what this object is
//		SceneType::ObjectSetType::iterator it = m_scene->GetIteratorFromLabel(m_labelToUndo);
//		m_objectTypeId = it->second->objectTypeId;
//		m_insertionMode = it->second->insertionMode;
//		m_oldParams = it->second->object->GetParameters();
//		//3: kill the object
//		m_scene->RemoveObject(m_labelToUndo);
//		return true;
//	}
//
//	//move the scene to the state it used to have the last time we called Apply
//	void Undo() { //resurect the dead object...
//		//1: create an object of the right type, and give it the old parameters...
//		ObjectType::Pointer obj = m_library->GenerateNewObjectDefault(m_objectTypeId);
//		obj->SetParameters(m_oldParams); 
//		//2: re-add it to the scene - this should always succeed ... otherwise, there is trouble somewhere
//		unsigned int tmp_label;
//		tmp_label = m_scene->AddObject(obj, m_labelToUndo);//CLEAN: check if this went well?
//		//std::cout<<"undoing death of object with label: "<<m_label<<" ; assigned label: "<<tmp_label<<std::endl;
//	}
//
//protected:
//	RJKernel_KillBadlyFittingObject() : ReversibleSceneModifierKernel_Base() {};
//	~RJKernel_KillBadlyFittingObject() {};
//
//	unsigned int m_labelToUndo; vnl_vector<double> m_oldParams;
//	SCENEDRAWMODE m_insertionMode;	
//	unsigned int m_objectTypeId;
//	typename OptimizationManagerType::Pointer m_manager;
//
//private:
//	RJKernel_KillBadlyFittingObject(const Self&);		//purposely not implemented
//	const Self & operator=( const Self & );		//purposely not implemented
//};
//
//
////
////RJKernel_KillBadInteractionObject: kill an object that have a bad interaction cost
////assumes the temperature is between 0 and 1 !!! at low temperature, it will select the object with the highest cost
//template<class TScene>
//class RJKernel_KillBadInteractionObject : public ReversibleSceneModifierKernel_Base<TScene> {
//public:
//	/** Standard class typedefs. */
//	typedef RJKernel_KillBadInteractionObject		Self;
//	typedef ReversibleSceneModifierKernel_Base					Superclass;
//	typedef itk::SmartPointer<Self>			Pointer;
//	/** Run-time type information (and related methods). */
//	itkTypeMacro(RJKernel_KillBadInteractionObject,ReversibleSceneModifierKernel_Base);
//	itkNewMacro(Self);
//
//
//	unsigned int SelectBadObject(double T) {
//		std::vector<double> objectCosts; 
//		std::vector<unsigned int> labels;
//		SceneType::ObjectSetType* objectSet = m_scene->GetObjectSet();
//		if (objectSet->size()<=1) {return 0;}	//if there is only 1 object remaining, don't kill it!!
//		double cost, sumcosts = 0;
//		for (SceneType::ObjectSetIteratorType it = m_scene->GetObjectSet()->begin() ; it!=m_scene->GetObjectSet()->end() ; it++) {
//			cost = 0;
//			//check if this object is in interaction with others, and add the costs
//			for (SceneType::ObjectInteractionMapType::iterator mapit = it->second->interactions.begin() ; mapit!=it->second->interactions.end() ; mapit++) {
//				cost += m_scene->GetInteractionManager()->GetPairWiseInteractionValue(it, m_scene->GetIteratorFromLabel(mapit->first));
//			}
//			objectCosts.push_back(cost);
//			labels.push_back(it->first);
//			sumcosts+=cost;
//		}
//
//		unsigned int nobj = objectCosts.size();
//		std::vector<unsigned int> indices;		for (unsigned i = 0 ; i < nobj ; i++) { indices.push_back(i); }
//		std::sort(indices.begin(), indices.end(), index_less<std::vector<double>&>(objectCosts));
//		//sorted in ascending order
//		double threshold = m_rndgen->GetUniformVariate(0, sumcosts) * T;
//		double cumsum = 0;
//		for (unsigned i=0 ; i<nobj; i++) {
//			//look at the object, sorted in descending order ; stop on the first object for which the cumulated cost is larger than the threshold...
//			//if there is one object with a huge cost compared to all the others ; it will be selected with very high probability
//			cumsum+=objectCosts[ indices[nobj-1-i] ];
//			if (cumsum>=threshold) return labels[ indices[nobj-1-i] ];
//		}
//		return labels[indices[nobj-1]];
//	}
//
//	//temperature is given as an argument ; in case this is useful for deciding the amplitude of the "move"...
//	double Apply(double T) {
//		//1: decide which object should be shrunk
//		m_labelToUndo = SelectBadObject(T);
//		if (m_labelToUndo==0) return false;
//		//2: remember what this object is
//		SceneType::ObjectSetType::iterator it = m_scene->GetIteratorFromLabel(m_labelToUndo);
//		m_objectTypeId = it->second->objectTypeId;
//		m_insertionMode = it->second->insertionMode;
//		m_oldParams = it->second->object->GetParameters();
//		//3: kill the object
//		m_scene->RemoveObject(m_labelToUndo);
//		return true;
//	}
//
//	//move the scene to the state it used to have the last time we called Apply
//	void Undo() { //resurect the dead object...
//		//1: create an object of the right type, and give it the old parameters...
//		ObjectType::Pointer obj = m_library->GenerateNewObjectDefault(m_objectTypeId);
//		obj->SetParameters(m_oldParams); 
//		//2: re-add it to the scene - this should always succeed ... otherwise, there is trouble somewhere
//		unsigned int tmp_label;
//		tmp_label = m_scene->AddObject(obj, m_labelToUndo);//CLEAN: check if this went well?
//		//std::cout<<"undoing death of object with label: "<<m_label<<" ; assigned label: "<<tmp_label<<std::endl;
//	}
//
//protected:
//	RJKernel_KillBadInteractionObject() : ReversibleSceneModifierKernel_Base() {};
//	~RJKernel_KillBadInteractionObject() {};
//
//	unsigned int m_labelToUndo; vnl_vector<double> m_oldParams;
//	SCENEDRAWMODE m_insertionMode;	
//	unsigned int m_objectTypeId;
//
//private:
//	RJKernel_KillBadInteractionObject(const Self&);		//purposely not implemented
//	const Self & operator=( const Self & );		//purposely not implemented
//};
//
////
////RJKernel_KillUnlikelyObject: kill an object that have a low likelihood
////assumes the temperature is between 0 and 1 !!! at low temperature, it will select the object with the highest cost
//template<class TScene>
//class RJKernel_KillUnlikelyObject : public ReversibleSceneModifierKernel_Base<TScene> {
//public:
//	/** Standard class typedefs. */
//	typedef RJKernel_KillUnlikelyObject		Self;
//	typedef ReversibleSceneModifierKernel_Base					Superclass;
//	typedef itk::SmartPointer<Self>			Pointer;
//	/** Run-time type information (and related methods). */
//	itkTypeMacro(RJKernel_KillUnlikelyObject,ReversibleSceneModifierKernel_Base);
//	itkNewMacro(Self);
//
//
//	unsigned int SelectBadObject(double T) { //this may be quite slow and inefficient --- what about caching these interaction cost terms?
//		std::vector<double> objectCosts; 
//		std::vector<unsigned int> labels;
//		SceneType::ObjectSetType* objectSet = m_scene->GetObjectSet();
//		if (objectSet->size()<=1) {return 0;}	//if there is only 1 object remaining, don't kill it!!
//		double cost, sumcosts = 0;
//		for (SceneType::ObjectSetIteratorType it = m_scene->GetObjectSet()->begin() ; it!=m_scene->GetObjectSet()->end() ; it++) {
//			cost = 0;
//			if (it->second->priorCostFlag) { cost = it->second->priorCost; }
//			else {//update the value if necessary ;  use ( - loglikelihood ) so that high value means bad object... i.e. generate a COST function
//				cost = - m_library->GetObjectPDF( it->second->objectTypeId, PDF_OBJECTLIKELIHOOD )->GetLogLikelihood(it->second->object->GetParameters());
//				it->second->priorCost = cost;	it->second->priorCostFlag = true;
//			}
//			objectCosts.push_back(cost);
//			labels.push_back(it->first);
//			sumcosts+=cost;
//		}
//
//		unsigned int nobj = objectCosts.size();
//		std::vector<unsigned int> indices;		for (unsigned i = 0 ; i < nobj ; i++) { indices.push_back(i); }
//		std::sort(indices.begin(), indices.end(), index_less<std::vector<double>&>(objectCosts));
//		//sorted in ascending order
//		double threshold = m_rndgen->GetUniformVariate(0, sumcosts) * T;
//		double cumsum = 0;
//		for (unsigned i=0 ; i<nobj; i++) {
//			//look at the object, sorted in descending order ; stop on the first object for which the cumulated cost is larger than the threshold...
//			//if there is one object with a huge cost compared to all the others ; it will be selected with very high probability
//			cumsum+=objectCosts[ indices[nobj-1-i] ];
//			if (cumsum>=threshold) return labels[ indices[nobj-1-i] ];
//		}
//		return labels[indices[nobj-1]];
//	}
//
//	//temperature is given as an argument ; in case this is useful for deciding the amplitude of the "move"...
//	double Apply(double T) {
//		//1: decide which object should be shrunk
//		m_labelToUndo = SelectBadObject(T);
//		if (m_labelToUndo==0) return false;
//		//2: remember what this object is
//		SceneType::ObjectSetType::iterator it = m_scene->GetIteratorFromLabel(m_labelToUndo);
//		m_objectTypeId = it->second->objectTypeId;
//		m_insertionMode = it->second->insertionMode;
//		m_oldParams = it->second->object->GetParameters();
//		//3: kill the object
//		m_scene->RemoveObject(m_labelToUndo);
//		return true;
//	}
//
//	//move the scene to the state it used to have the last time we called Apply
//	void Undo() { //resurect the dead object...
//		//1: create an object of the right type, and give it the old parameters...
//		ObjectType::Pointer obj = m_library->GenerateNewObjectDefault(m_objectTypeId);
//		obj->SetParameters(m_oldParams); 
//		//2: re-add it to the scene - this should always succeed ... otherwise, there is trouble somewhere
//		unsigned int tmp_label;
//		tmp_label = m_scene->AddObject(obj, m_labelToUndo);//CLEAN: check if this went well?
//		//std::cout<<"undoing death of object with label: "<<m_label<<" ; assigned label: "<<tmp_label<<std::endl;
//	}
//
//protected:
//	RJKernel_KillUnlikelyObject() : ReversibleSceneModifierKernel_Base() {};
//	~RJKernel_KillUnlikelyObject() {};
//
//	unsigned int m_labelToUndo; vnl_vector<double> m_oldParams;
//	SCENEDRAWMODE m_insertionMode;	
//	unsigned int m_objectTypeId;
//
//private:
//	RJKernel_KillUnlikelyObject(const Self&);		//purposely not implemented
//	const Self & operator=( const Self & );		//purposely not implemented
//};

//
///**\class RJKernel_MultipleBirthAndDeathWithBirthMapAndSceneEnergy
//*  \brief RJKernel_MultipleBirthAndDeathWithBirthMapAndSceneEnergy
//* Sprout several candidates according to a provided birth map
//* Use an energy term including single object goodness-of-fit to a reference image
//* Sort them according to the this data cost
//* Browse them in this order, and kill objects with some probability if this does not penalize the energy 
//* (if the energy is higher with an object, the probability that it is kept diminishes as the penalty is large)
//*/
//template<class TScene, class TEnergy>
//class RJKernel_MultipleBirthAndDeathWithBirthMapAndSceneEnergy : public ReversibleSceneModifierKernel_Base<TScene> {
//public:
//	/** Standard class typedefs. */
//	typedef RJKernel_MultipleBirthAndDeathWithBirthMapAndSceneEnergy		Self;
//	typedef ReversibleSceneModifierKernel_Base					Superclass;
//	typedef itk::SmartPointer<Self>			Pointer;
//	/** Run-time type information (and related methods). */
//	itkTypeMacro(RJKernel_MultipleBirthAndDeathWithBirthMapAndSceneEnergy,ReversibleSceneModifierKernel_Base);
//	itkNewMacro(Self);
//
//	typedef itk::Image<float, TScene::Dimension>	ProbaImageType;
//	void SetBirthProbabilityMap(ProbaImageType *image) { m_birthMap = image; }
//
//	typedef TEnergy EnergyType;
//
//	void SetEnergyFunction(EnergyType *energy) { m_energy = energy; }
//
//	//
//	unsigned int SelectType() { //select an object type randomly among the one registered in the library
//		double tmp = m_rndgen->GetUniformVariate(0, m_library->GetSumTypeWeights());
//		unsigned int type = 0;
//		while (tmp>0) {
//			if ( tmp < m_library->GetObjectEntry(type)->weight ) break;
//			tmp-=m_library->GetObjectEntry(type)->weight; type++;
//		}
//		return type;
//	}
//
//	//
//	double Apply(double T) {
//		m_typesBorn.clear(); m_labelsBorn.clear(); m_typesDead.clear(); m_labelsDead.clear(); m_paramsDead.clear();
//		
//		std::set<IDType> initialLabels;		
//		for (unsigned i=0 ; i<m_scene->GetNumberOfObjects() ; i++) { initialLabels.insert( m_scene->GetLabelFromIndex(i) ); } //OPTIMIZATION: start from the end, and hint the insertion at the beginning
//		
//		ObjectType::Pointer obj; IDType tmp_label; unsigned tmp_type; vnl_vector<double> params; 		
//		clock_t t0=clock(), t1;
//		// BIRTHS
//		// browse through the images (their intersection, in case they are not perfectly overlapping... however, they are expected to have the same resolution...)
//		ProbaImageType::RegionType birthRegion = m_birthMap->GetLargestPossibleRegion();
//		TScene::LabelImageType::RegionType sceneRegion = m_scene->GetSceneImageRegion();
//
//		//WARNING - check that the birth map has the same grid as the scene, to ensure correct
//
//		typedef itk::ImageRegionConstIterator< ProbaImageType >					 IteratorTypeA;	IteratorTypeA birthMapIt( m_birthMap, birthRegion );
//
//		//OPTIMIZATION: isntead, interpolate the probability on the birthMap, this would also enable using different resolutions between the 2 images...
//		//other possibility would be to iterate on the smallest image of the 2 (in terms of number of pixels), and either interpolate the proba map, or reduce the nb of authorized birth locations => can induce a great gain, interesting if there is a subsequent optimization step on the parameters of the objects...
//
//
//		ProbaImageType::IndexType index; ProbaImageType::SpacingType spacing = m_scene->GetSceneSpacing();
//		for ( birthMapIt.GoToBegin() ; !birthMapIt.IsAtEnd(); ++birthMapIt ) { 
//			if ( m_rndgen->GetUniformVariate(0, 1) < 0.001*T*birthMapIt.Get()) { //then sprout an object at the current location with a probability related to T and birthMapIt.Get()
//				tmp_type = SelectType(); obj = m_library->GenerateNewRandomObject(tmp_type);
//				params = obj->GetParameters();
//				//index = sceneImageIt.GetIndex();
//				index = birthMapIt.GetIndex();
//				for (unsigned i=0 ; i<TScene::Dimension ; i++) params(i) = index[i]*spacing[i];
//				obj->SetParameters(params);					
//				tmp_label = m_scene->AddObject(obj);
//				if (tmp_label!=0) { 
//					m_labelsBorn.push_back(tmp_label); m_typesBorn.push_back(tmp_type); 
//				}
//			}
//		}
//		std::cout<<"  nb of candidates proposed: "<<m_labelsBorn.size()<<" ; time to propose candidates: "<<(clock()-t0)/((double)CLOCKS_PER_SEC)<<" s."<<std::endl;		
//
//		//SORT OBJECTS WITH RESPECT TO THEIR DATA COST, IN DESCENDING ORDER
//		unsigned int nobj = m_scene->GetNumberOfObjects();
//		std::vector<double> objectDataCosts; std::vector<TScene::IDType> labels;
//
//		m_energy->GetValue();
//		std::cout<<"  time to compute energy: "<<(clock()-t0)/((double)CLOCKS_PER_SEC)<<" s."<<std::endl;		
//		std::vector<unsigned int> indices;
//		for (unsigned i=0 ; i < nobj ; i++) {
//			tmp_label = m_scene->GetLabelFromIndex(i);
//
//			objectDataCosts.push_back( m_energy->GetSingleObjectCostForObject(tmp_label) ); 
//			labels.push_back(tmp_label);
//			indices.push_back(i);
//		}
//		std::cout<<"  time to compute individual data costs: "<<(clock()-t0)/((double)CLOCKS_PER_SEC)<<" s."<<std::endl;		
//
//		std::sort(indices.begin(), indices.end(), index_more<std::vector<double>&>(objectDataCosts)); //sorted in descending order
//
//		//KILL OBJECTS... look at their contribution to the energy, compare it to the min & max of all objects (normalize...)
//		double costWith, costWithout, normalizedDiff;
//		IDType backupLabel; vnl_vector<double> backupParams; unsigned int backupType;
//
//		std::cout<<"  time before killing objects: "<<(clock()-t0)/((double)CLOCKS_PER_SEC)<<" s. "<<std::endl;		
//		for (unsigned i=0 ; i<indices.size() ; i++) {
//			costWith = m_energy->GetValue();
//
//			backupLabel = labels[indices[i]];
//			backupParams = m_scene->GetParametersOfObject(backupLabel);
//			backupType = m_scene->GetObjectByLabel(backupLabel)->objectTypeId;
//
//			m_scene->RemoveObject( labels[indices[i]] );
//
//			costWithout = m_energy->GetValue();
//
//			normalizedDiff = exp( - (costWithout-costWith) / T );
//			//std::cout<<"testing object with label "<<backupLabel<<", cost with: "<<costWith<<", without: "<<costWithout<<", diff = "<<costWithout-costWith<<"normalized diff: "<<normalizedDiff<<", proba threshold: "<<normalizedDiff / (1.0+normalizedDiff)<<std::endl;
//			if ( m_rndgen->GetUniformVariate(0, 1) > normalizedDiff / (1.0+normalizedDiff) ) {
//				//revives the object if the cost has increased too much...
//				ObjectType::Pointer obj;
//				obj = m_library->GenerateNewObjectDefault(backupType);
//				obj->SetParameters(backupParams); 
//				tmp_label = m_scene->AddObject(obj, backupLabel);
//				//sanity check, but it should never happen
//				if (tmp_label!=backupLabel) std::cout<<"warning in MultipleBirthAndDeathWithBirthMap: could not revive an object with its initial label... assigned label: "<<tmp_label<<" instead of "<<backupLabel<<" ; THIS SHOULD NEVER HAPPEN"<<std::endl;
//			}
//			else { //do some backup in case...
//				if ( initialLabels.find(backupLabel) != initialLabels.end() ) {
//					m_labelsDead.push_back( backupLabel );
//					m_typesDead.push_back( backupType );
//					m_paramsDead.push_back( backupParams );
//				}
//			}
//		}
//
//		std::cout<<"   energy: "<<m_energy->GetValue()<<", nb objects: "<<m_scene->GetNumberOfObjects()<<" - time spent in this iteration: "<<(clock()-t0)/((double)CLOCKS_PER_SEC)<<" s."<<std::endl;
//
//		return true;
//	}
//
//	//move the scene to the state it used to have the last time we called Apply
//	void Undo() { //kill the new objects, and resurect those that were deleted
//		ObjectType::Pointer obj; unsigned int tmp_label;
//		//revive old object
//		for (unsigned i=0 ; i<m_labelsDead.size() ; i++) {
//			obj = m_library->GenerateNewObjectDefault(m_typesDead[i]);
//			obj->SetParameters(m_paramsDead[i]); 
//			tmp_label = m_scene->AddObject(obj, m_labelsDead[i]);
//			//sanity check, but it should never happen
//			if (tmp_label!=m_labelsDead[i]) std::cout<<"warning in MultipleBirthAndDeathWithBirthMap: could not revive an object with its initial label... assigned label: "<<tmp_label<<" instead of "<<m_labelsDead[i]<<std::endl;
//		}
//		//kill the new
//		for (unsigned i=0 ; i<m_labelsBorn.size() ; i++) m_scene->RemoveObject(m_labelsBorn[i]);
//	}
//
//protected:
//	RJKernel_MultipleBirthAndDeathWithBirthMapAndSceneEnergy() : ReversibleSceneModifierKernel_Base() {};
//	~RJKernel_MultipleBirthAndDeathWithBirthMapAndSceneEnergy() {};
//
//	std::vector<unsigned int> m_typesBorn, m_typesDead;
//	std::vector<IDType> m_labelsBorn, m_labelsDead;
//	std::vector<vnl_vector<double>> m_paramsDead;
//	typename ProbaImageType::Pointer m_birthMap;
//	typename EnergyType::Pointer m_energy;
//
//private:
//	RJKernel_MultipleBirthAndDeathWithBirthMapAndSceneEnergy(const Self&);	//purposely not implemented
//	const Self & operator=( const Self & );			//purposely not implemented
//};
//
//


//
//

} // namespace psciob

#endif /* _REVERSIBLESCENEMODIFIERKERNEL_BASE_H_ */
