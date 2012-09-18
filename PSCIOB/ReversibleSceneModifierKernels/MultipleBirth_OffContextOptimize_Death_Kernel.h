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
* \file MultipleBirth_OffContextOptimize_Death_Kernel.h
* \author Rémi Blanc 
* \date 20. June 2012
*/



#ifndef __MULTIPLEBIRTH_OFFCONTEXTOPTIMIZE_DEATH_KERNEL_H_
#define __MULTIPLEBIRTH_OFFCONTEXTOPTIMIZE_DEATH_KERNEL_H_

#include <KernelWithLocalSingleObjectOptimizer_Base.h>
#include "SingleSceneObjectOptimizationManager.h"

#include "UnivariatePDF.h"


namespace psciob {


/**\class MultipleBirth_OffContextOptimize_Death_Kernel
* \brief MultipleBirth_OffContextOptimize_Death_Kernel
* This kernel works in three steps:
* 1/ Birth: A random number of new objects is added to the scene, according to a poisson density with parameter lambda*T, where lambda is an input parameter (base number of object per unit surface in the scene)
* 2/ Optimize: Each old and new object is optimized independently from all the others using a SingleSceneObjectOptimizationManager (no interaction costs included)
* 3/ Death: sort all the objects according to their independent costs, and iteratively kill them according to their full cost (~ or scene energy with & without them)
*
* IMPORTANT: for this class, the local optimization manager is generated automatically from the provided manager.
*            It clones the cost function, which is assumed to derive from a SingleObjectEnergy, and only switches off the interaction component.
*/

//note, in the optimize step: another version could try only to optimize the new objects
// optimizing both old and new objects implies that some steps involve a non-gradient-descent-based modifications of these objects
//            otherwise, it just does not make any sense

//ideas for other classes:
//Idea1 Birth: an input Birth map should be provided, indicating the probability to sprout a pixel at each pixel
//        this map shall be updated at the end 
// \sa SingleSceneObjectOptimizationManager

// ++ make the birth map dynamic, depending on the hypotheses that were generated, and their evaluation results.

//Idea2: specialized versions with assumptions on the involved objects could use other maps, suggesting some parameters for newly born objects (orientation, size, ...)


template<class TScene>
class MultipleBirth_OffContextOptimize_Death_Kernel : public KernelWithLocalSingleObjectOptimizer_Base<TScene> {
public:
	/** Standard class typedefs. */
	typedef MultipleBirth_OffContextOptimize_Death_Kernel Self;
	typedef KernelWithLocalSingleObjectOptimizer_Base     Superclass;
	typedef itk::SmartPointer<Self>                       Pointer;
	/** Run-time type information (and related methods). */
	itkTypeMacro(MultipleBirth_OffContextOptimize_Death_Kernel, KernelWithLocalSingleObjectOptimizer_Base);
	itkNewMacro(Self);

	typedef SceneOptimizationManager_Base<SceneType> GlobalOptimizationManagerType;

	/** Set the global optimization manager, which is important to get the global cost of an object... */
	void SetGlobalOptimizationManager(GlobalOptimizationManagerType *m) {
		m_globalOptimizationManager = m; //-> and also set the scene at the same time... while we are at it...
	}

	/** Set the scene on which the kernel works	
	* creates another scene of the same type to perform internal, off context optimization without computing any interaction.
	*/
	virtual void SetScene(SceneType *scene) {	
		Superclass::SetScene(scene);
		m_nbPixelsScene = m_scene->GetSceneImageRegion().GetNumberOfPixels();
	}	

	/** Checks that all the parameters of the kernel have been set properly (e.g. the scene...) */
	virtual bool CheckKernelValidity() {
		Superclass::CheckKernelValidity();
		if (m_library->GetNumberOfEntries()==0) { throw DeformableModelException("MultipleBirth_OffContextOptimize_Death_Kernel is not valid: no object type defined in the scene library"); return false; }
		for (unsigned i=0 ; i<m_library->GetNumberOfEntries() ; i++) {
			m_library->GetObjectPDF( i, PDF_OBJECTGENERATIONPRIOR ); //this function throws an exception if the PDF is not set... TODO: improve the message...
		}
		if (!m_globalOptimizationManager) { throw DeformableModelException("MultipleBirth_OffContextOptimize_Death_Kernel is not valid: no associated global optimizer"); return false; }
		return true;
	}

	/** for this special kernel, there is an option to switch-off the undo method, which may not always be necessary...
	* this may save the overhead of copying the full scene state in such cases
	* by default, undo mode is on
	*/
	void SetUndoModeState(bool b=true) {m_enableUndoFlag = b;}

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

	void SetNumberOfBirthPerCall(unsigned n) { m_nbBirth = std::max<unsigned>(n,1); }

	//temperature is given as an argument ; is it usefull in this case??? not so sure...
	double Apply(double T) {
		//backup the scene state, if requested
		if (m_enableUndoFlag) { m_previousState.clear(); m_scene->GetObjectSetCopy(m_previousState); }
		
		double initialCost = m_globalOptimizationManager->GetValue();

		IDType tmpLabel, label; 
		std::vector<ObjectCosts> listObjects;
		//1: collect the local costs of already existing objects.
		SceneObjectIterator<SceneType> objectIt(m_scene);
		for (objectIt.GoToBegin() ; !objectIt.IsAtEnd() ; ++objectIt) {
			label = objectIt.GetID();
			//put the object in the local scene and optimize it there.
			tmpLabel = m_localScene->AddObject( objectIt.GetObject() );
			m_localOptimizationManager->SelectObject(tmpLabel);
			//m_localOptimizationManager->Optimize();
			//modify the object in the scene, and collect the local cost with the real object ID.
			//m_scene->ModifyObjectParameters(label, m_localScene->GetParametersOfObject(tmpLabel));
			listObjects.push_back( ObjectCosts(label, m_localOptimizationManager->GetValue()) );
			//clean the local scene.
			m_localScene->RemoveObject(tmpLabel);
		}
		//2: give birth to new objects
		//draw a random number of object to sprout -- this could be replaced by floor(m_baseDensity*m_nbPixelsScene*T) without too much effect, I guess...
		//m_poissonPDF->SetLambda( m_baseDensity*m_nbPixelsScene*T );
		std::set<IDType> setOfNewObjects;
		long nbObj;
		//nbObj = m_poissonPDF->DrawSample();
		nbObj = m_nbBirth;
//std::cout<<"gonna place "<<nbObj<<" new objects"<<std::endl;
		for (long i=0 ; i<nbObj ; i++) {
			//select a random type of object and add it to the local scene
			SelectType(); //I should test once and for all if their are multiple entries in the library, instead of testing each time...
			tmpLabel = m_localScene->AddObject(  m_library->GenerateNewRandomObject(m_requestedType) );	// what about specific object insertion policies??	//should child classes of Scene_Base force specific policies (e.g. label map ; or binary map <-> boolean model ; or ADD mode <-> shot noise, ...)
			//optimize that object
			m_localOptimizationManager->SelectObject(tmpLabel);
			m_localOptimizationManager->Optimize();

			//add the optimized object into the real scene, and clean the local scene.
			label = m_scene->AddObject( m_localScene->GetObject(tmpLabel)->obj );
			listObjects.push_back( ObjectCosts(label, m_localOptimizationManager->GetValue()) );
			m_localScene->RemoveObject(tmpLabel);
			setOfNewObjects.insert(label);
		}

		//3: sort the object by decreasing local costs (check with the definition of the comparator, below)
		std::sort(listObjects.begin(), listObjects.end(), m_objectCostComparator);
		//4: iteratively kill objects
		m_currentGlobalCost = m_globalOptimizationManager->GetValue();
//std::cout<<"ok, sorted the list of objects against their local costs, and iteratively killing them... ; current global energy = "<<m_currentGlobalCost<<std::endl;
		double tmpCost;
		for (std::vector<ObjectCosts>::iterator it = listObjects.begin() ; it!=listObjects.end() ; ++it) {
//std::cout<<"  examining object with label "<<it->id<<", with local cost = "<<it->localCost<<std::endl;
			SceneType::DeformableObjectType::Pointer tmpObj = m_scene->GetObject( it->id )->obj;
			m_scene->RemoveObject( it->id );
			tmpCost = m_globalOptimizationManager->GetValue();
//std::cout<<"     global cost without it = "<<tmpCost<<", vs with = "<<m_currentGlobalCost<<std::endl;
			if (tmpCost<m_currentGlobalCost) {
//std::cout<<"     SCENE IS BETTER WITHOUT THIS OBJECT!"<<std::endl;
				//the scene is better without that object => accept without question
				m_currentGlobalCost = tmpCost; setOfNewObjects.erase( it->id );
			}
			else {
				//accept with some probability
				if ( m_rndgen->GetUniformVariate() < exp( (m_currentGlobalCost - tmpCost)/(0.0000001*T) ) ) {
//std::cout<<"     ACCEPT REMOVAL DESPITE A WORST VALUE... decision threshold = "<<exp( (m_currentGlobalCost - tmpCost)/(0.0000001*T) )<<" with T = "<<T<<std::endl;
					m_currentGlobalCost = tmpCost; setOfNewObjects.erase( it->id );
				}
				else { //reject <-> reinsert the object in the scene
//std::cout<<"     KEEP THE OBJECT... decision threshold = "<<exp( (m_currentGlobalCost - tmpCost)/(0.0000001*T) )<<std::endl;
					label = m_scene->AddObject( tmpObj ); // the m_currentGlobalCost should stay the same as before
				}
			}

		}
		if (setOfNewObjects.empty()) return 0; // idea: indicate that things are not going too well ; either the temperature is too high, and lots of objects are killed without much reason, or we are about to converge
		//in both cases, the temperature should be decreased faster than usual.

		return 1;	//this is always OK? or maybe return false if no improvement happened?
	}

	//move the scene to the state it used to have the last time we called Apply
	void Undo() { //return to the previous state
		if (m_enableUndoFlag) m_scene->SetObjectSet(m_previousState);
	}

protected:
	MultipleBirth_OffContextOptimize_Death_Kernel() : KernelWithLocalSingleObjectOptimizer_Base() { 
		m_enableUndoFlag = false; 
		m_baseDensity = 0.001; //default average number of object per pixel
		m_poissonPDF = PoissonPDF::New();
		m_globalOptimizationManager = NULL;
		m_nbBirth = 1;
	};
	~MultipleBirth_OffContextOptimize_Death_Kernel() {};

	int m_nbPixelsScene;
	double m_baseDensity;
	PoissonPDF::Pointer m_poissonPDF;
	unsigned int m_requestedType;

	class ObjectCosts { 
	public: 
		ObjectCosts(IDType i, double c) : id(i), localCost(c) {};
		IDType id; double localCost; 
	};
	class ObjectCostsComparator { public: bool operator()(ObjectCosts const & objc1, ObjectCosts const & objc2) const { return objc1.localCost > objc2.localCost; } };

	ObjectCostsComparator m_objectCostComparator;

	unsigned m_nbBirth;

	bool m_enableUndoFlag;
	typename SceneType::ObjectSetType m_previousState;	
	double m_currentGlobalCost;
	typename GlobalOptimizationManagerType::Pointer m_globalOptimizationManager;

private:
	MultipleBirth_OffContextOptimize_Death_Kernel(const Self&);		//purposely not implemented
	const Self & operator=( const Self & );				//purposely not implemented
};


} // namespace psciob

#endif /* __MULTIPLEBIRTH_OFFCONTEXTOPTIMIZE_DEATH_KERNEL_H_ */
