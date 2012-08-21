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
* \file MultipleBirthOptimizeAndDeathOptimizer.h
* \author Rémi Blanc 
* \date 22. November 2011
*/


#ifndef __MULTIPLEBIRTHOPTIMIZEANDDEATHOPTIMIZER_H_
#define __MULTIPLEBIRTHOPTIMIZEANDDEATHOPTIMIZER_H_

#include "GeneralUtils.h"
#include "Optimizer_Base.h"
#include "OptimizerCostFunction.h"

#include "ScalarFunctions.h"
#include "MultivariatePDF.h"

#include "SingleSceneObjectOptimizationManager.h"

#include "MultipleBirth_OffContextOptimize_Death_Kernel.h"

namespace psciob {

/** \brief MultipleBirthOptimizeAndDeathOptimizer
* 
* The class is inspired from the MutlipleBirthAndDeath algorithm described by Descombes et al.
* in particular in the RR-6328 ; Détection de flamants roses par processus ponctuels marqués pour l'estimation de la taille des populations
* S. Descamps, X. Descombes, A. Béchet, J. Zérubia
* 2007
*
* steps at each iteration:
* - sprout a given number of objects randomly according to the available generative pdf
* - each object is optimized independently using a local optimization manager (within a local scene, i.e. no interactions)
* - sort the objects by decreasing local cost
* - iteratively kill the object with some probability, according to the difference of global energy with & without the object
* 
* (a list of the local costs is maintained, so they don't need to be recomputed for old objects.)
*/


template<class TScene>
class MultipleBirthOptimizeAndDeathOptimizer : public Optimizer_Base {
public:
	/** Standard class typedefs. */
	typedef MultipleBirthOptimizeAndDeathOptimizer	Self;
	typedef Optimizer_Base					Superclass;
	typedef itk::SmartPointer<Self>			Pointer;
	/** Run-time type information (and related methods). */
	itkTypeMacro(MultipleBirthOptimizeAndDeathOptimizer, Optimizer_Base);
	itkNewMacro(Self);

	typedef TScene SceneType;
	typedef typename SceneType::IDType                 IDType;
	typedef typename SceneType::ObjectTypesLibraryType ObjectTypesLibraryType;
	typedef typename SceneType::DeformableObjectType   ObjectType;
	/** The optimization manager must derive from a SceneOptimizationManager_Base */
	typedef SingleSceneObjectOptimizationManager<TScene> LocalOptimizationManagerType;	
	/** The birth map should be of format double. */
	typedef itk::Image<double, SceneType::Dimension> BirthMapType;
	/** The local optimization manager must derive from a SceneOptimizationManager_Base */
	typedef SceneOptimizationManager_Base<SceneType> OptimizationManagerType;

	/** Set the maximum number of successive failures to improve things before declaring the convergence 
	* increase to give more robustness...
	*/
	void SetMaximumNumberOfSucessiveFailures(unsigned n=1) {m_maxNbSuccessiveFailures = n;}

	/** Set the initial inverse temperature */
	void SetInitialInverseTemperature(double t = 50)	{ if (t<=0) throw DeformableModelException("MultipleBirthOptimizeAndDeathOptimizer::SetInitialInverseTemperature temperatures should be >0"); m_phi0=t; }

	/** default is a ScalarFunction_Geometric with geometric factor = 1/0.993 */
	void SetCoolingFunction(ScalarFunction_Base *fct)	{ m_coolingFunction = fct; }   

	/** Set the initial inverse temperature */
	void SetInitialDiscreteStep(double d = 20000)	{ if (d<=0) throw DeformableModelException("MultipleBirthOptimizeAndDeathOptimizer::SetInitialDiscreteStep should be >0"); m_d0=d; }

	/** default is a ScalarFunction_Geometric with geometric factor = 0.997 */
	void SetDiscreteStepFunction(ScalarFunction_Base *fct)	{ m_stepFunction = fct; }   

	/** Set the local optimization manager (deriving from SingleSceneObjectOptimizationManager) used for local optimization and costs
	* It will be reconfigured to work with a locally defined scene.
	*/
	void SetLocalOptimizationManager(LocalOptimizationManagerType *manager) { m_localOptimizationManager = manager; }

	/** Adds a birth map - if not provided, it is assumed to be constant 
	* The image is expected to have the proper size and to be normalized such that its pixel values sum to 1.
	* a warning will be issued if the size is wrong, and the map will be ignored if this is not the case.
	* However, no checks are performed on the pixel values...
	*/
	void AddBirthMap(BirthMapType *img) {
		m_birthMap = img;
	}

	double Optimize(OptimizationManager_Base *manager) {
		//CHECK  THE INPUTS
		if (!m_localOptimizationManager) throw DeformableModelException("Exception in MultipleBirthOptimizeAndDeathOptimizer: a local optimization manager is requested to compute local costs");
		m_manager = dynamic_cast<SceneOptimizationManager_Base<SceneType>*>(manager);
		if (!m_manager) throw DeformableModelException("Exception in MultipleBirthOptimizeAndDeathOptimizer: the optimization manager should derive from SceneOptimizationManager_Base");
		m_scene = m_manager->GetScene();
		if (!m_scene) throw DeformableModelException("Exception in MultipleBirthOptimizeAndDeathOptimizer: the given optimization manager is not related to any scene!");

		m_library = m_scene->GetObjectTypesLibrary();

		//create the local scene, which will be used for local optimizations
		m_localScene = SceneType::New(); //TODO: check that they are configured in the same way (insertion policy, etc...)
		m_localScene->SetPhysicalDimensions( m_scene->GetSceneBoundingBox(), m_scene->GetSceneSpacing().GetVnlVector() );
		m_localScene->FuseObjectTypesLibrary( m_scene->GetObjectTypesLibrary() );
		m_localScene->SetGlobalPrior( m_scene->GetGlobalPrior()->CreateClone() ); //TODO: I should make sure they are properly configured. (if there are parameters to set, ...)
		m_localScene->SetObjectPriorNormalizationFunction( m_scene->GetObjectPriorNormalizationFunction() );
		////m_localScene->SetTrackChanges( m_scene->GetTrackChangesStatus() );
		//for (SceneType::SensorInterfaceListType::iterator it = m_scene->GetListOfConnectedSensors().begin() ; 
		//	it != m_scene->GetListOfConnectedSensors().end() ; ++it) {
		//		m_localScene->ConnectSensor(*it);
		//}

		//configure the local optimizer to work on this local scene
		m_localOptimizationManager->SetScene(m_localScene);

		double			bestEnergy = m_manager->GetValue(), currentEnergy = bestEnergy, oldEnergy = -1e30, tmpEnergy;
		unsigned int	currentNb = m_scene->GetNumberOfObjects(), oldNb, requestedObjectType;
		double			phi = m_phi0, d = m_d0, invphi = 1.0/phi, tmp;

		bool converged = false;
		unsigned successiveFailures = 0;

		if (!m_birthMap) {}
		else {
			//here, I could re-interpolate and normalize the birth map...
			//TODO:
			//well... perhaps the user should make sure that the dimensions are right, but keep a low-resolution map
			//then, inside each pixel, one (or more) objects could be positioned using a uniform distribution inside the pixel...
			//it could also be interesting to pad the birthmap a little, so as to be able to propose objects that are slightly outside the scene (for border-touching objects)
			for (unsigned i=0 ; i<SceneType::Dimension ; i++) {				
			}
		}

		//before starting, generate a list of objects in the scene, and of their local costs
		//this list is maitained to avoid recomputing the local costs of old objects
		IDType tmpLabel, label; 
		std::vector<ObjectCosts> listObjects; unsigned nbRemovedObjects;
		SceneObjectIterator<SceneType> objectIt(m_scene);
		for (objectIt.GoToBegin() ; !objectIt.IsAtEnd() ; ++objectIt) {
			label = objectIt.GetID();
			//put the object in the local scene and get its local cost
			tmpLabel = m_localScene->AddObject( objectIt.GetObject()->obj );
			m_localOptimizationManager->SelectObject(tmpLabel);
			listObjects.push_back( ObjectCosts(label, m_localOptimizationManager->GetValue()) );
			//clean the local scene.
			m_localScene->RemoveObject(tmpLabel);
		}

		//
		//START OF THE OPTIMIZATION LOOP!
		//
		//CONVERGENCE CRITERION => all the objects proposed in the birth step have been killed in the death step.
		//it is translated here into => the number of objects before and after is the same, and the energy difference is negligible
clock_t t0=clock(), t1;
unsigned nbiter = 0;
		while (!converged) {
			nbiter++;
			oldEnergy = currentEnergy; oldNb = currentNb; 

			//
			//0: browse the list of object costs, and remove invalid entries
			//
			for (int i=0, j=listObjects.size()-1 ; i<=j ; i++) {
				if (listObjects[i].id==0) { //this entry is invalid -> replace it when
					while ( (listObjects[j].id==0) && (j>i) ) { //find the next valid entry that can
						listObjects.pop_back(); j--;
					}
					if (j<=i) { listObjects.pop_back(); break; } //if I reach the up-front, then remove that last invalid element and exit.
					else { //replace the invalid by the last, remove the last, and continue
						listObjects[i].id = listObjects[j].id;
						listObjects[i].localCost = listObjects[j].localCost;
						listObjects.pop_back(); j--;
					}
				}
			}

			//
			//1: birth and optimize
			//
			if (!m_birthMap) { //in this case, assume a uniform birth map -> draw a number of objects to position, following a Poisson law, and position them uniformly in the scene
				//for the moment, just draw d objects uniformly.
t1=clock();//std::cout<<"generating and optimizing "<<d<<" objects"<<std::endl;
				for (unsigned i=0 ; i<d ; i++) {
					//select a random type of object and add it to the local scene
					requestedObjectType = 0;
					//if (m_library->GetNumberOfEntries()>1) { //this is used to select an object type, in case multiple types of objects are supported by the scene.
					//	tmp = m_rndgen->GetUniformVariate(0, m_library->GetSumTypeWeights());
					//	while (tmp>0) {
					//		if ( tmp < m_library->GetObjectEntry(requestedObjectType)->weight ) break;
					//		tmp-=m_library->GetObjectEntry(requestedObjectType)->weight; requestedObjectType++;
					//	}
					//}
					//add it to the local scene
					tmpLabel = m_localScene->AddObject(  m_library->GenerateNewRandomObject(requestedObjectType) );	// what about specific object insertion policies??	//should child classes of Scene_Base force specific policies (e.g. label map ; or binary map <-> boolean model ; or ADD mode <-> shot noise, ...)
					if (tmpLabel==0) continue; //warning: it could happen that the object is not accepted inside the scene.
					//optimize that object locally
					m_localOptimizationManager->SelectObject(tmpLabel);
					m_localOptimizationManager->Optimize();
					//add the optimized object into the real scene, and clean the local scene.
					label = m_scene->AddObject( m_localScene->GetObject(tmpLabel)->obj );
					listObjects.push_back( ObjectCosts(label, m_localOptimizationManager->GetValue()) );
					m_localScene->RemoveObject(tmpLabel);
					//setOfNewObjects.insert(label);
				}
std::cout<<"gen. "<<d<<" obj. ("<<(clock()-t1)/((double)CLOCKS_PER_SEC)<<"s)";
			}
			else {
				//in this scenario, browse the birth map, and for each pixel, sprout a new object with a proba proportional to the pixel value (and d)
				// change the location (draw uniformly inside the current pixel?, or put it at the center?)
				//TODO
			}

			//
			//2: sort the objects according to their local costs, and kill sequentially the objects according to their impact on the global cost
			//
//t1=clock();//std::cout<<"sort objects "<<std::endl;
			std::sort(listObjects.begin(), listObjects.end(), m_objectCostComparator);
//std::cout<<"sorted the objects in "<<(clock()-t1)/((double)CLOCKS_PER_SEC)<<" s."<<std::endl;

			currentEnergy = m_manager->GetValue();
			//std::cout<<"ok, sorted the list of objects against their local costs, and iteratively killing them... ; current global energy = "<<m_currentGlobalCost<<std::endl;
			double a;
t1=clock();//std::cout<<"killing objects"<<std::endl;
			for (std::vector<ObjectCosts>::iterator it = listObjects.begin() ; it!=listObjects.end() ; ++it) {
				//std::cout<<"  examining object with label "<<it->id<<", with local cost = "<<it->localCost<<std::endl;
				if (it->id==0) { std::cout<<"WARNING -- label 0... should never happen!!"<<std::endl; continue; } //keep the warning until the class is tested.
				SceneType::DeformableObjectType::Pointer tmpObj = m_scene->GetObject( it->id )->obj; //keep a smart pointer to the object to avoid it is destroyed, in case it should be re-added to the scene
				m_scene->RemoveObject( it->id );
				tmpEnergy = m_manager->GetValue();

				a = exp(-phi*(currentEnergy-tmpEnergy));
				//std::cout<<"     global cost without it = "<<tmpCost<<", vs with = "<<m_currentGlobalCost<<std::endl;
				if ( m_rndgen->GetUniformVariate() > a/(1.0+a)  ) { //in that case, validate the object removal
					currentEnergy = tmpEnergy;
					it->id = 0; //mark it as invalid in the list
				}
				else { //re-add the object in the scene.
					label = m_scene->AddObject( tmpObj );
				}
			}
std::cout<<" ; kill ("<<(clock()-t1)/((double)CLOCKS_PER_SEC)<<"s)";

			//decrease the temperature at each iteration
			phi = m_coolingFunction->Evaluate(phi); invphi = 1.0/phi;
			d = m_stepFunction->Evaluate(d); 

			//compare old and new scenes...
			currentEnergy = manager->GetValue();
			currentNb = m_scene->GetNumberOfObjects();

			//update the best state, and check for convergence
			if (currentEnergy < bestEnergy) { manager->SaveCurrentStateAsBest(); bestEnergy = currentEnergy; successiveFailures = 0; }
			else { 
				if ( (currentNb==oldNb) && (fabs(currentEnergy-oldEnergy)<TINY) ) {
					successiveFailures++;
					if (successiveFailures>=m_maxNbSuccessiveFailures) converged = true; 
				}
				else {
					successiveFailures = 0;
				}
			}
			
std::cout<<" -- end it. "<<nbiter<<"phi = "<<phi<<", nb obj: "<<m_scene->GetNumberOfObjects()<<" ; energy: "<<currentEnergy<<" (best = "<<bestEnergy<<") ** time: "<<(clock()-t0)/((double)CLOCKS_PER_SEC)<<" s"<<std::endl;

		}

		return manager->GetValue();	//the manager will return to the best visited state in case this is not the case

	}

protected:
	MultipleBirthOptimizeAndDeathOptimizer() : Optimizer_Base() {
		m_phi0 = 50;
		m_d0 = 20000;
		//cooling function
		ScalarFunction_Geometric::Pointer tmpFct1 = ScalarFunction_Geometric::New(); tmpFct1->SetFactor(1/0.993); 
		m_coolingFunction = static_cast<ScalarFunction_Base::Pointer>(tmpFct1);
		//discretization step function
		ScalarFunction_Geometric::Pointer tmpFct2 = ScalarFunction_Geometric::New(); tmpFct2->SetFactor(0.997); 
		m_stepFunction = static_cast<ScalarFunction_Base::Pointer>(tmpFct2);
		//internal random generator
		m_rndgen = RandomVariableGenerator::New();

		m_manager = NULL;
		m_scene = NULL;
		m_birthMap = NULL;

		m_localOptimizationManager = NULL;
		m_maxNbSuccessiveFailures = 1;
	};
	~MultipleBirthOptimizeAndDeathOptimizer() {};

	typedef SingleSceneObjectOptimizationManager<TScene> LocalOptimizationManagerType;
	typename LocalOptimizationManagerType::Pointer m_localOptimizationManager;

	typename SceneOptimizationManager_Base<SceneType>::Pointer m_manager;
	typename SceneType::Pointer m_scene, m_localScene;
	typename ObjectTypesLibraryType::Pointer	m_library;

	unsigned m_maxNbSuccessiveFailures; //
	double m_phi0, m_d0;
	ScalarFunction_Base::Pointer m_coolingFunction, m_stepFunction;
	RandomVariableGenerator::Pointer m_rndgen;

	typename BirthMapType::Pointer m_birthMap;


	//local structures storing objects ids, and their local costs (according to the provided localOptimizationManager)
	class ObjectCosts { 
	public: 
		ObjectCosts(IDType i, double c) : id(i), localCost(c) {};
		IDType id; double localCost; 
	};
	class ObjectCostsComparator { public: bool operator()(ObjectCosts const & objc1, ObjectCosts const & objc2) const { return objc1.localCost > objc2.localCost; } };
	ObjectCostsComparator m_objectCostComparator;

private:
	MultipleBirthOptimizeAndDeathOptimizer(const Self&);	//purposely not implemented
	const Self & operator=( const Self & );		//purposely not implemented
};






} // namespace psciob

#endif /* __MULTIPLEBIRTHOPTIMIZEANDDEATHOPTIMIZER_H_ */
