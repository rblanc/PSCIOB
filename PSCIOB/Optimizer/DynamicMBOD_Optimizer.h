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
* \file DynamicMBOD_Optimizer.h
* \author Rémi Blanc 
* \date 23. July 2011
*/


#ifndef __DYNAMICMBOD_OPTIMIZER_H_
#define __DYNAMICMBOD_OPTIMIZER_H_

#include "GeneralUtils.h"
#include "Optimizer_Base.h"
#include "OptimizerCostFunction.h"

#include "ScalarFunctions.h"
#include "MultivariatePDF.h"

#include "SingleSceneObjectOptimizationManager.h"

#include "MultipleBirth_OffContextOptimize_Death_Kernel.h"

#include "itkImageRegionIterator.h"
#include "itkImageRegionIteratorWithIndex.h"

namespace psciob {

/** \brief DynamicMBOD_Optimizer
*
* Experimental variant of the DynamicMBOD_Optimizer, maintaining a dynamic birth map.
* If not provided, the birth map is initialized as uniform.
* It is used to generate an inhomogeneous probability for sprouting new objects, favoring certain areas, and avoiding some others.
*
* The birth map is dynamic in the sense that successfully positioning an object increases the probability of birth in the area,
* while failures to position an object decreases the probability to sprout new objects in the vicinity.
* The objective is to speed-up the convergence, by focusing only on the area of interest (and stop wasting time on areas that have already been extensively visited)
*
* /sa DynamicMBOD_Optimizer
*/


template<class TScene>
class DynamicMBOD_Optimizer : public Optimizer_Base {
public:
	/** Standard class typedefs. */
	typedef DynamicMBOD_Optimizer	Self;
	typedef Optimizer_Base          Superclass;
	typedef itk::SmartPointer<Self> Pointer;
	/** Run-time type information (and related methods). */
	itkTypeMacro(DynamicMBOD_Optimizer, Optimizer_Base);
	itkNewMacro(Self);

	typedef TScene SceneType;
	typedef typename SceneType::IDType                 IDType;
	typedef typename SceneType::ObjectTypesLibraryType ObjectTypesLibraryType;
	typedef typename SceneType::DeformableObjectType   ObjectType;
	/** The optimization manager must derive from a SceneOptimizationManager_Base */
	typedef SingleSceneObjectOptimizationManager<TScene> LocalOptimizationManagerType;	
	/** The birth map should be of format double. */
	typedef itk::Image<double, SceneType::Dimension>              BirthMapType;
	/** The local optimization manager must derive from a SceneOptimizationManager_Base */
	typedef SceneOptimizationManager_Base<SceneType>     OptimizationManagerType;

	/** Set the maximum number of successive failures to improve things before declaring the convergence 
	* increase to give more robustness...
	*/
	void SetMaximumNumberOfSucessiveFailures(unsigned n=1) {m_maxNbSuccessiveFailures = n;}

	/** Set the initial inverse temperature */
	void SetInitialInverseTemperature(double t = 50)	{ if (t<=0) throw DeformableModelException("DynamicMBOD_Optimizer::SetInitialInverseTemperature temperatures should be >0"); m_phi0=t; }

	/** default is a ScalarFunction_Geometric with geometric factor = 1/0.993 */
	void SetCoolingFunction(ScalarFunction_Base *fct)	{ m_coolingFunction = fct; }   

	/** Set the initial inverse temperature */
	void SetInitialDiscreteStep(double d = 20000)	{ if (d<=0) throw DeformableModelException("DynamicMBOD_Optimizer::SetInitialDiscreteStep should be >0"); m_d0=d; }

	/** default is a ScalarFunction_Geometric with geometric factor = 0.997 */
	void SetDiscreteStepFunction(ScalarFunction_Base *fct)	{ m_stepFunction = fct; }   

	/** Set the local optimization manager (deriving from SingleSceneObjectOptimizationManager) used for local optimization and costs
	* It will be reconfigured to work with a locally defined scene.
	*/
	void SetLocalOptimizationManager(LocalOptimizationManagerType *manager) { m_localOptimizationManager = manager; }


	/** Set the decrease rate for the birth probability when an hypothesis is rejected - default = 0.9 
	* must be in [0,1] ; capped with these value if necessary
	*/
	void SetBirthDecreaseRate(double r = 0.9) {
		if (r<0) {m_BMdecreaseRate=0; return;}
		if (r>1) {m_BMdecreaseRate=1; return;}
		m_BMdecreaseRate=r;
	}

	/** Set the increase rate for the birth probability when an hypothesis is accepted - default = 1.5 
	* must be >= 1 ; capped if necessary
	*/
	void SetBirthIncreaseRate(double r = 1.5) {
		if (r<1) {m_BMincreaseRate=1; return;}
		m_BMincreaseRate=r;
	}

	/** Re-initialize the random generator used for generating sample (all random generators are initialized at 0 by default)
	* special input : -1 => initialize against the clock
	*/
	void InitializeInternalRndGenerator(int seed = -1) {
		if (seed==-1) m_rndgen->Initialize();
		else m_rndgen->Initialize(seed);
	}

	/** Re-initialize the random generator used for positioning the samples (all random generators are initialized at 0 by default)
	* special input : -1 => initialize against the clock
	*/
	void InitializeInternalTranslationRndGenerator(int seed = -1) {
		if (seed==-1) m_translationRndGen->Initialize();
		else m_translationRndGen->Initialize(seed);
	}


	/** Adds a birth map - if not provided, it is assumed to be constant 
	* The content of this image can be modified, so that its pixel values sum to the number of pixels in the image.
	* The image is expected to have the proper size (i.e. cover the same region as the scene) -- unsure what may happen if this is not the case.
	*/
	void AddBirthMap(typename BirthMapType *img) {
		m_birthMap = img;

		itk::ImageRegionIterator< BirthMapType > itBM(m_birthMap, m_birthMap->GetLargestPossibleRegion());
		double sum=0;
		for (itBM.GoToBegin() ; !itBM.IsAtEnd() ; ++itBM) { sum += itBM.Get(); }
		if ( sum!=m_birthMap->GetLargestPossibleRegion().GetNumberOfPixels() ) {
			double c = static_cast<double>(m_birthMap->GetLargestPossibleRegion().GetNumberOfPixels())/sum;
			for ( itBM.GoToBegin() ; !itBM.IsAtEnd() ; ++itBM ) { itBM.Set( c*itBM.Get() ); }
		}
	}

	double Optimize(OptimizationManager_Base *manager) {
		//CHECK  THE INPUTS
		if (!m_localOptimizationManager) throw DeformableModelException("Exception in DynamicMBOD_Optimizer: a local optimization manager is requested to compute local costs");
		m_manager = dynamic_cast<SceneOptimizationManager_Base<SceneType>*>(manager);
		if (!m_manager) throw DeformableModelException("Exception in DynamicMBOD_Optimizer: the optimization manager should derive from SceneOptimizationManager_Base");
		m_scene = m_manager->GetScene();
		if (!m_scene) throw DeformableModelException("Exception in DynamicMBOD_Optimizer: the given optimization manager is not related to any scene!");

		m_library = m_scene->GetObjectTypesLibrary();

		//create the local scene, which will be used for local optimizations
		m_localScene = SceneType::New(); //TODO: check that they are configured in the same way (insertion policy, etc...)
		m_localScene->SetPhysicalDimensions( m_scene->GetSceneBoundingBox(), m_scene->GetSceneSpacing().GetVnlVector() );
		m_localScene->FuseObjectTypesLibrary( m_scene->GetObjectTypesLibrary() );
		m_localScene->SetGlobalPrior( m_scene->GetGlobalPrior()->CreateClone() ); //TODO: I should make sure they are properly configured. (if there are parameters to set, ...)
		m_localScene->SetObjectPriorNormalizationFunction( m_scene->GetObjectPriorNormalizationFunction() );
		m_localScene->SetTrackChanges( m_scene->GetTrackChangesStatus() );

		//configure the local optimizer to work on this local scene
		m_localOptimizationManager->SetScene(m_localScene);

		double			bestEnergy = m_manager->GetValue(), currentEnergy = bestEnergy, oldEnergy = -1e30, tmpEnergy;
		unsigned int	currentNb = m_scene->GetNumberOfObjects(), oldNb, requestedObjectType;
		double			phi = m_phi0, d = m_d0, invphi = 1.0/phi, tmp;

		bool converged = false;
		unsigned successiveFailures = 0;


		if (!m_birthMap) {
			//if the birth map is unspecified -> generate a uniform grid, the same size and resolution as the scene image, filled with 1
			m_birthMap = BirthMapType::New();
			m_birthMap->SetOrigin( m_scene->GetSceneOrigin() );
			m_birthMap->SetSpacing( m_scene->GetSceneSpacing() );
			m_birthMap->SetRegions( m_scene->GetSceneImageRegion() );
			m_birthMap->Allocate();
			m_birthMap->FillBuffer(1.0);
		}
		itk::ImageRegionIteratorWithIndex< BirthMapType > itBM(m_birthMap, m_birthMap->GetLargestPossibleRegion());
		double m;
		vnl_vector<double> pixelBBox(2*SceneType::Dimension), pos(SceneType::Dimension), tmpOffset;
		for (unsigned i=0 ; i<SceneType::Dimension ; i++) { pixelBBox(2*i) = -m_birthMap->GetSpacing()[i]/2.0; pixelBBox(2*i+1) = -pixelBBox(2*i); }

		m_translationRndGen->SetBox( pixelBBox );
		ObjectType::Pointer tmpObj;
		const BirthMapType::PointType &BMorigin = m_birthMap->GetOrigin();
		const BirthMapType::SpacingType &BMspacing = m_birthMap->GetSpacing();
		const BirthMapType::SizeType &BMsize = m_birthMap->GetLargestPossibleRegion().GetSize();
		BirthMapType::IndexType endIndex;
		vnl_vector<double> initialParam, finalParam;
		NewlyBornObjectListType newlyBornObjects;
		NewlyBornObjectListIterator NBOit;

		//before starting the MBOD loop, generate the list of all objects in the scene, and of their local costs
		//this list is maintained uptodate in the loop to avoid recomputing the local costs of old objects
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
			//this is necessary because during the iterations, objects which are removed from the scene are not removed from the local list.
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
			// browse the birth map, for each pixel, sprout a new object with a proba proportional to the pixel value (and d)
			// then, change the location - draw uniformly inside the current pixel, so that it can accomodate for any resolution of the birth map (e.g. if the resolution is lower than that of the scene...
			// PAY ATTENTION, the value of the birth map is initially normalized such that the sum of pixel value is the nb of pixels.
			newlyBornObjects.clear();
			m = d / static_cast<double>(m_birthMap->GetLargestPossibleRegion().GetNumberOfPixels());
			for (itBM.GoToBegin() ; !itBM.IsAtEnd() ; ++itBM) {
				if ( m_rndgen->GetUniformVariate() < itBM.Get() * m ) { //sprout an object there with a probability depending on the birth map & current 'temperature' => d
					requestedObjectType = 0;
					//if (m_library->GetNumberOfEntries()>1) { //this is used to select an object type, in case multiple types of objects are supported by the scene.
					//	tmp = m_rndgen->GetUniformVariate(0, m_library->GetSumTypeWeights());
					//	while (tmp>0) {
					//		if ( tmp < m_library->GetObjectEntry(requestedObjectType)->weight ) break;
					//		tmp-=m_library->GetObjectEntry(requestedObjectType)->weight; requestedObjectType++;
					//	}
					//}
					tmpObj = m_library->GenerateNewRandomObject(requestedObjectType);
					//redefine the object center (draw uniformly inside the birthMap pixel...
					const BirthMapType::IndexType &BMindex = itBM.GetIndex();
					tmpOffset = m_translationRndGen->DrawSample();
					for (unsigned i=0 ; i<SceneType::Dimension ; i++) pos(i) = BMorigin[i] + BMindex[i]*BMspacing[i] + tmpOffset(i);
					tmpObj->PositionAt(pos);

					//add it to the local scene
					tmpLabel = m_localScene->AddObject( tmpObj );	// what about specific object insertion policies??	//should child classes of Scene_Base force specific policies (e.g. label map ; or binary map <-> boolean model ; or ADD mode <-> shot noise, ...)
					if (tmpLabel==0) continue; //warning: it could happen that the object is not accepted inside the scene.
initialParam = m_localScene->GetParametersOfObject(tmpLabel);
					//optimize that object locally
					m_localOptimizationManager->SelectObject(tmpLabel);
					m_localOptimizationManager->Optimize();
					//add the optimized object into the real scene, and clean the local scene.
					label = m_scene->AddObject( m_localScene->GetObject(tmpLabel)->obj );
					listObjects.push_back( ObjectCosts(label, m_localOptimizationManager->GetValue()) );
					m_localScene->RemoveObject(tmpLabel);

					finalParam = m_scene->GetParametersOfObject(label);
					for (unsigned i=0 ; i<SceneType::Dimension ; i++) {
						endIndex[i] = round( (finalParam(i)-BMorigin[i])/BMspacing[i] );
						if (endIndex[i]<0) endIndex[i]=0;
						if (endIndex[i]>=BMsize[i]) endIndex[i]=BMsize[i]-1;
					}
					newlyBornObjects.insert( NewlyBornObjectEntryType(label, NewlyBornObjectData(itBM.GetIndex(), endIndex)) );
if (nbiter==1) std::cout<<"   initial params: "<<initialParam<<", initial index: "<<itBM.GetIndex()<<"\n    - after optimization, params = "<<finalParam<<", index: "<<endIndex<<std::endl;
				}
			}
std::cout<<"end of birth step, added "<<newlyBornObjects.size()<<" objects, in "<<(clock()-t1)/((double)CLOCKS_PER_SEC)<<" s."<<std::endl;
			//
			//2: sort the objects according to their local costs, and kill sequentially the objects according to their impact on the global cost
			//
t1=clock();//std::cout<<"sort objects "<<std::endl;
			std::sort(listObjects.begin(), listObjects.end(), m_objectCostComparator);
std::cout<<"sorted the objects in "<<(clock()-t1)/((double)CLOCKS_PER_SEC)<<" s."<<std::endl;

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

				//a = exp(-phi*(currentEnergy-tmpEnergy));
				//std::cout<<"     global cost without it = "<<tmpCost<<", vs with = "<<m_currentGlobalCost<<std::endl;
				//if ( m_rndgen->GetUniformVariate() > a/(1.0+a)  ) { //in that case, validate the object removal
				if ( tmpEnergy <= currentEnergy ) { //SIMPLIFY THE CONDITION, IT IS ENOUGH THAT THE SCENE IS BETTER WITHOUT THE OBJECT TO KILL IT... -> avoid killing good objects!!!
					//check if this is a newly born object, and if so, update the birth map (reduce the birth proba)
					NBOit = newlyBornObjects.find( it->id );
					if (NBOit != newlyBornObjects.end()) { //decrease the birth proba.
						itBM.SetIndex( NBOit->second.indexFin );
						itBM.Set( itBM.Get() * m_BMdecreaseRate );
						newlyBornObjects.erase( NBOit );
					}
					currentEnergy = tmpEnergy;
					it->id = 0; //mark it as invalid in the list
				}
				else { //re-add the object in the scene.
					label = m_scene->AddObject( tmpObj );
				}
			}

			//for all object remaining in the newlyBornObjects map, increase the birth proba.
			for (NBOit = newlyBornObjects.begin() ; NBOit!=newlyBornObjects.end() ; ++NBOit) {
				itBM.SetIndex( NBOit->second.indexFin );
				itBM.Set( itBM.Get() * m_BMincreaseRate );
			}

std::cout<<" ; kill ("<<(clock()-t1)/((double)CLOCKS_PER_SEC)<<"s) -- number of new objects kept: "<<newlyBornObjects.size()<<std::endl;

			//decrease the temperature at each iteration
			phi = m_coolingFunction->Evaluate(phi); invphi = 1.0/phi;
			d = m_stepFunction->Evaluate(d); 

			//compare old and new scenes...
			currentEnergy = manager->GetValue();
			currentNb = m_scene->GetNumberOfObjects();

			//update the best state, and check for convergence
			if (currentEnergy < bestEnergy) { manager->SaveCurrentStateAsBest(); bestEnergy = currentEnergy; successiveFailures = 0; }
			else { 
				//if ( (currentNb==oldNb) && (fabs(currentEnergy-oldEnergy)<TINY) ) {
				if ( (currentNb==oldNb) && ( currentEnergy-oldEnergy<TINY ) ) {
					successiveFailures++;
					if (successiveFailures>=m_maxNbSuccessiveFailures) converged = true; 
				}
				else { successiveFailures = 0; }
			}
			
std::cout<<" -- end it. "<<nbiter<<"phi = "<<phi<<", nb obj: "<<m_scene->GetNumberOfObjects()<<" ; energy: "<<currentEnergy<<" (best = "<<bestEnergy<<") ** time: "<<(clock()-t0)/((double)CLOCKS_PER_SEC)<<" s"<<std::endl;
Write2DGreyLevelRescaledImageToFile<BirthMapType>("birthMap_" + stringify(nbiter) + ".png", m_birthMap);
		}

		return manager->GetValue();	//the manager will return to the best visited state in case this is not the case

	}

protected:
	DynamicMBOD_Optimizer() : Optimizer_Base() {
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

		m_translationRndGen = UniformBoxPDF::New();
		//TODO: add control over these parameters.
		m_BMdecreaseRate = 0.9;
		m_BMincreaseRate = 1.5;
	};
	~DynamicMBOD_Optimizer() {};

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
	UniformBoxPDF::Pointer m_translationRndGen;

	double m_BMdecreaseRate, m_BMincreaseRate;

	//local structures storing objects ids, and their local costs (according to the provided localOptimizationManager)
	class ObjectCosts {
	public: 
		ObjectCosts(IDType i, double c) : id(i), localCost(c) {};
		IDType id; double localCost; 
	};
	class ObjectCostsComparator { public: bool operator()(ObjectCosts const & objc1, ObjectCosts const & objc2) const { return objc1.localCost > objc2.localCost; } };
	ObjectCostsComparator m_objectCostComparator;

	//Data associated with a newly born object 
	//indexIni is the index in which the object is initially sprout
	//indexFin is the index indicating in which pixel of the birth map the object center lies after the optimization.
	class NewlyBornObjectData {
	public: 
		NewlyBornObjectData(typename BirthMapType::IndexType ind1, typename BirthMapType::IndexType ind2) : indexIni(ind1), indexFin(ind2) {};
		typename BirthMapType::IndexType indexIni, indexFin;
	};
	/** useful types for the list of newly born objects */
	typedef typename std::map<IDType, NewlyBornObjectData>    NewlyBornObjectListType;
	typedef typename std::pair<IDType, NewlyBornObjectData>   NewlyBornObjectEntryType;
	typedef typename NewlyBornObjectListType::iterator        NewlyBornObjectListIterator;


private:
	DynamicMBOD_Optimizer(const Self&);	//purposely not implemented
	const Self & operator=( const Self & );		//purposely not implemented
};





} // namespace psciob

#endif /* __DYNAMICMBOD_OPTIMIZER_H_ */
