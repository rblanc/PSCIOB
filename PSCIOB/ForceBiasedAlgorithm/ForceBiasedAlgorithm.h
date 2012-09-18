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
* \file ForceBiasedAlgorithm.h
* \author Rémi Blanc 
* \date 31. July 2012
*/


#ifndef __FORCEBIASEDALGORITHM_H_
#define __FORCEBIASEDALGORITHM_H_

#include "BaseScene.h"
#include "FBMovementManager.h"

namespace psciob {

/** \brief ForceBiasedAlgorithm
*
* This is the base class for implementing the force-biased algorithm.
* This algorithm is a deterministic, collective re-arrangement method for removing overlaps between geometric objects
* From an initial collection, for each pair of intersecting objects, it proposes a displacment aiming at reducing the overlaps
* All proposal moves are composed, and applied simultaneously. The objects are slightly downscaled, and the process is iterated until
* no intersection remain.
*
* The algorithm works with a FBMovementManager to deal with different kinds of moves (translations, possibly rotations, scaling) for the various object types...
* The default FBMovementManager performs only translations
*
* This algorithm requires the scene to work with a interaction manager that deals with overlaps only.
*/

//IDEA: use a temporary scene ?? in which to perform all computations...?

//DEV: implement a structure, with one entry per object, in which to store the proposal moves, and perform the necessary compositions.
//how to store those moves??? -> matrices seem to be the most generic way.
//I could just store the parameter modifications to be applied here, and let the manager deal with this...


template<class TScene>
class ForceBiasedAlgorithm : public itk::LightObject {
public:
    /** Standard class typedefs. */
    typedef ForceBiasedAlgorithm    Self;
    typedef itk::LightObject        Superclass;
    typedef itk::SmartPointer<Self> Pointer;
    /** Run-time type information (and related methods). */
    itkTypeMacro(ForceBiasedAlgorithm, itk::LightObject);
    itkNewMacro(Self);

    typedef TScene                                     SceneType;
    static const unsigned int Dimension              = SceneType::Dimension;
    typedef typename SceneType::IDType                 IDType;
    typedef typename SceneType::ObjectTypesLibraryType ObjectTypesLibraryType;
    typedef typename SceneType::DeformableObjectType   ObjectType;
	typedef typename FBMovementManager<SceneType>      FBMovementManagerType;

	/** types of boundary effect 
	* VOIDBOUNDARIES (default): scene boundaries are void, they have no effect on objects
	* SOLIDBOUNDARIES: scene boundaries are solid, and thus tend to push inwards objects that are partially outside the scene.
	* PERIODICBOUNDARIES: considering periodic boundaries, a part of an object partially outside on the right side 're-enters' the scene on the left side.
	*/
	enum BOUNDARYEFFECT { VOIDBOUNDARIES, SOLIDBOUNDARIES, PERIODICBOUNDARIES };

	
    /** Attach a scene to the algorithm */
    void SetScene(SceneType* scene) { 
        m_scene = scene;
		for (unsigned d=0 ; d<Dimension ; d++) m_sceneWidth(d) = m_scene->GetSceneBoundingBox()(2*d+1)-m_scene->GetSceneBoundingBox()(2*d);
        m_mvtManager->SetScene(m_scene);
    }

	/** Set the Boundary effect behavior
	* \todo implement the non-void boundary conditions...
	*/
	void SetBoundaryEffectBehavior(BOUNDARYEFFECT eff) { 
		m_boundaryEffectCode = eff; 
	}
	
    /** Set a specific movement manager, this object is in charge of proposing and applying moves to the objects of the scene 
    * The default manager is a FBMovementManager, one can set a specialized child instead (e.g. for managing rotations).
    */
    void SetMovementManager(FBMovementManagerType* mm) { 
        m_mvtManager = mm;
        if (!m_scene) {} else { m_mvtManager->SetScene(m_scene); }
    }
    
    /** Get a pointer to the movement manager 
    * this is useful, in particular for (re)defining specific aspects of the manager
    */
    FBMovementManagerType* GetMovementManager() { return m_mvtManager; }

    /** Set the maximum number of iterations for the method IterateUntilConvergence - default is 1e10 */
    void SetMaximumNumberOfIterations(unsigned n) {m_maxNbIteration=n;}

    /** Get the maximum number of iterations for the method IterateUntilConvergence - default is 1e10 */
    unsigned GetMaximumNumberOfIterations() { return m_maxNbIteration;}
    

    /** Applies a single iteration of the algorithm 
	* returns false if no overlaps were present anyway.
	*/
	bool ApplyOneIteration(bool verbose = false) {
		clock_t t0=clock();
		m_proposedMoves.clear(); //make sure it starts empty.
		//initialize the moves to identity for all objects (m_proposedMoves)
		SceneObjectIterator<SceneType> objectIt(m_scene);
		for (objectIt.GoToBegin() ; !objectIt.IsAtEnd() ; ++objectIt) { 
			m_proposedMoves.insert(m_proposedMoves.end(), MovePairType(objectIt.GetID(), m_scene->GetParametersOfObject(objectIt.GetID())) );
		}

		std::map<IDType, vnl_vector<double>>::iterator it;
		//check for overlaps between objects 
		//for each pair of overlapping objects, propose a move, and compose it with the existing m_proposedMoves
		m_scene->GetInteractionManager()->TurnOnInteractionManagement(); //should not be necessary... but do it just in case...
        if (verbose) std::cout<<"  time after initializing moves, and making sure interactions are uptodate: "<<(clock()-t0)/((double)CLOCKS_PER_SEC)<<" s."<<std::endl;
		unsigned nbOverlaps = 0;
		for (it = m_proposedMoves.begin() ; it != m_proposedMoves.end() ; ++it) { 
			//look for the interactions for this object
			for (SceneType::ObjectInteractionIterator dit = m_scene->GetObject(it->first)->interactionData.begin() ; dit != m_scene->GetObject(it->first)->interactionData.end() ; dit++ , ++nbOverlaps) {
				//propose a move unilaterally for the current object, and compose it with the current proposed move
				it->second = m_mvtManager->UpdateMove( it->second, it->first, dit->first, dit->second );
				//the second object of the pair will be treated when its turn comes
			}
		}
		if (verbose) std::cout<<"  time after processing object overlaps ("<<nbOverlaps<<"): "<<(clock()-t0)/((double)CLOCKS_PER_SEC)<<" s."<<std::endl;

		//manage boundary effects
		switch (m_boundaryEffectCode) {
			case VOIDBOUNDARIES: //nothing to do.
				break; 
			case SOLIDBOUNDARIES: //translate inwards objects that are partially outside the scene
				for (it = m_proposedMoves.begin() ; it != m_proposedMoves.end() ; ++it) { 
					std::vector<unsigned> intersects;
					intersects = IdentifyIntersectionBoundingBoxes( m_scene->GetObject(it->first)->obj->GetPhysicalBoundingBox(), m_scene->GetSceneBoundingBox() );
					for (unsigned w = 0 ; w<intersects.size() ; w++, ++nbOverlaps) it->second = m_mvtManager->UpdateMoveWithSceneWall( it->second, it->first, intersects[w] );
				}
				break;
			case PERIODICBOUNDARIES: 
				//check with which border(s), if any, the object overlaps with, and update the move accordingly.
				//for each of these, add (a) temporary object(s) on the 'opposite' face(s) of the scene, and UpdateMove as usual in case of interaction
				//Furthermore, if an objects is too much outside the scene (more than half), than delete it and move it to the other side...
				struct mirroredObjectType {
					SceneType::IDType id; //id of the mirrored object
					std::vector<unsigned> mirroredWallIds; //list of wall Ids over which the original objects reflected itself to create this object
				};
				mirroredObjectType obj;
				SceneType::DeformableObjectType::Pointer tmpObj; 

				for (it = m_proposedMoves.begin() ; it != m_proposedMoves.end() ; ++it) { 
					
					std::vector<unsigned> wallIds, faceIds; //ids of the walls (in [0,2*d]) and faces (in [0,d]) that the current object overlaps
					std::set<unsigned> activeFaceIds; std::pair<std::set<unsigned>::iterator,bool> facesSetInsertResp; //set of already activated faces

					std::vector<mirroredObjectType> currentMirroredObjects, previousMirroredObjects;

					wallIds = IdentifyIntersectionBoundingBoxes( m_scene->GetObject(it->first)->obj->GetPhysicalBoundingBox(), m_scene->GetSceneBoundingBox() );
					if (wallIds.size()==0) continue;

					tmpObj = m_scene->GetObject(it->first)->obj->CreateClone();
					obj.id = it->first;
					currentMirroredObjects.push_back(obj);
					vnl_vector<double> tmpParams;

					for (unsigned i=0 ; i<wallIds.size() ; i++) { 
						faceIds.push_back(wallIds[i]/2);
						facesSetInsertResp = activeFaceIds.insert( faceIds.back() );
						if (facesSetInsertResp.second) { //new face: add the 'reflections' of all objects in the currentMirroredObjects
							previousMirroredObjects = currentMirroredObjects; //backup this set of objects, can be useful in some cases
							for (unsigned idt=0 ; idt<currentMirroredObjects.size() ; idt++) {
								obj = currentMirroredObjects[idt]; //set the parameters of the mirrored object, starting from the current object in the list
								tmpParams = m_scene->GetParametersOfObject(obj.id);
								if (wallIds[i] == 2*faceIds[i]) { //0 or 2 or 4, ... (first 'wall' of that 'face')
									obj.mirroredWallIds.push_back(wallIds[i]+1);
									tmpParams(faceIds[i]) += m_sceneWidth(faceIds[i]);
								}
								else { //1 or 3 or 5, ... (second 'wall' of that 'face')
									obj.mirroredWallIds.push_back(wallIds[i]-1);
									tmpParams(faceIds[i]) -= m_sceneWidth(faceIds[i]);
								}
								//add it to the scene
								tmpObj->SetParameters(tmpParams); 
								obj.id = m_scene->AddObject( tmpObj );
								if (obj.id == 0) { continue; } //it can happen that the bounding box intersects with the border, but still the original object was fully inside the scene, so let just skip those cases...
								else { 
									if ( m_scene->GetObject(obj.id)->interactionData.find( it->first ) != m_scene->GetObject(obj.id)->interactionData.end() ) { //if the mirrored object intersects the original, then remove it.
										std::cout<<"ForceBiasedAlgorithm with periodic boundary conditions: WARNING: the object 'reflection' overlaps the original object! this reflection is ignored..."<<std::endl;
										m_scene->RemoveObject( obj.id );
									}
									else currentMirroredObjects.push_back( obj ); 
								}
							}
						}
						else { // this is the same face as in the previous iteration, but the opposite wall, only the objects in previousMirroredObjects need to be 'reflected'.
							//this case shouldn't happen often, it means the object is crossing the entire scene
							for (unsigned idt=0 ; idt<previousMirroredObjects.size() ; idt++) {
								obj = previousMirroredObjects[idt]; //set the parameters of the mirrored object, starting from the current object in the list
								tmpParams = m_scene->GetParametersOfObject(obj.id);
								if (wallIds[i] == 2*faceIds[i]) { //0 or 2 or 4, ... (first 'wall' of that 'face')
									obj.mirroredWallIds.push_back(wallIds[i]+1);
									tmpParams(faceIds[i]) += m_sceneWidth(faceIds[i]);
								}
								else { //1 or 3 or 5, ... (second 'wall' of that 'face')
									obj.mirroredWallIds.push_back(wallIds[i]-1);
									tmpParams(faceIds[i]) -= m_sceneWidth(faceIds[i]);
								}
								//add it to the scene
								tmpObj->SetParameters(tmpParams);
								obj.id = m_scene->AddObject( tmpObj );
								if (obj.id == 0) { continue; } 
								else { 
									if ( m_scene->GetObject(obj.id)->interactionData.find( it->first ) != m_scene->GetObject(obj.id)->interactionData.end() ) { //if the mirrored object intersects the original, then remove it.
										std::cout<<"ForceBiasedAlgorithm with periodic boundary conditions: WARNING: the object 'reflection' overlaps the original object! this reflection is ignored..."<<std::endl;
										m_scene->RemoveObject( obj.id );
									}
									else currentMirroredObjects.push_back( obj ); 
								}
							}
						}
					}
					

					//treat all temporary objects that have been created
					std::vector<unsigned> distantWallIds; bool distantCrossesSameFace; //wall ids that the distant object crosses
					for (unsigned i = 1 ; i<currentMirroredObjects.size() ; i++) { //index 0 is the original object, so skip it 
						// for all temporary reflections of the current object, check for intersections with other objects of the scene
						for (SceneType::ObjectInteractionIterator dit = m_scene->GetObject(currentMirroredObjects[i].id)->interactionData.begin() ; dit != m_scene->GetObject(currentMirroredObjects[i].id)->interactionData.end() ; dit++) {
							//check whether the distant object touches the same border
							distantWallIds = IdentifyIntersectionBoundingBoxes( m_scene->GetObject(dit->first)->obj->GetPhysicalBoundingBox(), m_scene->GetSceneBoundingBox() );
							//look for common entries between distantWallIds and currentMirroredObjects[i].mirroredWallIds
							std::vector<unsigned> intersectWalldIds;
							std::set_intersection(currentMirroredObjects[i].mirroredWallIds.begin(), currentMirroredObjects[i].mirroredWallIds.end(), 
								distantWallIds.begin(), distantWallIds.end(), std::inserter(intersectWalldIds, intersectWalldIds.end()) );
							if ( (!intersectWalldIds.empty()) && (dit->first < it->first) ) continue; //this overlap was already treated via the reflections of the object dit->first
							//else ... move both objects (the second object may not cross the boundary, so it should be treated immediately
							++nbOverlaps;
							it->second = m_mvtManager->UpdateMove( it->second, currentMirroredObjects[i].id, dit->first, dit->second );
							m_proposedMoves[dit->first] = m_mvtManager->UpdateMove( m_proposedMoves[dit->first], dit->first, currentMirroredObjects[i].id, dit->second );							
						}
						//finished with the overlaps on this face ; now remove the temporary object
						m_scene->RemoveObject(currentMirroredObjects[i].id);
					}

				}

				break;
		}
		if (verbose) std::cout<<"  time after dealing with boundary conditions: "<<(clock()-t0)/((double)CLOCKS_PER_SEC)<<" s."<<std::endl;


		if (verbose) std::cout<<"  nb of overlaps: "<<nbOverlaps<<std::endl;
		if (nbOverlaps==0) return false;

		//apply the moves
		m_scene->GetInteractionManager()->TurnOffInteractionManagement(); 
		unsigned nbModifs = 0;
		for (it = m_proposedMoves.begin() ; it != m_proposedMoves.end() ; ++it) { 
			//downscale the objects -> compute the parameters that realizes this downscaling and apply the moves
			if (m_boundaryEffectCode==PERIODICBOUNDARIES) {
				for (unsigned i=0 ; i<Dimension ; i++) {
					if ( it->second(i) < m_scene->GetSceneBoundingBox()( 2*i ) ) { it->second(i) += m_sceneWidth(i); }
					if ( it->second(i) > m_scene->GetSceneBoundingBox()(2*i+1) ) { it->second(i) -= m_sceneWidth(i); }
				}
			}
			if (m_scene->ModifyObjectParameters( it->first, m_mvtManager->GetScaledParameters(it->first, it->second) )) nbModifs++;
		}

		m_scene->GetInteractionManager()->TurnOnInteractionManagement();         
		if (verbose) std::cout<<"  total time for this iteration, after updating the interactions: "<<(clock()-t0)/((double)CLOCKS_PER_SEC)<<" s."<<std::endl;
		if (nbModifs==0) return false;
		return true;
	}
    
    /** Iterate until convergence 
    * returns -1 if it exited after reaching the maximum number of allowed iterations
    * and 1 otherwise.
    */
    int IterateUntilConvergence(bool verbose = false) {
        if (!m_scene) throw DeformableModelException("ForceBiasedAlgorithm::IterateUntilConvergence -- the scene must be set first.");
        unsigned nbIter=0;
        int converged = 0;
        //draw the scene now
        while (converged==0) {
			if (verbose) std::cout<<"\nIteration "<<nbIter<<std::endl;
            nbIter++;
            if (nbIter>=m_maxNbIteration) { converged = -1; break; }
            if ( !ApplyOneIteration(verbose) ) converged = 1;
			if (verbose) {
				if (Dimension==2) psciob::Write2DGreyLevelRescaledImageToFile<SceneType::LabelImageType>("FB_it" + stringify(nbIter) + ".png", m_scene->GetSceneAsLabelImage());
				if (Dimension==3) psciob::WriteMirrorPolyDataToFile("FB_it" + stringify(nbIter) + ".vtk", m_scene->GetSceneAsVTKPolyData());
			}
        }
        return converged;
    }


protected:
	ForceBiasedAlgorithm() : m_sceneWidth(Dimension) {
        m_scene = 0;
        m_maxNbIteration = 1e10;
        m_mvtManager = FBMovementManagerType::New();
		m_boundaryEffectCode = VOIDBOUNDARIES;
    };
    ~ForceBiasedAlgorithm() {};

    typename SceneType::Pointer m_scene;
    unsigned m_maxNbIteration;
    typename FBMovementManagerType::Pointer m_mvtManager;
    
	vnl_vector<double> m_sceneWidth;

    //temporary storage for the proposal movements
    //it associate a vnl_vector representing the parameters of the proposed moves.
    //those are initialized, filled, and composed using the FBMovementManager
    typedef std::map<IDType, vnl_vector<double>> ProposedMovesType;
    typedef std::pair<IDType, vnl_vector<double>> MovePairType;
    std::map<IDType, vnl_vector<double>> m_proposedMoves;

	BOUNDARYEFFECT m_boundaryEffectCode;	
private:
    ForceBiasedAlgorithm(const Self&);      //purposely not implemented
    const Self & operator=( const Self & ); //purposely not implemented
};





} // namespace psciob

#endif /* __FORCEBIASEDALGORITHM_H_ */
