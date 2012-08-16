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

    /** Attach a scene to the algorithm */
    void SetScene(SceneType* scene) { 
        m_scene = scene;
        m_mvtManager->SetScene(m_scene);
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
    bool ApplyOneIteration() {
        m_proposedMoves.clear(); //make sure it starts empty.
        bool noOverlaps = true;
        //initialize the moves to identity for all objects (m_proposedMoves)
        SceneObjectIterator<SceneType> it(m_scene);
        for (it.GoToBegin() ; !it.IsAtEnd() ; ++it) { 
            m_proposedMoves.insert(m_proposedMoves.end(), MovePairType(it.GetID(), m_scene->GetParametersOfObject(it.GetID())) );
        }

        //check for overlaps between objects 
        //for each pair of overlapping objects, propose a move, and compose it with the existing m_proposedMoves
        m_scene->GetInteractionManager()->TurnOnInteractionManagement(); //should not be necessary... but do it just in case...
        for (it.GoToBegin() ; !it.IsAtEnd() ; ++it) { 
            //look for the interactions for this object
            if (!it.GetObject()->interactionData.empty()) noOverlaps = false;
            for (SceneType::ObjectInteractionIterator dit = it.GetObject()->interactionData.begin() ; dit != it.GetObject()->interactionData.end() ; dit++ ) {
                //propose a move unilaterally for the current object, and compose it with the current proposed move
                m_proposedMoves[it.GetID()] = m_mvtManager->UpdateMove( m_proposedMoves[it.GetID()], it.GetID(), dit->first, dit->second );
                //the second object of the pair will be treated when its turn comes
            }
        }
        
        if (noOverlaps) return false;
    
        //apply the moves
        m_scene->GetInteractionManager()->TurnOffInteractionManagement(); 
        for (it.GoToBegin() ; !it.IsAtEnd() ; ++it) { 
            //downscale the objects -> compute the parameters that realizes this downscaling and apply the moves
            m_scene->ModifyObjectParameters( it.GetID(), m_mvtManager->GetScaledParameters(it.GetID(), m_proposedMoves[it.GetID()]) );
            
            
        }
        m_scene->GetInteractionManager()->TurnOnInteractionManagement();         
        
        return true;
    }
    
    /** Iterate until convergence 
    * returns -1 if it exited after reaching the maximum number of allowed iterations
    * and 1 otherwise.
    */
    int IterateUntilConvergence() {
        if (!m_scene) throw DeformableModelException("ForceBiasedAlgorithm::IterateUntilConvergence -- the scene must be set first.");
        unsigned nbIter=0;
        int converged = 0;
        //draw the scene now
        //psciob::Write2DGreyLevelRescaledImageToFile<SceneType::LabelImageType>("FB_it" + stringify(nbIter) + ".png", m_scene->GetSceneAsLabelImage());
        while (converged==0) {
            nbIter++;
            if (nbIter>=m_maxNbIteration) { converged = -1; break; }            
            if ( !ApplyOneIteration() ) converged = 1;
            //draw the scene now
            //psciob::Write2DGreyLevelRescaledImageToFile<SceneType::LabelImageType>("FB_it" + stringify(nbIter) + ".png", m_scene->GetSceneAsLabelImage());
        }
        return converged;
    }


protected:
    ForceBiasedAlgorithm() {
        m_scene = 0;
        m_maxNbIteration = 1e10;
        m_mvtManager = FBMovementManagerType::New();
    };
    ~ForceBiasedAlgorithm() {};

    typename SceneType::Pointer m_scene;
    unsigned m_maxNbIteration;
    typename FBMovementManagerType::Pointer m_mvtManager;
    
    //temporary storage for the proposal movements
    //it associate a vnl_vector representing the parameters of the proposed moves.
    //those are initialized, filled, and composed using the FBMovementManager
    typedef std::map<IDType, vnl_vector<double>> ProposedMovesType;
    typedef std::pair<IDType, vnl_vector<double>> MovePairType;
    std::map<IDType, vnl_vector<double>> m_proposedMoves;


private:
    ForceBiasedAlgorithm(const Self&);      //purposely not implemented
    const Self & operator=( const Self & ); //purposely not implemented
};





} // namespace psciob

#endif /* __FORCEBIASEDALGORITHM_H_ */
