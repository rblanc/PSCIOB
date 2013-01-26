/*
 * This file is part of the psciob library.
 *
 * Copyright (c) 2012, Remi Blanc
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
* \file GeneralizedHoughTransformAlgorithm_SimpleBinning.h
* \author Rémi Blanc 
* \date 15. October 2012
*/


#ifndef __GENERALIZEDHOUGHTRANSFORMALGORITHM_SIMPLEBINNING_H_
#define __GENERALIZEDHOUGHTRANSFORMALGORITHM_SIMPLEBINNING_H_

#include "GeneralizedHoughTransformAlgorithm_Base.h"

namespace psciob {

/** \brief GeneralizedHoughTransformAlgorithm_SimpleBinning
 * use binning at all stages of the algorithm...
 * in the feature space:
 *  -> to cluster feature vectors and their associated votes during the training phase
 *  -> to identify the set of votes corresponding to a input feature vector
 * in the actual image space
 *  -> to regroup and accumulate votes in the detection stage
*/


template<class InputImagePixelType, unsigned int InputImageDimension, unsigned int ObjectDimension = InputImageDimension>
class GeneralizedHoughTransformAlgorithm_SimpleBinning : public GeneralizedHoughTransformAlgorithm_Base<InputImagePixelType, InputImageDimension, ObjectDimension> {
public:
  /** Standard class typedefs. */
  typedef GeneralizedHoughTransformAlgorithm_SimpleBinning   Self;
  typedef GeneralizedHoughTransformAlgorithm_Base   Superclass;
  typedef itk::SmartPointer<Self>                   Pointer;
  /** Run-time type information (and related methods). */
  itkTypeMacro(GeneralizedHoughTransformAlgorithm_SimpleBinning, GeneralizedHoughTransformAlgorithm_Base);
  itkNewMacro(Self);


	/** Defines a grid defining classes for the feature space.
	* the (outer) std::vector should be the same dimensionality as the feature vector
	* for each feature element, the (inner) std::vector indicates a list of bins ( v0, v1, v2, ..., vn, index i is attributed to a value x if vi-1<x<=vi ; 0: x<=v0 ; n if x>vn)
	*/
	void SetFeatureBins(std::vector< std::vector<double> > featureGrid) {
		if ( featureGrid.size() != m_detector->GetFeatureVectorDimension() ) throw DeformableModelException("GeneralizedHoughTransformAlgorithm_SimpleBinning::SetFeatureBins() size mismatch.");
		m_featureGrid = featureGrid;
    std::sort( m_featureGrid[0].begin(), m_featureGrid[0].end()); //make sure the entries are sorted, and an upper limit is provided
		unsigned totalNbGridPoints = m_featureGrid[0].size();
		for (unsigned i=1 ; i<m_featureGrid.size() ; i++) {
      std::sort( m_featureGrid[i].begin(), m_featureGrid[i].end()); //make sure the entries are sorted, and an upper limit is provided
			totalNbGridPoints *= m_featureGrid[i].size();
		}
		m_TrainingFVAssociationStructure.resize(totalNbGridPoints); //add one for the values higher than the expected max...
	}

	/** Defines the discrete grid spacing for aggregating votes
	* by default, the same spacing is used as that of the input image
	* The grid origin and size are derived from the input image, but enlarged such that the center of objects are necessarily inside.
	*/
	void SetDetectorGridSpacing(vnl_vector<double> sp) {
		if (sp.size()!=ObjectDimension) throw DeformableModelException("GeneralizedHoughTransformAlgorithm_SimpleBinning::SetDetectorGridSpacing() size mismatch.");
		m_detectorSpacing = sp;
	}

	void SetDetectorGridSpacing(double sp) {
    m_detectorSpacing.set_size(ObjectDimension);
    for (unsigned i=0 ; i<ObjectDimension ; i++) m_detectorSpacing(i) = sp;
	}

	/** Prints, to the provided stream, the correspondence table between feature vectors and votes, generated in the training step. */
	void PrintFVStructure(std::ostream &stream, itk::Indent indent = 0) const {
		stream << indent << "Printing the table of correspondences between feature vectors and votes" << std::endl;
		stream << indent << "number of entries: " <<m_TrainingFVAssociationStructure.size()<<std::endl;
		FeatureVectorType fv(m_detector->GetFeatureVectorDimension());
		for (unsigned i=0 ; i<m_detector->GetFeatureVectorDimension() ; i++) { fv(i) = m_featureGrid[i][0]; }
		unsigned index=0, mainIndex=0, subIndex=0;
		for (index = 0 ; index<m_TrainingFVAssociationStructure.size() ; index++, subIndex++) {
			//update the feature vector
			if (subIndex>m_featureGrid[mainIndex].size()) { subIndex = 0; fv(mainIndex)=m_featureGrid[mainIndex][0]; mainIndex++; }
			fv(mainIndex)=m_featureGrid[mainIndex][subIndex];
			stream << indent << index <<" - feature = "<<fv<<", votes ("<<m_TrainingFVAssociationStructure[index].size()<<"): ";
			if (m_TrainingFVAssociationStructure[index].size()>0) stream<<m_TrainingFVAssociationStructure[index][0].parameters;
			for (unsigned i=1 ; i<m_TrainingFVAssociationStructure[index].size() ; i++) stream<<" ; "<<m_TrainingFVAssociationStructure[index][i].parameters;
			stream<<std::endl;
		}
		stream << indent << "done printing the FVStruct"<<std::endl;
	}


protected:
	GeneralizedHoughTransformAlgorithm_SimpleBinning() : GeneralizedHoughTransformAlgorithm_Base() {
    };
    ~GeneralizedHoughTransformAlgorithm_SimpleBinning() {};

	//define bins for clustering the votes...
	std::vector< std::vector<double> > m_featureGrid;
	vnl_vector<double> m_detectorSpacing;

	//structure defining the training table - the index corresponds to a feature vector class
	//behind each entry, a list of precursor votes are stored
	std::vector< std::vector<VotePrecursorType> > m_TrainingFVAssociationStructure;

	//supervote: used to aggregate votes in the final stage
	class SuperVoteValueType { // this is only used to remember the base votes, and to accumulate the weights
		public:
		SuperVoteValueType() {};
		SuperVoteValueType(double w, unsigned ind) : weight(w), voteIndices(1,ind) {};
		double weight;                     // sum of weights of base votes
		std::vector<unsigned> voteIndices; // list of indices of the base votes aggregated here, from the m_pollBox
	};
	typedef std::pair< vnl_vector<double>, SuperVoteValueType > SuperVoteEntryType; //pair associating a vector (=object params, excepted the position) and the SuperVoteValueType
	typedef std::map< vnl_vector<double>, SuperVoteValueType, less_vnlvector< vnl_vector<double> > > SuperVoteMapType; //the KEY vector corresponds to the parameters of the object (position excluded to reduce memory), the VALUE indicate the weight and voters for a given hypothesis
	typedef std::pair< vnl_vector_fixed<double, ObjectDimension>, SuperVoteMapType > ObjectHypothesesEntryType; //hypothesis entry: at a location (vnl_vector_fixed), have one or multiple object hypotheses.
	typedef std::map< vnl_vector_fixed<double, ObjectDimension>, SuperVoteMapType, less_vnlfixedvector< vnl_vector<double> > > ObjectHypothesesType;
	ObjectHypothesesType m_ObjectHypotheses; //the KEY here is a vector corresponding to a spatial coordinate (location of a candidate), the VALUE is the list of candidates that (may) share this location.
	

	// Add an entry in the training database, which relate feature vectors and precursor votes.
  // describes how a training feature vector is translated into a vote in the database
	void AddTrainingFeaturePointVote(const FeatureVectorType &featureVector, const VotePrecursorType &votePrecursor) {
		//identify in which bin the feature vector lies... //TODO: this can be optimized using a true 'search' method (/sort), check e.g. lower_bound
		unsigned index=0, increment = 1;
		bool doneWith_i;
		for (unsigned i=0 ; i<featureVector.size() ; i++) { //browse each parameter
      //find the first grid point that is >= than the feature value. If none, select the last grid point.
      unsigned j = std::lower_bound(m_featureGrid[i].begin(), m_featureGrid[i].end(), featureVector(i)) - m_featureGrid[i].begin();
      //update the index, which 'flattens' all dimensions of the feature space...
      index += std::min(j, m_featureGrid[i].size()-1) * increment;
      increment += m_featureGrid[i].size();
    }
		//add the corresponding precursor vote to this index
		m_TrainingFVAssociationStructure[index].push_back(votePrecursor);
	}


	// nothing to do here, since the training elements are already clustered in predefined bins
	void FinalizeTrainingPhase() {
		m_pollBox.clear();
	}




	// Method intended to generate (a set of) vote(s) from a feature point, and add these to a internal container that accumulates all the votes
	// It uses the structure associating feature vectors and votes learned in the training phase, clustering the votes.
  // Fills the m_pollBox
	void GenerateVote(const FeaturePointType &FP) {
		//FP.first  -> Index of the source point
		//FP.second -> associated feature vector
		VoteType vote;
		vote.parameters.set_size(m_sampleObject->GetNumberOfParameters());
		vote.sourcePixel = FP.first;    // coordinates of the pixel at the source of the vote - useful only in the "apply" phase... maybe...

		vnl_vector<double> sourceCoords(InputImageDimension); //physical coordinates of the source <=> offset to apply to the vote to get the predicted object center
		const InputImageType::SpacingType &spacing = m_inputImage->GetSpacing();
		const InputImageType::PointType &origin = m_inputImage->GetOrigin();
		for (unsigned d=0 ; d<InputImageDimension ; d++) {
			sourceCoords(d) = origin[d] + FP.first[d]*spacing[d];
		}

		//look into m_TrainingFVAssociationStructure for entries similar to FP.second
		//1: identify the index in the database which corresponds to the current feature value
		unsigned index=0, increment = 1;
		for (unsigned i=0 ; i<FP.second.size() ; i++) { //browse each parameter 		//TODO: share the code with the method AddTrainingFeaturePointVote (create a function specialized for finding the index...)
      unsigned j = std::lower_bound(m_featureGrid[i].begin(), m_featureGrid[i].end(), FP.second(i)) - m_featureGrid[i].begin();
      index += std::min(j, m_featureGrid[i].size()-1) * increment;
      increment += m_featureGrid[i].size();
		}

		//2: each entry of m_TrainingFVAssociationStructure[index] in the table should cast a vote.
		//...should I already cluster the votes at this stage? this can be done by creating an 'image' where each pixel contains a vector of votes (e.g to handle different parameters), or through a tree-like structure (e.g. quad/octree ; kdtree...)
		//if the votes are agregated, remember to increase the weight of the cluster, rather than 'adding' the vote...
		for (unsigned i=0 ; i<m_TrainingFVAssociationStructure[index].size() ; i++) {
			for (unsigned d=0 ; d<InputImageDimension ; d++) {
				vote.parameters(d) = sourceCoords(d) + m_TrainingFVAssociationStructure[index][i].parameters(d); //the vote uses the translation parameters to indicate the translation to apply to get from the source to the center of the object
			}
			for (unsigned d=InputImageDimension ; d<vote.parameters.size() ; d++) vote.parameters(d) = m_TrainingFVAssociationStructure[index][i].parameters(d);
			vote.weight = m_TrainingFVAssociationStructure[index][i].weight;

			m_pollBox.push_back(vote);
		}
	}





	// GenerateSceneFromVotes() 
	//
	// Method used to find local maxima in the voting space, and detect objects out of it
  // Clusters the votes in a grid, the spacing of which is given by m_detectorSpacing, and the size defined by the input image
  // At each 'pixel' of that grid, different hypotheses can be formulated, and their weight accumulates from the various individual votes
  //TODO (here or in another class?): add a 'kernel' which spreads each vote to the surroundigs. This requires an additional 
  //     parameter controlling the spread, and how the vote-weight vanishes with the distance.
	void GenerateSceneFromVotes() {
	
		//First, check that the grid spacing of the detector is properly set
		if (m_detectorSpacing.size() == 0) { //empty..?
			m_detectorSpacing.set_size(ObjectDimension);
			for (unsigned i=0 ; i<ObjectDimension ; i++) m_detectorSpacing(i) = m_inputImage->GetSpacing()[i];
		}
	
    //Define the output scene
    m_scene->RemoveAllObjects();
    m_scene->SetPhysicalDimensions(BoundingBoxFromITKImage< typename InputImageType >(m_inputImage), m_inputImage->GetSpacing().GetVnlVector());
			
		// I should now aggregate the votes based on their spatial location, whilst preserving the possibility of multiple candidates per location
		vnl_vector_fixed<double, ObjectDimension> pos;
		vnl_vector<double> candidateParams;
		unsigned voteIndex=0;
		for (std::vector<VoteType>::iterator it=m_pollBox.begin() ; it!=m_pollBox.end() ; ++it, ++voteIndex) {
		
			for (unsigned d=0 ; d<ObjectDimension ; d++) { 
				pos(d)= std::floor(it->parameters(d)/m_detectorSpacing(d))*m_detectorSpacing(d); // round it to the closest 'grid' value so that aggregation can take place...
			} 

      //candidate parameters for the object, without the position parameters
      candidateParams = it->parameters.extract( it->parameters.size()-ObjectDimension, ObjectDimension);

			//look into m_ObjectHypotheses if an hypothesis exists at this location.
      //IDEA: use a pow2DTree to speed up the search, through clustering of the spatial locations
			ObjectHypothesesType::iterator hit = m_ObjectHypotheses.find(pos);
			if (hit==m_ObjectHypotheses.end()) { //if the hypothesis is new, create a new one and add it
				SuperVoteMapType voteMap;
				voteMap.insert( SuperVoteEntryType(candidateParams, SuperVoteValueType(it->weight, voteIndex)) );			
				m_ObjectHypotheses.insert( ObjectHypothesesEntryType(pos, voteMap) );
			}
			else { //aggregate this vote to the local hypothesis
        SuperVoteMapType::iterator localIt = hit->second.find(candidateParams);
        if (localIt==hit->second.end()) { //this particular object was not yet considered at this location
          hit->second.insert( SuperVoteEntryType(candidateParams, SuperVoteValueType(it->weight, voteIndex)) );
        }
        else {//this particular object param were already proposed here => just increment the weight
          localIt->second.voteIndices.push_back(voteIndex);
          localIt->second.weight += it->weight;
        }								
			}

			//IDEA: here, I can possibly loop over a kernel to cast this vote on the neighborhood...
			
		}
		
    std::cout<<"initial number of votes: "<<m_pollBox.size()<<", nb of object hypotheses: "<<m_ObjectHypotheses.size()<<std::endl;

		//Once this is done, I can process the hypotheses in m_ObjectHypotheses
		//first, I should put them all in a vector, and sort them against their weight
		//then, I retain the one with highest weight, add the corresponding object to the output scene
		//then, remove the votes (at least those for this retained candidate) from the feature points that were supporting this hypothesis
		//continue until some minimum weight value is reached...
    
    bool continueDetection = true;
    vnl_vector<double> objectParams;
    ObjectHypothesesType::iterator bestloc_it;
    SuperVoteMapType::iterator bestobj_it;
    while (continueDetection) {
      //browse all active votes and identify the best hypothesis
      double bestWeight=-1;
      for (ObjectHypothesesType::iterator hit = m_ObjectHypotheses.begin() ; hit!=m_ObjectHypotheses.end() ; ++hit) {
        for (SuperVoteMapType::iterator localIt = hit->second.begin() ; localIt!=hit->second.end() ; ++localIt) {
          if (localIt->second.weight>bestWeight) { bestWeight = localIt->second.weight; bestloc_it = hit; bestobj_it = localIt; }
        }
      }

      //if (bestWeight<0.5) {
      //  continueDetection=false;
      //  break;
      //}
      
      //add the best hypothesis as an object in the output scene.
      objectParams.set_size(ObjectDimension + bestobj_it->first.size());
      //the object parameters are directly given by the hypothesis
      for (unsigned d=0 ; d<bestobj_it->first.size() ; d++) objectParams(d+ObjectDimension) = bestobj_it->first(d);
      //the location is averaged from the location of original votes.
      for (unsigned d=0 ; d<ObjectDimension ; d++) objectParams(d) = 0;
      for (std::vector<unsigned>::iterator nit=bestobj_it->second.voteIndices.begin() ; nit!=bestobj_it->second.voteIndices.end() ; ++nit) {
        for (unsigned d=0 ; d<ObjectDimension ; d++) objectParams(d) += m_pollBox[*nit].parameters(d);
      }
      for (unsigned d=0 ; d<ObjectDimension ; d++) objectParams(d) /= bestobj_it->second.voteIndices.size();

      std::cout<<"bestWeight = "<<bestWeight<<", corresponding parameters: "<<objectParams<<std::endl;

      if (!m_sampleObject->SetParameters( objectParams )) throw DeformableModelException("GeneralizedHoughTransformAlgorithm_SimpleBinning::GenerateSceneFromVotes() cannot set parameters of the detected object -- should never happen");
      if (!m_scene->AddObject(m_sampleObject)) throw DeformableModelException("GeneralizedHoughTransformAlgorithm_SimpleBinning::GenerateSceneFromVotes() cannot add detected parameter to the output scene -- should never happen");

      //at least, remove this particular vote from m_ObjectHypotheses
      //IDEA: also remove the votes that were related to this location -> look at localIt->second.voteIndices
      //      those votes should be discarded? ... re-think  about that, perhaps not so clear..
      // --> it some cases, it can be perfectly acceptable to have multiple object centers at similar locations
      //     while in other cases, one may want to enforce non-overlapping objects ; in which case, it would be good to clear hypotheses around (e.g. inside the current object)
      bestloc_it->second.erase(bestobj_it);
      if (bestloc_it->second.empty()) {
        m_ObjectHypotheses.erase(bestloc_it);
        if (m_ObjectHypotheses.empty()) {
          std::cout<<"exit: no remaining hypotheses..."<<std::endl;
          continueDetection = false;
        }
      }

      //continue to add object? stop when bestWeight goes below some threshold?
      if (m_scene->GetNumberOfObjects()>=15) continueDetection = false;

    }
    std::cout<<"TODO. continue implementation ..."<<std::endl;
		
		
  }
  
	
	
private:
  GeneralizedHoughTransformAlgorithm_SimpleBinning(const Self&);            //purposely not implemented
  const Self & operator=( const Self & ); //purposely not implemented
};


} // namespace psciob

#endif /* __GENERALIZEDHOUGHTRANSFORMALGORITHM_SIMPLEBINNING_H_ */
