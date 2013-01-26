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
* \file GeneralizedHoughTransformAlgorithm_Test.h
* \author Rémi Blanc 
* \date 15. October 2012
*/

//TODO: rename class to indicate binning_based GHT ...

#ifndef __GENERALIZEDHOUGHTRANSFORMALGORITHM_TEST_H_
#define __GENERALIZEDHOUGHTRANSFORMALGORITHM_TEST_H_

#include "GeneralizedHoughTransformAlgorithm_Base.h"

namespace psciob {

/** \brief GeneralizedHoughTransformAlgorithm_Test
 * use binning at all stages of the algorithm...
 * in the feature space:
 *  -> to cluster feature vectors and their associated votes during the training phase
 *  -> to identify the set of votes corresponding to a input feature vector
 * in the actual image space
 *  -> to regroup and accumulate votes in the detection stage
*/


template<class InputImagePixelType, unsigned int InputImageDimension, unsigned int ObjectDimension = InputImageDimension>
class GeneralizedHoughTransformAlgorithm_Test : public GeneralizedHoughTransformAlgorithm_Base<InputImagePixelType, InputImageDimension, ObjectDimension> {
public:
    /** Standard class typedefs. */
    typedef GeneralizedHoughTransformAlgorithm_Test   Self;
    typedef GeneralizedHoughTransformAlgorithm_Base   Superclass;
    typedef itk::SmartPointer<Self>                   Pointer;
    /** Run-time type information (and related methods). */
    itkTypeMacro(GeneralizedHoughTransformAlgorithm_Test, GeneralizedHoughTransformAlgorithm_Base);
    itkNewMacro(Self);


	/** Defines a grid defining classes for the feature space.
	* the (outer) std::vector should be the same dimensionality as the feature vector
	* for each feature element, the (inner) std::vector indicates a list of bins ( v0, v1, v2, ..., vn, index i is attributed to a value x if vi-1<x<=vi ; 0: x<=v0 ; n if x>vn)
	*/
	void SetFeatureBins(std::vector< std::vector<double> > featureGrid) {
		if ( featureGrid.size() != m_detector->GetFeatureVectorDimension() ) throw DeformableModelException("GeneralizedHoughTransformAlgorithm_Test::SetFeatureBins() size mismatch.");
		m_featureGrid = featureGrid;
		unsigned totalNbGridPoints = m_featureGrid[0].size();
		for (unsigned i=1 ; i<m_featureGrid.size() ; i++) {
			totalNbGridPoints *= m_featureGrid[i].size();
		}
		m_TrainingFVAssociationStructure.resize(totalNbGridPoints+1); //add one for the values higher than the expected max...
	}

	/** Defines the discrete grid spacing for aggregating votes
	* by default, the same spacing is used as that of the input image
	* The grid origin and size are derived from the input image, but enlarged such that the center of objects are necessarily inside.
	*/
	void SetDetectorGridSpacing(vnl_vector<double> sp) {
		if (sp.size()!=ObjectDimension) throw DeformableModelException("GeneralizedHoughTransformAlgorithm_Test::SetDetectorGridSpacing() size mismatch.");
		m_detectorSpacing = sp;
	}


	/** Prints the correspondence table to the standard output */
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


	/** Prints the list of votes*/
	void PrintVotes(std::ostream &stream, itk::Indent indent = 0) const {
		stream << indent << "Printing the set of votes generated by the input image" << std::endl;
		stream << indent << "number of entries: " <<m_pollBox.size()<<std::endl;

		for (unsigned i=0 ; i<m_pollBox.size() ; i++) {
			stream << indent << "  entry "<<i<<", weight = "<<m_pollBox[i].weight<<", sourcePixel: "<<m_pollBox[i].sourcePixel<<", vote: "<<m_pollBox[i].parameters<<std::endl;
		}
	}

protected:
	GeneralizedHoughTransformAlgorithm_Test() : GeneralizedHoughTransformAlgorithm_Base() {
    };
    ~GeneralizedHoughTransformAlgorithm_Test() {};

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
	void AddTrainingFeaturePointVote(const FeatureVectorType &featureVector, const VotePrecursorType &votePrecursor) {
		//identify in which bin the feature vector lies... //TODO: this can be optimized using a true 'search' method (/sort), check e.g. lower_bound
		unsigned index=0, increment = 1;
		bool doneWith_i;
		for (unsigned i=0 ; i<featureVector.size() ; i++) { //browse each parameter
			doneWith_i = false;
			for (unsigned j=0 ; j<m_featureGrid[i].size() ; j++) {
				if ( featureVector(i) < m_featureGrid[i][j] ) {//found the bin for this dimension
					increment += m_featureGrid[i].size();
					doneWith_i = true; continue;
				}
				else { //continue browsing through this dimension
					index += increment;
				}
			}
			if (!doneWith_i) { //then put it in the last bin of this dimension...
				increment += m_featureGrid[i].size();
			}
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
		//identify in which bin the feature vector lies... //TODO: this can be optimized using a true 'search' method (/sort)
		//TODO: share the code with the method AddTrainingFeaturePointVote (create a function specialized for finding the index...)
		unsigned index=0, increment = 1;
		bool doneWith_i;
		for (unsigned i=0 ; i<FP.second.size() ; i++) { //browse each parameter
			doneWith_i = false;
			for (unsigned j=0 ; j<m_featureGrid[i].size() ; j++) {
				if ( FP.second(i) < m_featureGrid[i][j] ) {//found the bin for this dimension
					increment += m_featureGrid[i].size();
					doneWith_i = true; continue;
				}
				else { //continue browsing through this dimension
					index += increment;
				}
			}
			if (!doneWith_i) { //then put it in the last bin of this dimension...
				increment += m_featureGrid[i].size();
			}
		}

		//each entry in the table should cast a vote.
		//...should I already cluster the votes at this stage?
		//this can be done by creating an 'image' where each pixel contains a vector of votes (e.g to handle different parameters)
		//or through a tree-like structure (e.g. quad/octree ; kdtree...)
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
	void GenerateSceneFromVotes() {
	
		//First, check that the grid spacing of the detector is properly set
		if (m_detectorSpacing.size() == 0) { //empty..?
			m_detectorSpacing.set_size(ObjectDimension);
			for (unsigned i=0 ; i<ObjectDimension ; i++) m_detectorSpacing(i) = m_inputImage->GetSpacing()[i];
		}
	
    //Define the output scene
    m_scene->RemoveAllObjects();
    m_scene->SetPhysicalDimensions(BoundingBoxFromITKImage< typename InputImageType >(m_inputImage), m_detectorSpacing);
			
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
			if (hit==m_ObjectHypotheses.end()) { //if the hypothesis is new, add it
				SuperVoteMapType voteMap;
				voteMap.insert( SuperVoteEntryType(candidateParams, SuperVoteValueType(it->weight, voteIndex)) );			
				m_ObjectHypotheses.insert( ObjectHypothesesEntryType(pos, voteMap) );
			}
			else { //aggregate this vote to the local hypothesis
        SuperVoteMapType::iterator localIt = hit->second.find(candidateParams);
        if (localIt==hit->second.end()) { //this particular object was not yet considered at this location
          hit->second.insert( SuperVoteEntryType(candidateParams, SuperVoteValueType(it->weight, voteIndex)) );
        }
        else {//this particular object param were already proposed here, in which case I just need to increment the weight
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

      std::cout<<"bestWeight = "<<bestWeight<<std::endl;
      //if (bestWeight<0.5) {
      //  continueDetection=false;
      //  break;
      //}
      
      //add the best hypothesis as an object in the output scene.
      objectParams.set_size(ObjectDimension + bestobj_it->first.size());
      for (unsigned d=0 ; d<ObjectDimension ; d++) objectParams(d) = bestloc_it->first(d);
      for (unsigned d=0 ; d<bestobj_it->first.size() ; d++) objectParams(d+ObjectDimension) = bestobj_it->first(d);
      std::cout<<"parameters of the current best hypothesis = "<<objectParams<<std::endl;
      std::cout<<"number of parameters expected for this object type: "<<m_sampleObject->GetNumberOfParameters()<<", ObjectDim: "<<ObjectDimension<<", bestobj_it->first.size() = "<<bestobj_it->first.size()<<std::endl;
      if (!m_sampleObject->SetParameters( objectParams )) throw DeformableModelException("GeneralizedHoughTransformAlgorithm_Test::GenerateSceneFromVotes() cannot set parameters of the detected object -- should never happen");
      if (!m_scene->AddObject(m_sampleObject)) throw DeformableModelException("GeneralizedHoughTransformAlgorithm_Test::GenerateSceneFromVotes() cannot add detected parameter to the output scene -- should never happen");

      //at least, remove this particular vote from m_ObjectHypotheses
      //IDEA: also remove the votes that were related to this location -> look at localIt->second.voteIndices
      //      those votes should be discarded? ... re-think  about that, perhaps not so clear..
      // --> it some cases, it can be perfectly acceptable to have multiple object centers at similar locations
      //     while in other cases, one may want to enforce non-overlapping objects ; in which case, it would be good to clear hypotheses around (e.g. inside the current object)
      bestloc_it->second.erase(bestobj_it);
      if (bestloc_it->second.empty()) m_ObjectHypotheses.erase(bestloc_it);
    

      //continue to add object? stop when bestWeight goes below some threshold?
      if (m_scene->GetNumberOfObjects()>=10) continueDetection = false;
      std::cout<<"TODO. continue implementation here..."<<std::endl;
    }
		
		
  }
  
	
	
private:
    GeneralizedHoughTransformAlgorithm_Test(const Self&);            //purposely not implemented
    const Self & operator=( const Self & ); //purposely not implemented
};


} // namespace psciob

#endif /* __GENERALIZEDHOUGHTRANSFORMALGORITHM_TEST_H_ */
