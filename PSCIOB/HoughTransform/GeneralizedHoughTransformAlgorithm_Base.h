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
* \file GeneralizedHoughTransformAlgorithm_Base.h
* \author Rémi Blanc 
* \date 15. October 2012
*/


#ifndef __GENERALIZEDHOUGHTRANSFORMALGORITHM_BASE_H_
#define __GENERALIZEDHOUGHTRANSFORMALGORITHM_BASE_H_


#include "ParametricObject.h" //requires one or more shape types
#include "VTKScene.h"         //uses internally a VTKScene to collect object (should be fast...)
#include "ImageSensor_Base.h" //a sensor to produce images from object instances
#include "ImageFeaturePointDetector_Base.h"

namespace psciob {

/** \brief GeneralizedHoughTransformAlgorithm_Base
*
* This is the base class for implementing the Hough-transform-like algorithms
* There are 2 main steps in the algorithm: a training phase, and an object detector.
* The algorithm is intended to be trained on a given ParametricObject, with a grid of parameters (for multiple object types, one should create multiple instances of the HT and merge the results).
* The training phase requires a sensor (to generate images of the objects), and computes an R-table mapping feature descriptors with object parameters
* The detector phase exploits the same feature detector&descriptor, and the learned R-table to produce votes, and extract object hypotheses from them.
* 
* It is necessary to set the input image, the sensor, the feature point detector and the sample object (+grid) before training or detecting objects
* (the input image is necessary to define the proper image spacing for the training phase)
*/


template<class InputImagePixelType, unsigned int InputImageDimension, unsigned int ObjectDimension = InputImageDimension>
class GeneralizedHoughTransformAlgorithm_Base : public itk::LightObject {
public:
    /** Standard class typedefs. */
    typedef GeneralizedHoughTransformAlgorithm_Base Self;
    typedef itk::LightObject                        Superclass;
    typedef itk::SmartPointer<Self>                 Pointer;
    /** Run-time type information (and related methods). */
    itkTypeMacro(GeneralizedHoughTransformAlgorithm_Base, itk::LightObject);

	/** type of the image on which to perform the detection */
	typedef itk::Image<InputImagePixelType, InputImageDimension>  InputImageType;
	/** Type of pixel indices in the input image */
	typedef typename InputImageType::IndexType                    ImageIndexType;

	/** base type for the parametric object to be detected */
	typedef ParametricObject<ObjectDimension>  BaseObjectType; 
	/** the transform internally uses a VTKScene for the training phase, it should be the most light-weight kind of scene */
	typedef VTKScene<ObjectDimension>          SceneType;

	/** base type for the detector used in the transform */
	typedef ImageFeaturePointDetector_Base<InputImagePixelType, InputImageDimension> FeaturePointDetectorType;
	
	/** base type for the sensor used in the transform */
	typedef typename ImageSensor_Base<SceneType, InputImageType>  SensorType;

	/** type of the feature vector */
	typedef typename FeaturePointDetectorType::FeatureVectorType     FeatureVectorType;
	typedef typename FeaturePointDetectorType::FeatureVectorLessType FeatureVectorLessType;
	/** Type defining a feature point - a pair containin point index & feature vector */
	typedef typename FeaturePointDetectorType::FeaturePointType      FeaturePointType;

protected:
	//vote precursor: light-weight versoin used in the training phase
	class VotePrecursorType {
	public:
		VotePrecursorType() : weight(0) {}
		vnl_vector<double> parameters; //parameters: the translation params indicate the translation to apply to find the object center from the given source
		double weight;                 // typically 1/nb of feature points for this set of shapeparameters
	};

  //useful for comparing votes based on their weight
	struct less_PreVoteWeight : public std::binary_function<VotePrecursorType, VotePrecursorType,bool> {
		bool operator() (const VotePrecursorType& x, const VotePrecursorType& y) const {
      return x.weight<y.weight;
		}
	};

	// type of a vote : a list of parameters, a weight, and a pointer to the coordinates of the feature point that generated it.
	class VoteType : public VotePrecursorType {
		public:
		VoteType() : VotePrecursorType() {}
		ImageIndexType sourcePixel;    // coordinates of the pixel at the source of the vote - useful only in the "apply" phase... maybe...
		bool sameAs(VoteType v) {
			for (unsigned i=ObjectDimension ; i<parameters.size() ; i++) {
				if ( parameters(i)!=v(i) ) return false;
			}
			return true;
		}
	};
	
	//useful for comparing votes based on the location they point to.
	struct less_Vote : public std::binary_function<VoteType, VoteType,bool> {
		bool operator() (const VoteType& x, const VoteType& y) const {
			for (unsigned i=0 ; i<ObjectDimension ; i++) {
				if (x.parameters(i)!=y.parameters(i)) 
					return x.parameters(i)!=y.parameters(i);
			}
		}
	};



public:
	// /**	Set the input image, on which the object(s) shall be detected */
	void SetInputImage(InputImageType *image) {
		m_inputImage = image;
		m_trainingDone = false;
	}

	/** Set the Feature Point detector 
	* perhaps I could, by default, use a standard gradient-based feature with a contour detector...
	* \TODO clone the detector?
	*/
	void SetFeaturePointDetector(FeaturePointDetectorType *detector) {
		m_detector = detector;
		m_trainingDone = false;
	}
	
	/** Set the Sensor which is used to form an image out of the objects
	 * Necessary for the training phase.
	*/
	void SetSensor(SensorType *sensor) { 
		m_trainingDone = false;
		m_sensor = sensor->CreateClone(); 
		m_sensor->SetScene(m_scene);
	}
		
	/** Set a sample of the object type to be detected, together with the grid of parameters (if not specified, only the parameters of the input object will be used) 
	* the (outer) std::vector should be the same dimensionality as the number of parameters of the object.
	* for each parameter, the (inner) std::vector indicates the list of parameter values to be considered for the detection
	* note: translation parameters must be incorporated in the vector, but are ignored (unless some specialized child class overloads the method..., which could perhaps be useful to implement for special handling of rotation & scaling parameters...)
	*/
	virtual void SetSampleObjectAndParameterGrid(BaseObjectType *sample, std::vector< std::vector<double> > grid = std::vector< std::vector<double> >() ) {
		m_sampleObject = sample;
		//if the grid is invalid / not set, use the parameters of the current object.
		if ( grid.size() != m_sampleObject->GetNumberOfParameters() ) {
			m_parameterGrid.clear();			
			for (unsigned i=0 ; i<m_sampleObject->GetNumberOfParameters() ; i++) {
				std::vector<double> pgrid;
				pgrid.push_back( m_sampleObject->GetParameters()(i) );
				m_parameterGrid.push_back(pgrid);
			}
		}
		else {
			m_parameterGrid = grid;
		}
	}


	/** Apply the training phase 
    * base behavior: for each combination of parameters
	*    generate an image of the object using the sensor
	*    apply the detector
	*    for each feature point ('on' pixel), associate the feature vector and the corresponding vote
	*      feature vector = given by the detector
	*      vote = translation + rest of the parameters, weight (1/nb of feature points)
	* -> first, collect the list of associated features & votes (the actual method is pure virtual, 
	*    and can either collect the raw list, or bin the items into predefined classes)
	* -> a pure virtual postprocessing step is then performed, e.g. to cluster features in some optimal way...
	*/
	void Train(bool verbose = false) {
		if (!m_sampleObject) throw DeformableModelException("GeneralizedHoughTransformAlgorithm_Base::Train() a sample object must be available for training, as well as a set of corresponding object parameters.");
		if (!m_sensor) throw DeformableModelException("GeneralizedHoughTransformAlgorithm_Base::Train() a sensor must be set prior to training, in order to generate images from objects.");
		if (!m_detector) throw DeformableModelException("GeneralizedHoughTransformAlgorithm_Base::Train() a feature point detector must be provided to identify points of interest and extract feature vectors for them.");
		if (!m_inputImage) throw DeformableModelException("GeneralizedHoughTransformAlgorithm_Base::Train() an input image must be available (it should at least have the correct pixel spacing...)");
		
		//The goal is to learn a table associating feature vectors (for feature points) with specific votes, indicating the object center & parameters that may have produced this feature point
		//there can be multiple votes for a same feature.
		FeatureVectorType f;
		VotePrecursorType v; v.parameters.set_size(m_sampleObject->GetNumberOfParameters());
			
		vnl_vector<double> currentParams(m_parameterGrid.size());
		vnl_vector<double> pos(m_parameterGrid.size()); pos.fill(0); //indicate the index of the current combination of parameters, within the m_parameterGrid
		bool doneAllCombinations=false, nextFound; int indPos;
		const InputImageType::SpacingType &spacing = m_inputImage->GetSpacing();
		
		if (verbose) std::cout<<"Beginning the GHT training phase"<<std::endl;
		unsigned n_combination=0;
		//browse the set of all possible combinations of parameters... 
		while (!doneAllCombinations) {
			n_combination++;
			m_scene->RemoveAllObjects();
			
			//get the current parameters
			if (verbose) std::cout<<"training combination "<<n_combination<<", object parameters: "<<std::endl;
			for (unsigned i = 0 ; i < m_parameterGrid.size() ; i++) {
				currentParams(i) = m_parameterGrid[i][pos(i)];
				if (verbose) std::cout<<currentParams(i)<<" ";
			}
			
			//synthesize the object and its image 
			m_sampleObject->SetParameters( currentParams );
			vnl_vector<double> objectBoundingBox = m_sampleObject->GetPhysicalBoundingBox(); double boxWidth;
			for (unsigned i=0 ; i<ObjectDimension ; i++) { 
				boxWidth = objectBoundingBox(2*i+1)-objectBoundingBox(2*i); 
				m_maxObjectBBox(2*i) = min( m_maxObjectBBox(2*i), objectBoundingBox(2*i)); m_maxObjectBBox(2*i+1) = max( m_maxObjectBBox(2*i+1), objectBoundingBox(2*i+1));
				objectBoundingBox( 2*i ) -= boxWidth/10.0; objectBoundingBox(2*i+1) += boxWidth/10.0; 				
			}
			m_scene->SetPhysicalDimensions(objectBoundingBox, spacing.GetVnlVector()); //make sure the scene is large enough
			m_scene->AddObject(m_sampleObject);
			if (verbose) {
				std::cout<<"\n  writing the sensor output, filename: trainingObject"<<n_combination<<".png"<<std::endl;
				WriteITKImageToFile< SensorType::OutputImageType >("trainingObject" + stringify(n_combination) + ".png", m_sensor->GetOutputImage());
			}

			//detect feature points on it
			m_detector->SetInputImage( m_sensor->GetOutputImage() );
			const InputImageType::PointType &origin = m_detector->GetInputImage()->GetOrigin();
			const FeaturePointDetectorType::FeaturePointListType &fpList = m_detector->GetFeaturePointList();
			if (verbose) std::cout<<"   number of feature points detected: "<<fpList.size()<<std::endl;

			//associate votes to feature vectors
			//the shape parameters are always the same...
			for (unsigned i=InputImageDimension ; i<m_sampleObject->GetNumberOfParameters() ; i++) v.parameters(i) = currentParams(i);
			v.weight = 1.0/fpList.size();
			if (verbose) std::cout<<"   associating votes to feature vectors"<<std::endl;
			for (FeaturePointDetectorType::FeaturePointListConstIterator it = fpList.begin() ; it != fpList.end() ; ++it) {
				//the translation to apply varies...
				for (unsigned i=0 ; i<InputImageDimension ; i++) {
					v.parameters(i) = currentParams(i)-(origin[i] + it->first[i]*spacing[i]); //the vote uses the translation parameters to indicate the translation to apply to get from the source to the center of the object
				}
				AddTrainingFeaturePointVote(it->second, v);
			}
					
			//go the next set of parameters
			nextFound = false; indPos = m_parameterGrid.size()-1; //indPos indicate which parameter is modified to get the next combination of parameters from the grid
			while (!nextFound) {
				//try to get to the next parameter in the grid
				if (pos(indPos)+1>=m_parameterGrid[indPos].size()) { pos(indPos)=0;  indPos--; } //go to the next parameter if we reach the end of possible values for the current one
				else                                               { pos(indPos)+=1; nextFound=true; }
				if (indPos<0) { nextFound=true; doneAllCombinations=true; } //exit when all combinations have been tried.
			}			
		} //end while, all combinations of parameters have been treated.
		
		FinalizeTrainingPhase();
		m_trainingDone = true;
	}


	/** Save the training table to a file 
	* \todo find a proper way to save/load structured data in a file
	* info to write/load: object type, parameter grid, ...
	* => look for hdf5? as in statismo?
	*/
	//WARNING: remember to overload with the child class to save the learned structure...
	void SaveTrainingDataToFile(std::string filename) {
		throw DeformableModelException("GeneralizedHoughTransformAlgorithm_Base::SaveTrainingDataToFile(), NOT IMPLEMENTED YET -- TODO.");
	}

	/** Load the training data from a file
	* \todo find a proper way to save/load structured data in a file
	* info to write/load: object type, parameter grid, image spacing
	* return a code indicating how things went...
	*/
	int LoadTrainingDataToFile(std::string filename) {
		throw DeformableModelException("GeneralizedHoughTransformAlgorithm_Base::LoadTrainingDataToFile(), NOT IMPLEMENTED YET -- TODO.");
	}
	
	/** Performs the detection, redo it if necessary 
	* This requires that Train() has been performed first.
	* The method works in 3 stages.
	* First: feature points are detected on the input images, and each point casts (a) vote(s) 
	*        by looking for similar feature vector in the training database
	* Second: the set of votes is reduced by aggregating them on a spatial criterion
	* Finally, the detection is performed on the set of the aggregated votes.
	* These steps are also pure virtual.
	*/
	virtual void DetectObjectsOnImage(bool verbose = false) {
		//check if the training has been performed.
		if (!m_trainingDone) throw DeformableModelException("GeneralizedHoughTransformAlgorithm_Base::DetectObjectsOnImage() the training phase must be performed first");
		const typename InputImageType::PointType &origin = m_inputImage->GetOrigin();
		const typename InputImageType::SpacingType &spacing = m_inputImage->GetSpacing();

		//apply the feature point detector on the image
		m_detector->SetInputImage( m_inputImage );
		const FeaturePointDetectorType::FeaturePointListType &fpList = m_detector->GetFeaturePointList();
		
		if (verbose) std::cout<<"   number of feature points detected on the input image: "<<fpList.size()<<std::endl;
		//for each feature point, convert the feature vector into a vote, and fill the pollBox
		for (FeaturePointDetectorType::FeaturePointListConstIterator it = fpList.begin() ; it != fpList.end() ; ++it) {
			GenerateVote(*it); // <- calling a pure virtual class (protected), implemented in the specialized HTAlgo
		}
		
		//convert the set of votes into a set of detected objects, when the 'likelihood' of an object is high (e.g. taking weights into account...)
		GenerateSceneFromVotes(); //a second pure virtual method...
		
		m_uptodate = true;
	}
	
	/** Get the scene generated by the DetectObjectsOnImage() method, calls it if necessary */
	SceneType *GetDetectedScene() {
		if (!m_uptodate) { DetectObjectsOnImage(); }
		return m_scene;
	}
	
protected:
	GeneralizedHoughTransformAlgorithm_Base() {
		m_scene = SceneType::New();
		m_inputImage = NULL;
		m_sampleObject = NULL;
		m_detector = NULL;
		m_sensor = NULL;

		m_uptodate = false; m_trainingDone = false;
		m_maxObjectBBox.set_size(2*ObjectDimension); m_maxObjectBBox.fill(0);
    };
    ~GeneralizedHoughTransformAlgorithm_Base() {};

	typename InputImageType::Pointer m_inputImage;
	vnl_vector<double> m_maxObjectBBox; //0-centered bounding box that is large enough to containg any of the training object.
	
	typename SceneType::Pointer      m_scene;
	typename BaseObjectType::Pointer m_sampleObject; //keep a set of sample objects
	std::vector< std::vector<double> > m_parameterGrid; //the vnl_vector should be the same dimensionality as the object. For each entry, a std::vector indicates which parameter values should be considered.

	typename SensorType::Pointer     m_sensor;
	
	typename FeaturePointDetectorType::Pointer  m_detector;

	bool m_uptodate, m_trainingDone;
	
	std::vector<VoteType> m_pollBox; //in the detection stage, all votes from feature points are collected here.

	
	// Add a pair <featureVector - vote precursos> to the training database
	virtual void AddTrainingFeaturePointVote(const FeatureVectorType &featureVector, const VotePrecursorType &votePrecursor) = 0;

	// Indicate that the training phase is complete - in case the child class needs to postprocess the training database
	virtual void FinalizeTrainingPhase() = 0;

	// Method intended to generate (a set of) vote(s) from a feature point, and add these to a internal container that accumulates all the votes
	// It uses the structure learned in the training phase, clustering the votes from similar feature vectors encountered during training.
	// This requires to identify the entry(ies) of m_TrainingFVMap with features similar to the current feature vector FP.second
	//   there are many options for this:
	//		1: define a 'distance' measure between feature vectors, and find the elements with distance less than a given threshold
	//		2: use binning of the feature space to cluster votes
	virtual void GenerateVote(const FeaturePointType &FP) = 0;
		
	
	// Method used to find local maxima in the voting space, and detect objects out of it 
	virtual void GenerateSceneFromVotes() = 0;
	
	
private:
    GeneralizedHoughTransformAlgorithm_Base(const Self&);            //purposely not implemented
    const Self & operator=( const Self & ); //purposely not implemented
};


} // namespace psciob

#endif /* __GENERALIZEDHOUGHTRANSFORMALGORITHM_BASE_H_ */
