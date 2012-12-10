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


#ifndef __GENERALIZEDHOUGHTRANSFORMALGORITHM_TEST_H_
#define __GENERALIZEDHOUGHTRANSFORMALGORITHM_TEST_H_

#include "GeneralizedHoughTransformAlgorithm_Base.h"

namespace psciob {

/** \brief GeneralizedHoughTransformAlgorithm_Test
* specialized classes must provide a structure for collecting all votes, and implement the methods
* GenerateVote(feature point)
* GenerateSceneFromVotes
* In this case, the following is performed:
*
*/


template<class InputImagePixelType, unsigned int InputImageDimension, unsigned int ObjectDimension = InputImageDimension>
class GeneralizedHoughTransformAlgorithm_Test : public itk::LightObject {
public:
    /** Standard class typedefs. */
    typedef GeneralizedHoughTransformAlgorithm_Test          Self;
    typedef itk::LightObject        Superclass;
    typedef itk::SmartPointer<Self> Pointer;
    /** Run-time type information (and related methods). */
    itkTypeMacro(GeneralizedHoughTransformAlgorithm_Test, itk::LightObject);
    itkNewMacro(Self);

protected:
	GeneralizedHoughTransformAlgorithm_Test() : GeneralizedHoughTransformAlgorithm_Base() {
    };
    ~GeneralizedHoughTransformAlgorithm_Test() {};

	//structure collecting the votes

	// + structures used to store the data about clustering the features...

	// Method intended to generate (a set of) vote(s) from a feature point, and add these to a internal container that accumulates all the votes
	//It uses the m_TrainingFVMap learned in the training phase, clustering the votes from similar feature vectors encountered during training.
	void GenerateVote(const FeaturePointType &FP) {
		
	}

	// Method used to find local maxima in the voting space, and detect objects out of it
	void GenerateSceneFromVotes() {

	}
	
	
private:
    GeneralizedHoughTransformAlgorithm_Test(const Self&);            //purposely not implemented
    const Self & operator=( const Self & ); //purposely not implemented
};


} // namespace psciob

#endif /* __GENERALIZEDHOUGHTRANSFORMALGORITHM_TEST_H_ */
