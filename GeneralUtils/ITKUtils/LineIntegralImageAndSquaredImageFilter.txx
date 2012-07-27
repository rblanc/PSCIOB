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
* \file LineIntegralImageAndSquaredImageFilter.txx
* \author Rémi Blanc 
* \date 25. July 2012
*/


#include "LineIntegralImageAndSquaredImageFilter.h"


#include "itkObjectFactory.h"
#include "itkImageRegionIterator.h"
#include "itkImageRegionConstIterator.h"

namespace psciob {


template< typename TInputImage, typename TOutputImage1, typename TOutputImage2>
LineIntegralImageAndSquaredImageFilter<TInputImage, TOutputImage1, TOutputImage2>::LineIntegralImageAndSquaredImageFilter()
{
	this->SetNumberOfRequiredOutputs(2);
	this->SetNumberOfRequiredInputs(0);

	this->SetNthOutput( 0, this->MakeOutput(0) );
	this->SetNthOutput( 1, this->MakeOutput(1) );
}

template< typename TInputImage, typename TOutputImage1, typename TOutputImage2>
void LineIntegralImageAndSquaredImageFilter<TInputImage, TOutputImage1, TOutputImage2>::GenerateData()
{
	const TInputImage *inputImage = (this->GetInput());

	// Setup output 1
	typename TOutputImage1::Pointer output1 = this->GetOutput1();
	output1->CopyInformation(inputImage); output1->SetRegions(inputImage->GetLargestPossibleRegion()); output1->Allocate();

	// Setup output 2
	typename TOutputImage2::Pointer output2 = this->GetOutput2();
	output2->CopyInformation(inputImage); output2->SetRegions(inputImage->GetLargestPossibleRegion()); output2->Allocate();


	typedef itk::ImageLinearConstIteratorWithIndex< TInputImage > ConstInIteratorType;
	typedef itk::ImageLinearIteratorWithIndex< TOutputImage1 > Out1IteratorType;
	typedef itk::ImageLinearIteratorWithIndex< TOutputImage2 > Out2IteratorType;

	ConstInIteratorType  inIt(inputImage, inputImage->GetLargestPossibleRegion() );  inIt.SetDirection(0);
	Out1IteratorType     outIt1( output1, inputImage->GetLargestPossibleRegion() ); outIt1.SetDirection(0);
	Out2IteratorType     outIt2( output2, inputImage->GetLargestPossibleRegion() ); outIt2.SetDirection(0);

	TOutputImage1::PixelType partialSum;
	TOutputImage2::PixelType partialSqSum;
	for ( inIt.GoToBegin(), outIt1.GoToBegin() , outIt2.GoToBegin(); ! inIt.IsAtEnd(); outIt1.NextLine(), outIt2.NextLine(), inIt.NextLine())
	{
		inIt.GoToBeginOfLine(); outIt1.GoToBeginOfLine(); outIt2.GoToBeginOfLine(); 
		partialSum = 0; partialSqSum = 0;
		while ( ! inIt.IsAtEndOfLine() ) {
			partialSum += static_cast<TOutputImage1::PixelType>(inIt.Get());
			partialSqSum += static_cast<TOutputImage2::PixelType>(inIt.Get())*static_cast<TOutputImage2::PixelType>(inIt.Get());
			outIt1.Set( partialSum );
			outIt2.Set( partialSqSum );
			++inIt; ++outIt1; ++outIt2;
		}
	}


}

template< typename TInputImage, typename TOutputImage1, typename TOutputImage2>
itk::DataObject::Pointer LineIntegralImageAndSquaredImageFilter<TInputImage, TOutputImage1, TOutputImage2>::MakeOutput(unsigned int idx)
{
	itk::DataObject::Pointer output;

	switch ( idx )
	{
	case 0:
		output = ( TOutputImage1::New() ).GetPointer();
		break;
	case 1:
		output = ( TOutputImage2::New() ).GetPointer();
		break;
	default:
		std::cerr << "No output " << idx << std::endl;
		output = NULL;
		break;
	}
	return output.GetPointer();
}

template< typename TInputImage, typename TOutputImage1, typename TOutputImage2>
TOutputImage1* LineIntegralImageAndSquaredImageFilter<TInputImage, TOutputImage1, TOutputImage2>::GetOutput1()
{
	return dynamic_cast< TOutputImage1 * >(this->ProcessObject::GetOutput(0) );
}

template< typename TInputImage, typename TOutputImage1, typename TOutputImage2>
TOutputImage2* LineIntegralImageAndSquaredImageFilter<TInputImage, TOutputImage1, TOutputImage2>::GetOutput2()
{
	return dynamic_cast< TOutputImage2 * >(this->ProcessObject::GetOutput(1) );
}

} // namespace psciob
