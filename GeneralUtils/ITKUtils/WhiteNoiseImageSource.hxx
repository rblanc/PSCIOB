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
* \file WhiteNoiseImageSource.cxx
* \author Rémi Blanc 
* \date 25. May 2012
* 
*/

#ifndef __WhiteNoiseImageSource_hxx
#define __WhiteNoiseImageSource_hxx

#include "WhiteNoiseImageSource.h"
#include "itkImageRegionIterator.h"
#include "itkObjectFactory.h"
#include "itkProgressReporter.h"

namespace psciob {

template< class TOutputImage >
WhiteNoiseImageSource< TOutputImage >
::WhiteNoiseImageSource()
{
  //Initial image is 64 wide in each direction.
  for ( unsigned int i = 0; i < TOutputImage::GetImageDimension(); i++ )
    {
    m_Size[i] = 64;
    m_Spacing[i] = 1.0;
    m_Origin[i] = 0.0;
    }
	
	//default generator is a uniform variate generator
	UniformPDF::Pointer tmpGen = UniformPDF::New();
	tmpGen->SetParameters(itk::NumericTraits< OutputImagePixelType >::NonpositiveMin(), itk::NumericTraits< OutputImagePixelType >::max());
  
	m_rndGen = tmpGen.GetPointer();
	
	this->SetNumberOfThreads(1); //seems necessary, probably something to do with the internal random number generator not supporting multiplethreads -> TODO: implement the GenerateData method instead...
}

template< class TOutputImage >
WhiteNoiseImageSource< TOutputImage >
::~WhiteNoiseImageSource()
{}

template< class TOutputImage >
void
WhiteNoiseImageSource< TOutputImage >
::SetSize(SizeValueArrayType sizeArray)
{
  const unsigned int count = TOutputImage::ImageDimension;
  unsigned int       i;

  for ( i = 0; i < count; i++ )
    {
    if ( sizeArray[i] != this->m_Size[i] )
      {
      break;
      }
    }
  if ( i < count )
    {
    this->Modified();
    for ( i = 0; i < count; i++ )
      {
      this->m_Size[i] = sizeArray[i];
      }
    }
}

template< class TOutputImage >
const typename WhiteNoiseImageSource< TOutputImage >::SizeValueType *
WhiteNoiseImageSource< TOutputImage >
::GetSize() const
{
  return this->m_Size.GetSize();
}

template< class TOutputImage >
void
WhiteNoiseImageSource< TOutputImage >
::SetSpacing(SpacingValueArrayType spacingArray)
{
  const unsigned int count = TOutputImage::ImageDimension;
  unsigned int       i;

  for ( i = 0; i < count; i++ )
    {
    if ( spacingArray[i] != this->m_Spacing[i] )
      {
      break;
      }
    }
  if ( i < count )
    {
    this->Modified();
    for ( i = 0; i < count; i++ )
      {
      this->m_Spacing[i] = spacingArray[i];
      }
    }
}

template< class TOutputImage >
void
WhiteNoiseImageSource< TOutputImage >
::SetOrigin(PointValueArrayType originArray)
{
  const unsigned int count = TOutputImage::ImageDimension;
  unsigned int       i;

  for ( i = 0; i < count; i++ )
    {
    if ( originArray[i] != this->m_Origin[i] )
      {
      break;
      }
    }
  if ( i < count )
    {
    this->Modified();
    for ( i = 0; i < count; i++ )
      {
      this->m_Origin[i] = originArray[i];
      }
    }
}

template< class TOutputImage >
const typename WhiteNoiseImageSource< TOutputImage >::PointValueType *
WhiteNoiseImageSource< TOutputImage >
::GetOrigin() const
{
  for ( unsigned int i = 0; i < TOutputImage::ImageDimension; i++ )
    {
    this->m_OriginArray[i] = this->m_Origin[i];
    }
  return this->m_OriginArray;
}

template< class TOutputImage >
const typename WhiteNoiseImageSource< TOutputImage >::SpacingValueType *
WhiteNoiseImageSource< TOutputImage >
::GetSpacing() const
{
  for ( unsigned int i = 0; i < TOutputImage::ImageDimension; i++ )
    {
    this->m_SpacingArray[i] = this->m_Spacing[i];
    }
  return this->m_SpacingArray;
}

/**
 *
 */
template< class TOutputImage >
void
WhiteNoiseImageSource< TOutputImage >
::PrintSelf(std::ostream & os, itk::Indent indent) const
{
  Superclass::PrintSelf(os, indent);
  os << indent << "Max: "
     << static_cast< typename itk::NumericTraits< OutputImagePixelType >::PrintType >( m_Max )
     << std::endl;
  os << indent << "Min: "
     << static_cast< typename itk::NumericTraits< OutputImagePixelType >::PrintType >( m_Min )
     << std::endl;
  unsigned int i;
  os << indent << "Origin: [";
  for ( i = 0; i < TOutputImage::ImageDimension - 1; i++ )
    {
    os << m_Origin[i] << ", ";
    }
  os << m_Origin[i] << "]" << std::endl;

  os << indent << "Spacing: [";
  for ( i = 0; i < TOutputImage::ImageDimension - 1; i++ )
    {
    os << m_Spacing[i] << ", ";
    }
  os << m_Spacing[i] << "]" << std::endl;

  os << indent << "Size: [";
  for ( i = 0; i < TOutputImage::ImageDimension - 1; i++ )
    {
    os << m_Size[i] << ", ";
    }
  os << m_Size[i] << "]" << std::endl;
}

//----------------------------------------------------------------------------
template< typename TOutputImage >
void
WhiteNoiseImageSource< TOutputImage >
::GenerateOutputInformation()
{
  TOutputImage *output;
  IndexType     index;

  index.Fill(0);

  output = this->GetOutput(0);

  typename TOutputImage::RegionType largestPossibleRegion;
  largestPossibleRegion.SetSize(this->m_Size);
  largestPossibleRegion.SetIndex(index);
  output->SetLargestPossibleRegion(largestPossibleRegion);

  output->SetSpacing(m_Spacing);
  output->SetOrigin(m_Origin);
}

//----------------------------------------------------------------------------
template< typename TOutputImage >
void
WhiteNoiseImageSource< TOutputImage >
::ThreadedGenerateData(const OutputImageRegionType & outputRegionForThread,
					   itk::ThreadIdType threadId)
{
  itkDebugMacro(<< "Generating a random image of scalars");

  // Support progress methods/callbacks
  itk::ProgressReporter progress( this, threadId, outputRegionForThread.GetNumberOfPixels() );

  typedef typename TOutputImage::PixelType scalarType;
  typename TOutputImage::Pointer image = this->GetOutput(0);

  itk::ImageRegionIterator< TOutputImage > it(image, outputRegionForThread);

  // Random number seed
  unsigned int sample_seed = 12345 + threadId;
  double       u;
  double       rnd;

  
//  double dMin = static_cast< double >( m_Min );
//  double dMax = static_cast< double >( m_Max );

  for (; !it.IsAtEnd(); ++it )
    {
//    sample_seed = ( sample_seed * 16807 ) % 2147483647L;
//    u = static_cast< double >( sample_seed ) / 2147483711UL;
//    rnd = ( 1.0 - u ) * dMin + u * dMax;

	rnd = m_rndGen->DrawSample();

    it.Set( (scalarType)rnd );
    progress.CompletedPixel();
    }
}

} // namespace psciob

#endif
