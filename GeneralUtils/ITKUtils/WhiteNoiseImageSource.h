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
* \file WhiteNoiseImageSource.h
* \author Rémi Blanc 
* \date 3. July 2012
* 
*/


#ifndef __WhiteNoiseImageSource_h
#define __WhiteNoiseImageSource_h

#include "itkImageSource.h"
#include "itkNumericTraits.h"

#include "UnivariatePDF.h"

namespace psciob {

/** \class WhiteNoiseImageSource
 * \brief Generate an n-dimensional scalar image with random pixel values using the provided random number generator
 * \sa UnivariatePDF
 */
template< typename TOutputImage >
class WhiteNoiseImageSource : public itk::ImageSource< TOutputImage >
{
public:
  /** Standard class typedefs. */
  typedef WhiteNoiseImageSource           Self;
  typedef itk::ImageSource< TOutputImage > Superclass;
  typedef itk::SmartPointer< Self >        Pointer;
  typedef itk::SmartPointer< const Self >  ConstPointer;

  /** Typedef for the output image PixelType. */
  typedef typename TOutputImage::PixelType OutputImagePixelType;

  /** Typedef to describe the output image region type. */
  typedef typename TOutputImage::RegionType OutputImageRegionType;

  /** Run-time type information (and related methods). */
  itkTypeMacro(WhiteNoiseImageSource, ImageSource);

  /** Method for creation through the object factory. */
  itkNewMacro(Self);

  /** Basic types from the OutputImageType */
  typedef typename TOutputImage::SizeType         SizeType;
  typedef typename TOutputImage::IndexType        IndexType;
  typedef typename TOutputImage::SpacingType      SpacingType;
  typedef typename TOutputImage::PointType        PointType;
  typedef typename SizeType::SizeValueType        SizeValueType;
  typedef SizeValueType                           SizeValueArrayType[TOutputImage::ImageDimension];
  typedef typename TOutputImage::SpacingValueType SpacingValueType;
  typedef SpacingValueType                        SpacingValueArrayType[TOutputImage::ImageDimension];
  typedef typename TOutputImage::PointValueType   PointValueType;
  typedef PointValueType                          PointValueArrayType[TOutputImage::ImageDimension];

  /** Set/Get size of the output image */
  itkSetMacro(Size, SizeType);
  virtual void SetSize(SizeValueArrayType sizeArray);

  virtual const SizeValueType * GetSize() const;

  /** Set/Get spacing of the output image */
  itkSetMacro(Spacing, SpacingType);
  virtual void SetSpacing(SpacingValueArrayType spacingArray);

  virtual const SpacingValueType * GetSpacing() const;

  /** Set/Get origin of the output image */
  itkSetMacro(Origin, PointType);
  virtual void SetOrigin(PointValueArrayType originArray);

  virtual const PointValueType * GetOrigin() const;

  /** Type of the internal random variable generator */
  typedef UnivariatePDF InternalRandomNumberGeneratorType;
  
  /** Set the internal random variable generator
  * the default is a uniform variate between NumericTraits< OutputImagePixelType >::NonpositiveMin() and NumericTraits< OutputImagePixelType >::max();
  */
  void SetRandomNumberGenerator(InternalRandomNumberGeneratorType *gen) { m_rndGen=gen; }  

protected:
  WhiteNoiseImageSource();
  ~WhiteNoiseImageSource();
  void PrintSelf(std::ostream & os, itk::Indent indent) const;

  virtual void
  ThreadedGenerateData(const OutputImageRegionType &outputRegionForThread, itk::ThreadIdType threadId);

  virtual void GenerateOutputInformation();

  typename InternalRandomNumberGeneratorType::Pointer m_rndGen;
  
private:
  WhiteNoiseImageSource(const WhiteNoiseImageSource &); //purposely not implemented
  void operator=(const WhiteNoiseImageSource &);    //purposely not implemented

  SizeType    m_Size;       //size of the output image
  SpacingType m_Spacing;    //spacing
  PointType   m_Origin;     //origin

  typename TOutputImage::PixelType m_Min; //minimum possible value
  typename TOutputImage::PixelType m_Max; //maximum possible value

  // The following variables are deprecated, and provided here just for
  // backward compatibility. It use is discouraged.
  mutable PointValueArrayType   m_OriginArray;
  mutable SpacingValueArrayType m_SpacingArray;
};

} // namespace psciob

#include "WhiteNoiseImageSource.hxx"

#endif
