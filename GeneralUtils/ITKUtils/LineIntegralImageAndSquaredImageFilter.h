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
* \file LineIntegralImageAndSquaredImageFilter.h
* \author Rémi Blanc 
* \date 25. July 2012
*/


#ifndef __LINEINTEGRALIMAGEANDSQUAREDIMAGEFILTER_H
#define __LINEINTEGRALIMAGEANDSQUAREDIMAGEFILTER_H
 
#include "itkImageToImageFilter.h"
 
namespace psciob {

/**
* \class LineIntegralImageAndSquaredImageFilter
* \brief LineIntegralImageAndSquaredImageFilter
* 
* ITK-style filter that computes a sort of integral image, integrating values along each line of the input image (only the first coordinate is varying along each line) 
* computes, at the same time, the sum of squared pixel values.
*/

template< typename TInputImage, typename TOutputImage1, typename TOutputImage2 = TOutputImage1>
class LineIntegralImageAndSquaredImageFilter : public itk::ImageToImageFilter< TInputImage, TOutputImage1 > 
{
public:
  /** Standard class typedefs. */
  typedef LineIntegralImageAndSquaredImageFilter                Self;
  typedef itk::ImageToImageFilter< TInputImage, TOutputImage1 > Superclass;
  typedef itk::SmartPointer< Self >                             Pointer;
 
  /** Method for creation through the object factory. */
  itkNewMacro(Self);
 
  /** Run-time type information (and related methods). */
  itkTypeMacro(LineIntegralImageAndSquaredImageFilter, ImageToImageFilter);
 
  TOutputImage1* GetOutput1();
  TOutputImage2* GetOutput2();
 
protected:
  LineIntegralImageAndSquaredImageFilter();
  ~LineIntegralImageAndSquaredImageFilter(){}
 
  /** Does the real work. */
  virtual void GenerateData();
 
  /**  Create the Output */
  itk::DataObject::Pointer MakeOutput(unsigned int idx);
 
private:
  LineIntegralImageAndSquaredImageFilter(const Self &); //purposely not implemented
  void operator=(const Self &);  //purposely not implemented
 
};

} // namespace psciob

#include "LineIntegralImageAndSquaredImageFilter.txx"
 
#endif // __LINEINTEGRALIMAGEANDSQUAREDIMAGEFILTER_H