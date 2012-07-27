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

/*=========================================================================

  Program:   Visualization Toolkit
  Module:    vtkITK2DUCharLabelMapSource.h

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
// .NAME vtkITK2DUCharLabelMapSource - generate an image stencil
// .SECTION Description
// vtkITK2DUCharLabelMapSource is a superclass for filters that generate image
// stencils.  Given a clipping object such as a vtkImplicitFunction, it
// will set up a list of clipping extents for each x-row through the
// image data.  The extents for each x-row can be retrieved via the 
// GetNextExtent() method after the extent lists have been built
// with the BuildExtents() method.  For large images, using clipping
// extents is much more memory efficient (and slightly more time-efficient)
// than building a mask.  This class can be subclassed to allow clipping
// with objects other than vtkImplicitFunction.
// .SECTION see also
// vtkImplicitFunction vtkImageStencil vtkPolyDataToImageStencil

#ifndef __vtkITK2DUCharLabelMapSource_h
#define __vtkITK2DUCharLabelMapSource_h


#include "vtkITK2DUCharLabelMapAlgorithm.h"


class vtkImageData;

namespace psciob {
class vtkITK2DUCharLabelMapData;

//class VTK_IMAGING_EXPORT vtkITK2DUCharLabelMapSource : public vtkITK2DUCharLabelMapAlgorithm
class vtkITK2DUCharLabelMapSource : public vtkITK2DUCharLabelMapAlgorithm
{
public:
  static vtkITK2DUCharLabelMapSource *New();
  vtkTypeMacro(vtkITK2DUCharLabelMapSource, vtkITK2DUCharLabelMapAlgorithm);

  void PrintSelf(ostream& os, vtkIndent indent);

  // Description:
  // Set a vtkImageData that has the Spacing, Origin, and
  // WholeExtent that will be used for the stencil.  This
  // input should be set to the image that you wish to
  // apply the stencil to.  If you use this method, then
  // any values set with the SetOutputSpacing, SetOutputOrigin,
  // and SetOutputWholeExtent methods will be ignored.
  virtual void SetInformationInput(vtkImageData*);
  vtkGetObjectMacro(InformationInput, vtkImageData);

  // Description:
  // Set the Origin to be used for the stencil.  It should be
  // set to the Origin of the image you intend to apply the
  // stencil to. The default value is (0,0,0).
  vtkSetVector3Macro(OutputOrigin, double);
  vtkGetVector3Macro(OutputOrigin, double);

  // Description:
  // Set the Spacing to be used for the stencil. It should be
  // set to the Spacing of the image you intend to apply the
  // stencil to. The default value is (1,1,1)
  vtkSetVector3Macro(OutputSpacing, double);
  vtkGetVector3Macro(OutputSpacing, double);

  // Description:
  // Set the whole extent for the stencil (anything outside
  // this extent will be considered to be "outside" the stencil).
  vtkSetVector6Macro(OutputWholeExtent, int);
  vtkGetVector6Macro(OutputWholeExtent, int);

  // Description:
  // Report object referenced by instances of this class.
  virtual void ReportReferences(vtkGarbageCollector*);

protected:
  vtkITK2DUCharLabelMapSource();
  ~vtkITK2DUCharLabelMapSource();

  virtual int RequestInformation(vtkInformation *, vtkInformationVector **,
                                 vtkInformationVector *);

  vtkImageData *InformationInput;

  int OutputWholeExtent[6];
  double OutputOrigin[3];
  double OutputSpacing[3];

private:
  vtkITK2DUCharLabelMapSource(const vtkITK2DUCharLabelMapSource&);  // Not implemented.
  void operator=(const vtkITK2DUCharLabelMapSource&);  // Not implemented.
};


} // namespace psciob

#endif

