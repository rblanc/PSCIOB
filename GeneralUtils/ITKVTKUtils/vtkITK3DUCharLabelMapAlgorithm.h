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
  Module:    vtkITK3DUCharLabelMapAlgorithm.h

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
// .NAME vtkITK3DUCharLabelMapAlgorithm - producer of vtkITK3DUCharLabelMapData
// .SECTION Description
// vtkITK3DUCharLabelMapAlgorithm is a superclass for filters that generate
// the special vtkITK3DUCharLabelMapData type.  This data type is a special
// representation of a binary image that can be used as a mask by
// several imaging filters.
// .SECTION see also
// vtkITK3DUCharLabelMapData vtkImageStencilSource

#ifndef __vtkITK3DUCharLabelMapAlgorithm_h
#define __vtkITK3DUCharLabelMapAlgorithm_h


#include "vtkAlgorithm.h"

namespace psciob {

class vtkITK3DUCharLabelMapData;

//class VTK_IMAGING_EXPORT vtkITK3DUCharLabelMapAlgorithm : public vtkAlgorithm
class vtkITK3DUCharLabelMapAlgorithm : public vtkAlgorithm
{
public:
  static vtkITK3DUCharLabelMapAlgorithm *New();
  vtkTypeMacro(vtkITK3DUCharLabelMapAlgorithm, vtkAlgorithm);

  void PrintSelf(ostream& os, vtkIndent indent);

  // Description:
  // Get or set the output for this source.
  void SetOutput(vtkITK3DUCharLabelMapData *output);
  vtkITK3DUCharLabelMapData *GetOutput();

  // Description:
  // see vtkAlgorithm for details
  virtual int ProcessRequest(vtkInformation*,
                             vtkInformationVector**,
                             vtkInformationVector*);

protected:
  vtkITK3DUCharLabelMapAlgorithm();
  ~vtkITK3DUCharLabelMapAlgorithm();

  virtual int RequestData(vtkInformation *, vtkInformationVector **,
                  vtkInformationVector *);
  virtual int RequestInformation(vtkInformation *, vtkInformationVector **,
                                 vtkInformationVector *);
  virtual int RequestUpdateExtent(vtkInformation *, vtkInformationVector **,
                                  vtkInformationVector *);
  vtkITK3DUCharLabelMapData *AllocateOutputData(vtkDataObject *out, int* updateExt);

  virtual int FillOutputPortInformation(int, vtkInformation*);

private:
  vtkITK3DUCharLabelMapAlgorithm(const vtkITK3DUCharLabelMapAlgorithm&);  // Not implemented.
  void operator=(const vtkITK3DUCharLabelMapAlgorithm&);  // Not implemented.
};


} // namespace psciob

#endif
