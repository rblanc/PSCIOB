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
  Module:    vtkITK2DUIntLabelMapAlgorithm.h

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
// .NAME vtkITK2DUIntLabelMapAlgorithm - producer of vtkITK2DUIntLabelMapData
// .SECTION Description
// vtkITK2DUIntLabelMapAlgorithm is a superclass for filters that generate
// the special vtkITK2DUIntLabelMapData type.  This data type is a special
// representation of a binary image that can be used as a mask by
// several imaging filters.
// .SECTION see also
// vtkITK2DUIntLabelMapData vtkImageStencilSource

#ifndef __vtkITK2DUIntLabelMapAlgorithm_h
#define __vtkITK2DUIntLabelMapAlgorithm_h


#include "vtkAlgorithm.h"

namespace psciob {

class vtkITK2DUIntLabelMapData;

//class VTK_IMAGING_EXPORT vtkITK2DUIntLabelMapAlgorithm : public vtkAlgorithm
class vtkITK2DUIntLabelMapAlgorithm : public vtkAlgorithm
{
public:
  static vtkITK2DUIntLabelMapAlgorithm *New();
  vtkTypeMacro(vtkITK2DUIntLabelMapAlgorithm, vtkAlgorithm);

  void PrintSelf(ostream& os, vtkIndent indent);

  // Description:
  // Get or set the output for this source.
  void SetOutput(vtkITK2DUIntLabelMapData *output);
  vtkITK2DUIntLabelMapData *GetOutput();

  // Description:
  // see vtkAlgorithm for details
  virtual int ProcessRequest(vtkInformation*,
                             vtkInformationVector**,
                             vtkInformationVector*);

protected:
  vtkITK2DUIntLabelMapAlgorithm();
  ~vtkITK2DUIntLabelMapAlgorithm();

  virtual int RequestData(vtkInformation *, vtkInformationVector **,
                  vtkInformationVector *);
  virtual int RequestInformation(vtkInformation *, vtkInformationVector **,
                                 vtkInformationVector *);
  virtual int RequestUpdateExtent(vtkInformation *, vtkInformationVector **,
                                  vtkInformationVector *);
  vtkITK2DUIntLabelMapData *AllocateOutputData(vtkDataObject *out, int* updateExt);

  virtual int FillOutputPortInformation(int, vtkInformation*);

private:
  vtkITK2DUIntLabelMapAlgorithm(const vtkITK2DUIntLabelMapAlgorithm&);  // Not implemented.
  void operator=(const vtkITK2DUIntLabelMapAlgorithm&);  // Not implemented.
};


} // namespace psciob

#endif
