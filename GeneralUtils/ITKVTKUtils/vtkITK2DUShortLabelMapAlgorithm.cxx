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
  Module:    vtkITK2DUShortLabelMapAlgorithm.cxx

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
#include "vtkITK2DUShortLabelMapAlgorithm.h"

#include "vtkITK2DUShortLabelMapData.h"
#include "vtkInformation.h"
#include "vtkInformationVector.h"
#include "vtkObjectFactory.h"
#include "vtkStreamingDemandDrivenPipeline.h"

using namespace psciob;

vtkStandardNewMacro(vtkITK2DUShortLabelMapAlgorithm);

//----------------------------------------------------------------------------
vtkITK2DUShortLabelMapAlgorithm::vtkITK2DUShortLabelMapAlgorithm()
{
  this->SetNumberOfInputPorts(1);
  this->SetNumberOfOutputPorts(1);

  vtkITK2DUShortLabelMapData *output = vtkITK2DUShortLabelMapData::New();
  this->GetExecutive()->SetOutputData(0, output);

  // Releasing data for pipeline parallism.
  // Filters will know it is empty.
  output->ReleaseData();
  output->Delete();
}

//----------------------------------------------------------------------------
vtkITK2DUShortLabelMapAlgorithm::~vtkITK2DUShortLabelMapAlgorithm()
{
}

//----------------------------------------------------------------------------
void vtkITK2DUShortLabelMapAlgorithm::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os,indent);
}

//----------------------------------------------------------------------------
void vtkITK2DUShortLabelMapAlgorithm::SetOutput(vtkITK2DUShortLabelMapData *output)
{
  this->GetExecutive()->SetOutputData(0, output);
}

//----------------------------------------------------------------------------
vtkITK2DUShortLabelMapData *vtkITK2DUShortLabelMapAlgorithm::GetOutput()
{
  if (this->GetNumberOfOutputPorts() < 1)
    {
    return NULL;
    }

  return vtkITK2DUShortLabelMapData::SafeDownCast(
    this->GetExecutive()->GetOutputData(0));
}

//----------------------------------------------------------------------------
vtkITK2DUShortLabelMapData *vtkITK2DUShortLabelMapAlgorithm::AllocateOutputData(
  vtkDataObject *out, int* uExt)
{
  vtkITK2DUShortLabelMapData *res = vtkITK2DUShortLabelMapData::SafeDownCast(out);
  if (!res)
    {
    vtkWarningMacro("Call to AllocateOutputData with non vtkITK2DUShortLabelMapData"
                    " output");
    return NULL;
    }
  res->SetExtent(uExt);
  res->AllocateExtents();

  return res;
}

//----------------------------------------------------------------------------
int vtkITK2DUShortLabelMapAlgorithm::RequestData(
  vtkInformation *,
  vtkInformationVector **,
  vtkInformationVector *outputVector)
{
  vtkInformation *outInfo = outputVector->GetInformationObject(0);
  vtkDataObject *out = outInfo->Get(vtkDataObject::DATA_OBJECT());
  this->AllocateOutputData(
    out,
    outInfo->Get(vtkStreamingDemandDrivenPipeline::UPDATE_EXTENT()));

  return 1;
}

//----------------------------------------------------------------------------
int vtkITK2DUShortLabelMapAlgorithm::RequestInformation(
  vtkInformation *,
  vtkInformationVector **,
  vtkInformationVector *)
{
  return 1;
}

//----------------------------------------------------------------------------
int vtkITK2DUShortLabelMapAlgorithm::RequestUpdateExtent(
  vtkInformation *,
  vtkInformationVector **,
  vtkInformationVector *)
{
  return 1;
}

//----------------------------------------------------------------------------
int vtkITK2DUShortLabelMapAlgorithm::FillOutputPortInformation(
  int, vtkInformation* info)
{
  info->Set(vtkDataObject::DATA_TYPE_NAME(), "vtkITK2DUShortLabelMapData");
  return 1;
}

//----------------------------------------------------------------------------
int vtkITK2DUShortLabelMapAlgorithm::ProcessRequest(
  vtkInformation* request,
  vtkInformationVector** inputVector,
  vtkInformationVector* outputVector)
{
  // generate the data
  if(request->Has(vtkDemandDrivenPipeline::REQUEST_DATA()))
    {
    this->RequestData(request, inputVector, outputVector);
    return 1;
    }

  // execute information
  if(request->Has(vtkDemandDrivenPipeline::REQUEST_INFORMATION()))
    {
    this->RequestInformation(request, inputVector, outputVector);
    return 1;
    }

  if (request->Has(vtkStreamingDemandDrivenPipeline::REQUEST_UPDATE_EXTENT()))
    {
    this->RequestUpdateExtent(request, inputVector, outputVector);
    return 1;
    }

  return this->Superclass::ProcessRequest(request, inputVector, outputVector);
}

