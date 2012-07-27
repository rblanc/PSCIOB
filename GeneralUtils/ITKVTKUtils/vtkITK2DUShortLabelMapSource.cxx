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
Module:    vtkITK2DUShortLabelMapSource.cxx

Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
All rights reserved.
See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

This software is distributed WITHOUT ANY WARRANTY; without even
the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
#include "vtkITK2DUShortLabelMapSource.h"

#include "vtkITK2DUShortLabelMapData.h"
#include "vtkImageData.h"
#include "vtkInformation.h"
#include "vtkInformationVector.h"
#include "vtkObjectFactory.h"
#include "vtkStreamingDemandDrivenPipeline.h"
#include "vtkGarbageCollector.h"

using namespace psciob;

vtkStandardNewMacro(vtkITK2DUShortLabelMapSource);
vtkCxxSetObjectMacro(vtkITK2DUShortLabelMapSource, InformationInput, vtkImageData);

//----------------------------------------------------------------------------
vtkITK2DUShortLabelMapSource::vtkITK2DUShortLabelMapSource()
{
	this->InformationInput = NULL;

	this->OutputOrigin[0] = 0;
	this->OutputOrigin[1] = 0;
	this->OutputOrigin[2] = 0;

	this->OutputSpacing[0] = 1;
	this->OutputSpacing[1] = 1;
	this->OutputSpacing[2] = 1;

	this->OutputWholeExtent[0] = 0;
	this->OutputWholeExtent[1] = -1;
	this->OutputWholeExtent[2] = 0;
	this->OutputWholeExtent[3] = -1;
	this->OutputWholeExtent[4] = 0;
	this->OutputWholeExtent[5] = -1;
}

//----------------------------------------------------------------------------
vtkITK2DUShortLabelMapSource::~vtkITK2DUShortLabelMapSource()
{
	this->SetInformationInput(NULL);
}

//----------------------------------------------------------------------------
void vtkITK2DUShortLabelMapSource::PrintSelf(ostream& os, vtkIndent indent)
{
	this->Superclass::PrintSelf(os,indent);

	os << indent << "InformationInput: " << this->InformationInput << "\n";

	os << indent << "OutputSpacing: " << this->OutputSpacing[0] << " " <<
		this->OutputSpacing[1] << " " << this->OutputSpacing[2] << "\n";
	os << indent << "OutputOrigin: " << this->OutputOrigin[0] << " " <<
		this->OutputOrigin[1] << " " << this->OutputOrigin[2] << "\n";
	os << indent << "OutputWholeExtent: " << this->OutputWholeExtent[0] << " " <<
		this->OutputWholeExtent[1] << " " << this->OutputWholeExtent[2] << " " <<
		this->OutputWholeExtent[3] << " " << this->OutputWholeExtent[4] << " " <<
		this->OutputWholeExtent[5] << "\n";
}

//----------------------------------------------------------------------------
void vtkITK2DUShortLabelMapSource::ReportReferences(vtkGarbageCollector* collector)
{
	this->Superclass::ReportReferences(collector);
	vtkGarbageCollectorReport(collector, this->InformationInput,
		"InformationInput");
}

//----------------------------------------------------------------------------
int vtkITK2DUShortLabelMapSource::RequestInformation(
	vtkInformation *,
	vtkInformationVector **,
	vtkInformationVector *outputVector)
{
	int wholeExtent[6];
	double spacing[3];
	double origin[3];

	vtkInformation *outInfo = outputVector->GetInformationObject(0);

	for (int i = 0; i < 3; i++)
	{
		wholeExtent[2*i] = this->OutputWholeExtent[2*i];
		wholeExtent[2*i+1] = this->OutputWholeExtent[2*i+1];
		spacing[i] = this->OutputSpacing[i];
		origin[i] = this->OutputOrigin[i];
	}

	// If InformationInput is set, then get the spacing,
	// origin, and whole extent from it.
	if (this->InformationInput)
	{
		this->InformationInput->UpdateInformation();
		this->InformationInput->GetWholeExtent(wholeExtent);
		this->InformationInput->GetSpacing(spacing);
		this->InformationInput->GetOrigin(origin);
	}

	outInfo->Set(vtkStreamingDemandDrivenPipeline::WHOLE_EXTENT(),
		wholeExtent, 6);
	outInfo->Set(vtkDataObject::SPACING(), spacing, 3);
	outInfo->Set(vtkDataObject::ORIGIN(), origin, 3);

	outInfo->Set(
		vtkStreamingDemandDrivenPipeline::UNRESTRICTED_UPDATE_EXTENT(), 1);

	return 1;
}

