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
 * \file VTKUtils.cpp
 * \author Rémi Blanc 
 * \date 27. February 2012
*/

#include "VTKUtils.h"

using namespace psciob;

void psciob::WriteMirrorPolyDataToFile(std::string filename, vtkSmartPointer<vtkPolyData> polydata) {
	//mirror the x and y coordinates before writing file.
	vtkSmartPointer<vtkPolyData> mirror = vtkSmartPointer<vtkPolyData>::New();
	vtkSmartPointer<vtkPoints> mirror_points = vtkSmartPointer<vtkPoints>::New();
	mirror_points->SetNumberOfPoints(polydata->GetNumberOfPoints());
	mirror->DeepCopy(polydata);
	double p[3];
	for (unsigned i=0 ; i<polydata->GetNumberOfPoints() ; i++)
	{
		polydata->GetPoint(i, p);
		mirror_points->SetPoint(i, -p[0], -p[1], p[2]);
	}
	mirror->SetPoints(mirror_points);

	vtkSmartPointer<vtkPolyDataWriter> writer = vtkSmartPointer<vtkPolyDataWriter>::New();
	writer->SetFileName(filename.c_str());
	writer->SetInput(mirror);
	writer->Update();
}


void psciob::WriteMirrorPolyDataToSTLFile(std::string filename, vtkSmartPointer<vtkPolyData> polydata) {
	//mirror the x and y coordinates before writing file.
	vtkSmartPointer<vtkPolyData> mirror = vtkSmartPointer<vtkPolyData>::New();
	vtkSmartPointer<vtkPoints> mirror_points = vtkSmartPointer<vtkPoints>::New();
	mirror_points->SetNumberOfPoints(polydata->GetNumberOfPoints());
	mirror->DeepCopy(polydata);
	double p[3];
	for (unsigned i=0 ; i<polydata->GetNumberOfPoints() ; i++)
	{
		polydata->GetPoint(i, p);
		mirror_points->SetPoint(i, -p[0], -p[1], p[2]);
	}
	mirror->SetPoints(mirror_points);

	vtkSmartPointer<vtkSTLWriter> writer = vtkSmartPointer<vtkSTLWriter>::New();
	writer->SetFileName(filename.c_str());
	writer->SetInput(mirror);
	writer->Update();
}
