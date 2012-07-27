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

//seems to enable IntelliSence for VisualStudio...
#pragma once

#include <iostream>
#include <typeinfo>

#include "clipper.hpp"
#include "vtkBooleanOperation2DPolygons.h"

using namespace psciob;


void TestClipper() {
	std::cout<<"\n\n TestClipper(), taken from http://www.angusj.com/delphi/clipper.php#features"<<std::endl;
	ClipperLib::Polygons subj(2), clip(1), solution;

	subj[0].push_back(ClipperLib::IntPoint(180,200));
	subj[0].push_back(ClipperLib::IntPoint(260,200));
	subj[0].push_back(ClipperLib::IntPoint(260,150));
	subj[0].push_back(ClipperLib::IntPoint(180,150));

	subj[1].push_back(ClipperLib::IntPoint(215,160));
	subj[1].push_back(ClipperLib::IntPoint(230,190));
	subj[1].push_back(ClipperLib::IntPoint(200,190));

	clip[0].push_back(ClipperLib::IntPoint(190,210));
	clip[0].push_back(ClipperLib::IntPoint(240,210));
	clip[0].push_back(ClipperLib::IntPoint(240,130));
	clip[0].push_back(ClipperLib::IntPoint(190,130));


	ClipperLib::Clipper c;
	c.AddPolygons(subj, ClipperLib::ptSubject);
	c.AddPolygons(clip, ClipperLib::ptClip);
	if (!c.Execute(ClipperLib::ctIntersection, solution)) std::cout<<"failed computing the intersection"<<std::endl;

	std::cout<<"nb of polygons in solution: "<<solution.size()<<std::endl;
	for (unsigned i=0 ; i<solution.size() ; i++) {
		std::cout<<"nb of points in polygon "<<i+1<<" = "<<solution[i].size()<<std::endl;
		std::cout<<"coordinates of the first point: "<<solution[i][0].X<<", "<<solution[i][0].Y<<std::endl;
	}

	vtkSmartPointer<vtkPolyData> obj1 = vtkSmartPointer<vtkPolyData>::New();
	vtkSmartPointer<vtkPolyData> obj2 = vtkSmartPointer<vtkPolyData>::New();
	vtkSmartPointer<vtkPolyData> obj3 = vtkSmartPointer<vtkPolyData>::New();
	ConvertClipperPolygonToVTKPolyData(subj, obj1);
	ConvertClipperPolygonToVTKPolyData(clip, obj2);
	ConvertClipperPolygonToVTKPolyData(solution, obj3);

	WriteMirrorPolyDataToFile("ClipTestConvert_obj1.vtk", obj1);
	WriteMirrorPolyDataToFile("ClipTestConvert_obj2.vtk", obj2);
	WriteMirrorPolyDataToFile("ClipTestConvert_obj_inter.vtk", obj3);

}

void TestWrappedClipper() {
	std::cout<<"\n\n TestWrappedClipper(), test intersection and union of 2 self-constructed polygons"<<std::endl;

	vtkSmartPointer<vtkPolyData> obj1 = vtkSmartPointer<vtkPolyData>::New();
	vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();	points->Allocate(4);
	vtkSmartPointer<vtkCellArray> cells = vtkSmartPointer<vtkCellArray>::New();	cells->Allocate( cells->EstimateSize(4,2) ); 

	//points must be ordered
	points->InsertNextPoint( -2 , -2, 0 ); 	points->InsertNextPoint( 3 , -1, 0 );
	points->InsertNextPoint( 2.5 , 4, 0 );	points->InsertNextPoint( -4 , 2, 0 );

	vtkIdType lin[2];
	lin[0] = 0; lin[1] = 1; cells->InsertNextCell(2, lin);
	lin[0] = 1; lin[1] = 2; cells->InsertNextCell(2, lin);
	lin[0] = 2; lin[1] = 3; cells->InsertNextCell(2, lin);
	lin[0] = 3; lin[1] = 0; cells->InsertNextCell(2, lin);

	obj1->SetPoints(points);
	cells->Squeeze(); obj1->SetLines(cells);

	//
	vtkSmartPointer<vtkPolyData> obj2 = vtkSmartPointer<vtkPolyData>::New();
	points = vtkSmartPointer<vtkPoints>::New();	points->Allocate(20);
	cells = vtkSmartPointer<vtkCellArray>::New();	cells->Allocate( cells->EstimateSize(20,2) ); 

	double theta;
	//1st point
	points->InsertNextPoint( 0.5 , 0 , 0 ); //
	//all points and all segments but the last
	for (unsigned i=1 ; i<20 ; i++) {
		theta = 2.0*i*PI/(20.0);
		points->InsertNextPoint( 0.5*cos(theta), 0.5*sin(theta), 0 ); 
		lin[0] = i-1; 	lin[1] = i; cells->InsertNextCell(2, lin);
	}
	//last segment
	lin[0] = 19; lin[1] = 0; cells->InsertNextCell(2, lin);

	obj2->SetPoints(points);
	cells->Squeeze(); obj2->SetLines(cells);


	vtkSmartPointer<vtkPolyData> intersect = vtkSmartPointer<vtkPolyData>::New();
	std::cout<<"area of intersection: "<<BooleanOperation2DPolygons(obj1, obj2, intersect)<<std::endl;

	WriteMirrorPolyDataToFile("ClipTest_obj1.vtk", obj1);
	WriteMirrorPolyDataToFile("ClipTest_obj2.vtk", obj2);
	WriteMirrorPolyDataToFile("ClipTest_obj_inter.vtk", intersect);

	std::cout<<"area of union: "<<BooleanOperation2DPolygons(obj1, obj2, intersect, SETUNION)<<std::endl;
	WriteMirrorPolyDataToFile("ClipTest_obj_union.vtk", intersect);

}

int main(int argc, char** argv) {

	TestClipper();

	TestWrappedClipper();

	return 1;
}
