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
* \file vtkBooleanOperation2DPolygons.cpp
* \author Rémi Blanc 
* \date 1. March 2012
*/

#include "vtkBooleanOperation2DPolygons.h"

using namespace psciob;

double psciob::BooleanOperation2DPolygons(vtkPolyData *poly1, vtkPolyData *poly2, vtkPolyData *outputPoly, PolygonBooleanOperationType code, double coeff) {
	ClipperLib::Polygons subj(1), clip(1), solution;

	double pt[3];
	//
	for (unsigned i=0 ; i<poly1->GetNumberOfPoints() ; i++) {
		poly1->GetPoint(i, pt);
		subj[0].push_back(ClipperLib::IntPoint(coeff*pt[0],coeff*pt[1]));
	}
	//
	for (unsigned i=0 ; i<poly2->GetNumberOfPoints() ; i++) {
		poly2->GetPoint(i, pt);
		clip[0].push_back(ClipperLib::IntPoint(coeff*pt[0],coeff*pt[1]));
	}

	ClipperLib::Clipper c;
	c.AddPolygons(subj, ClipperLib::ptSubject);
	c.AddPolygons(clip, ClipperLib::ptClip);

	switch(code) {
		case SETUNION:
			if (!c.Execute(ClipperLib::ctUnion, solution)) std::cout<<"failed computing the union"<<std::endl;
			break;
		case SETINTERSECTION: 
			if (!c.Execute(ClipperLib::ctIntersection, solution)) std::cout<<"failed computing the intersection"<<std::endl;
			break;
		case SETDIFFERENCE:
			if (!c.Execute(ClipperLib::ctDifference, solution)) std::cout<<"failed computing the difference"<<std::endl;
			break;
		case SETXOR:
			if (!c.Execute(ClipperLib::ctXor, solution)) std::cout<<"failed computing the XOR"<<std::endl;
			break;
	}

	//create a vtkPolyData out of the 'solution'
	double invcoeff = 1.0/coeff, invcoeff2 = invcoeff*invcoeff;
	vtkIdType segm[2];
	outputPoly->Initialize();
	
	unsigned totalNb = 0;
	for (unsigned i=0 ; i<solution.size() ; i++) { totalNb += solution[i].size(); }

	vtkPoints *points = vtkPoints::New();      points->Allocate(totalNb);
	vtkCellArray *lines = vtkCellArray::New(); lines->Allocate( lines->EstimateSize(totalNb,2) );

	totalNb = 0;
	double totalArea = 0;
	for (unsigned i=0 ; i<solution.size() ; i++) {
		points->InsertNextPoint( invcoeff*static_cast<double>(solution[i][0].X), invcoeff*static_cast<double>(solution[i][0].Y), 0 );
		for (unsigned j=1 ; j<solution[i].size() ; j++) {
			points->InsertNextPoint( invcoeff*static_cast<double>(solution[i][j].X), invcoeff*static_cast<double>(solution[i][j].Y), 0 );
			segm[0] = totalNb + j-1; segm[1] = totalNb + j; lines->InsertNextCell(2, segm);
		}
		segm[0] = totalNb + solution[i].size()-1; segm[1] = totalNb + 0; lines->InsertNextCell(2, segm);
		totalNb += solution[i].size();
		totalArea += ClipperLib::Area(solution[i])*invcoeff2;
	}

	outputPoly->SetPoints(points);
	lines->Squeeze(); outputPoly->SetLines(lines);

	points->Delete();
	lines->Delete();
	return totalArea;
}


/** same as BooleanOperation2DPolygons, but fills in a double[3] array with the area of poly1, poly2 and outputPoly
*/
void psciob::BooleanOperation2DPolygons_WithAreaComputation(vtkPolyData *poly1, vtkPolyData *poly2, vtkPolyData *outputPoly, double *areas, PolygonBooleanOperationType code, double coeff) {
	ClipperLib::Polygons subj(1), clip(1), solution;
	double invcoeff = 1.0/coeff, invcoeff2 = invcoeff*invcoeff;
	double pt[3];
	//
	for (unsigned i=0 ; i<poly1->GetNumberOfPoints() ; i++) {
		poly1->GetPoint(i, pt);
		subj[0].push_back(ClipperLib::IntPoint(coeff*pt[0],coeff*pt[1]));
	}
	areas[0] = ClipperLib::Area(subj[0])*invcoeff2;
	
	//
	for (unsigned i=0 ; i<poly2->GetNumberOfPoints() ; i++) {
		poly2->GetPoint(i, pt);
		clip[0].push_back(ClipperLib::IntPoint(coeff*pt[0],coeff*pt[1]));
	}
	areas[1] = ClipperLib::Area(clip[0])*invcoeff2;

	ClipperLib::Clipper c;
	c.AddPolygons(subj, ClipperLib::ptSubject);
	c.AddPolygons(clip, ClipperLib::ptClip);

	switch(code) {
		case SETUNION:
			if (!c.Execute(ClipperLib::ctUnion, solution)) std::cout<<"failed computing the union"<<std::endl;
			break;
		case SETINTERSECTION: 
			if (!c.Execute(ClipperLib::ctIntersection, solution)) std::cout<<"failed computing the intersection"<<std::endl;
			break;
		case SETDIFFERENCE:
			if (!c.Execute(ClipperLib::ctDifference, solution)) std::cout<<"failed computing the difference"<<std::endl;
			break;
		case SETXOR:
			if (!c.Execute(ClipperLib::ctXor, solution)) std::cout<<"failed computing the XOR"<<std::endl;
			break;
	}

	//create a vtkPolyData out of the 'solution'
	vtkIdType segm[2];
	outputPoly->Initialize();

	unsigned totalNb = 0;
	for (unsigned i=0 ; i<solution.size() ; i++) { totalNb += solution[i].size(); }

	vtkPoints *points = vtkPoints::New();      points->Allocate(totalNb);
	vtkCellArray *lines = vtkCellArray::New(); lines->Allocate( lines->EstimateSize(totalNb,2) );

	totalNb = 0;
	areas[2] = 0;
	for (unsigned i=0 ; i<solution.size() ; i++) {
		points->InsertNextPoint( invcoeff*static_cast<double>(solution[i][0].X), invcoeff*static_cast<double>(solution[i][0].Y), 0 );
		for (unsigned j=1 ; j<solution[i].size() ; j++) {
			points->InsertNextPoint( invcoeff*static_cast<double>(solution[i][j].X), invcoeff*static_cast<double>(solution[i][j].Y), 0 );
			segm[0] = totalNb + j-1; segm[1] = totalNb + j; lines->InsertNextCell(2, segm);
		}
		segm[0] = totalNb + solution[i].size()-1; segm[1] = totalNb + 0; lines->InsertNextCell(2, segm);
		totalNb += solution[i].size();
		areas[2] += ClipperLib::Area(solution[i])*invcoeff2;
	}

	outputPoly->SetPoints(points);
	lines->Squeeze(); outputPoly->SetLines(lines);
	points->Delete();
	lines->Delete();

}




/** Converts a ClipperPolygon into a vtkPolyData */
void psciob::ConvertClipperPolygonToVTKPolyData(ClipperLib::Polygons cpoly, vtkPolyData *outputPoly) {
	vtkIdType segm[2];
	outputPoly->Initialize();

	unsigned totalNb = 0;
	for (unsigned i=0 ; i<cpoly.size() ; i++) { totalNb += cpoly[i].size(); }

	vtkPoints *points = vtkPoints::New();      points->Allocate(totalNb);
	vtkCellArray *lines = vtkCellArray::New(); lines->Allocate( lines->EstimateSize(totalNb,2) );

	totalNb = 0;
	for (unsigned i=0 ; i<cpoly.size() ; i++) {
		points->InsertNextPoint( cpoly[i][0].X, cpoly[i][0].Y, 0 );
		for (unsigned j=1 ; j<cpoly[i].size() ; j++) {
			points->InsertNextPoint( cpoly[i][j].X, cpoly[i][j].Y, 0 );
			segm[0] = totalNb + j-1; segm[1] = totalNb + j; lines->InsertNextCell(2, segm);
		}
		segm[0] = totalNb + cpoly[i].size()-1; segm[1] = totalNb + 0; lines->InsertNextCell(2, segm);
		totalNb += cpoly[i].size();
	}

	outputPoly->SetPoints(points);
	lines->Squeeze(); outputPoly->SetLines(lines);
	points->Delete();
	lines->Delete();
}
