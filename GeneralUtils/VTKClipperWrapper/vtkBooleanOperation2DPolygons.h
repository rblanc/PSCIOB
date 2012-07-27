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
* \file vtkBooleanOperation2DPolygons.h
* \author Rémi Blanc 
* \date 13. March 2012
*/

#include "clipper.hpp"
#include "VTKUtils.h"


namespace psciob {


/** Types of boolean operation to perform between two 2D polygons*/
typedef enum {
	SETUNION, 
	SETINTERSECTION,
	SETDIFFERENCE,
	SETXOR
} PolygonBooleanOperationType;

/** Wraps the Clipper library for vtkPolyData objects
* the input objects are assumed to have a specific structure: the points should be stored in order so that they are connected with the previous and next points
* the output appends all polygons that correspond to the requested operation (check PolygonBooleanOperationType)
* Since Clipper internally uses integer-type coordinates, the real coordinates are internally multiplied by the provided coeff (default = 10000) before converting to integer
*
* returns the area of the outputPoly
*/
double BooleanOperation2DPolygons(vtkPolyData *poly1, vtkPolyData *poly2, vtkPolyData *outputPoly, PolygonBooleanOperationType code = SETINTERSECTION, double coeff = 10000);

/** same as BooleanOperation2DPolygons, but fills in a double[3] array with the area of poly1, poly2 and outputPoly
*/
void BooleanOperation2DPolygons_WithAreaComputation(vtkPolyData *poly1, vtkPolyData *poly2, vtkPolyData *outputPoly, double *areas, PolygonBooleanOperationType code = SETINTERSECTION, double coeff = 10000);


/** Converts 
*/
void ConvertClipperPolygonToVTKPolyData(ClipperLib::Polygons cpoly, vtkPolyData *outputPoly);

} // namespace psciob

