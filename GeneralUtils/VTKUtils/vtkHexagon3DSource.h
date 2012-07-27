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
  Module:    vtkHexagon3DSource.h

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
// .NAME vtkHexagon3DSource - create a polygonal representation of a Hexagon3D
// .SECTION Description
// vtkHexagon3DSource creates a Hexagon3D centered at origin. The Hexagon3D is represented
// with four-sided polygons. It is possible to specify the length, width, 
// and height of the Hexagon3D independently.

#ifndef __vtkHexagon3DSource_h
#define __vtkHexagon3DSource_h

#include "vtkPolyDataAlgorithm.h"

namespace psciob {


class vtkHexagon3DSource : public vtkPolyDataAlgorithm 
{
public:
  static vtkHexagon3DSource *New();
  vtkTypeMacro(vtkHexagon3DSource,vtkPolyDataAlgorithm);
  void PrintSelf(ostream& os, vtkIndent indent);

  // Description:
  // Set the length of the Hexagon3D in the x-direction.
  vtkSetClampMacro(XLength,double,0.0,VTK_DOUBLE_MAX);
  vtkGetMacro(XLength,double);

  // Description:
  // Set the length of the Hexagon3D in the y-direction.
  vtkSetClampMacro(YLength,double,0.0,VTK_DOUBLE_MAX);
  vtkGetMacro(YLength,double);

  // Description:
  // Set the length of the Hexagon3D in the z-direction.
  vtkSetClampMacro(ZLength,double,0.0,VTK_DOUBLE_MAX);
  vtkGetMacro(ZLength,double);

  // Description:
  // Set the center of the Hexagon3D.
  vtkSetVector3Macro(Center,double);
  vtkGetVectorMacro(Center,double,3);

  // Description:
  // Convenience method allows creation of Hexagon3D by specifying bounding box.
  void SetBounds(double xMin, double xMax,
                 double yMin, double yMax,
                 double zMin, double zMax);
  void SetBounds(double bounds[6]);

protected:
  vtkHexagon3DSource(double xL=1.0, double yL=1.0, double zL=1.0);
  ~vtkHexagon3DSource() {};

  int RequestData(vtkInformation *, vtkInformationVector **, vtkInformationVector *);
  double XLength;
  double YLength;
  double ZLength;
  double Center[3];
private:
  vtkHexagon3DSource(const vtkHexagon3DSource&);  // Not implemented.
  void operator=(const vtkHexagon3DSource&);  // Not implemented.
};


} // namespace psciob

#endif
