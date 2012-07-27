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
 * \file VTKUtils.h
 * \author Rémi Blanc 
 * \date 29. August 2011
 * \brief useful vtk-related functions
*/

#ifndef VTKUTILS_H_
#define VTKUTILS_H_

#include "GeneralUtils.h"
#include <vtkPolyDataWriter.h>
#include <vtkSTLWriter.h>

namespace psciob {



/** Write polydata as a vtk file, mirroring the first 2 coordinates, so that the data superposes with images written with ITK, when opened with Slicer */
void WriteMirrorPolyDataToFile(std::string filename, vtkSmartPointer<vtkPolyData> polydata);


/** Write polydata as an STL file, mirroring the first 2 coordinates, so that the data superposes with images written with ITK, when opened with Slicer */
void WriteMirrorPolyDataToSTLFile(std::string filename, vtkSmartPointer<vtkPolyData> polydata);


} // namespace psciob


#endif //VTKUTILS_H_