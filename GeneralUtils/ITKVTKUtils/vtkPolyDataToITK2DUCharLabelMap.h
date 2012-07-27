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
Module:    vtkPolyDataToITK2DUCharLabelMap.h

Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
All rights reserved.
See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

This software is distributed WITHOUT ANY WARRANTY; without even
the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
/*=========================================================================

Copyright (c) 2008 Atamai, Inc.

Use, modification and redistribution of the software, in source or
binary forms, are permitted provided that the following terms and
conditions are met:

1) Redistribution of the source code, in verbatim or modified
form, must retain the above copyright notice, this license,
the following disclaimer, and any notices that refer to this
license and/or the following disclaimer.  

2) Redistribution in binary form must include the above copyright
notice, a copy of this license and the following disclaimer
in the documentation or with other materials provided with the
distribution.

3) Modified copies of the source code must be clearly marked as such,
and must not be misrepresented as verbatim copies of the source code.

THE COPYRIGHT HOLDERS AND/OR OTHER PARTIES PROVIDE THE SOFTWARE "AS IS"
WITHOUT EXPRESSED OR IMPLIED WARRANTY INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
PURPOSE.  IN NO EVENT SHALL ANY COPYRIGHT HOLDER OR OTHER PARTY WHO MAY
MODIFY AND/OR REDISTRIBUTE THE SOFTWARE UNDER THE TERMS OF THIS LICENSE
BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, LOSS OF DATA OR DATA BECOMING INACCURATE
OR LOSS OF PROFIT OR BUSINESS INTERRUPTION) ARISING IN ANY WAY OUT OF
THE USE OR INABILITY TO USE THE SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGES.

=========================================================================*/
// .NAME vtkPolyDataToITK2DUCharLabelMap - use polydata to mask an image
// .SECTION Description
// The vtkPolyDataToITK2DUCharLabelMap class will convert polydata into
// an image stencil.  The polydata can either be a closed surface
// mesh or a series of polyline contours (one contour per slice).
// .SECTION Caveats
// If contours are provided, the contours must be aligned with the
// Z planes.  Other contour orientations are not supported.
// .SECTION See Also
// vtkImageStencil vtkImageAccumulate vtkImageBlend vtkImageReslice

#ifndef __vtkPolyDataToITK2DUCharLabelMap_h
#define __vtkPolyDataToITK2DUCharLabelMap_h

#include "vtkITK2DUCharLabelMapSource.h"

#include "GeneralUtils.h"
#include "itkLabelMap.h"
#include "itkLabelObject.h"

class vtkMergePoints;
class vtkDataSet;
class vtkPolyData;

namespace psciob {


//class VTK_HYBRID_EXPORT vtkPolyDataToITK2DUCharLabelMap : public vtkITK2DUCharLabelMapSource
class vtkPolyDataToITK2DUCharLabelMap : public vtkITK2DUCharLabelMapSource
{
public:
	static vtkPolyDataToITK2DUCharLabelMap* New();
	vtkTypeMacro(vtkPolyDataToITK2DUCharLabelMap, vtkITK2DUCharLabelMapSource);
	void PrintSelf(ostream& os, vtkIndent indent);

	// Description:
	// Specify the implicit function to convert into a stencil.
	virtual void SetInput(vtkPolyData*);
	vtkPolyData *GetInput();

	// Description:
	// The tolerance to apply in when determining whether a voxel
	// is inside the stencil, given as a fraction of a voxel.
	// Only used in X and Y, not in Z.
	vtkSetClampMacro(Tolerance, double, 0.0, 1.0);
	vtkGetMacro(Tolerance, double);

	//
	//
	//
	typedef itk::LabelObject<unsigned char, 2>	LabelObjectType;
	typedef itk::LabelMap<LabelObjectType>		LabelMapType;
	
	/** Set the input label map */
	void SetLabelMap(LabelMapType::Pointer labelMap) { 
		m_labelMap = labelMap; m_labelMapFlag=true;	  
	}
	/** Get the output (modified) label map */ 
	LabelMapType* GetLabelMap() { return m_labelMap; }

	//
	//
	//

protected:
	vtkPolyDataToITK2DUCharLabelMap();
	~vtkPolyDataToITK2DUCharLabelMap();

	//
	//
	//
	LabelMapType::Pointer m_labelMap;
	bool m_labelMapFlag;
	//
	//
	//


	void ThreadedExecute(vtkITK2DUCharLabelMapData *output,
		int extent[6], int threadId);

	static void PolyDataCutter(vtkPolyData *input, vtkPolyData *output,
		double z, vtkMergePoints *locator);

	static void PolyDataSelector(vtkPolyData *input, vtkPolyData *output,
		double z, double thickness,
		vtkMergePoints *locator);

	virtual int RequestData(vtkInformation *, vtkInformationVector **,
		vtkInformationVector *);

	virtual int FillInputPortInformation(int, vtkInformation*);

	// Description:
	// The tolerance distance for favoring the inside of the stencil
	double Tolerance;

private:
	vtkPolyDataToITK2DUCharLabelMap(const vtkPolyDataToITK2DUCharLabelMap&);  // Not implemented.
	void operator=(const vtkPolyDataToITK2DUCharLabelMap&);  // Not implemented.
};


} // namespace psciob

#endif
