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
Module:    vtkITK3DUCharLabelMapData.h

Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
All rights reserved.
See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

This software is distributed WITHOUT ANY WARRANTY; without even
the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
// .NAME vtkITK3DUCharLabelMapData - efficient description of an image stencil
// .SECTION Description
// vtkITK3DUCharLabelMapData describes an image stencil in a manner which is
// efficient both in terms of speed and storage space.  The stencil extents
// are stored for each x-row across the image (multiple extents per row if
// necessary) and can be retrieved via the GetNextExtent() method.
// .SECTION see also
// vtkITK3DUCharLabelMapSource vtkImageStencil

#ifndef __vtkITK3DUCharLabelMapData_h
#define __vtkITK3DUCharLabelMapData_h


#include "vtkDataObject.h"
#include "itkLabelMap.h"
#include "itkLabelObject.h"

namespace psciob {

//class VTK_IMAGING_EXPORT vtkITK3DUCharLabelMapData : public vtkDataObject
class vtkITK3DUCharLabelMapData : public vtkDataObject
{
public:
	static vtkITK3DUCharLabelMapData *New();
	vtkTypeMacro(vtkITK3DUCharLabelMapData, vtkDataObject);
	void PrintSelf(ostream& os, vtkIndent indent);

	void Initialize();
	void DeepCopy(vtkDataObject *o);
	void ShallowCopy(vtkDataObject *f);
	void InternalImageStencilDataCopy(vtkITK3DUCharLabelMapData *s);

	// Description:
	// Get the data type as an integer (this will return VTK_DATA_OBJECT
	// for now, maybe a proper type constant will be reserved later).
	int GetDataObjectType() { return VTK_DATA_OBJECT; }

	// Description:
	// The extent type is 3D, just like vtkImageData.
	int GetExtentType() { return VTK_3D_EXTENT; };

	// Description:
	// Given the total output x extent [xMin,xMax] and the current y, z indices,
	// return each sub-extent [r1,r2] that lies within within the unclipped
	// region in sequence.  A value of '0' is returned if no more sub-extents
	// are available.  The variable 'iter' must be initialized to zero before
	// the first call, unless you want the complementary sub-extents in which
	// case you must initialize 'iter' to -1.  The variable 'iter' is used
	// internally to keep track of which sub-extent should be returned next.
	int GetNextExtent(int &r1, int &r2, int xMin, int xMax,
		int yIdx, int zIdx, int &iter);

	// Description:
	// Checks if an index is inside the stencil.
	// This can be faster than GetNextExtent if called on a voxel. If calling 
	// sequentially, along a scan line, the preferred way is to use 
	// GetNextExtent and then loop over the returned [r1,r2] extents.
	int IsInside( int xIdx, int yIdx, int zIdx );

	// Description:
	// This method is used by vtkITK3DUCharLabelMapDataSource to add an x 
	// sub extent [r1,r2] for the x row (yIdx,zIdx).  The specified sub
	// extent must not intersect any other sub extents along the same x row.
	// As well, r1 and r2 must both be within the total x extent
	// [Extent[0],Extent[1]].
	void InsertNextExtent(int r1, int r2, int yIdx, int zIdx);

	// Description:
	// Similar to InsertNextExtent, except that the extent (r1,r2) at yIdx, 
	// zIdx is merged with other extents, (if any) on that row. So a 
	// unique extent may not necessarily be added. For instance, if an extent 
	// [5,11] already exists adding an extent, [7,9] will not affect the 
	// stencil. Likewise adding [10, 13] will replace the existing extent 
	// with [5,13].
	void InsertAndMergeExtent(int r1, int r2, int yIdx, int zIdx);

	// Description:
	// Remove the extent from (r1,r2) at yIdx, zIdx
	void RemoveExtent(int r1, int r2, int yIdx, int zIdx);

	// Description:
	// Set the desired spacing for the stencil.
	// This must be called before the stencil is Updated, ideally 
	// in the ExecuteInformation method of the imaging filter that
	// is using the stencil.
	vtkSetVector3Macro(Spacing, double);
	vtkGetVector3Macro(Spacing, double);

	// Description:
	// Set the desired origin for the stencil.
	// This must be called before the stencil is Updated, ideally 
	// in the ExecuteInformation method of the imaging filter that
	// is using the stencil.
	vtkSetVector3Macro(Origin, double);
	vtkGetVector3Macro(Origin, double);

	// Description:
	// Set the extent of the data.  This is should be called only 
	// by vtkITK3DUCharLabelMapSource, as it is part of the basic pipeline
	// functionality.
	void SetExtent(int extent[6]);
	void SetExtent(int x1, int x2, int y1, int y2, int z1, int z2);
	vtkGetVector6Macro(Extent, int);

	// Description:
	// Allocate space for the sub-extents.  This is called by
	// vtkITK3DUCharLabelMapSource.
	void AllocateExtents();

	// Description:
	// Fill the sub-extents.
	void Fill();

	// Description:
	// Override these to handle origin, spacing, scalar type, and scalar
	// number of components.  See vtkDataObject for details.
	  virtual void CopyInformationToPipeline(vtkInformation* request,
                                         vtkInformation* input,
                                         vtkInformation* output,
                                         int forceCopy);
  virtual void CopyInformationFromPipeline(vtkInformation* request);

	//BTX
	// Description:
	// Retrieve an instance of this class from an information object.
	static vtkITK3DUCharLabelMapData* GetData(vtkInformation* info);
	static vtkITK3DUCharLabelMapData* GetData(vtkInformationVector* v, int i=0);
	//ETX

	// Description:
	// Add merges the stencil supplied as argument into Self.
	virtual void Add     ( vtkITK3DUCharLabelMapData * ); 

	// Description:
	// Subtract removes the portion of the stencil, supplied as argument, 
	// that lies within Self from Self.   
	virtual void Subtract( vtkITK3DUCharLabelMapData * ); 

	// Description:
	// Replaces the portion of the stencil, supplied as argument, 
	// that lies within Self from Self.   
	virtual void Replace( vtkITK3DUCharLabelMapData * ); 

	// Description:
	// Clip the stencil with the supplied extents. In other words, discard data
	// outside the specified extents. Return 1 if something changed.
	virtual int Clip( int extent[6] );

	//
	//
	//
	typedef itk::LabelObject<unsigned char, 3>	LabelObjectType;
	typedef itk::LabelMap<LabelObjectType>		LabelMapType;
	typedef LabelMapType::IndexType				IndexType;
	typedef LabelMapType::LengthType			LengthType;
	typedef LabelMapType::SizeType				SizeType;

	/** Set the input label map */
	void SetLabelMap(LabelMapType::Pointer labelMap) { 
		m_labelMap = labelMap; 
		m_size = m_labelMap->GetLargestPossibleRegion().GetSize();
		m_labelObject = m_labelMap->GetNthLabelObject(0);
	}
	/** Get the output (modified) label map */ 
	LabelMapType* GetLabelMap() { return m_labelMap; }

	//
	//
	//

protected:
	vtkITK3DUCharLabelMapData();
	~vtkITK3DUCharLabelMapData();

	//
	//
	//
	LabelMapType::Pointer m_labelMap;
	LabelObjectType::Pointer m_labelObject;
	IndexType m_index;
	LengthType m_length;
	SizeType m_size;
	//
	//
	//

	// Description:
	// Get important info from pipeline.
	void CopyOriginAndSpacingFromPipeline();

	// Description:
	// Merges portions of the stencil that are within Self's extents into 
	// itself. 
	virtual void InternalAdd( vtkITK3DUCharLabelMapData * );

	void CollapseAdditionalIntersections(int r2, int idx, int *clist, 
		int &clistlen);

	// Description:
	// The Spacing and Origin of the data.
	double Spacing[3];
	double Origin[3];

	int Extent[6];

	// Description:
	// The actual 'data' is stored here.
	int NumberOfExtentEntries;
	int *ExtentListLengths;
	int **ExtentLists;

private:
	vtkITK3DUCharLabelMapData(const vtkITK3DUCharLabelMapData&);  // Not implemented.
	void operator=(const vtkITK3DUCharLabelMapData&);  // Not implemented.

	friend class vtkImageStencilIteratorFriendship;
};

//BTX
// Description:
// This is a helper class for stencil creation.  It is a raster with
// infinite resolution in the X direction (approximately, since it uses
// double precision).  Lines that represent polygon edges can be drawn
// into this raster, and then filled given a tolerance.
//class VTK_IMAGING_EXPORT vtkITK3DUCharLabelMapRaster
class vtkITK3DUCharLabelMapRaster
{
public:
	// Description:
	// Create a raster with the specified whole y extent.
	vtkITK3DUCharLabelMapRaster(const int wholeExtent[2]);

	// Description:
	// Destructor.
	~vtkITK3DUCharLabelMapRaster();

	// Description:
	// Reset the raster to its original state, but keep the same whole
	// extent. Pre-allocate the specified 1D allocateExtent, which must be
	// within the whole extent.
	void PrepareForNewData(const int allocateExtent[2] = 0);

	// Description:
	// Insert a line into the raster, given the two end points.
	// The "inflect1" and "inflect2" should be set if you want
	// to add a small vertical tolerance to either endpoints.
	void InsertLine(const double p1[2], const double p2[2],
		bool inflect1, bool inflect2);

	// Description:
	// Fill the specified extent of a vtkITK3DUCharLabelMapData with the raster,
	// after permuting the raster according to xj and yj.
	void FillStencilData(vtkITK3DUCharLabelMapData *data, const int extent[6],
		int xj = 0, int yj = 1);

	// Description:
	// The tolerance for float-to-int conversions.
	void SetTolerance(double tol) { this->Tolerance = tol; }
	double GetTolerance() { return this->Tolerance; }

protected:
	// Description:
	// Ensure that the raster is initialized for the specified range
	// of y values, which must be within the Extent.
	void PrepareExtent(int ymin, int ymax);

	// Description:
	// Insert an x point into the raster.  If the y value is larger
	// than the y extent, the extent will grow automatically.
	void InsertPoint(int y, double x);

	int Extent[2];
	int UsedExtent[2];
	double **Raster;
	double Tolerance;

private:
	vtkITK3DUCharLabelMapRaster(const vtkITK3DUCharLabelMapRaster&);  // Not implemented.
	void operator=(const vtkITK3DUCharLabelMapRaster&);  // Not implemented.
};
//ETX


} // namespace psciob

#endif



