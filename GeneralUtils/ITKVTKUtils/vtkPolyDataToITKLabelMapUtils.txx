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
* \file vtkPolyDataToITKLabelMapUtils.txx
* \author Rémi Blanc 
* \date 1. March 2012
*/

#include "ITKUtils.h"
#include "vtkPolyDataToITKLabelMapUtils.h"


namespace psciob {


/** Convert a vtkPolyData to a binary itkImage (unsigned char)
* The image is defined with respect to the minimal bounding box containing the object (with the convention that coordinate 0 is center of a pixel)
*/
template <unsigned VDimension>
void VTKPolyDataToITKImage( vtkPolyData *polydata, itk::Image<unsigned char, VDimension> *image, typename itk::Image<unsigned char, VDimension>::SpacingType spacing ) {
	typedef itk::Image<unsigned char, VDimension> ImageType;
	typedef itk::LabelObject<unsigned char, VDimension> LabelObjectType;
	typedef itk::LabelMap<LabelObjectType> LabelMapType;
	LabelMapType::Pointer labelMap = LabelMapType::New();

	VTKPolyDataToLabelMap<LabelMapType>(polydata, labelMap, spacing);
	
	typedef itk::LabelMapToLabelImageFilter< LabelMapType, ImageType> LabelMapToLabelImageFilterType;
	LabelMapToLabelImageFilterType::Pointer labelMapToLabelImageFilter = LabelMapToLabelImageFilterType::New();
	labelMapToLabelImageFilter->SetInput(labelMap);
	labelMapToLabelImageFilter->Update();

	image = labelMapToLabelImageFilter->GetOutput();
}


/** Convert a vtkPolyData to a itkLabelMap containing a single itkLabelObject with label value 1
* The vtkPolyData is assumed to be 2D (only the first 2 coordinates being used) and convex
* Furthermore, 2 points with consecutive indices are assumed to be connected (and first is connected to last...)
* The map is defined with respect to the minimal bounding box containing the object (with the convention that coordinate 0 is center of a pixel)
*/
template <class TLabelMap>
void VTK2DConvexPolyDataToLabelMap( vtkPolyData *polydata, TLabelMap *outputMap, typename TLabelMap::SpacingType spacing) {
	outputMap->ClearLabels();

	// Set spacing and physical extent
	outputMap->SetSpacing(spacing);
	TLabelMap::SpacingType invspacing;
	double bounds[6]; polydata->GetBounds(bounds);
	vnl_vector<double> bbox(4);
	invspacing[0] = 1.0/spacing[0]; invspacing[1] = 1.0/spacing[1]; 
	bbox(0) = std::min<double>(bounds[0], bounds[1]); bbox(1) = std::max<double>(bounds[0], bounds[1]);
	bbox(2) = std::min<double>(bounds[2], bounds[3]); bbox(3) = std::max<double>(bounds[2], bounds[3]);
	
	// Compute and set image meta-data
	TLabelMap::PointType origin; 
	TLabelMap::RegionType region;
	ITKGridImageInformationFromPhysicalBoundingBoxAndSpacing(bbox, spacing.GetVnlVector(), &origin, &region); 
	bbox = BoundingBoxFromITKImageInformation<TLabelMap>(origin, spacing, region);

	outputMap->SetOrigin(origin);
	outputMap->SetRegions(region);
	const TLabelMap::SizeType &size = region.GetSize();

	// Create the LabelObject that will host the object representation
	TLabelMap::LabelObjectType::Pointer labelObject = TLabelMap::LabelObjectType::New();
	labelObject->SetLabel(1);
	outputMap->AddLabelObject(labelObject);
	TLabelMap::IndexType index;
	TLabelMap::LengthType length;

	// Effective processing.
	long n = polydata->GetNumberOfPoints();
	vertice2D tmpv;
	std::vector<vertice2D> v;
	int *vl, *al;
	vl=(int*)malloc(n*sizeof(int)); //list of indices ordering the 'vertice2Ds'
	al=(int*)malloc(n*sizeof(int)); //list of active vertice2Ds
	int i, k=0, vertproc=0, actnum=0;

	double pt1[3], pt2[3]; //physical coordinates
	double ci1[2], ci2[2]; //continuous index (accounts for origin and spacing of the image)
	//computation of v
	//1st point
	i=0;k=0; //convert coordinates to continuous indices relative to the requested grid
	polydata->GetPoint( i , pt1); ci1[0] = (pt1[0]-origin[0])*invspacing[0]; ci1[1] = (pt1[1]-origin[1])*invspacing[1];
	polydata->GetPoint(i+1, pt2); ci2[0] = (pt2[0]-origin[0])*invspacing[0]; ci2[1] = (pt2[1]-origin[1])*invspacing[1];
	//convert the point coordinates to index coordinates				
	if      ( ci1[1] < ci2[1] ) { tmpv.y0= ci1[1]; tmpv.x0= ci1[0]; tmpv.y1= ci2[1]; tmpv.m = (ci1[0]-ci2[0])/(ci1[1]-ci2[1]); v.push_back(tmpv);vl[k]=k;k++; }
	else if ( ci2[1] < ci1[1] ) { tmpv.y0= ci2[1]; tmpv.x0= ci2[0]; tmpv.y1= ci1[1]; tmpv.m = (ci2[0]-ci1[0])/(ci2[1]-ci1[1]); v.push_back(tmpv);vl[k]=k;k++; }			
	//next points
	for(i=1;i<n-1;i++){ //re-use the previously computed points
		ci1[0] = ci2[0]; ci1[1] = ci2[1];  polydata->GetPoint(i+1, pt2); ci2[0] = (pt2[0]-origin[0])*invspacing[0]; ci2[1] = (pt2[1]-origin[1])*invspacing[1];
		if     (ci1[1] < ci2[1]){ tmpv.y0= ci1[1]; tmpv.x0= ci1[0]; tmpv.y1= ci2[1]; tmpv.m = (ci1[0]-ci2[0])/(ci1[1]-ci2[1]); v.push_back(tmpv);vl[k]=k;k++; }
		else if(ci2[1] < ci1[1]){ tmpv.y0= ci2[1]; tmpv.x0= ci2[0]; tmpv.y1= ci1[1]; tmpv.m = (ci2[0]-ci1[0])/(ci2[1]-ci1[1]); v.push_back(tmpv);vl[k]=k;k++; }
	}
	//last point
	ci1[0] = ci2[0]; ci1[1] = ci2[1]; //re-use the previously computed point
	polydata->GetPoint(0, pt2); ci2[0] = (pt2[0]-origin[0])*invspacing[0]; ci2[1] = (pt2[1]-origin[1])*invspacing[1];
	//convert the point coordinates to index coordinates				
	if      ( ci1[1] < ci2[1] ) { tmpv.y0= ci1[1]; tmpv.x0= ci1[0]; tmpv.y1= ci2[1]; tmpv.m = (ci1[0]-ci2[0])/(ci1[1]-ci2[1]); v.push_back(tmpv);vl[k]=k;k++; }
	else if ( ci2[1] < ci1[1] ) { tmpv.y0= ci2[1]; tmpv.x0= ci2[0]; tmpv.y1= ci1[1]; tmpv.m = (ci2[0]-ci1[0])/(ci2[1]-ci1[1]); v.push_back(tmpv);vl[k]=k;k++; }

	////
	n=k;
	std::sort(vl, vl+k, vertice2D_less(v));

	//select the line which is sufficently covered by the first pixel
	index[1] = round(v[vl[0]].y0);

	//select the active vertice2Ds for this line
	vertproc = 0; actnum=0; 
	while ( (vertproc<n) && (v[vl[vertproc]].y0 <= 0.5 + index[1]) ) {
		al[actnum]=vl[vertproc]; //al: liste des vertice2Ds actives pour cette ligne
		actnum++;vertproc++;	 //actnum: nombre de vertice2Ds actives
	}

	double minx, maxx;
	do{
		//compute length
		if(index[1]>=0) {
			minx = size[0]; maxx=-1;
			for(i=0 ; i<actnum ; i++) { 
				minx = std::min(minx, v[al[i]].x0);
				maxx = std::max(maxx, v[al[i]].x0);
			}
			if ( (maxx-minx)<0.5 ) {} //skip tiny lines.. ?? 
			else {
				index[0] = std::max<int>(0, round(minx));
				length = std::min<int>(size[0]-1, round(maxx)) - index[0] + 1;
				labelObject->AddLine(index, length);
			}
		}
		//increment line
		index[1]++;
		//remove finished segments -- 
		for(i=0;i<actnum;i++) if( v[al[i]].y1 < 0.5+index[1]-TINY ) {al[i]=al[actnum-1];actnum--;i--;}
		//update X //implicitly, there is also y0+=1 (moving along the segment
		for(i=0;i<actnum;i++) { v[al[i]].x0+=v[al[i]].m; }
		//add segments
		while( (vertproc<n) && ( v[vl[vertproc]].y0 <= 0.5+index[1] ) )
		{al[actnum]=vl[vertproc];actnum++;vertproc++;}
	} while( (index[1]<size[1]) && (actnum>0) );
	free(vl);free(al);
}


} // namespace psciob

