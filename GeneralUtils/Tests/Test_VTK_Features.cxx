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

#include "GeneralUtils.h"
#include "SetsUtils.h"
#include "ITKUtils.h"
#include "time.h"

#include <typeinfo>


#include "itkImage.h"
#include "itkImageFileWriter.h"
#include "itkImageRegionIterator.h"
#include "itkBinaryImageToLabelMapFilter.h"
#include "itkLabelMapToLabelImageFilter.h"
#include "itkLabelObject.h" 

#include "vtkPolyDataToITKLabelMapUtils.h"


#include "vtkPolyDataToImageStencil.h"
#include "vtkImageData.h"
#include "vtkImageStencil.h"
#include "vtkImageStencilData.h"
#include "vtkMetaImageWriter.h"
#include "vtkLinearExtrusionFilter.h"
#include "vtkPointData.h"

#include "vtkImageStencilToImage.h"
#include <vtkDiskSource.h>
#include "vtkSphereSource.h"
#include "vtkAppendPolyData.h"

//#include <vtkBooleanOperationPolyDataFilter.h>
#include <vtkIntersectionPolyDataFilter.h>
#include "vtkCubeSource.h"
#include "vtkSphereSource.h"
#include <vtkTriangleFilter.h>
#include "vtkFloatArray.h"
#include "vtkCellData.h"
#include "vtkMassProperties.h"

using namespace psciob;


void Test3DVTKPolyDataTolabelMap() {
	std::cout<<"Entering TestVTKPolyDataToImageStencil"<<std::endl;

	//mesh has 16 points
	vtkSmartPointer<vtkPolyData> hollowcube = vtkSmartPointer<vtkPolyData>::New();
	vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();	points->Allocate(16);
	// 8 rectangles
	vtkSmartPointer<vtkCellArray> cells = vtkSmartPointer<vtkCellArray>::New();	cells->Allocate( cells->EstimateSize(8,4) ); 

	double l1 = 12, w1 = 10, h1 = 5, l2 = 8, w2 = 5, h2 = 2; 
	//outer points
	points->InsertNextPoint(-l1, -h1, -w1);	points->InsertNextPoint(-l1, +h1, -w1);	points->InsertNextPoint(+l1, +h1, -w1);	points->InsertNextPoint(+l1, -h1, -w1);
	points->InsertNextPoint(+l1, -h1, +w1);	points->InsertNextPoint(-l1, -h1, +w1);	points->InsertNextPoint(-l1, +h1, +w1);	points->InsertNextPoint(+l1, +h1, +w1);
	//inner points
	points->InsertNextPoint(-l2, -h2, -w2);	points->InsertNextPoint(-l2, +h2, -w2);	points->InsertNextPoint(+l2, +h2, -w2);	points->InsertNextPoint(+l2, -h2, -w2);
	points->InsertNextPoint(+l2, -h2, +w2);	points->InsertNextPoint(-l2, -h2, +w2);	points->InsertNextPoint(-l2, +h2, +w2);	points->InsertNextPoint(+l2, +h2, +w2);

	vtkIdType quad[4];
	//outer faces
	quad[0]= 0; quad[1]= 1; quad[2]= 2; quad[3]= 3;	cells->InsertNextCell(4, quad);
	quad[0]= 0; quad[1]= 1; quad[2]= 6; quad[3]= 5;	cells->InsertNextCell(4, quad);
	quad[0]= 4; quad[1]= 7; quad[2]= 6; quad[3]= 5;	cells->InsertNextCell(4, quad);
	quad[0]= 4; quad[1]= 7; quad[2]= 2; quad[3]= 3;	cells->InsertNextCell(4, quad);
	quad[0]= 0; quad[1]= 3; quad[2]= 4; quad[3]= 5;	cells->InsertNextCell(4, quad);
	quad[0]= 1; quad[1]= 2; quad[2]= 7; quad[3]= 6;	cells->InsertNextCell(4, quad);
	//inner faces
	quad[0]= 8; quad[1]= 9; quad[2]=10; quad[3]=11;	cells->InsertNextCell(4, quad);
	quad[0]= 8; quad[1]= 9; quad[2]=14; quad[3]=13;	cells->InsertNextCell(4, quad);
	quad[0]=12; quad[1]=15; quad[2]=14; quad[3]=13;	cells->InsertNextCell(4, quad);
	quad[0]=12; quad[1]=15; quad[2]=10; quad[3]=11;	cells->InsertNextCell(4, quad);
	quad[0]= 8; quad[1]=11; quad[2]=12; quad[3]=13;	cells->InsertNextCell(4, quad);
	quad[0]= 9; quad[1]=10; quad[2]=15; quad[3]=14;	cells->InsertNextCell(4, quad);

	hollowcube->SetPoints(points);
	cells->Squeeze();
	hollowcube->SetPolys(cells);

	WriteMirrorPolyDataToFile("TEST_VTK_hollowcube.vtk", hollowcube);


	clock_t t0=clock(), t00=clock();

	double sp = 0.5;
	vnl_vector<double> spacing(3); spacing.fill(sp);
	vnl_vector<double> bbox(6); 
	bbox(0) = -15; bbox(1) = +15;
	bbox(2) = -15; bbox(3) = +15;
	bbox(4) = -15; bbox(5) = +15;

	itk::Image<unsigned char,3>::PointType origin;
	itk::Image<unsigned char,3>::RegionType region;
	ITKGridImageInformationFromPhysicalBoundingBoxAndSpacing<3>(bbox, spacing, &origin, &region);
	
	unsigned char inval = 255;
	unsigned char outval = 0;

	// polygonal data --> image stencil:
	//vtkSmartPointer<customPolyDataToStencil> pol2stenc = vtkSmartPointer<customPolyDataToStencil>::New();	//DOES NOT WORK WITH THE HOLLOW CUBE...
	vtkSmartPointer<vtkPolyDataToImageStencil> pol2stenc = vtkSmartPointer<vtkPolyDataToImageStencil>::New(); //takes ~0.25 s to update the stencil!!!, when the resolution is quite high...
	pol2stenc->SetTolerance(0); // important if extruder->SetVector(0, 0, 1) !!!
	//pol2stenc->SetInputConnection(extruder->GetOutputPort());
	pol2stenc->SetInput( hollowcube );
	pol2stenc->SetOutputOrigin(origin[0], origin[1], origin[2]);
	pol2stenc->SetOutputSpacing(sp,sp,sp);
	pol2stenc->SetOutputWholeExtent(0, region.GetSize()[0] - 1, 0, region.GetSize()[1] - 1, 0, region.GetSize()[2] - 1);
	pol2stenc->Update();
	std::cout<<"time to update the stencil... "<<(clock()-t0)/((double)CLOCKS_PER_SEC)<<" s."<<std::endl;
	// cut the corresponding white image and set the background:

	vtkSmartPointer<vtkImageStencilToImage> imgstenc2img = vtkSmartPointer<vtkImageStencilToImage>::New();
	imgstenc2img->SetInputConnection( pol2stenc->GetOutputPort() );
	imgstenc2img->SetInsideValue(250);
	imgstenc2img->SetOutsideValue(0);
	imgstenc2img->Update();
	std::cout<<"time to update the image... "<<(clock()-t0)/((double)CLOCKS_PER_SEC)<<" s."<<std::endl;

	vtkSmartPointer<vtkMetaImageWriter> imageWriter = vtkSmartPointer<vtkMetaImageWriter>::New();
	imageWriter->SetFileName("ImageStencil_hollowcube.mhd");
	imageWriter->SetInputConnection(imgstenc2img->GetOutputPort());
	imageWriter->Write();
	std::cout<<"time to get and write the image from the stencil... "<<(clock()-t0)/((double)CLOCKS_PER_SEC)<<" s."<<std::endl;

	t0=clock();
	//WriteITKImageToFile<ShapeAndPoseDeformableObject<3>::BinaryImageType>("ImageStencil_hexa.nii", obj->GetObjectAsBinaryImage());

	std::cout<<"time to get and write the binary image from the object... "<<(clock()-t0)/((double)CLOCKS_PER_SEC)<<" s."<<std::endl;

	std::cout<<"Exiting TestVTKPolyDataTolabelMap successfully"<<std::endl;
}



void TestAppendVTKPolyData() {
	vtkSmartPointer<vtkSphereSource> sphereSource1 = vtkSmartPointer<vtkSphereSource>::New();
	sphereSource1->SetCenter(0,0,0); sphereSource1->SetRadius(5); sphereSource1->Update();
	vtkSmartPointer<vtkSphereSource> sphereSource2 = vtkSmartPointer<vtkSphereSource>::New();
	sphereSource1->SetCenter(1,1,1); sphereSource2->SetRadius(10); sphereSource2->SetPhiResolution(25); sphereSource2->Update();
	vtkSmartPointer<vtkPolyData> sphere1 = sphereSource1->GetOutput();
	vtkSmartPointer<vtkPolyData> sphere2 = sphereSource2->GetOutput();

	vtkSmartPointer<vtkAppendPolyData> appendPolyDataFilter = vtkSmartPointer<vtkAppendPolyData>::New();
	appendPolyDataFilter->AddInput( sphere1 );
	appendPolyDataFilter->AddInput( sphere2 );
	std::cout<<"number of points of the first input: "<<sphere1->GetNumberOfPoints()<<std::endl;
	std::cout<<"number of points of the second input: "<<sphere2->GetNumberOfPoints()<<std::endl;
	appendPolyDataFilter->Update();
	std::cout<<"number of points of the output: "<<appendPolyDataFilter->GetOutput()->GetNumberOfPoints()<<std::endl;
	appendPolyDataFilter->RemoveInput( sphere1 );
	appendPolyDataFilter->Update();
	std::cout<<"number of points of the output after removing 1st input: "<<appendPolyDataFilter->GetOutput()->GetNumberOfPoints()<<std::endl;
}


#include <vtkBooleanOperationPolyDataFilter.h>
//WARNING!!! I HAD TO MODIFY VTK BY RENAMING DIFFERENCE in SETDIFFERENCE in this include file to avoid a clash with windows predefined variables...
//using #undef does not seem to be working
void TestIntersection3DPolyData() {
	std::cout<<"\n\n ENTERING TestIntersection3DPolyData"<<std::endl;

	//vtkSmartPointer<vtkSphereSource> source1 = vtkSmartPointer<vtkSphereSource>::New();
	//source1->SetCenter(0,0,-15); source1->SetRadius(5); source1->Update();
	////vtkSmartPointer<vtkSphereSource> source2 = vtkSmartPointer<vtkSphereSource>::New();
	////source2->SetCenter(3,0,0); source2->SetRadius(5); source2->Update();
	//vtkSmartPointer<vtkCubeSource> source2 = vtkSmartPointer<vtkCubeSource>::New();
	//source2->SetCenter(3,3,0); source2->SetBounds(2,5, -1, 4, -2, 2); //source2->Update();

	//vtkSmartPointer<vtkPolyData> obj1 = source1->GetOutput();
	//vtkSmartPointer<vtkPolyData> obj2 = source2->GetOutput();


	//DOES NOT SEEM TO WORK WITH THESE OBJECTS
	vtkSmartPointer<vtkPolyData> obj1 = vtkSmartPointer<vtkPolyData>::New();
	vtkSmartPointer<vtkPolyData> obj2 = vtkSmartPointer<vtkPolyData>::New();

	vtkSmartPointer<vtkPoints> points1 = vtkSmartPointer<vtkPoints>::New();	points1->Allocate(12);
	vtkSmartPointer<vtkPoints> points2 = vtkSmartPointer<vtkPoints>::New();	points2->Allocate(12);
	//vtkSmartPointer<vtkCellArray> cells1 = vtkSmartPointer<vtkCellArray>::New();	cells1->Allocate( cells1->EstimateSize(8,4) + cells1->EstimateSize(4,3) ); 
	//vtkSmartPointer<vtkCellArray> cells1 = vtkSmartPointer<vtkCellArray>::New();	cells1->Allocate( cells1->EstimateSize(6,4) + cells1->EstimateSize(2,6) ); 
	vtkSmartPointer<vtkCellArray> cells1 = vtkSmartPointer<vtkCellArray>::New();	cells1->Allocate( cells1->EstimateSize(20,3) ); 

	vtkFloatArray *pnormals1 = vtkFloatArray::New(); pnormals1->SetNumberOfComponents(3);// pnormals1->SetNumberOfTuples(20);
	vtkFloatArray *pnormals2 = vtkFloatArray::New(); pnormals2->SetNumberOfComponents(3);// pnormals2->SetNumberOfTuples(20);

	double tx = 0;
	double halflength1 = 5, halfwidth1 = 4, halfheight1 = 2, h1 = 4.5;
	double halflength2 = 4, halfwidth2 = 6, halfheight2 = 1, h2 = 3;
	points1->InsertNextPoint(-halflength1, 0.0,-halfheight1);	points1->InsertNextPoint(-h1, halfwidth1,-halfheight1);	points1->InsertNextPoint( h1, halfwidth1,-halfheight1);	points1->InsertNextPoint( halflength1, 0.0,-halfheight1);	points1->InsertNextPoint( h1,-halfwidth1,-halfheight1);	points1->InsertNextPoint(-h1,-halfwidth1,-halfheight1);
	points1->InsertNextPoint(-halflength1, 0.0, halfheight1);	points1->InsertNextPoint(-h1, halfwidth1, halfheight1);	points1->InsertNextPoint( h1, halfwidth1, halfheight1);	points1->InsertNextPoint( halflength1, 0.0, halfheight1);	points1->InsertNextPoint( h1,-halfwidth1, halfheight1);	points1->InsertNextPoint(-h1,-halfwidth1, halfheight1);
	points2->InsertNextPoint(tx-halflength2, 0.0,-halfheight2);	points2->InsertNextPoint(tx-h2, halfwidth2,-halfheight2);	points2->InsertNextPoint(tx+ h2, halfwidth2,-halfheight2);	points2->InsertNextPoint(tx+ halflength2, 0.0,-halfheight2);	points2->InsertNextPoint(tx+ h2,-halfwidth2,-halfheight2);	points2->InsertNextPoint(tx-h2,-halfwidth2,-halfheight2);
	points2->InsertNextPoint(tx-halflength2, 0.0, halfheight2);	points2->InsertNextPoint(tx-h2, halfwidth2, halfheight2);	points2->InsertNextPoint(tx+ h2, halfwidth2, halfheight2);	points2->InsertNextPoint(tx+ halflength2, 0.0, halfheight2);	points2->InsertNextPoint(tx+ h2,-halfwidth2, halfheight2);	points2->InsertNextPoint(tx-h2,-halfwidth2, halfheight2);

	//vtkIdType quad[4], tri[3];
	////'upper' faces
	//tri[0] = 0; tri[1] = 1; tri[2] = 5;				cells1->InsertNextCell(3, tri); 
	//quad[0]= 1; quad[1]= 2; quad[2]= 4; quad[3]= 5;	cells1->InsertNextCell(4, quad);
	//tri[0] = 2; tri[1] = 3; tri[2] = 4;				cells1->InsertNextCell(3, tri);
	////'side' faces
	//quad[0]= 0; quad[1]= 1; quad[2]= 7; quad[3]= 6;	cells1->InsertNextCell(4, quad);
	//quad[0]= 1; quad[1]= 2; quad[2]= 8; quad[3]= 7;	cells1->InsertNextCell(4, quad);
	//quad[0]= 2; quad[1]= 3; quad[2]= 9; quad[3]= 8;	cells1->InsertNextCell(4, quad);
	//quad[0]= 4; quad[1]= 3; quad[2]= 9; quad[3]=10;	cells1->InsertNextCell(4, quad);
	//quad[0]= 5; quad[1]= 4; quad[2]=10; quad[3]=11;	cells1->InsertNextCell(4, quad);
	//quad[0]= 0; quad[1]= 5; quad[2]=11; quad[3]= 6;	cells1->InsertNextCell(4, quad);
	////'below' faces
	//tri[0] = 6; tri[1] = 7; tri[2] =11;				cells1->InsertNextCell(3, tri);
	//quad[0]= 7; quad[1]= 8; quad[2]=10; quad[3]=11;	cells1->InsertNextCell(4, quad);
	//tri[0] = 8; tri[1] = 9; tri[2] =10;				cells1->InsertNextCell(3, tri);
	
	//vtkIdType quad[4], hexa[6];
	////'below' face
	//hexa[0] = 0; hexa[1] = 1; hexa[2] = 2; hexa[3] = 3; hexa[4] = 4; hexa[5] = 5; cells1->InsertNextCell(6, hexa);
	////'side' faces
	//quad[0]= 0; quad[1]= 1; quad[2]= 7; quad[3]= 6;	cells1->InsertNextCell(4, quad);
	//quad[0]= 1; quad[1]= 2; quad[2]= 8; quad[3]= 7;	cells1->InsertNextCell(4, quad);
	//quad[0]= 2; quad[1]= 3; quad[2]= 9; quad[3]= 8;	cells1->InsertNextCell(4, quad);
	//quad[0]= 4; quad[1]= 3; quad[2]= 9; quad[3]=10;	cells1->InsertNextCell(4, quad);
	//quad[0]= 5; quad[1]= 4; quad[2]=10; quad[3]=11;	cells1->InsertNextCell(4, quad);
	//quad[0]= 0; quad[1]= 5; quad[2]=11; quad[3]= 6;	cells1->InsertNextCell(4, quad);
	////'below' faces
	//hexa[0] = 6; hexa[1] = 7; hexa[2] = 8; hexa[3] = 9; hexa[4] =10; hexa[5] =11; cells1->InsertNextCell(6, hexa);

	double nor1 = sqrt(halfwidth1*halfwidth1 + (halflength1-h1)*(halflength1-h1));
	double nor2 = sqrt(halfwidth2*halfwidth2 + (halflength2-h2)*(halflength2-h2));


	vtkIdType tri[3]; 
	float nrmls[3];
	//'below' face
	tri[0] = 0; tri[1] = 1; tri[2] = 5; cells1->InsertNextCell(3, tri); nrmls[0] = 0; nrmls[1] = 0; nrmls[2] =-1; pnormals1->InsertNextTuple(nrmls);   pnormals2->InsertNextTuple(nrmls);
	tri[0] = 1; tri[1] = 2; tri[2] = 5; cells1->InsertNextCell(3, tri); nrmls[0] = 0; nrmls[1] = 0; nrmls[2] =-1; pnormals1->InsertNextTuple(nrmls);   pnormals2->InsertNextTuple(nrmls);
	tri[0] = 2; tri[1] = 4; tri[2] = 5; cells1->InsertNextCell(3, tri); nrmls[0] = 0; nrmls[1] = 0; nrmls[2] =-1; pnormals1->InsertNextTuple(nrmls);   pnormals2->InsertNextTuple(nrmls);
	tri[0] = 2; tri[1] = 3; tri[2] = 4; cells1->InsertNextCell(3, tri); nrmls[0] = 0; nrmls[1] = 0; nrmls[2] =-1; pnormals1->InsertNextTuple(nrmls);   pnormals2->InsertNextTuple(nrmls);
	//'side' faces
	tri[0] = 0; tri[1] = 1; tri[2] = 7; cells1->InsertNextCell(3, tri); nrmls[0] =-halfwidth1/nor1; nrmls[1] = (halflength1-h1)/nor1; nrmls[2] = 0; pnormals1->InsertNextTuple(nrmls);   nrmls[0] =-halfwidth2/nor2; nrmls[1] = (halflength2-h2)/nor2; nrmls[2] = 0; pnormals2->InsertNextTuple(nrmls);
	tri[0] = 0; tri[1] = 7; tri[2] = 6; cells1->InsertNextCell(3, tri); nrmls[0] =-halfwidth1/nor1; nrmls[1] = (halflength1-h1)/nor1; nrmls[2] = 0; pnormals1->InsertNextTuple(nrmls);   nrmls[0] =-halfwidth2/nor2; nrmls[1] = (halflength2-h2)/nor2; nrmls[2] = 0; pnormals2->InsertNextTuple(nrmls);
	tri[0] = 1; tri[1] = 2; tri[2] = 8; cells1->InsertNextCell(3, tri); nrmls[0] = 0; nrmls[1] = 1; nrmls[2] = 0; pnormals1->InsertNextTuple(nrmls);   pnormals2->InsertNextTuple(nrmls);
	tri[0] = 1; tri[1] = 8; tri[2] = 7; cells1->InsertNextCell(3, tri); nrmls[0] = 0; nrmls[1] = 1; nrmls[2] = 0; pnormals1->InsertNextTuple(nrmls);   pnormals2->InsertNextTuple(nrmls);
	tri[0] = 2; tri[1] = 3; tri[2] = 9; cells1->InsertNextCell(3, tri); nrmls[0] = halfwidth1/nor1; nrmls[1] = (halflength1-h1)/nor1; nrmls[2] = 0; pnormals1->InsertNextTuple(nrmls);   nrmls[0] = halfwidth2/nor2; nrmls[1] = (halflength2-h2)/nor2; nrmls[2] = 0; pnormals2->InsertNextTuple(nrmls);
	tri[0] = 2; tri[1] = 9; tri[2] = 8; cells1->InsertNextCell(3, tri); nrmls[0] = halfwidth1/nor1; nrmls[1] = (halflength1-h1)/nor1; nrmls[2] = 0; pnormals1->InsertNextTuple(nrmls);   nrmls[0] = halfwidth2/nor2; nrmls[1] = (halflength2-h2)/nor2; nrmls[2] = 0; pnormals2->InsertNextTuple(nrmls);
	tri[0] = 3; tri[1] = 4; tri[2] =10; cells1->InsertNextCell(3, tri); nrmls[0] = halfwidth1/nor1; nrmls[1] =-(halflength1-h1)/nor1; nrmls[2] = 0; pnormals1->InsertNextTuple(nrmls);   nrmls[0] = halfwidth2/nor2; nrmls[1] =-(halflength2-h2)/nor2; nrmls[2] = 0; pnormals2->InsertNextTuple(nrmls);
	tri[0] = 3; tri[1] =10; tri[2] = 9; cells1->InsertNextCell(3, tri); nrmls[0] = halfwidth1/nor1; nrmls[1] =-(halflength1-h1)/nor1; nrmls[2] = 0; pnormals1->InsertNextTuple(nrmls);   nrmls[0] = halfwidth2/nor2; nrmls[1] =-(halflength2-h2)/nor2; nrmls[2] = 0; pnormals2->InsertNextTuple(nrmls);
	tri[0] = 4; tri[1] = 5; tri[2] =11; cells1->InsertNextCell(3, tri); nrmls[0] = 0; nrmls[1] =-1; nrmls[2] = 0; pnormals1->InsertNextTuple(nrmls);   pnormals2->InsertNextTuple(nrmls);
	tri[0] = 4; tri[1] =11; tri[2] =10; cells1->InsertNextCell(3, tri); nrmls[0] = 0; nrmls[1] =-1; nrmls[2] = 0; pnormals1->InsertNextTuple(nrmls);   pnormals2->InsertNextTuple(nrmls);
	tri[0] = 0; tri[1] =11; tri[2] = 5; cells1->InsertNextCell(3, tri); nrmls[0] =-halfwidth1/nor1; nrmls[1] =-(halflength1-h1)/nor1; nrmls[2] = 0; pnormals1->InsertNextTuple(nrmls);   nrmls[0] =-halfwidth2/nor2; nrmls[1] =-(halflength2-h2)/nor2; nrmls[2] = 0; pnormals2->InsertNextTuple(nrmls);
	tri[0] = 0; tri[1] = 6; tri[2] =11; cells1->InsertNextCell(3, tri); nrmls[0] =-halfwidth1/nor1; nrmls[1] =-(halflength1-h1)/nor1; nrmls[2] = 0; pnormals1->InsertNextTuple(nrmls);   nrmls[0] =-halfwidth2/nor2; nrmls[1] =-(halflength2-h2)/nor2; nrmls[2] = 0; pnormals2->InsertNextTuple(nrmls);
	//upper faces
	tri[0] = 6; tri[1] = 7; tri[2] =11; cells1->InsertNextCell(3, tri); nrmls[0] = 0; nrmls[1] = 0; nrmls[2] =-1; pnormals1->InsertNextTuple(nrmls);   pnormals2->InsertNextTuple(nrmls);
	tri[0] = 7; tri[1] = 8; tri[2] =11; cells1->InsertNextCell(3, tri); nrmls[0] = 0; nrmls[1] = 0; nrmls[2] =-1; pnormals1->InsertNextTuple(nrmls);   pnormals2->InsertNextTuple(nrmls);
	tri[0] = 8; tri[1] =10; tri[2] =11; cells1->InsertNextCell(3, tri); nrmls[0] = 0; nrmls[1] = 0; nrmls[2] =-1; pnormals1->InsertNextTuple(nrmls);   pnormals2->InsertNextTuple(nrmls);
	tri[0] = 8; tri[1] = 9; tri[2] =10; cells1->InsertNextCell(3, tri); nrmls[0] = 0; nrmls[1] = 0; nrmls[2] =-1; pnormals1->InsertNextTuple(nrmls);   pnormals2->InsertNextTuple(nrmls);

	obj1->SetPoints(points1); obj2->SetPoints(points2); 
	cells1->Squeeze();
	obj1->SetPolys(cells1);   obj2->SetPolys(cells1);
	obj1->GetCellData()->SetNormals( pnormals1 ); obj2->GetCellData()->SetNormals( pnormals2 );

	vtkSmartPointer<vtkTriangleFilter> triangFilter1 = vtkSmartPointer<vtkTriangleFilter>::New();
	triangFilter1->SetInput(obj1); triangFilter1->Update();
	vtkSmartPointer<vtkTriangleFilter> triangFilter2 = vtkSmartPointer<vtkTriangleFilter>::New();
	triangFilter2->SetInput(obj2); triangFilter2->Update();




	//vtkSmartPointer<vtkIntersectionPolyDataFilter>filter = vtkSmartPointer<vtkIntersectionPolyDataFilter>::New();
	vtkIntersectionPolyDataFilter *filter = vtkIntersectionPolyDataFilter::New();
	filter->SetInput(0, triangFilter1->GetOutput() );
	filter->SetInput(1, triangFilter2->GetOutput() );
	filter->Update();
	std::cout<<"number of points in the output: "<<filter->GetOutput()->GetNumberOfPoints()<<std::endl;
	
	WriteMirrorPolyDataToFile("3DPolyIntersection_obj1.vtk", triangFilter1->GetOutput() );
	WriteMirrorPolyDataToFile("3DPolyIntersection_obj2.vtk", triangFilter2->GetOutput() );
	WriteMirrorPolyDataToFile("3DPolyIntersection_obj_inter.vtk", filter->GetOutput());

	filter->Delete();
	
	vtkBooleanOperationPolyDataFilter *booleanFilter = vtkBooleanOperationPolyDataFilter::New();
	booleanFilter->SetOperationToIntersection();
	booleanFilter->SetInput(0, triangFilter1->GetOutput() );
	booleanFilter->SetInput(1, triangFilter2->GetOutput() );
	booleanFilter->Update();
	WriteMirrorPolyDataToFile("3DPolyIntersection_obj_BooleanIntersection.vtk", booleanFilter->GetOutput());

	std::cout<<"nb of vertices in the intersection polydata: "<<booleanFilter->GetOutput()->GetNumberOfPoints()<<std::endl;

	std::cout<<" (for 2D data, check TestClipper) "<<std::endl;

	vtkSmartPointer<vtkMassProperties> massProp1 = vtkSmartPointer<vtkMassProperties>::New();
	massProp1->SetInput( triangFilter1->GetOutput() ); massProp1->Update();
	vtkSmartPointer<vtkMassProperties> massProp2 = vtkSmartPointer<vtkMassProperties>::New();
	massProp2->SetInput( triangFilter2->GetOutput() ); massProp2->Update();
	vtkSmartPointer<vtkMassProperties> massProp3 = vtkSmartPointer<vtkMassProperties>::New();
	massProp3->SetInput( booleanFilter->GetOutput() ); massProp3->Update();
	std::cout<<"volume of object1: "<<massProp1->GetVolume()<<std::endl;
	std::cout<<"volume of object2: "<<massProp2->GetVolume()<<std::endl;
	std::cout<<"volume of intersection: "<<massProp3->GetVolume()<<std::endl;

}

#include "vtkPolyDataMapper.h"
#include "vtkCamera.h"
#include "vtkLight.h"
#include <vtkRenderer.h> 
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkPolyDataMapper.h> 
#include <vtkActor.h> 
#include <vtkActorCollection.h>
#include <vtkProperty.h> 

void TestVTKRenderImageAndZBuffer() {
	std::cout<<"test vtk rendering image and zbuffer..."<<std::endl;

	vtkSphereSource *sphereSource = vtkSphereSource::New();
	sphereSource->SetThetaResolution(20);sphereSource->SetPhiResolution(20);
	sphereSource->SetCenter(0,0,0); sphereSource->SetRadius(8); 

	vtkRenderer *renderer = vtkRenderer::New();  renderer->SetBackground(0,0,0);
	vtkRenderWindow *renderWindow = vtkRenderWindow::New(); 
	renderWindow->SetOffScreenRendering(1);
	renderWindow->SetSize(100, 100);

	vtkPolyDataMapper *map = vtkPolyDataMapper::New(); map->SetInput( sphereSource->GetOutput() );
	vtkActor *actor = vtkActor::New(); actor->SetMapper(map); 

	renderer->AddActor(actor);

	renderWindow->AddRenderer( renderer ); 
	vtkCamera *camera = vtkCamera::New();
	camera->ParallelProjectionOn();

	double cameraOffset = 10*0.5;
	camera->SetFocalPoint( 0, 0, 0 );
	camera->SetPosition( -16, 0, 0);
	camera->SetViewUp(0,0,1);
	camera->SetParallelScale( 8 );
	camera->SetClippingRange( 0, cameraOffset + 10 );

	renderer->SetActiveCamera(camera);

vtkRenderWindowInteractor *iren = vtkRenderWindowInteractor::New(); 
iren->SetRenderWindow(renderWindow); 

	renderWindow->Render();
	float *zbuff = renderWindow->GetZbufferData(0, 0, 99, 99);
	unsigned char *pixbuff = renderWindow->GetPixelData(0, 0, 99, 99, 0);

	itk::Image<float>::Pointer zbuffImg = itk::Image<float>::New();
	itk::Image<unsigned char>::Pointer pixbuffImg = itk::Image<unsigned char>::New();
	itk::Image<float>::SizeType size; size[0]=100;size[1]=100;
	itk::Image<float>::IndexType index; index[0]=0;index[1]=0;
	itk::Image<float>::RegionType region; region.SetSize(size);region.SetIndex(index);
	zbuffImg->SetRegions(region); pixbuffImg->SetRegions(region);
	zbuffImg->Allocate(); pixbuffImg->Allocate();
	float *zb = zbuffImg->GetBufferPointer();
	unsigned char *pb = pixbuffImg->GetBufferPointer();

	for (unsigned i=0 ; i<100*100 ; i++) { 
		zb[i] = zbuff[i];
		pb[i] = (pixbuff[3*i] + pixbuff[3*i+1] + pixbuff[3*i+2])/3;
	}
	
	Write2DGreyLevelRescaledImageToFile<itk::Image<float>>("TestVTK_ZBuffer.png", zbuffImg);
	Write2DGreyLevelRescaledImageToFile<itk::Image<unsigned char>>("TestVTK_Img.png", pixbuffImg);
//iren->Start();

	delete [] zbuff;
	delete [] pixbuff;
iren->Delete();
}

void TestVTKRenderLeak() {
	std::cout<<"test leak..."<<std::endl;
	vtkSphereSource *sphereSource = vtkSphereSource::New();
	sphereSource->SetThetaResolution(20);sphereSource->SetPhiResolution(20);
	sphereSource->SetCenter(0,0,0); sphereSource->SetRadius(5); 

//	vtkRenderWindowInteractor *iren = vtkRenderWindowInteractor::New(); 
//	iren->SetRenderWindow(renderWindow); 

	vtkRenderer *renderer = vtkRenderer::New();  renderer->SetBackground(0,0,0);
	vtkRenderWindow *renderWindow = vtkRenderWindow::New(); 
	renderWindow->SetOffScreenRendering(1);
	renderWindow->SetSize(100, 100);

	double radius = 5;
	clock_t t0 = clock();

	for (unsigned i=0 ; i<5001 ; i++) {
		radius += 0.001; sphereSource->SetRadius(radius); sphereSource->Update();

		vtkPolyDataMapper *map = vtkPolyDataMapper::New(); map->SetInput( sphereSource->GetOutput() );
		vtkActor *actor = vtkActor::New(); actor->SetMapper(map); 

		renderer->AddActor(actor);

		renderWindow->AddRenderer( renderer ); 
		vtkCamera *camera = vtkCamera::New();
		camera->ParallelProjectionOn();
//		renderer->SetActiveCamera(camera);


		renderWindow->Render();

		float *zbuff = renderWindow->GetZbufferData(0, 0, 99, 99);
		unsigned char *pixbuff = renderWindow->GetPixelData(0, 0, 99, 99, 0);

	//	iren->Start(); ///... does not leak, here...
		if (i==100) std::cout<<"100 iterations, after "<<(clock()-t0)/((double)CLOCKS_PER_SEC)<<" s."<<std::endl;
		if (i==500) std::cout<<"500 iterations, after "<<(clock()-t0)/((double)CLOCKS_PER_SEC)<<" s."<<std::endl;
		if (i==1000) std::cout<<"1000 iterations, after "<<(clock()-t0)/((double)CLOCKS_PER_SEC)<<" s."<<std::endl;
		if (i==5000) std::cout<<"5000 iterations, after "<<(clock()-t0)/((double)CLOCKS_PER_SEC)<<" s."<<std::endl;

		delete [] zbuff;
		delete [] pixbuff;

		renderWindow->RemoveRenderer(renderer);
		renderer->RemoveActor(actor);

		map->Delete();
		actor->Delete();
		camera->Delete();
	}

		renderWindow->Delete();
		renderer->Delete();
		sphereSource->Delete();
	//iren->Delete();
	std::cout<<"end leak..."<<std::endl; 
}


void TestVTKRenderNoLeak() {
	std::cout<<"test no leak..."<<std::endl;
	vtkSphereSource *sphereSource = vtkSphereSource::New();
	sphereSource->SetThetaResolution(20);sphereSource->SetPhiResolution(20);
	sphereSource->SetCenter(0,0,0); sphereSource->SetRadius(5); 

	vtkRenderer *renderer = vtkRenderer::New();  renderer->SetBackground(0,0,0);
	vtkPolyDataMapper *map = vtkPolyDataMapper::New(); map->SetInput( sphereSource->GetOutput() );
	vtkActor *actor = vtkActor::New(); actor->SetMapper(map); 

	renderer->AddActor(actor);
	vtkRenderWindow *renderWindow = vtkRenderWindow::New(); 
	renderWindow->SetOffScreenRendering(1);
	renderWindow->SetSize(100, 100);
	renderWindow->AddRenderer( renderer ); 
	vtkCamera *camera = vtkCamera::New();
	camera->ParallelProjectionOn();
	//renderer->SetActiveCamera(camera);
	vtkRenderWindowInteractor *iren = vtkRenderWindowInteractor::New(); 
	iren->SetRenderWindow(renderWindow); 
 
	double radius = 5;
	clock_t t0 = clock();

	for (unsigned i=0 ; i<5001 ; i++) {
		radius += 0.001; sphereSource->SetRadius(radius); sphereSource->Update();

		renderWindow->Render();
//		iren->Start(); ///... does not leak, here...
//tester removeactor / addactor...
		float *zbuff = renderWindow->GetZbufferData(0, 0, 99, 99);
		unsigned char *pixbuff = renderWindow->GetPixelData(0, 0, 99, 99, 0);

		if (i==100) std::cout<<"100 iterations, after "<<(clock()-t0)/((double)CLOCKS_PER_SEC)<<" s."<<std::endl;
		if (i==500) std::cout<<"500 iterations, after "<<(clock()-t0)/((double)CLOCKS_PER_SEC)<<" s."<<std::endl;
		if (i==1000) std::cout<<"1000 iterations, after "<<(clock()-t0)/((double)CLOCKS_PER_SEC)<<" s."<<std::endl;
		if (i==5000) std::cout<<"5000 iterations, after "<<(clock()-t0)/((double)CLOCKS_PER_SEC)<<" s."<<std::endl;

		delete [] zbuff;
		delete [] pixbuff;

	}

	map->Delete();
	actor->Delete();
	renderWindow->Delete();
	camera->Delete();
	renderer->Delete();
	sphereSource->Delete();
	iren->Delete();
	std::cout<<"end leak..."<<std::endl; 
}


int main(int argc, char** argv) {

	//Test3DVTKPolyDataTolabelMap();

	//TestAppendVTKPolyData();

	//TestIntersection3DPolyData(); 

	TestVTKRenderImageAndZBuffer();

	//TestVTKRenderLeak();

	//TestVTKRenderNoLeak();

	return 1;
}
