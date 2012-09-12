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
#include "LabelMapUtils.h"
#include <vtkSphereSource.h>

#include "itkBinaryDilateImageFilter.h"
#include "itkBinaryBallStructuringElement.h"
#include <itkLabelMapMaskImageFilter.h>
#include <itkBinaryImageToLabelMapFilter.h>

#include "PixelSetUtils.h"

using namespace psciob;


//
void TestLabelObject() {

	typedef itk::Image<unsigned char, 2>		ImageType;
	typedef itk::LabelObject<unsigned char, 2>	LabelObjectType;
	typedef itk::LabelMap<LabelObjectType>		LabelMapType;

	LabelObjectType::Pointer labelObject = LabelObjectType::New();
	LabelMapType::Pointer labelMap = LabelMapType::New();

	labelMap->AddLabelObject(labelObject);
	LabelMapType::PointType origin; origin[0] = 0; origin[1] = 0;
	LabelMapType::RegionType region;
	LabelMapType::SizeType size; size.Fill(20);
	region.SetSize(size);
	labelMap->SetRegions(region);
	labelMap->SetOrigin(origin);

	labelObject->SetLabel(1);
	ImageType::IndexType index;
	index[0] = 5; index[1] = 0; labelObject->AddLine(index, 10);
	index[0] = 4; index[1] = 1; labelObject->AddLine(index, 12);
	index[0] = 4; index[1] = 2; labelObject->AddLine(index, 12);
	index[0] = 4; index[1] = 3; labelObject->AddLine(index, 12);
	index[0] = 3; index[1] = 4; labelObject->AddLine(index, 14);
	index[0] = 3; index[1] = 5; labelObject->AddLine(index, 14);
	index[0] = 2; index[1] = 6; labelObject->AddLine(index, 16);
	index[0] = 1; index[1] = 7; labelObject->AddLine(index, 18);
	index[0] = 1; index[1] = 8; labelObject->AddLine(index, 18);
	index[0] = 0; index[1] = 9; labelObject->AddLine(index, 20);
	index[0] = 0; index[1] =10; labelObject->AddLine(index, 20);
	index[0] = 1; index[1] =11; labelObject->AddLine(index, 18);
	index[0] = 1; index[1] =12; labelObject->AddLine(index, 18);
	index[0] = 2; index[1] =13; labelObject->AddLine(index, 16);
	index[0] = 3; index[1] =14; labelObject->AddLine(index, 14);
	index[0] = 3; index[1] =15; labelObject->AddLine(index, 14);
	index[0] = 4; index[1] =16; labelObject->AddLine(index, 12);
	index[0] = 4; index[1] =17; labelObject->AddLine(index, 12);
	index[0] = 4; index[1] =18; labelObject->AddLine(index, 12);
	index[0] = 5; index[1] =19; labelObject->AddLine(index, 10);

	typedef itk::LabelMapToLabelImageFilter< LabelMapType, ImageType> LabelMapToLabelImageFilterType;
	LabelMapToLabelImageFilterType::Pointer labelMapToLabelImageFilter = LabelMapToLabelImageFilterType::New();
	labelMapToLabelImageFilter->SetInput(labelMap);
	labelMapToLabelImageFilter->Update();

	Write2DGreyLevelRescaledImageToFile<ImageType>("TestITK_ConvertedLabelImage.png", labelMapToLabelImageFilter->GetOutput() );

	std::cout<<"getting labelObject back..."<<std::endl;
	std::cout<<"nb of label objects: "<<labelMap->GetNumberOfLabelObjects()<<std::endl;

	LabelObjectType::Pointer labelObject_bis = labelMap->GetNthLabelObject( 0 );
	std::cout<<"number of lines: "<<labelObject_bis->GetNumberOfLines()<<std::endl;

	std::cout<<"Casting the label map to a smaller frame, cropping the left side, and enlarging the rest"<<std::endl;

	LabelMapType::Pointer labelMap2 = LabelMapType::New();
	LabelMapType::PointType origin2; origin2[0] = -3; origin2[1] = -2;
	LabelMapType::RegionType region2;
	LabelMapType::SizeType size2; size2[0] = 26; size2[1] = 38;
	region2.SetSize(size2);
	labelMap2->SetRegions(region2);
	labelMap2->SetOrigin(origin2);

	//CastLabelMapToOtherFrame<LabelMapType, LabelMapType>(labelMap, labelMap2);
	InsertSingleObjectLabelMapIntoAnother<LabelMapType, LabelMapType>(labelMap, labelMap2);
	InsertSingleObjectLabelMapIntoAnother<LabelMapType, LabelMapType>(labelMap, labelMap2);

	labelMapToLabelImageFilter->SetInput(labelMap2);
	labelMapToLabelImageFilter->Update();

	Write2DGreyLevelRescaledImageToFile<ImageType>("TestITK_ConvertedLabelImageReFramed.png", labelMapToLabelImageFilter->GetOutput() );
	
	std::cout<<"Exit from TestLabelObject() \n\n"<<std::endl;
}

void TestStructuringElementToLabelObjectAndSurroundings() {

	typedef itk::LabelObject<unsigned char, 2>	LabelObjectType;
	typedef itk::LabelMap<LabelObjectType>		LabelMapType;

	LabelMapType::Pointer labelMap = LabelMapType::New();
	LabelMapType::Pointer outputMap = LabelMapType::New();
	LabelMapType::Pointer dilatedMap = LabelMapType::New();
	LabelMapType::PointType origin; origin[0] = 0; origin[1] = 0; LabelMapType::RegionType region; LabelMapType::SizeType size; size.Fill(20);
	region.SetSize(size); labelMap->SetRegions(region); labelMap->SetOrigin(origin);
	
	LabelObjectType::Pointer obj1 = LabelObjectType::New(); obj1->SetLabel(1);
	LabelObjectType::Pointer obj2 = LabelObjectType::New(); obj2->SetLabel(2);
	LabelMapType::IndexType index;
	index[0] = 0; index[1] = 0; obj1->AddLine(index, 10);	index[0] =15; index[1] = 0; obj1->AddLine(index, 5);	
	index[0] = 0; index[1] = 1; obj1->AddLine(index, 12);
	index[0] = 4; index[1] = 2; obj1->AddLine(index, 12);	index[0] = 4; index[1] = 3; obj1->AddLine(index, 12);
	index[0] = 3; index[1] = 4; obj1->AddLine(index, 14);	index[0] = 3; index[1] = 5; obj1->AddLine(index, 14);
	labelMap->AddLabelObject(obj1);

	index[0] = 2; index[1] =13; obj2->AddLine(index, 16);	index[0] = 3; index[1] =14; obj2->AddLine(index, 14);
	index[0] = 3; index[1] =15; obj2->AddLine(index, 14);	index[0] = 4; index[1] =16; obj2->AddLine(index, 12);
	index[0] = 7; index[1] =17; obj2->AddLine(index, 12);
	index[0] = 0; index[1] =18; obj2->AddLine(index, 10);   index[0] = 12; index[1] =18; obj2->AddLine(index, 8);
	index[0] = 7; index[1] =19; obj2->AddLine(index, 12);
	labelMap->AddLabelObject(obj2);

	LabelObjectType::Pointer seObject = LabelObjectType::New();

	std::cout<<"calling GenerateBallStructuringLabelObject"<<std::endl;
	GenerateBallStructuringLabelObject<LabelObjectType>(1, seObject);
	std::cout<<"OK ; looking into the label object"<<std::endl;
	std::cout<<"number of lines: "<<seObject->GetNumberOfLines()<<std::endl;
	for (unsigned i=0 ; i<seObject->GetNumberOfLines() ; i++) {
		std::cout<<"line "<<i<<", index: "<<seObject->GetLine(i).GetIndex()[0]<<", "<<seObject->GetLine(i).GetIndex()[1]<<", length: "<<seObject->GetLine(i).GetLength()<<std::endl;
	}

	DilateOffContextObjects<LabelMapType>(labelMap, seObject, dilatedMap); 

	GetOffContextObjectsSurroundings<LabelMapType>(labelMap, seObject, outputMap);

	std::cout<<"number of objects in the input map: "<<labelMap->GetNumberOfLabelObjects()<<std::endl;
	std::cout<<"number of objects in the output map: "<<outputMap->GetNumberOfLabelObjects()<<std::endl;

	LabelMapType::Iterator it1 = LabelMapType::Iterator(labelMap); 
	LabelMapType::Iterator it2 = LabelMapType::Iterator(outputMap);
	unsigned i=0;
	for (it1.GoToBegin(), it2.GoToBegin() ; !it1.IsAtEnd() ; ++it1, ++it2) {
		std::cout<<"looking at object "<<i<<std::endl;
	}

	typedef itk::Image<unsigned char, 2>		ImageType;
	typedef itk::LabelMapToLabelImageFilter< LabelMapType, ImageType> LabelMapToLabelImageFilterType;
	LabelMapToLabelImageFilterType::Pointer labelMapToLabelImageFilter = LabelMapToLabelImageFilterType::New();
	labelMapToLabelImageFilter->SetInput(labelMap);
	labelMapToLabelImageFilter->Update();

	Write2DGreyLevelRescaledImageToFile<ImageType>("TestITK_LabelImage1input.png", labelMapToLabelImageFilter->GetOutput() );
	
	labelMapToLabelImageFilter->SetInput(dilatedMap);
	labelMapToLabelImageFilter->Update();
	Write2DGreyLevelRescaledImageToFile<ImageType>("TestITK_LabelImage2dilated.png", labelMapToLabelImageFilter->GetOutput() );

	labelMapToLabelImageFilter->SetInput(outputMap);
	labelMapToLabelImageFilter->Update();
	Write2DGreyLevelRescaledImageToFile<ImageType>("TestITK_LabelImage3surroundings.png", labelMapToLabelImageFilter->GetOutput() );


	std::cout<<"\n\n TESTing also the intersection between 2 label objects. in this case the dilated object (the 1st) and the subtracted object..."<<std::endl;

	LabelObjectType::Pointer test_obj1 = LabelObjectType::New();
	unsigned nbPix = GetLabelObjectIntersection<LabelObjectType>( dilatedMap->GetLabelObject(1), outputMap->GetLabelObject(1), test_obj1 );
	std::cout<<"nb of pixels: "<<nbPix<<", nb lines: "<<test_obj1->GetNumberOfLines()<<std::endl;
	std::cout<<"comparison with original object: "<<obj1->Size()<<", nb lines: "<<obj1->GetNumberOfLines()<<std::endl;

	for (unsigned i=0 ; i<test_obj1->GetNumberOfLines() ; i++) {
		std::cout<<"  line "<<i<<", start index: "<<test_obj1->GetLine(i).GetIndex()<<", length = "<<test_obj1->GetLine(i).GetLength()<<std::endl;
	}


}
//
void TestVTKToITKLabelMapConversion() {
	std::cout<<"\n\n Entering TestVTKToITKLabelMapConversion"<<std::endl;
	vtkSmartPointer<vtkSphereSource> m_sphereSource = vtkSmartPointer<vtkSphereSource>::New();
	m_sphereSource->SetPhiResolution(25);
	m_sphereSource->SetThetaResolution(12);
	m_sphereSource->SetCenter(2,1.5,0);
	m_sphereSource->SetRadius(10.3);
	m_sphereSource->Update();

	WriteMirrorPolyDataToFile("TestVTK2ITK_Sphere.vtk", m_sphereSource->GetOutput());

	typedef itk::Image<unsigned char, 3>		Image3DType;
	typedef itk::LabelObject<unsigned char, 3>	LabelObject3DType;
	typedef itk::LabelMap<LabelObject3DType>	LabelMap3DType;

	LabelObject3DType::Pointer labelObject = LabelObject3DType::New();
	LabelMap3DType::Pointer labelMap = LabelMap3DType::New();
clock_t t0 = clock();
	VTKPolyDataToLabelMap<LabelMap3DType>(m_sphereSource->GetOutput(), labelMap, labelMap->GetSpacing());
std::cout<<"time to convert the object: "<<(clock()-t0)/((double)CLOCKS_PER_SEC)<<" s."<<std::endl;
	typedef itk::LabelMapToLabelImageFilter< LabelMap3DType, Image3DType> LabelMapToLabelImageFilterType;
	LabelMapToLabelImageFilterType::Pointer labelMapToLabelImageFilter = LabelMapToLabelImageFilterType::New();
	labelMapToLabelImageFilter->SetInput(labelMap);
	labelMapToLabelImageFilter->Update();

	WriteITKImageToFile<Image3DType>("TestVTK2ITK_Sphere.nii", labelMapToLabelImageFilter->GetOutput() );


}




//
void TestVTKToITK2DLabelMapConversion() {
	std::cout<<"\n\n Entering TestVTKToITK2DLabelMapConversion"<<std::endl;
	
	vtkSmartPointer<vtkPolyData> m_outputPolyData = vtkSmartPointer<vtkPolyData>::New();
	m_outputPolyData->Initialize();
	vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();	points->Allocate(25);
	vtkSmartPointer<vtkCellArray> cells = vtkSmartPointer<vtkCellArray>::New();	cells->Allocate( cells->EstimateSize(25,2) ); 

	//points must be ordered
	double theta, longaxis = 15.0;
	double ct = cos(-25*180.0/PI), st = sin(-25*180.0/PI), xx, yy;
	vtkIdType segm[2];

	//1st point
	points->InsertNextPoint( 3 + longaxis , -1.5 , 0 );
	//all points and all segments but the last
	for (unsigned i=1 ; i<25 ; i++) {
		theta = 2.0*i*PI/((double)25);
		xx = longaxis*cos(theta);
		yy = 10.0*sin(theta);
		points->InsertNextPoint( 3.0 + xx*ct - yy*st,
								 -1.5 + xx*st + yy*ct, 0 ); 
		segm[0] = i-1; 	segm[1] = i; cells->InsertNextCell(2, segm);
	}
	//last segment
	segm[0] = 25-1; segm[1] = 0; cells->InsertNextCell(2, segm);

	m_outputPolyData->SetPoints(points);
	cells->Squeeze();
	m_outputPolyData->SetLines(cells);

	WriteMirrorPolyDataToFile("TestVTK2ITK_2DEllipse.vtk", m_outputPolyData);

	typedef itk::Image<unsigned short, 2>		Image2DType;
	typedef itk::Image<unsigned short, 3>		Image3DType;
	typedef itk::LabelObject<unsigned short, 2>	LabelObject2DType;
	typedef itk::LabelMap<LabelObject2DType>	LabelMap2DType;

	LabelObject2DType::Pointer labelObject = LabelObject2DType::New();
	LabelMap2DType::Pointer labelMap = LabelMap2DType::New();
	
	clock_t t0 = clock();
	vnl_vector<double> bbox;
	VTKPolyDataToLabelMap<LabelMap2DType>(m_outputPolyData, labelMap, labelMap->GetSpacing());
	std::cout<<"time to convert the object: "<<(clock()-t0)/((double)CLOCKS_PER_SEC)<<" s."<<std::endl;
	
	typedef itk::LabelMapToLabelImageFilter< LabelMap2DType, Image2DType> LabelMapToLabelImageFilterType;
	LabelMapToLabelImageFilterType::Pointer labelMapToLabelImageFilter = LabelMapToLabelImageFilterType::New();
	labelMapToLabelImageFilter->SetInput(labelMap);
	labelMapToLabelImageFilter->Update();

	Write2DGreyLevelRescaledImageToFile<Image2DType>("TestVTK2ITK_2DEllipse.png", labelMapToLabelImageFilter->GetOutput() );

	Image3DType::Pointer img = Image3DType::New();
	Convert2DITKImageToFlat3D<unsigned short, unsigned short>(labelMapToLabelImageFilter->GetOutput(), img, HORIZONTAL, 0);

	WriteITKImageToFile<Image3DType>("TestVTK2ITK_2DEllipse.nii", img );


}



//
void TestGraft() {
	std::cout<<"\n\n Entering TestGraft"<<std::endl;
	vtkSmartPointer<vtkSphereSource> m_sphereSource = vtkSmartPointer<vtkSphereSource>::New();
	vtkSmartPointer<vtkSphereSource> m_sphereSource2= vtkSmartPointer<vtkSphereSource>::New();

	m_sphereSource->SetPhiResolution(25); m_sphereSource->SetThetaResolution(12); m_sphereSource->SetCenter(2,1.5,0); m_sphereSource->SetRadius(10.3); m_sphereSource->Update();
	m_sphereSource2->SetPhiResolution(25);m_sphereSource2->SetThetaResolution(12);m_sphereSource2->SetCenter(2,1.5,0);m_sphereSource2->SetRadius(5);   m_sphereSource2->Update();

	typedef itk::Image<unsigned char, 3>		Image3DType;
	typedef itk::LabelObject<unsigned char, 3>	LabelObject3DType;
	typedef itk::LabelMap<LabelObject3DType>	LabelMap3DType;
	typedef itk::LabelMapToLabelImageFilter< LabelMap3DType, Image3DType> LabelMapToLabelImageFilterType;
	LabelObject3DType::Pointer labelObject = LabelObject3DType::New(); labelObject->SetLabel(2);
	LabelMap3DType::Pointer labelMap = LabelMap3DType::New();
	LabelMap3DType::Pointer labelMap2= LabelMap3DType::New();

	vnl_vector<double> bbox;
	VTKPolyDataToLabelMap<LabelMap3DType>(m_sphereSource->GetOutput(), labelMap, labelMap->GetSpacing());
	LabelMapToLabelImageFilterType::Pointer labelMapToLabelImageFilter = LabelMapToLabelImageFilterType::New();
	labelMapToLabelImageFilter->SetInput(labelMap); labelMapToLabelImageFilter->Update();

	vnl_vector<double> bbox2;
	VTKPolyDataToLabelMap<LabelMap3DType>(m_sphereSource2->GetOutput(), labelMap2, labelMap2->GetSpacing());
	LabelMapToLabelImageFilterType::Pointer labelMapToLabelImageFilter2 = LabelMapToLabelImageFilterType::New();
	labelMapToLabelImageFilter2->SetInput(labelMap2); labelMapToLabelImageFilter2->Update();

	std::cout<<"number of objects in labelMap: "<<labelMap->GetNumberOfLabelObjects()<<std::endl;
	std::cout<<"number of lines in 1st object: "<<labelMap->GetNthLabelObject(0)->GetNumberOfLines()<<std::endl;

	std::cout<<"\nnumber of objects in labelMap2: "<<labelMap2->GetNumberOfLabelObjects()<<std::endl;
	std::cout<<"number of lines in 1st object: "<<labelMap2->GetNthLabelObject(0)->GetNumberOfLines()<<std::endl;

	std::cout<<"\n Graft 1 on 2, and add an empty object in 2"<<std::endl;
	labelMap2->Graft( labelMap );
	labelMap2->ClearLabels();
	labelMap2->AddLabelObject( labelObject );

	std::cout<<"\nnumber of objects in labelMap: "<<labelMap->GetNumberOfLabelObjects()<<std::endl;
	std::cout<<"number of lines in 1st object: "<<labelMap->GetNthLabelObject(0)->GetNumberOfLines()<<std::endl;

	std::cout<<"\nnumber of objects in labelMap2: "<<labelMap2->GetNumberOfLabelObjects()<<std::endl;
	std::cout<<"number of lines in 1st object: "<<labelMap2->GetNthLabelObject(0)->GetNumberOfLines()<<std::endl;

}


//
void TestGetSurroundingPixels() {
	std::cout<<"\n\n Entering TestGetSurroundingPixels"<<std::endl;
	vtkSmartPointer<vtkSphereSource> m_sphereSource = vtkSmartPointer<vtkSphereSource>::New();
	m_sphereSource->SetPhiResolution(25); m_sphereSource->SetThetaResolution(12); m_sphereSource->SetCenter(2,1.5,0); m_sphereSource->SetRadius(10.3); m_sphereSource->Update();

	typedef itk::Image<unsigned char, 3>		Image3DType;
	typedef itk::LabelObject<unsigned char, 3>	LabelObject3DType;
	typedef itk::LabelMap<LabelObject3DType>	LabelMap3DType;
	typedef itk::LabelMapToBinaryImageFilter< LabelMap3DType, Image3DType> LabelMapToBinaryImageFilterType;
	LabelMapToBinaryImageFilterType::Pointer mapToImageFilter = LabelMapToBinaryImageFilterType::New(); mapToImageFilter->SetForegroundValue(1);

	LabelMap3DType::Pointer labelMap = LabelMap3DType::New();
	VTKPolyDataToLabelMap<LabelMap3DType>(m_sphereSource->GetOutput(), labelMap, labelMap->GetSpacing());

	LabelMap3DType::Pointer labelMapLarge = LabelMap3DType::New();
	LabelMap3DType::RegionType region = labelMap->GetLargestPossibleRegion();
	LabelMap3DType::SizeType size = region.GetSize(); size[0] +=5; size[1] +=5; size[2] +=5; region.SetSize(size);
	labelMapLarge->SetSpacing( labelMap->GetSpacing() );
	labelMapLarge->SetOrigin( labelMap->GetOrigin() );
	labelMapLarge->SetRegions(region);
	
	InsertSingleObjectLabelMapIntoAnother_FullyInside<LabelMap3DType, LabelMap3DType>(labelMap, labelMapLarge, 1);
	
	mapToImageFilter->SetInput(labelMapLarge); mapToImageFilter->Update();
	WriteITKImageToFile<Image3DType>("TestGetSurroundingPixels_1.nii", mapToImageFilter->GetOutput() );
	
	//binary dilation of this object:
	typedef itk::BinaryBallStructuringElement<unsigned char, 3 > StructuringElementType;
	StructuringElementType structuringElement; 
	structuringElement.SetRadius( 2 ); 
	structuringElement.CreateStructuringElement();
	typedef itk::BinaryDilateImageFilter<Image3DType,Image3DType,StructuringElementType > BinaryDilateFilterType;

	BinaryDilateFilterType::Pointer binaryDilateFilter = BinaryDilateFilterType::New();
	binaryDilateFilter->SetKernel( structuringElement ); binaryDilateFilter->SetForegroundValue(1);
	binaryDilateFilter->SetInput( mapToImageFilter->GetOutput() );

	binaryDilateFilter->Update();//mapToImageFilter->SetInput(binaryDilateFilter->GetOutput()); mapToImageFilter->Update();
	WriteITKImageToFile<Image3DType>("TestGetSurroundingPixels_2_dilated.nii", binaryDilateFilter->GetOutput() );

	//mask out the initial object
	typedef itk::LabelMapMaskImageFilter<LabelMap3DType, Image3DType> LabelMapMaskImageFilterType;
	LabelMapMaskImageFilterType::Pointer maskFilter = LabelMapMaskImageFilterType::New();
	maskFilter->SetNegated(true); maskFilter->SetLabel(1);
	maskFilter->SetInput1( labelMapLarge );
	maskFilter->SetInput2( binaryDilateFilter->GetOutput() );

	maskFilter->Update();//mapToImageFilter->SetInput(maskFilter->GetOutput()); mapToImageFilter->Update();
	WriteITKImageToFile<Image3DType>("TestGetSurroundingPixels_3_surrounding.nii", maskFilter->GetOutput() );

	//
	typedef itk::BinaryImageToLabelMapFilter<Image3DType, LabelMap3DType> LabelMapToImageFilterType;
	LabelMapToImageFilterType::Pointer labelMapToImageFilter = LabelMapToImageFilterType::New();
	labelMapToImageFilter->SetInput( maskFilter->GetOutput() );
	labelMapToImageFilter->SetInputForegroundValue(1);
	labelMapToImageFilter->SetOutputBackgroundValue(0);
	labelMapToImageFilter->Update();

	std::cout<<"number of objects in the output: "<<labelMapToImageFilter->GetOutput()->GetNumberOfLabelObjects()<<std::endl;

}


//
void TestPixelSet() {
	std::cout<<"\n\n Entering TestPixelSet"<<std::endl;
	//Generate a VTK Sphere
	vtkSmartPointer<vtkSphereSource> m_sphereSource = vtkSmartPointer<vtkSphereSource>::New();
	m_sphereSource->SetPhiResolution(25);
	m_sphereSource->SetThetaResolution(12);
	m_sphereSource->SetCenter(2,1.5,0);
	m_sphereSource->SetRadius(10.3);
	m_sphereSource->Update();

	typedef itk::Image<unsigned char, 3>		Image3DType;
	typedef itk::LabelObject<unsigned char, 3>	LabelObject3DType;
	typedef itk::LabelMap<LabelObject3DType>	LabelMap3DType;

	LabelObject3DType::Pointer labelObject = LabelObject3DType::New();
	LabelMap3DType::Pointer labelMap = LabelMap3DType::New();
clock_t t0 = clock();
	VTKPolyDataToLabelMap<LabelMap3DType>(m_sphereSource->GetOutput(), labelMap, labelMap->GetSpacing());
std::cout<<"time to convert the object: "<<(clock()-t0)/((double)CLOCKS_PER_SEC)<<" s."<<std::endl;
	typedef itk::LabelMapToLabelImageFilter< LabelMap3DType, Image3DType> LabelMapToLabelImageFilterType;
	LabelMapToLabelImageFilterType::Pointer labelMapToLabelImageFilter = LabelMapToLabelImageFilterType::New();
	labelMapToLabelImageFilter->SetInput(labelMap);
	labelMapToLabelImageFilter->Update();

	WriteITKImageToFile<Image3DType>("TestVTK2ITK_Sphere.nii", labelMapToLabelImageFilter->GetOutput() );


	std::cout<<"new PixelSet"<<std::endl;
	t0 = clock();
	ObjectPixelSet::Pointer pixelSet = ObjectPixelSet::New();
	ConvertSingleObjectLabelMapToPixelSet<LabelMap3DType>(labelMap, pixelSet); //, labelMap->GetOrigin(), labelMap->GetLargestPossibleRegion().GetSize(), labelMap->GetSpacing()
	std::cout<<"time to convert the object into a pixel set: "<<(clock()-t0)/((double)CLOCKS_PER_SEC)<<" s. ; nb pixels = "<<pixelSet->GetNumberOfPixels()<<std::endl;

	std::cout<<"new PixelList..."<<std::endl;
	ObjectPixelSet::SetType pixelList;
	t0 = clock();
	ConvertSingleObjectLabelMapToPixelList<LabelMap3DType>(labelMap, pixelList);
	std::cout<<"time to convert the object: "<<(clock()-t0)/((double)CLOCKS_PER_SEC)<<" s. ; nb pixels = "<<pixelList.size()<<std::endl;

	ObjectPixelSet::SetType *pixelList2 = pixelSet->GetPixelSet();
	for (unsigned i=0 ; i<10 ; i++) { std::cout<<"list1: "<<pixelList[i]<<", list2: "<<(*pixelList2)[i]<<std::endl; }

	std::cout<<"total number of pixels in the grid: "<<labelMap->GetLargestPossibleRegion().GetNumberOfPixels()<<std::endl;
	
	vnl_vector<long> listPxl(pixelSet->GetNumberOfPixels());
	unsigned i=0;
	for (ObjectPixelSet::IteratorType it=pixelSet->GetPixelSet()->begin() ; it!=pixelSet->GetPixelSet()->end()  ; it++) { listPxl(i) = *it; i++; }
	WriteVNLVectorToTxtFile<long>("TestVTK2ITK_Sphere_listPixels.txt",listPxl);
}

//
#include <itkGaussianImageSource.h>
//this class draws a gaussian profile, not a random noise.
void TestGaussianSource() {
	std::cout<<"\n\nEntering TestGaussianSource()"<<std::endl;

	typedef itk::Image<unsigned char, 2> ImageType;
	typedef itk::GaussianImageSource<ImageType> GaussianImageSourceType;

	GaussianImageSourceType::Pointer gaussSource = GaussianImageSourceType::New();
	ImageType::SizeType size;
	size[0] = 20; size[1] = 20;
	gaussSource->SetSize(size);
	std::cout<<"updating the gaussian source..."<<std::endl;
	gaussSource->Update();
	std::cout<<"updating the gaussian source... OK!"<<std::endl;
	
	Write2DGreyLevelRescaledImageToFile<ImageType>("TestGaussianSource.png", gaussSource->GetOutput() );
}
//
#include "itkRandomImageSource.h"
void TestRandomImageSource() {
	std::cout<<"\n\nEntering TestRandomImageSource()"<<std::endl;

	typedef itk::Image<unsigned char, 2> ImageType;
	typedef itk::RandomImageSource<ImageType> RandomImageSourceType;

	RandomImageSourceType::Pointer noiseSource = RandomImageSourceType::New();
	noiseSource->SetMin(0);
	noiseSource->SetMax(20);
	std::cout<<"updating the noise source..."<<std::endl;
	noiseSource->Update();
	std::cout<<"updating the noise source... OK!"<<std::endl;
	
	Write2DGreyLevelRescaledImageToFile<ImageType>("TestNoiseSource.png", noiseSource->GetOutput() );

	
}
//
//
int main(int argc, char** argv) {

	try {

		//TestLabelObject();
		//TestStructuringElementToLabelObjectAndSurroundings();

		//TestVTKToITKLabelMapConversion();

		//TestVTKToITK2DLabelMapConversion();

		//TestGraft(); //B->Graft( A ); copies the contents of A into B

		//TestGetSurroundingPixels();

		//TestPixelSet();

		//TestGaussianSource();
		TestRandomImageSource();
	}
	catch (DeformableModelException& e) {
		std::cout << "Exception occured while running the test cases" << std::endl;
		std::cout << e.what() << std::endl;
	}
	return 1;
}
