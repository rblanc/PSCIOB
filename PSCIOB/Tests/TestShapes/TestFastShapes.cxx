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

#pragma once


#include <iostream>
#include <typeinfo>

#include <vtkDataSet.h>

#include "ITKUtils.h"
#include "VTKUtils.h"
#include "GeneralUtils.h"

#include "Fast2DDisk.h"
#include "Fast2DEllipse.h"


using namespace psciob;

//
void fastDisks() {
	std::cout<<"\n\nfastDisks: draw a few disks using various parameters"<<std::endl;

	Fast2DDisk::Pointer object = Fast2DDisk::New();

	std::cout<<" WARNING: using 'Fast'Objects, the 'image' on which they are drawn (with GetObjectAsBinaryImage) is defined \n \
	 as the smallest grid of spacing 1 which contains the object when first generated \n \
	 This is never updated automatically afterwards, so don't forget to modify it as requested... calling SetPixelSetImageInformation()\n"<<std::endl;

	std::cout<<"here, I generate a first object which is large, so I don't update anything later"<<std::endl;
	vnl_vector<double> params(3);
	params(0) = 0; params(1) = 0; params(2) = 15.63045; object->SetParameters(params);

	object->PrintInfo();
	std::cout<<"  physical bounding box of fastDisk_0: "<<object->GetPhysicalBoundingBox()<<std::endl;
	WriteMirrorPolyDataToFile( "fastDisk_0.vtk", object->GetObjectAsVTKPolyData());
	Write2DGreyLevelRescaledImageToFile<Fast2DDisk::BinaryImageType>("fastDisk_0.png", object->GetObjectAsBinaryImage());

	params(0) = 0; params(1) = 0; params(2) = 5; object->SetParameters(params);
	std::cout<<"   params: "<<params<<std::endl;
	std::cout<<"  physical bounding box of fastDisk_1: "<<object->GetPhysicalBoundingBox()<<std::endl;
	Write2DGreyLevelRescaledImageToFile<Fast2DDisk::BinaryImageType>("fastDisk_1.png", object->GetObjectAsBinaryImage());

	params(0) = 0.5; params(1) = 0; params(2) = 5; object->SetParameters(params);
	std::cout<<"   params: "<<params<<std::endl;
	std::cout<<"is the image still uptodate? "<<object->IsObjectBinaryImageUpToDate()<<std::endl;
	Write2DGreyLevelRescaledImageToFile<Fast2DDisk::BinaryImageType>("fastDisk_2.png", object->GetObjectAsBinaryImage());

	params(0) = 0.5; params(1) = 1; params(2) = 5; object->SetParameters(params);
	std::cout<<"   params: "<<params<<std::endl;
	std::cout<<"is the image still uptodate? "<<object->IsObjectBinaryImageUpToDate()<<std::endl;
	Write2DGreyLevelRescaledImageToFile<Fast2DDisk::BinaryImageType>("fastDisk_3.png", object->GetObjectAsBinaryImage());

	params(0) = 0.5; params(1) = 0.5; params(2) = 5.2; object->SetParameters(params);
	std::cout<<"   params: "<<params<<std::endl;
	Write2DGreyLevelRescaledImageToFile<Fast2DDisk::BinaryImageType>("fastDisk_4.png", object->GetObjectAsBinaryImage());

	params(0) = 0.5; params(1) = 0.5; params(2) = 15.2; object->SetParameters(params);
	std::cout<<"   params: "<<params<<std::endl;
	Write2DGreyLevelRescaledImageToFile<Fast2DDisk::BinaryImageType>("fastDisk_5.png", object->GetObjectAsBinaryImage());

}

//
void fastEllipses() {
	std::cout<<"\n\nfastEllipses: draw a few ellipses using various parameters"<<std::endl;

	Fast2DEllipse::Pointer object = Fast2DEllipse::New();

	vnl_vector<double> params(5);//centre, petit axe, rapport petit/grand axe in [0,1], orientation
	params(0) = 3; params(1) = -3; params(2) = 15.63045; params(3) = 1; params(4) = 0; object->SetParameters(params);

	vnl_vector<double> center; vnl_matrix<double> inertia;

	vnl_vector<double> v1(3), v2(3);
	v1(0) = 0.5; v1(1) = 2;   v1(2) = -1;
	v2(0) = -1;  v2(1) =-0.5; v2(2) = 2;
	std::cout<<"test... v1 = "<<v1<<", v2 = "<<v2<<" ; outer_product = \n"<<outer_product(v1, v2)<<std::endl;

	object->PrintInfo();
	WriteMirrorPolyDataToFile( "fastEllipse_0.vtk", object->GetObjectAsVTKPolyData());
	Write2DGreyLevelRescaledImageToFile<Fast2DEllipse::BinaryImageType>("fastEllipse_0.png", object->GetObjectAsBinaryImage());

	object->GetObjectCenterAndInertiaMatrix(center, inertia);
	std::cout<<"center of disk...: "<<center<<", inertia matrix :\n"<<inertia<<std::endl;
	std::cout<<"image origin: "<<object->GetImageOrigin()<<std::endl;

	params(0) = 0; params(1) = 0; params(2) = 5; params(3) = 0.71; params(4) = PI/4.0; object->SetParameters(params);
	std::cout<<"   params: "<<params<<std::endl;
	WriteMirrorPolyDataToFile( "fastEllipse_1.vtk", object->GetObjectAsVTKPolyData());
	Write2DGreyLevelRescaledImageToFile<Fast2DEllipse::BinaryImageType>("fastEllipse_1.png", object->GetObjectAsBinaryImage());

	params(0) = 0; params(1) = 0; params(2) = 5; params(3) = 0.21; params(4) = PI/2.0; object->SetParameters(params);
	std::cout<<"   params: "<<params<<std::endl;
	Write2DGreyLevelRescaledImageToFile<Fast2DEllipse::BinaryImageType>("fastEllipse_2.png", object->GetObjectAsBinaryImage());

	params(0) = 0.5; params(1) = 0.5; params(2) = 5; params(3) = 0.2; params(4) = -0.1564; object->SetParameters(params);
	std::cout<<"   params: "<<params<<std::endl;
	Write2DGreyLevelRescaledImageToFile<Fast2DEllipse::BinaryImageType>("fastEllipse_3.png", object->GetObjectAsBinaryImage());

	params(0) = 0.5; params(1) = 0.5; params(2) = 5.2; params(3) = 0.25; params(4) = 0; object->SetParameters(params);
	std::cout<<"   params: "<<params<<std::endl;
	Write2DGreyLevelRescaledImageToFile<Fast2DEllipse::BinaryImageType>("fastEllipse_4.png", object->GetObjectAsBinaryImage());

	params(0) = 0.5; params(1) = 0.5; params(2) = 9.1; params(3) = 2.5; params(4) = 0.564; object->SetParameters(params);
	std::cout<<"   params: "<<params<<std::endl;
	Write2DGreyLevelRescaledImageToFile<Fast2DEllipse::BinaryImageType>("fastEllipse_5.png", object->GetObjectAsBinaryImage());

	object->GetObjectCenterAndInertiaMatrix(center, inertia);
	std::cout<<"center of ellipse: "<<center<<", inertia matrix :\n"<<inertia<<std::endl;

}
int main(int argc, char** argv) {
	fastDisks();
	fastEllipses();
}