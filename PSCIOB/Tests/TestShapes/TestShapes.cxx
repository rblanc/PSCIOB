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

#include <vtkDataSet.h>

#include "GeneralUtils.h"

#include "PoseTransformedBinaryShape.h"

#include "shape2DDisk.h"
#include "shape2DEllipse.h"

#include "TranslationScale2DTransform.h"
#include "Similarity2DTransform.h"


#include "shape2DRectangle.h"
#include "Rigid2DTransform.h"
#include "shape3DCuboid.h"
#include "Similarity3DTransform.h"
#include "shape3DSphere.h"
#include "Translation3DTransform.h"
#include "Translation2DTransform.h"

#include "shape3DCylinder.h"

using namespace psciob;

//disks with translation & scale
void disks() {
	std::cout<<"\n\n\n********DISKS: draw a few unit disks with translation & scale, using various parameters"<<std::endl;

	PoseTransformedBinaryShape<2>::Pointer object = PoseTransformedBinaryShape<2>::New();
	TranslationScale2DTransform::Pointer transf = TranslationScale2DTransform::New();
	shape2DDisk::Pointer shape = shape2DDisk::New();

	vnl_vector<double> pp(3); pp(0)=0; pp(1)=0; pp(2) = 5.63045;
	transf->SetParameters(pp);

	object->SetShapeAndTransform(shape, transf);

	std::cout<<"Writing vtk representation of a disk: disk_0.vtk"<<std::endl;
	WriteMirrorPolyDataToFile("disk_0.vtk", object->GetObjectAsVTKPolyData());
	std::cout<<"Writing it as a binary image: disk_0.png"<<std::endl;
	Write2DGreyLevelRescaledImageToFile<shape2DDisk::BinaryImageType>("disk_0.png", object->GetObjectAsBinaryImage());

	vnl_vector<double> params(3);
	params(0) = 5; params(1) = 0; params(2) = 5; 
	std::cout<<"\n\n set parameters of a new object"<<std::endl;
	object->SetParameters(params);
	std::cout<<"   params: "<<params<<std::endl;
	Write2DGreyLevelRescaledImageToFile<shape2DDisk::BinaryImageType>("disk_1.png", object->GetObjectAsBinaryImage());

	params(0) = 0.5; params(1) = 3; params(2) = 5; object->SetParameters(params);
	std::cout<<"   params: "<<params<<std::endl;
	Write2DGreyLevelRescaledImageToFile<shape2DDisk::BinaryImageType>("disk_2.png", object->GetObjectAsBinaryImage());

	params(0) = 0.5; params(1) = 4.5; params(2) = 5; object->SetParameters(params);
	std::cout<<"   params: "<<params<<std::endl;
	Write2DGreyLevelRescaledImageToFile<shape2DDisk::BinaryImageType>("disk_3.png", object->GetObjectAsBinaryImage());

	params(0) = 2.5; params(1) = 0.5; params(2) = 5.2; object->SetParameters(params);
	std::cout<<"   params: "<<params<<std::endl;
	Write2DGreyLevelRescaledImageToFile<shape2DDisk::BinaryImageType>("disk_4.png", object->GetObjectAsBinaryImage());

	params(0) = 0.5; params(1) = 5.5; params(2) = 15.2; object->SetParameters(params);
	std::cout<<"   params: "<<params<<std::endl;
	Write2DGreyLevelRescaledImageToFile<shape2DDisk::BinaryImageType>("disk_5.png", object->GetObjectAsBinaryImage());
	std::cout<<"********EXITING DISKS *************************************************************************"<<std::endl;
}

//ellipse with similarity transform
void ellipses() {
	std::cout<<"\n\n\n*******ELLIPSES: draw a few unit ellipses with similarity transforms, using various parameters"<<std::endl;

	shape2DEllipse::Pointer shape = shape2DEllipse::New();
	Similarity2DTransform::Pointer transf = Similarity2DTransform::New();
	PoseTransformedBinaryShape<2>::Pointer object = PoseTransformedBinaryShape<2>::New();

	vnl_vector<double> sp(1); sp(0) = 2; shape->SetParameters(sp);
	vnl_vector<double> pp(4); pp(0)=0; pp(1)=0; pp(2)=0; pp(3) = 5;
	transf->SetParameters(pp);

	object->SetShapeAndTransform(shape, transf);

	std::cout<<"   shape name: "<<object->GetShape()->GetClassName()<<" ; nb of object parameters: "<<object->GetNumberOfParameters()<<std::endl;
	std::cout<<"object parameters: "<<object->GetParameters()<<std::endl;
	WriteMirrorPolyDataToFile("ellipse_0.vtk", object->GetObjectAsVTKPolyData());	
	Write2DGreyLevelRescaledImageToFile<shape2DEllipse::BinaryImageType>("ellipse_0.png", object->GetObjectAsBinaryImage());

	std::cout<<"\n\n\n MODIFYING THE OBJECT"<<std::endl;

	vnl_vector<double> params(5);
	params(0) = 3; params(1) = 0; params(2) = PI/3.0; params(3) = 5;params(4) = 1;
	object->SetParameters(params);
	std::cout<<"   params: "<<object->GetParameters()<<std::endl;
	WriteMirrorPolyDataToFile("ellipse_1.vtk", object->GetObjectAsVTKPolyData());	
	Write2DGreyLevelRescaledImageToFile<shape2DEllipse::BinaryImageType>("ellipse_1.png", object->GetObjectAsBinaryImage());

	params(0) = 0.5; params(1) = 4; params(2) = 0; params(3) = 2.354;params(4) = 2; object->SetParameters(params);
	std::cout<<"   params: "<<params<<std::endl;
	Write2DGreyLevelRescaledImageToFile<shape2DEllipse::BinaryImageType>("ellipse_2.png", object->GetObjectAsBinaryImage());

	params(0) = 0.5; params(1) = 2.5; params(2) = 0; params(3) = 5;params(4) = 3; object->SetParameters(params);
	std::cout<<"   params: "<<params<<std::endl;
	Write2DGreyLevelRescaledImageToFile<shape2DEllipse::BinaryImageType>("ellipse_3.png", object->GetObjectAsBinaryImage());

	params(0) = 3.5; params(1) = 0.5; params(2) = 0.25; params(3) = 5.12;params(4) = 2; object->SetParameters(params);
	std::cout<<"   params: "<<params<<std::endl;
	Write2DGreyLevelRescaledImageToFile<shape2DEllipse::BinaryImageType>("ellipse_4.png", object->GetObjectAsBinaryImage());

	params(0) = 0; params(1) = 1; params(2) = -0.5; params(3) = 5;params(4) = 1.2; object->SetParameters(params);
	std::cout<<"   params: "<<params<<std::endl;
	Write2DGreyLevelRescaledImageToFile<shape2DEllipse::BinaryImageType>("ellipse_5.png", object->GetObjectAsBinaryImage());

	std::cout<<"\n\n\n   EXITING FROM ellipses() \n\n\n"<<std::endl;

	std::cout<<"nb of references of object: "<<object->GetReferenceCount()<<std::endl;
	std::cout<<"nb of references of its shape: "<<object->GetShape()->GetReferenceCount()<<std::endl;
	std::cout<<"nb of references of its transform: "<<object->GetTransform()->GetReferenceCount()<<std::endl;
	std::cout<<"\n\n\n*******Exit from ELLIPSES"<<std::endl;
}

//rectangle with similarity ; cuboid with similarity ; sphere with translation ; cylinder with translation
//void simpleObjectsExample() {
	//std::cout<<"\n\n\nsimpleObjectsExample: \n1:single rectangle with similarity transform\n2:3D cuboid with similarity"<<std::endl;

	//PoseTransformedBinaryShape<2>::Pointer object = PoseTransformedBinaryShape<2>::New();
	//shape2DRectangle::Pointer shape = shape2DRectangle::New();
	//Similarity2DTransform::Pointer transf = Similarity2DTransform::New();

	//vnl_vector<double> sp(2); sp(0) = 5.63045; sp(1) = 2.12;	//->2nd parameter correspond to the elongation (ratio long/small lengths)
	//shape->SetParameters(sp);
	//vnl_vector<double> pp(4); pp(0)=1; pp(1)=50; pp(2)=-25; pp(3)=0.85;
	//transf->SetParameters(pp);

	//object->SetShapeAndTransform(shape, transf);

	//std::cout<<"   shape name: "<<object->GetShape()->GetClassName()<<" ; nb of object parameters: "<<object->GetNumberOfParameters()<<std::endl;
	//
	//WriteMirrorPolyDataToFile("simpleObjectsExample_rectangle.vtk", object->GetObjectAsVTKPolyData());

	//object->SetImageSpacing(0.127951);
	//std::cout<<"   spacing is set"<<std::endl;
	//typedef itk::Image<unsigned char, 2> ImageType;
	//WriteITKImageToFile<ImageType>("simpleObjectsExample_rectangle.png", object->GetObjectAsBinaryImage());
	//typedef itk::Image<unsigned char, 3> Image3DType;
	//Image3DType::Pointer image3D = Image3DType::New();
	//Convert2DITKImageToFlat3D<unsigned char, unsigned char>(object->GetObjectAsBinaryImage(), image3D, CORONAL);
	//WriteITKImageToFile<Image3DType>("simpleObjectsExample_rectangle.nii", image3D);

	//std::cout<<"\n\n\n *** Now the same for a 3D cuboid... ++ play with the parameters !! ***"<<std::endl;

	//shape3DCuboid::Pointer shape2 = shape3DCuboid::New();
	//Similarity3DTransform::Pointer transform2 = Similarity3DTransform::New();
	//std::cout<<"created the shape & transform"<<std::endl;
	//PoseTransformedBinaryShape<3>::Pointer object2 = PoseTransformedBinaryShape<3>::New();
	//std::cout<<"created object"<<std::endl;
	//vnl_vector<double> sp2(3); sp2(0) = 5.63045; sp2(1) = 6.42235; sp2(2) = 2.5;
	//shape2->SetParameters(sp2);std::cout<<"set shape params"<<std::endl;
	//vnl_vector<double> pp2(7); pp2(0)=1; pp2(1)=50; pp2(2)=-25; pp2(3)=0; pp2(4)=0; pp2(5)=0; pp2(6) = 1;
	//vnl_vector<double> allparams;
	//std::cout<<"number of transform params: "<<transform2->GetNumberOfParameters()<<std::endl;
	//transform2->SetParameters(pp2);std::cout<<"set transform params"<<std::endl;
	//object2->SetShapeAndTransform(shape2, transform2);
	//std::cout<<"set transform and shape to object..."<<std::endl;

	//std::cout<<"   shape name: "<<object2->GetShape()->GetClassName()<<" ; nb of object parameters: "<<object2->GetNumberOfParameters()<<" ; parameters: "<<object2->GetParameters()<<std::endl;		
	//WriteMirrorPolyDataToFile("simpleObjectsExample_cuboid_0.vtk", object2->GetObjectAsVTKPolyData());
	//std::cout<<"bounding box: "<<object2->GetPhysicalBoundingBox()<<std::endl;
	//object2->SetImageSpacing(1.5);
	//std::cout<<"   spacing is set"<<std::endl;
	//WriteITKImageToFile<Image3DType>("simpleObjectsExample_cuboid_0.nii", object2->GetObjectAsBinaryImage());

	////NOW, MODIFY THE PARAMETERS...
	//allparams = object2->GetParameters();
	//std::cout<<"initial parameters: "<<allparams<<std::endl;
	//allparams(0)+=2.123;
	//std::cout<<"setting new parameters: "<<allparams<<std::endl;
	//object2->SetParameters(allparams);
	//std::cout<<"getting them back     : "<<object2->GetParameters()<<std::endl;
	//std::cout<<"is the image still uptodate? "<<object2->IsObjectBinaryImageUpToDate()<<std::endl;
	//std::cout<<"is the vtk mesh still uptodate? "<<object2->IsObjectPolyDataUpToDate()<<std::endl;
	//WriteITKImageToFile<Image3DType>("simpleObjectsExample_cuboid_1.nii", object2->GetObjectAsBinaryImage());
	//WriteMirrorPolyDataToFile("simpleObjectsExample_cuboid_1.vtk", object2->GetObjectAsVTKPolyData());

	//std::cout<<"now, an interger grid translation in another direction..."<<std::endl;
	//allparams(1)+=3;
	//std::cout<<"setting new parameters: "<<allparams<<std::endl;
	//object2->SetParameters(allparams);
	//std::cout<<"getting them back     : "<<object2->GetParameters()<<std::endl;
	//std::cout<<"is the image still uptodate? "<<object2->IsObjectBinaryImageUpToDate()<<std::endl;
	//std::cout<<"is the vtk mesh still uptodate? "<<object2->IsObjectPolyDataUpToDate()<<std::endl;
	//WriteITKImageToFile<Image3DType>("simpleObjectsExample_cuboid_2.nii", object2->GetObjectAsBinaryImage());
	//WriteMirrorPolyDataToFile("simpleObjectsExample_cuboid_2.vtk", object2->GetObjectAsVTKPolyData());

	//std::cout<<"now, just modify the shape parameters"<<std::endl;
	//allparams(9) = 12;
	//std::cout<<"setting new parameters: "<<allparams<<std::endl;
	//object2->SetParameters(allparams);
	//std::cout<<"getting them back     : "<<object2->GetParameters()<<std::endl;
	//WriteITKImageToFile<Image3DType>("simpleObjectsExample_cuboid_3.nii", object2->GetObjectAsBinaryImage());
	//WriteMirrorPolyDataToFile("simpleObjectsExample_cuboid_3.vtk", object2->GetObjectAsVTKPolyData());

	//std::cout<<"now, just rotate it around x, scaled it *2"<<std::endl;
	//allparams(9) = 4; allparams(3) = PI/3.0; allparams(6) = 2;
	//std::cout<<"setting new parameters: "<<allparams<<std::endl;
	//object2->SetParameters(allparams);
	//std::cout<<"getting them back     : "<<object2->GetParameters()<<std::endl;
	//WriteITKImageToFile<Image3DType>("simpleObjectsExample_cuboid_4.nii", object2->GetObjectAsBinaryImage());
	//WriteMirrorPolyDataToFile("simpleObjectsExample_cuboid_4.vtk", object2->GetObjectAsVTKPolyData());




	//std::cout<<"\n\n\nNow the same for a 3D sphere..."<<std::endl;

	//shape3DSphere::Pointer shape3 = shape3DSphere::New();
	//Translation3DTransform::Pointer transform3 = Translation3DTransform::New();

	//PoseTransformedBinaryShape<3>::Pointer object3 = PoseTransformedBinaryShape<3>::New();
	//shape3->SetRadius(2.25);
	//vnl_vector<double> pp3(3); pp3(0)=1; pp3(1)=-4; pp3(2)=-2.5; 
	//transform3->SetParameters(pp3);std::cout<<"translation set"<<std::endl;
	//shape3->SetVTKPolyDataResolution(15,15);
	//object3->SetShapeAndTransform(shape3, transform3);
	//std::cout<<"   shape name: "<<object3->GetShape()->GetClassName()<<" ; nb of object parameters: "<<object3->GetNumberOfParameters()<<" ; parameters: "<<object3->GetParameters()<<std::endl;		
	//WriteMirrorPolyDataToFile("simpleObjectsExample_sphere.vtk", object3->GetObjectAsVTKPolyData());
	//std::cout<<"bounding box: "<<object3->GetPhysicalBoundingBox()<<std::endl;
	//object3->SetImageSpacing(0.25);
	//WriteITKImageToFile<Image3DType>("simpleObjectsExample_sphere.nii", object3->GetObjectAsBinaryImage());

	////try integer grid translation
	//pp3(0)=-0.5; pp3(1)=-1; pp3(2)=-2.5; 
	//transform3->SetParameters(pp3);
	//object3->GetObjectAsBinaryImage();

	//std::cout<<"\nNow the same for a 3D cylinder..."<<std::endl;

	//shape3DCylinder::Pointer shape4 = shape3DCylinder::New();
	//Translation3DTransform::Pointer transform4 = Translation3DTransform::New();

	//PoseTransformedBinaryShape<3>::Pointer object4 = PoseTransformedBinaryShape<3>::New();
	//shape4->SetRadius(5);
	//shape4->SetHeight(25);
	//vnl_vector<double> pp4(3); pp4(0)=1; pp4(1)=-4; pp4(2)=-2.5; 
	//transform4->SetParameters(pp4);std::cout<<"translation set"<<std::endl;
	//shape4->SetVTKPolyDataResolution(15);
	//object4->SetShapeAndTransform(shape4, transform4);
	//std::cout<<"   shape name: "<<object4->GetShape()->GetClassName()<<" ; nb of object parameters: "<<object4->GetNumberOfParameters()<<" ; parameters: "<<object4->GetParameters()<<std::endl;		
	//WriteMirrorPolyDataToFile("simpleObjectsExample_cylinder.vtk", object4->GetObjectAsVTKPolyData());
	//WriteMirrorPolyDataToSTLFile("simpleObjectsExample_cylinder.stl", object4->GetObjectAsVTKPolyData());
	//std::cout<<"bounding box: "<<object4->GetPhysicalBoundingBox()<<std::endl;
	//object4->SetImageSpacing(0.25);
	//WriteITKImageToFile<Image3DType>("simpleObjectsExample_cylinder.nii", object4->GetObjectAsBinaryImage());


	//std::cout<<"\nsimpleObjectsExample: EXIT, writing files simpleObjectsExample_{rectangle/cuboid/sphere/cylinder}.{nii/vtk} "<<std::endl;
//}

//
void LabelMapExample() {
	std::cout<<"\n\n LabelMapExample: accesses an object as a set of pixel, and play a bit with the output"<<std::endl;

	Similarity2DTransform::Pointer transf = Similarity2DTransform::New();
	shape2DRectangle::Pointer shape = shape2DRectangle::New();

	vnl_vector<double> sp(1); sp(0) = 2.5;
	shape->SetParameters(sp);
	vnl_vector<double> pp(4); pp(0)=1; pp(1)=50; pp(2)=-25; pp(3)=5.12;
	transf->SetParameters(pp);

	PoseTransformedBinaryShape<2>::Pointer object = PoseTransformedBinaryShape<2>::New();
	object->SetShapeAndTransform(shape, transf);

	typedef PoseTransformedBinaryShape<2>::LabelMapType LabelMapType;
	LabelMapType *ptr = object->GetObjectAsLabelMap();
	std::cout<<"image size: "<<ptr->GetLargestPossibleRegion().GetSize()[0]<<", "<<ptr->GetLargestPossibleRegion().GetSize()[1]<<std::endl;
	std::cout<<"image spacing: "<<ptr->GetSpacing()[0]<<", "<<ptr->GetSpacing()[1]<<std::endl;
	
	std::cout<<"LabelMapExample: EXIT"<<std::endl;
}


//
int main(int argc, char** argv) {

	try {
		disks();
		ellipses();
		//simpleObjectsExample();
		LabelMapExample();
		//TODO! test a 3D shape...

	}
	catch (DeformableModelException& e) {
		std::cout << "Exception occured while running the test cases" << std::endl;
		std::cout << e.what() << std::endl;
	}

}
