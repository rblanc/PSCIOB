
//seems to enable IntelliSence for VisualStudio...
#pragma once

#include <iostream>
#include <typeinfo>

#include <GeneralUtils.h>

#include "shape3DCuboid.h"
#include "shape3DCylinder.h"
#include "Similarity3DTransform.h"
#include "PoseTransformedBinaryShape.h"

#include "LabelMapScene.h"
#include "ObjectOverlap_Pow2DTree.h"
#include "LabelObjectOverlap.h"
#include "LabelObjectOverlapDetector.h"

#include <SimplePixelBasedSEMSensor.h>

#include "ForceBiasedAlgorithm.h"

using namespace psciob;

//
//
int main(int argc, char** argv) {
	

	try{
		std::string o_outputText= "FB_log.txt";
		std::ofstream file_out;
		file_out.open(o_outputText.c_str());
		file_out <<"Force-Biased algorithm for 3D synthesis of ferrites packings"<< std::endl;

		/* 
		* Define the scene : 100*100*100 cube of unit resolution.
		*/
		static const unsigned int D = 3;
		typedef unsigned short IDType;
		typedef unsigned char ScenePixelType;
		typedef LabelMapScene<D, ScenePixelType, IDType, ObjectCostsContainer, LabelObjectIntersectionContainer<IDType, D>> SceneType;
		
		SceneType::Pointer scene = SceneType::New();
		vnl_vector<double> bbox(6);
		bbox(0) = -50; bbox(1) = 50; 
		bbox(2) = -50; bbox(3) = 50;
		bbox(4) = -50; bbox(5) = 50;

		vnl_vector<double> sp(3);
		sp(0) = 1;//'spacing in depth'
		sp(1) = 1; sp(2) = 1;
		scene->SetPhysicalDimensions(bbox, sp);

		file_out <<"scene size: "<<bbox(1)-bbox(0)<<" x "<<bbox(3)-bbox(2)<<" x "<<bbox(5)-bbox(4)<< std::endl;
		file_out <<"scene bounding box: "<<bbox<< std::endl;

		typedef ObjectOverlap_Pow2DTree<SceneType::BaseClass, LabelObjectOverlapDetector<SceneType::BaseClass>> ObjectInteractionManagerType;
		ObjectInteractionManagerType::Pointer interactionManager = ObjectInteractionManagerType::New();
		scene->SetInteractionManager(interactionManager);

		/* 
		* Define a generative distribution for ferrites birth...
		*/ 
		shape3DCuboid::Pointer cub = shape3DCuboid::New();
		shape3DCuboid::Pointer cyl = shape3DCuboid::New();
		Similarity3DTransform::Pointer transform_cub = Similarity3DTransform::New();
		Similarity3DTransform::Pointer transform_cyl = Similarity3DTransform::New();
		
		PoseTransformedBinaryShape<3>::Pointer sampleCub = PoseTransformedBinaryShape<3>::New();
		PoseTransformedBinaryShape<3>::Pointer sampleCyl = PoseTransformedBinaryShape<3>::New();
		sampleCub->SetShapeAndTransform(cub, transform_cub);
		sampleCyl->SetShapeAndTransform(cyl, transform_cyl);
		
		//parameters: 3 translation, 3 rotation, 1 scale (ferrite length)
		//            1 elongation (>1), 1 pointiness ([0,1]), 1 thickness (>=1)

		unsigned int typeCode_cub = scene->GetObjectTypesLibrary()->RegisterObjectType(sampleCub);
		unsigned int typeCode_cyl = scene->GetObjectTypesLibrary()->RegisterObjectType(sampleCyl);

		//shrink the bounding box a little, to avoid putting objects too close to the borders
		vnl_vector<double> enlargedBBox = scene->GetSceneBoundingBox();
		enlargedBBox(0)+=5; enlargedBBox(1)-=5;
		enlargedBBox(2)+=5; enlargedBBox(3)-=5;
		enlargedBBox(4)+=5; enlargedBBox(5)-=5; 

		//translation
		UniformBoxPDF::Pointer transPDF = UniformBoxPDF::New(); transPDF->SetBox(enlargedBBox);
		file_out <<"\ninitial distribution: position uniform in "<<enlargedBBox<< std::endl;
		//rotation
		IndependentEulerRotationsPDFs::Pointer rotationPrior = IndependentEulerRotationsPDFs::New();
		//1st: rotation around z, so that the object is mostly oriented along y (horizontal axis of the image)
		TriangularPDF::Pointer prior_alpha = TriangularPDF::New(); prior_alpha->SetParameters(-PI/2.0, PI/2.0, 3.0*PI/2.0); 
		rotationPrior->AddIndependentRotationPDF(IndependentEulerRotationsPDFs::Z_AXIS, prior_alpha);	
		//2nd: rotate slightly along x, then along y
		NormalPDF::Pointer prior_theta = NormalPDF::New(); prior_theta->SetParameters(0, (30.0*PI/180.0)*(30.0*PI/180.0));
		rotationPrior->AddIndependentRotationPDF(IndependentEulerRotationsPDFs::X_AXIS, prior_theta);
		NormalPDF::Pointer prior_phi = NormalPDF::New(); prior_phi->SetParameters(0, (30.0*PI/180.0)*(30.0*PI/180.0));
		rotationPrior->AddIndependentRotationPDF(IndependentEulerRotationsPDFs::Y_AXIS, prior_phi);
		file_out <<"  rotation, 3 independent rotations"<< std::endl;
		file_out <<"    1: around Z, angle follows a triangular distribution with parameters: -PI/2.0, PI/2.0, 3.0*PI/2.0"<< std::endl;
		file_out <<"    2: around X, angle follows a normal distribution with mean 0 and std 30°"<< std::endl;
		file_out <<"    3: around Z, angle follows a normal distribution with mean 0 and std 30°"<< std::endl;

		TrapezoidalPDF::Pointer  scalePDF= TrapezoidalPDF::New();scalePDF->SetParameters(5, 6, 18, 23);    
		file_out <<"  scale follows a TrapezoidalPDF with parameters 5, 6, 18, 23"<< std::endl;
		
		//elongation for the cube
		TrapezoidalPDF::Pointer elongPDF= TrapezoidalPDF::New();elongPDF->SetParameters(1, 1.25, 2.5, 3);
		file_out <<"  cube elongation: TrapezoidalPDF with parameters 1, 1.25, 2.5, 3"<< std::endl;
		//thickness for cuboids
		TrapezoidalPDF::Pointer thicknessPDF = TrapezoidalPDF::New(); thicknessPDF->SetParameters(1.2, 1.5, 4, 7);
		file_out <<"  cube thickness follows a TrapezoidalPDF with parameters 1.2, 1.5, 4, 7"<< std::endl;
		
		//elongation for the cylinder ~ length / diamter
		TrapezoidalPDF::Pointer cylelongPDF= TrapezoidalPDF::New();elongPDF->SetParameters(1, 1.25, 2.5, 3);
		file_out <<"  cube elongation: TrapezoidalPDF with parameters 1, 1.25, 2.5, 3"<< std::endl;

		IndependentPDFs::Pointer cubGenerationPDF = IndependentPDFs::New();
		cubGenerationPDF->AddMultivariatePDF(transPDF);   cubGenerationPDF->AddMultivariatePDF(rotationPrior);
		cubGenerationPDF->AddUnivariatePDF(scalePDF);     cubGenerationPDF->AddUnivariatePDF(elongPDF);
		cubGenerationPDF->AddUnivariatePDF(thicknessPDF); 

		scene->GetObjectTypesLibrary()->SetObjectPDF(typeCode_cub, PDF_OBJECTGENERATIONPRIOR, cubGenerationPDF);

		IndependentPDFs::Pointer cylGenerationPDF = IndependentPDFs::New();
		cylGenerationPDF->AddMultivariatePDF(transPDF);   cylGenerationPDF->AddMultivariatePDF(rotationPrior);
		cylGenerationPDF->AddUnivariatePDF(scalePDF);     cylGenerationPDF->AddUnivariatePDF(cylelongPDF);

		scene->GetObjectTypesLibrary()->SetObjectPDF(typeCode_cyl, PDF_OBJECTGENERATIONPRIOR, cylGenerationPDF);



		clock_t t0=clock();
		scene->GetInteractionManager()->TurnOffInteractionManagement();
		///*
		//* put 100 random objects in the scene
		//*/
		for (unsigned i=0 ; i<5 ; i++) {
			scene->AddObject(scene->GetObjectTypesLibrary()->GenerateNewRandomObject(typeCode_cub));
		}
		for (unsigned i=0 ; i<5 ; i++) {
			scene->AddObject(scene->GetObjectTypesLibrary()->GenerateNewRandomObject(typeCode_cyl));
		}

		/* Test with only 2 objects, to check movements are correctly estimated and applied
		* scale, rotation & translation look fine with the base algorithm 
		*/


		/*
		* 
		*/
		file_out <<"Number of objects inserted: "<<scene->GetNumberOfObjects()<< std::endl;

		psciob::WriteMirrorPolyDataToFile("FB_initialState.vtk", scene->GetSceneAsVTKPolyData());
		psciob::WriteITKImageToFile("FB_initialState.nii", scene->GetSceneAsLabelImage());

		std::cout<<"time to add "<<scene->GetNumberOfObjects()<<" objects, without computing interactions: "<<(clock()-t0)/((double)CLOCKS_PER_SEC)<<" s."<<std::endl;
		scene->GetInteractionManager()->TurnOnInteractionManagement();
		std::cout<<"time to compute all interactions: "<<(clock()-t0)/((double)CLOCKS_PER_SEC)<<" s."<<std::endl;
		unsigned nbOverlaps=0;
		
		SceneObjectIterator<SceneType> objectIt(scene);
		for (objectIt.GoToBegin() ; !objectIt.IsAtEnd() ; ++objectIt) {
			nbOverlaps += objectIt.GetObjectInScene()->interactionData.size();
		}
		std::cout<<"total nb of overlaps detected: "<<nbOverlaps<<std::endl;
		

		/*
		* Force-biased
		*/
		typedef psciob::ForceBiasedAlgorithm<SceneType> ForceBiasedAlgorithmType;
		ForceBiasedAlgorithmType::Pointer fbAlgo = ForceBiasedAlgorithmType::New();
		fbAlgo->SetScene(scene);
		fbAlgo->GetMovementManager()->SetTranslationFactor(1);
		fbAlgo->GetMovementManager()->SetScalingFactor(0.999);
		fbAlgo->GetMovementManager()->SetRotationFactor(0.2);
		fbAlgo->SetBoundaryEffectBehavior(ForceBiasedAlgorithmType::PERIODICBOUNDARIES);

		file_out <<"Applying the Force-Biased algorithm with PERIODICBOUNDARIES"<< std::endl;

		fbAlgo->IterateUntilConvergence(true);

		file_out <<"Total time to convergence: "<<(clock()-t0)/((double)CLOCKS_PER_SEC)<<" s."<< std::endl;

		psciob::WriteMirrorPolyDataToFile("FB_finalState.vtk", scene->GetSceneAsVTKPolyData());
		psciob::WriteITKImageToFile("FB_finalState.nii", scene->GetSceneAsLabelImage());
	}
	catch (DeformableModelException& e) {
		std::cout << "Exception occured while performing the program..." << std::endl;
		std::cout << e.what() << std::endl;
	}


	return 1;
}

