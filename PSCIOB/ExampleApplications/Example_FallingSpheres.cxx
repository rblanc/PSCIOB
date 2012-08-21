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

#include <GeneralUtils.h>
#include "LabelMapScene.h"
#include "ObjectOverlap_Pow2DTree.h"
#include "LabelObjectOverlap.h"
#include "Direct3DSphere.h"

#include "RandomVariables.h"
#include "UnivariatePDF.h"

using namespace psciob;

//
// In this example, we generate spheres at the top of the scene bbox, and let them fall under 'gravity',
// they stop when they start intersecting the scene borders, and bounce a little when touching other spheres.
// the process stops when the balls reach the top of the box.
//
int main(int argc, char** argv) {

	bool verbose = true;
	try {

		/* 
		* Define the scene, we want a 3D scene, to associate velocity data to each object
		* and to manage intersection between object through the intersection of the set of voxels occupied by the objects (it could be simplified by using an analytical intersection test, but this is not the purpose here)
		*/
		typedef LabelMapScene<3,unsigned char, unsigned short, ObjectVelocityContainer, LabelObjectIntersectionContainer<unsigned short, 3>> SceneType;

		SceneType::Pointer scene = SceneType::New();
		//we define a 100*100*100 cube in which the objects will fall.
		vnl_vector<double> bbox(6); 
		bbox(0) = 0; bbox(1) = 100;
		bbox(2) = 0; bbox(3) = 100;
		bbox(4) = 0; bbox(5) = 100;
		scene->SetPhysicalDimensions(bbox, 1);
		if (verbose) std::cout<<"The scene is defined, bbox: "<<scene->GetSceneBoundingBox()<<std::endl; 

		//interacting object = overlapping objects
		//we generate an intersection manager exploiting an octree structure for storing objects
		typedef ObjectOverlap_Pow2DTree<SceneType::BaseClass, LabelObjectOverlap<SceneType::BaseClass>> ObjectInteractionManagerType;
		ObjectInteractionManagerType::Pointer interactionManager = ObjectInteractionManagerType::New();
		scene->SetInteractionManager(interactionManager);

		//general purpose random number generator, aimed at generating the random positions of the objects
		RandomVariableGenerator::Pointer rndGen = RandomVariableGenerator::New();
		rndGen->Initialize(); //initialize it on the clock

		//pdf for sphere radius
		NormalPDF::Pointer radiusPDF = NormalPDF::New();   
		radiusPDF->SetParameters(10, 1);


		/*
		* Insert a sphere of radius 10 at the top-center of the scene
		*/
		SceneType::IDType id;
		Direct3DSphere::Pointer sampleObject = Direct3DSphere::New();
		vnl_vector<double> sphereParams(4); //4 parameters: 3 centers, and radius

		sphereParams(3) = 10;
		sphereParams(0) = 50;
		sphereParams(1) = 50;
		sphereParams(2) = bbox(3)-sphereParams(3);

		sampleObject->SetParameters(sphereParams);

		//Add the sphere to the scene
		id = scene->AddObject(sampleObject);
		scene->GetObject(id)->objectData.m = sphereParams(3)*sphereParams(3)*sphereParams(3);

		/*
		* Let the sphere fall under gravity
		* here, only the z direction is useful, so this simplifies the code a little.
		*/
		double sumV;int nbItAtZero=0;
		bool converged = false;
		double G = 9.81; //gravité
		double kw = 0.8;  //energie dissipée lors d'un choc avec un mur...
		double ks = 0.95;  //energie dissipée lors d'un choc entre spheres
		double t = 0, dt = 0.01;//discrete time step.
		vnl_vector<double> nv(3);
		WriteMirrorPolyDataToFile("sphere_t0.vtk", scene->GetSceneAsVTKPolyData());
		while (!converged) {
			sumV = 0;
			//get the current position and velocity of the object
			sphereParams = scene->GetParametersOfObject(id);
			nv(0) = scene->GetObject(id)->objectData.vx;
			nv(1) = scene->GetObject(id)->objectData.vy;
			nv(2) = scene->GetObject(id)->objectData.vz;

			//move the object according to its velocity
			sphereParams(0) += nv(0) * dt;
			sphereParams(1) += nv(1) * dt;
			sphereParams(2) += nv(2) * dt;
			scene->ModifyObjectParameters(id, sphereParams);

			//apply the various forces to compute the new velocity
			//gravity
			nv(2) -= G * dt;
			//bouncing on the bottom wall
			if (sphereParams(2)-sphereParams(3)<=0) {
				//first, move the sphere back on the wall...
				sphereParams(2) = sphereParams(3);
				//
				if (nv(2)<0) {
					nv(2) = fabs(nv(2)); //make sure the velocity is pointing upwards
					nv *= kw;
				}
			}

			//update the velocity
			scene->GetObject(id)->objectData.vx = nv(0);
			scene->GetObject(id)->objectData.vy = nv(1);
			scene->GetObject(id)->objectData.vz = nv(2);

			//advance time.
			t+=dt;
			if (fabs(t-round(t))<TINY) {
				WriteMirrorPolyDataToFile("sphere1_t" + stringify(t) + ".vtk", scene->GetSceneAsVTKPolyData());				
			}

			//check if things are still moving
			sumV = nv.magnitude();
			if (sumV<G*dt+TINY) nbItAtZero++;
			else nbItAtZero=0;

			if (nbItAtZero==10) {
				converged = true;
			}

			if (t>15) converged=true;
		}


		/*
		* add a second sphere and let it fall
		*/
		converged = false;
		vnl_vector<double> distantParams, v1(3), v2(3), n(3);
		double m1, m2, a;
		t=0;
		//draw a new sphere with random radius, somewhere at the top of the box
		sphereParams(3) = 12;
		sphereParams(0) = 55;
		sphereParams(1) = 50;
		sphereParams(2) = bbox(3)-sphereParams(3);
		sampleObject->SetParameters(sphereParams);
		id = scene->AddObject(sampleObject);
		scene->GetObject(id)->objectData.m = sphereParams(3)*sphereParams(3)*sphereParams(3);
		std::cout<<"let a second ball fall on the first one..., radius = "<<sphereParams(3)<<", m = "<<scene->GetObject(id)->objectData.m<<std::endl;

		while (!converged) {

			//move each spheres, unless it is 'frozen'
			SceneObjectIterator<SceneType> objectIt(scene);
			for (objectIt.GoToBegin() ; !objectIt.IsAtEnd() ; ++objectIt) {
				if (objectIt.GetObject()->objectData.frozen) continue;

				sphereParams = scene->GetParametersOfObject(objectIt.GetID());
				//std::cout<<"  id "<<objectIt.GetID()<<" ; before: pos = "<<sphereParams<<", v = "<<objectIt.GetObject()->objectData.vx<<", "<<objectIt.GetObject()->objectData.vy<<", "<<objectIt.GetObject()->objectData.vz<<std::endl;
				sphereParams(0) += objectIt.GetObject()->objectData.vx * dt;
				sphereParams(1) += objectIt.GetObject()->objectData.vy * dt;
				sphereParams(2) += objectIt.GetObject()->objectData.vz * dt;
				if (!scene->ModifyObjectParameters(objectIt.GetID(), sphereParams)) std::cout<<"failed to move to new params: "<<sphereParams<<" ; should NEVER happen..."<<std::endl;
				//std::cout<<"  id "<<objectIt.GetID()<<" ; after: pos = "<<scene->GetParametersOfObject(objectIt.GetID())<<std::endl;

			}

			//for each sphere, update the velocities with various forces.
			for (objectIt.GoToBegin() ; !objectIt.IsAtEnd() ; ++objectIt) {
				if (objectIt.GetObject()->objectData.frozen) continue;

				sphereParams = scene->GetParametersOfObject(objectIt.GetID());
				v1(0) = objectIt.GetObject()->objectData.vx;
				v1(1) = objectIt.GetObject()->objectData.vy;
				v1(2) = objectIt.GetObject()->objectData.vz;
				m1 = objectIt.GetObject()->objectData.m;
				nv=v1;

				//1: gravity
				nv(2) -= G * dt;

				//2: check collision with other spheres
				for (SceneType::ObjectInteractionIterator iit = objectIt.GetObject()->interactionData.begin() ; iit!= objectIt.GetObject()->interactionData.end() ; iit++) {
					distantParams = scene->GetParametersOfObject(iit->first);
					v2(0) = scene->GetObject(iit->first)->objectData.vx; v2(1) = scene->GetObject(iit->first)->objectData.vy; v2(2) = scene->GetObject(iit->first)->objectData.vz;
					//vector normal to both sphere
					n(0) = distantParams(0)-sphereParams(0); n(1) = distantParams(1)-sphereParams(1); n(2) = distantParams(2)-sphereParams(2);
					n.normalize();
					//check they are moving toward each other
					if ( dot_product(v2-v1,n)>0 ) {continue;}
					else { 
						//http://www.sjsu.edu/faculty/watkins/collision.htm
						m2 = scene->GetObject(iit->first)->objectData.m;
						a = 2.0*m1*m2/(m1+m2)*dot_product(n, v1-v2);
						std::cout<<" ball "<<objectIt.GetID()<<" : "<<sphereParams<<", speed: "<<v1<<", m = "<<m1<<std::endl;
						std::cout<<" ball "<<iit->first<<" : "<<distantParams<<", speed: "<<v2<<", m = "<<m2<<std::endl;
						std::cout<<" a = "<<a<<std::endl;
						nv -= a/m1*n; 
						nv *= ks;
					}
				}
				
				//3: check collision with the walls
				//left, right
				if (sphereParams(0)-sphereParams(3)<bbox(0)) { if (nv(0)<0) nv(0) = - nv(0); nv*=kw;}
				if (sphereParams(0)+sphereParams(3)>bbox(1)) { if (nv(0)>0) nv(0) = - nv(0); nv*=kw;}
				//back, front
				if (sphereParams(1)-sphereParams(3)<bbox(2)) { if (nv(1)<0) nv(1) = - nv(1); nv*=kw;}
				if (sphereParams(1)+sphereParams(3)>bbox(3)) { if (nv(1)>0) nv(1) = - nv(1); nv*=kw;}
				//bottom, up
				if (sphereParams(2)-sphereParams(3)<bbox(4)) { if (nv(2)<0) nv(2) = - nv(2); nv*=kw;}
				if (sphereParams(2)+sphereParams(3)>bbox(5)) { if (nv(2)>0) nv(2) = - nv(2); nv*=kw;}

				//OK, update the velocity of the object.
				objectIt.GetObject()->objectData.vx = nv(0);
				objectIt.GetObject()->objectData.vy = nv(1);
				objectIt.GetObject()->objectData.vz = nv(2);

			}

			//advance time.
			t+=dt;
			if (fabs(t-round(t))<TINY) {
				WriteMirrorPolyDataToFile("sphere2_t" + stringify(t) + ".vtk", scene->GetSceneAsVTKPolyData());				
			}

			//check if things are still moving...

			//
			if (t>20) converged=true;

		}


		/*
		* add a new sphere every 3 'second' and let it fall
		*/
		t=0;converged=false;
		while (!converged) {
			//check if a new sphere shall be added.
			if (fabs(t-3.0*round(t/3.0))<TINY) {
				//draw a new sphere with random radius, somewhere at the top of the box
				sphereParams(3) = max(1.0, radiusPDF->DrawSample());
				sphereParams(0) = rndGen->GetUniformVariate(bbox(0)+sphereParams(3), bbox(1)-sphereParams(3));
				sphereParams(1) = rndGen->GetUniformVariate(bbox(2)+sphereParams(3), bbox(3)-sphereParams(3));
				sphereParams(2) = bbox(3)-sphereParams(3);
				sampleObject->SetParameters(sphereParams);
				id = scene->AddObject(sampleObject);
				scene->GetObject(id)->objectData.m = sphereParams(3)*sphereParams(3)*sphereParams(3);
			}

			//move each spheres, unless it is 'frozen'
			SceneObjectIterator<SceneType> objectIt(scene);
			for (objectIt.GoToBegin() ; !objectIt.IsAtEnd() ; ++objectIt) {
				if (objectIt.GetObject()->objectData.frozen) continue;

				sphereParams = scene->GetParametersOfObject(objectIt.GetID());
				//std::cout<<"  id "<<objectIt.GetID()<<" ; before: pos = "<<sphereParams<<", v = "<<objectIt.GetObject()->objectData.vx<<", "<<objectIt.GetObject()->objectData.vy<<", "<<objectIt.GetObject()->objectData.vz<<std::endl;
				sphereParams(0) += objectIt.GetObject()->objectData.vx * dt;
				sphereParams(1) += objectIt.GetObject()->objectData.vy * dt;
				sphereParams(2) += objectIt.GetObject()->objectData.vz * dt;
				if (!scene->ModifyObjectParameters(objectIt.GetID(), sphereParams)) std::cout<<"failed to move to new params: "<<sphereParams<<" ; should NEVER happen..."<<std::endl;
				//std::cout<<"  id "<<objectIt.GetID()<<" ; after: pos = "<<scene->GetParametersOfObject(objectIt.GetID())<<std::endl;

			}

			//for each sphere, update the velocities with various forces.
			for (objectIt.GoToBegin() ; !objectIt.IsAtEnd() ; ++objectIt) {
				if (objectIt.GetObject()->objectData.frozen) continue;

				sphereParams = scene->GetParametersOfObject(objectIt.GetID());
				v1(0) = objectIt.GetObject()->objectData.vx;
				v1(1) = objectIt.GetObject()->objectData.vy;
				v1(2) = objectIt.GetObject()->objectData.vz;
				m1 = objectIt.GetObject()->objectData.m;
				nv=v1;

				//1: gravity
				nv(2) -= G * dt;

				//2: check collision with other spheres
				for (SceneType::ObjectInteractionIterator iit = objectIt.GetObject()->interactionData.begin() ; iit!= objectIt.GetObject()->interactionData.end() ; iit++) {
					distantParams = scene->GetParametersOfObject(iit->first);
					v2(0) = scene->GetObject(iit->first)->objectData.vx; v2(1) = scene->GetObject(iit->first)->objectData.vy; v2(2) = scene->GetObject(iit->first)->objectData.vz;
					//vector normal to both sphere
					n(0) = distantParams(0)-sphereParams(0); n(1) = distantParams(1)-sphereParams(1); n(2) = distantParams(2)-sphereParams(2);
					n.normalize();
					//check they are moving toward each other
					if ( dot_product(v2-v1,n)>0 ) {continue;}
					else { 
						//http://www.sjsu.edu/faculty/watkins/collision.htm
						m2 = scene->GetObject(iit->first)->objectData.m;
						a = 2.0*m1*m2/(m1+m2)*dot_product(n, v1-v2);
						nv -= a/m1*n; 
						nv *= ks;
					}
				}
				
				//3: check collision with the walls
				//left, right
				if (sphereParams(0)-sphereParams(3)<bbox(0)) { if (nv(0)<0) nv(0) = - nv(0); nv*=kw;}
				if (sphereParams(0)+sphereParams(3)>bbox(1)) { if (nv(0)>0) nv(0) = - nv(0); nv*=kw;}
				//back, front
				if (sphereParams(1)-sphereParams(3)<bbox(2)) { if (nv(1)<0) nv(1) = - nv(1); nv*=kw;}
				if (sphereParams(1)+sphereParams(3)>bbox(3)) { if (nv(1)>0) nv(1) = - nv(1); nv*=kw;}
				//bottom, up
				if (sphereParams(2)-sphereParams(3)<bbox(4)) { if (nv(2)<0) nv(2) = - nv(2); nv*=kw;}
				if (sphereParams(2)+sphereParams(3)>bbox(5)) { if (nv(2)>0) nv(2) = - nv(2); nv*=kw;}

				//OK, update the velocity of the object.
				objectIt.GetObject()->objectData.vx = nv(0);
				objectIt.GetObject()->objectData.vy = nv(1);
				objectIt.GetObject()->objectData.vz = nv(2);

			}


			//advance time.
			t+=dt;
			if (fabs(t-round(t))<TINY) {
				WriteMirrorPolyDataToFile("spheres_t" + stringify(t) + ".vtk", scene->GetSceneAsVTKPolyData());				
			}

			//check if things are still moving...

			//
			if (t>200) converged=true;

		}


	}
	catch (DeformableModelException& e) {
		std::cout << "Exception occured while performing the program..." << std::endl;
		std::cout << e.what() << std::endl;
	}


	return 1;
}

