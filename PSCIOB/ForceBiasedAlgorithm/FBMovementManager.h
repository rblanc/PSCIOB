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
* \file FBMovementManager.h
* \author Rémi Blanc 
* \date 31. July 2012
*/


#ifndef __FBMOVEMENTMANAGER_H_
#define __FBMOVEMENTMANAGER_H_

#include "BaseScene.h"
#include "ForceBiasedAlgorithm.h"

#include "2DTransformUtils.h"
#include "3DTransformUtils.h"
#include <vnl/algo/vnl_symmetric_eigensystem.h>

namespace psciob {

/** \brief FBMovementManager
*
* This is the default movement manager for the ForceBiasedAlgorithm, which only proposes translations, and scaling
* Translation is straightforward, as the first Dimension parameters of the objects correspond to translation
* The direction of the translation is simply set as the direction which links both object centers. 
* In this implementation, the amount of translation is 1 unit, multiplied by a parameter (translation factor)
* For non-unit scaling, the user must indicate which parameter (for each object type) corresponds to scale.
*/

//DEV: Child classes should implement a system similar to the ObjectTypesLibrary, to associate 

template<class TScene>
class FBMovementManager : public itk::LightObject {
public:
	/** Standard class typedefs. */
	typedef FBMovementManager       Self;
	typedef itk::LightObject        Superclass;
	typedef itk::SmartPointer<Self> Pointer;
	/** Run-time type information (and related methods). */
	itkTypeMacro(FBMovementManager, itk::LightObject);
	itkNewMacro(Self);

	typedef TScene                                     SceneType;
	static const unsigned int Dimension              = SceneType::Dimension;
	typedef typename SceneType::IDType                 IDType;
	typedef typename SceneType::ObjectTypesLibraryType ObjectTypesLibraryType;
	typedef typename SceneType::DeformableObjectType   ObjectType;
	typedef typename SceneType::InteractionDataType    InteractionDataType;
	

	/** Attach a scene to the algorithm */
	void SetScene(SceneType* scene) { 
		m_scene = scene; 
	}

	/** Set the scaling factor, to be applied at the end of each iteration 
	* Values should be in ]0,1] ; if the input is outside this range, value 1 is set instead
	* If the value is 1 (default behavior), no scaling is applied
	* Otherwise, it should be specified, for each object type, which is the coefficient that performs scaling... this is done through the object movement manager...
	*/
	void SetScalingFactor(double s) {
		if (s<=0) {m_scaleFactor=1;return;}
		if (s>=1) {m_scaleFactor=1;return;}
		m_scaleFactor = s;
	}
	
	/** Get the value of the scaling factor */
	double GetScalingFactor() { return m_scaleFactor; }

	/** Set the translation factor, multiplying the default translation vector */
	void SetTranslationFactor(double f) {
		if (f<=0) {m_translationFactor=1;return;}
		m_translationFactor = f;
	}
	
	/** Get the value of the translation factor */
	double GetTranslationFactor() {return m_translationFactor;}

	/** Set rotation factor (in [0 0.5])
	* it interpolates the rotation which bring the principal axes of inertia of both objects in alignement
	* 0.5 => they should end up aligned
	*/
	void SetRotationFactor(double f) { 
		m_rotationFactor=f;
	}

	/** Get the value of the translation factor */
	double GetRotationFactor() {return m_rotationFactor;}

	/** update the proposed move of object id1 with respect to its interaction with id2
	* \param currentMove is the current vector of parameters describing the move (would-be object parameters)
	* \param id1 is the id of the object to be moved
	* \param id2 is the id of the overlapping object
	* \param overlapData is the structure containing the overlap information.
	* In this class, a simple unit translation is applied, in the direction joining the center of both objects, and optionally a rotation that tends to make the objects inertia matrices 'parallel'.
	*/
	virtual vnl_vector<double> UpdateMove(const vnl_vector<double> &currentMove, IDType id1, IDType id2, InteractionDataType overlapData) {
		vnl_vector<double> t(Dimension), proposedMove = currentMove;
		
		//translation
		t = m_translationFactor * (m_scene->GetParametersOfObject(id1).extract(Dimension) - m_scene->GetParametersOfObject(id2).extract(Dimension))/2.0;
		for (unsigned i=0 ; i<Dimension ; i++) proposedMove(i)+=t(i);

		//rotation
		if (m_rotationFactor!=0) {
			//get the center of gravity, and matrices of inertia of both objects
			vnl_vector<double> c1, c2; vnl_matrix<double> I1, I2;
			m_scene->GetObject(id1)->obj->GetObjectCenterAndInertiaMatrix(c1, I1);
			m_scene->GetObject(id2)->obj->GetObjectCenterAndInertiaMatrix(c2, I2);

			vnl_symmetric_eigensystem<double> eig1(I1);
			vnl_symmetric_eigensystem<double> eig2(I2);
std::cout<<"matrix 1: \n"<<eig1.V<<std::endl;
std::cout<<"matrix 2: \n"<<eig2.V<<std::endl;
			vnl_matrix<double> RotMat_Full = eig2.V * eig1.V.transpose(), RotMat_Part;
			double ang;
			vnl_vector<double> Q;vnl_vector<double> vr(3);

			switch(Dimension) {
				case 2: //just interpolate the angle that brings both systems in alignement
					ang = psciob::GetAngleFrom2DRotationMatrix( RotMat_Full );
					RotMat_Part = psciob::Get2DRotationMatrixFromAngle(m_rotationFactor*ang);
					break;
				case 3:
					//do the same as for 2D...
					//use quaternion to get the axis of rotation, and angle
std::cout<<"RotMat_Full: \n"<<RotMat_Full<<std::endl;
					Q = psciob::GetQuaternionFrom3DRotationMatrix( RotMat_Full );
std::cout<<"    Q: "<<Q<<std::endl;
RotMat_Part = psciob::Get3DRotationMatrixFromQuaternion_33(Q);
std::cout<<"RotMat_Part: \n"<<RotMat_Part<<std::endl;

					psciob::GetVectorAndAngleFromQuaternion( vr, ang, Q );
std::cout<<"    vect: "<<vr<<", angle: "<<ang<<std::endl;
					//and get the rotation matrix corresponding to the requested fraction of the angle
					psciob::GetQuaternionFromVectorAndAngle( vr, m_rotationFactor*ang, Q );
std::cout<<"eff Q: "<<Q<<std::endl;
psciob::GetVectorAndAngleFromQuaternion( vr, ang, Q );
std::cout<<"eff vect: "<<vr<<", eff angle: "<<ang<<std::endl;

					RotMat_Part = psciob::Get3DRotationMatrixFromQuaternion_33(Q);
std::cout<<"RotMat_Part: \n"<<RotMat_Part<<std::endl;
					break;
				default: throw DeformableModelException("FBMovementManager::UpdateMove : dimension should be 2 or 3!");
			}
			m_scene->GetObject(id1)->obj->ApplyRotationToParameters( RotMat_Part, proposedMove);
		}

		return proposedMove;
	}
	
	/** 	*/
	vnl_vector<double> GetScaledParameters(IDType id, const vnl_vector<double> &currentMove) {
		vnl_vector<double> proposedMove = currentMove;
		if (m_scaleFactor!=1) {
			//check this
			m_scene->GetObject(id)->obj->ApplyScalingToParameters(m_scaleFactor, proposedMove);
		}
		return proposedMove;
	}


	/** update the proposed move of object id1 with respect to its interaction with the scene wall
	* \param currentMove is the current vector of parameters describing the move (would-be object parameters)
	* \param id1 is the id of the object to be moved
	* \param wall is the index describing which wall is hit (0=left, 1=right, ...)
	* In this class, a simple unit translation is applied, in the direction joining the center of both objects, and optionally a rotation that tends to make the objects inertia matrices 'parallel'.
	*/
	virtual vnl_vector<double> UpdateMoveWithSceneWall(const vnl_vector<double> &currentMove, IDType id1, unsigned wallId) {
		vnl_vector<double> t(Dimension), proposedMove = currentMove;

		//translation
		t.fill(0);
		unsigned i = wallId/2;

		if (wallId == 2*i) proposedMove(i) += max(m_translationFactor * (m_scene->GetSceneBoundingBox()(wallId)-m_scene->GetObject(id1)->obj->GetPhysicalBoundingBox()(wallId)), m_scene->GetSceneSpacing()[i]);
		else               proposedMove(i) -= max(m_translationFactor * (m_scene->GetSceneBoundingBox()(wallId)-m_scene->GetObject(id1)->obj->GetPhysicalBoundingBox()(wallId)), m_scene->GetSceneSpacing()[i]);
		////rotation
		//if (m_rotationFactor!=0) {
		//	//get the center of gravity, and matrices of inertia of both objects
		//	vnl_vector<double> c1; vnl_matrix<double> I1, I2(Dimension, Dimension);
		//	m_scene->GetObject(id1)->obj->GetObjectCenterAndInertiaMatrix(c1, I1);
		//	I2.fill(0); for (unsigned j=0 ; j<Dimension ; j++) { I2(j,j) = 10;} I2(i,i) = 1; // just set the inertia matrix as aligned with the wall

		//	vnl_symmetric_eigensystem<double> eig1(I1);
		//	vnl_symmetric_eigensystem<double> eig2(I2);
		//	
		//	vnl_matrix<double> RotMat_Full = eig1.V.transpose() * eig2.V, RotMat_Part;
		//	double ang;
		//	vnl_vector<double> Q; vnl_vector<double> vr(3);

		//	switch(Dimension) {
		//		case 2: //just interpolate the angle that brings both systems in alignement
		//			ang = psciob::GetAngleFrom2DRotationMatrix( RotMat_Full );
		//			RotMat_Part = psciob::Get2DRotationMatrixFromAngle(m_rotationFactor*ang);
		//			break;
		//		case 3:
		//			//do the same as for 2D...
		//			//use quaternion to get the axis of rotation, and angle
		//			Q = psciob::GetQuaternionFrom3DRotationMatrix( RotMat_Full );
		//			psciob::GetVectorAndAngleFromQuaternion( vr, ang, Q );
		//			//and get the rotation matrix corresponding to the requested fraction of the angle
		//			psciob::GetQuaternionFromVectorAndAngle( vr, m_rotationFactor*ang, Q );
		//			RotMat_Part = psciob::Get3DRotationMatrixFromQuaternion_33(Q);
		//			break;
		//		default: throw DeformableModelException("FBMovementManager::UpdateMove : dimension should be 2 or 3!");
		//	}
		//	m_scene->GetObject(id1)->obj->ApplyRotationToParameters( RotMat_Part, proposedMove);
		//}

		return proposedMove;
	}

protected:
	FBMovementManager() : m_scaleFactor(1), m_translationFactor(1), m_rotationFactor(0) { //by default, no rotations are applied
		m_scene = 0;
		m_identityMove.set_size(Dimension); m_identityMove.fill(0); //translation only in this case.
	};
	~FBMovementManager() {};

    typename SceneType::Pointer m_scene;
	double m_scaleFactor;
	double m_translationFactor;
	double m_rotationFactor;
	//bool m_scaleInfoSet; //flag indicating whether all the necessary information is available -> e.g. which parameters correspond to scale
	vnl_vector<double> m_identityMove;

private:
	FBMovementManager(const Self&);      //purposely not implemented
	const Self & operator=( const Self & ); //purposely not implemented
};





} // namespace psciob

#endif /* __FBMOVEMENTMANAGER_H_ */
