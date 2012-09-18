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
* \file VTKObjectOverlap.h
* \author Rémi Blanc 
* \date 13. March 2011
*/

#ifndef __VTKOBJECTOVERLAP_H_
#define __VTKOBJECTOVERLAP_H_

#include "ObjectInteractionManager.h"

#include "vtkBooleanOperation2DPolygons.h"
#include <vtkBooleanOperationPolyDataFilter.h>
#include <vtkTriangleFilter.h>
#include <vtkMassProperties.h>

namespace psciob {

/**\class VTKObjectOverlap
* \brief VTKObjectOverlap: detect objects intersection based on their VTK representation as meshes
* requires the scene that can use a VTKIntersectionContainer
*
* the computation of the intersection does not work for some custom-generated shapes...
*
* the cost computed is the ratio : volume of intersection / (volume 1 + volume 2)
*/

//CONCRETE CLASS
template<class TScene>
class VTKObjectOverlap : public ObjectInteractionManager<TScene> {
public:
	/** Standard class typedefs. */
	typedef VTKObjectOverlap               Self;
	typedef ObjectInteractionManager       Superclass;
	typedef itk::SmartPointer<Self>        Pointer;
	typedef itk::SmartPointer<const Self>  ConstPointer;
	/** Run-time type information (and related methods). */
	itkTypeMacro(VTKObjectOverlap, ObjectInteractionManager);
	itkNewMacro(VTKObjectOverlap);


	///** Add an object to the Interaction Manager */
	//void AddObject(ObjectInScene *objectPtr) {
	//	//test all objects of the scene
	//	SceneObjectIterator<SceneType> it(m_scene);
	//	for (it.GoToBegin() ; !it.IsAtEnd() ; ++it) { 
	//		if ( objectPtr->id != it.GetID() ) {
	//			//first, check for bbox
	//			if ( !TestBoundingBoxesIntersection_NoCheck(objectPtr->obj->GetPhysicalBoundingBox(), it.GetObject()->GetPhysicalBoundingBox()) ) return;
	//			//if the bboxes intersect, then do the exact test.
	//			InteractionDataType interactionData;
	//			ComputePairWiseObjectInteractionData(objectPtr->obj, it.GetObjectInScene(), interactionData);
	//			if ( !interactionData.interactionCostFlag ) return;
	//			//Register the interaction, bilaterally: store the information in both objects
	//			m_scene->GetObject(objectPtr->id)->interactionData[it.GetID()] = interactionData;
	//			it.GetObjectInScene()->interactionData[objectPtr->id] = interactionData;
	//		}
	//	}
	//}


	///** Update the interaction of an object after a change in its parameters */
	//inline void ModifyObjectParameters(ObjectInScene *objectPtr, ObjectInScene *newObject) {
	//	//test all objects of the scene
	//	SceneObjectIterator<SceneType> it(m_scene);
	//	for (it.GoToBegin() ; !it.IsAtEnd() ; ++it) { 
	//		if ( objectPtr->id != it.GetID() ) {
	//			//first, check if their bbox intersect
	//			if ( !TestBoundingBoxesIntersection_NoCheck(newObject->obj->GetPhysicalBoundingBox(), it.GetObject()->GetPhysicalBoundingBox()) ) { _UnregisterInteraction_Internal(objectPtr, it.GetObjectInScene()); continue; }
	//			//if the bboxes intersect, then do the exact test.
	//			InteractionDataType interactionData;
	//			ComputePairWiseObjectInteractionData(newObject->obj, it.GetObjectInScene(), interactionData);
	//			if ( !interactionData.interactionCostFlag ) { _UnregisterInteraction_Internal(objectPtr, it.GetObjectInScene()); continue; }
	//			//Register the interaction, bilaterally: store the information in both objects
	//			m_scene->GetObject(objectPtr->id)->interactionData[it.GetID()] = interactionData;
	//			it.GetObjectInScene()->interactionData[objectPtr->id] = interactionData;
	//		}
	//	}
	//}



	/** given two arbitrary objects, compute what their interaction cost would be */
	inline void ComputePairWiseObjectInteractionData(DeformableObjectType *object1, DeformableObjectType *object2, InteractionDataType &interactionData) {		
		switch(SceneType::Dimension) {
			case 2:
				ComputePairWiseObjectInteractionData_2D(object1, object2, interactionData);
				break;
			case 3:
				ComputePairWiseObjectInteractionData_3D(object1, object2, interactionData);
				break;
			default: throw DeformableModelException("VTKObjectOverlap : unexpected dimension ; should never happen!"); 
		}	

/*
		switch(SceneType::Dimension) {
			case 2:
				double areas[3];
				for (it.GoToBegin() ; !it.IsAtEnd() ; ++it) {
					if ( objectPtr->id != it.GetID() ) {
						InteractionDataType interactionData = GetPairWiseObjectInteractionData_2D(objectPtr->obj, it.GetObject());
						if ( interactionData.interactionCostFlag ) {
							//intersection is bilateral, store the interaction information in both objects
							m_scene->GetObject(objectPtr->id)->interactionData.insert( ObjectInteractionPairType(it.GetID(), interactionData) );
							it.GetObjectInScene()->interactionData.insert( ObjectInteractionPairType(objectPtr->id, interactionData) );
						}				
					}
				}
				break;
			case 3: //does not call GetPairWiseObjectInteractionData_3D: it would add an overhead for the recomputation of the volume of objectPtr
				triangFilter1 = vtkTriangleFilter::New(); triangFilter2 = vtkTriangleFilter::New();
				filter = vtkBooleanOperationPolyDataFilter::New(); filter->SetOperationToIntersection();
				massProp1 = vtkMassProperties::New(); massProp2 = vtkMassProperties::New(); massProp3 = vtkMassProperties::New();

				triangFilter1->SetInput(objectPtr->obj->GetObjectAsVTKPolyData()); triangFilter1->Update();
				filter->SetOperationToIntersection(); filter->SetInput(0, triangFilter1->GetOutput() );
				massProp1->SetInput( triangFilter1->GetOutput() ); massProp1->Update();

				for (it.GoToBegin() ; !it.IsAtEnd() ; ++it) {
					if (objectPtr->id!=it.GetID()) {
						if (!TestBoundingBoxesIntersection( it.GetObject()->GetPhysicalBoundingBox(), m_scene->GetObject(objectPtr->id)->obj->GetPhysicalBoundingBox() )) continue;
						triangFilter2->SetInput( it.GetObject()->GetObjectAsVTKPolyData() ); triangFilter2->Update();
						filter->SetInput(1, triangFilter2->GetOutput() ); filter->Update();
						if (filter->GetOutput()->GetNumberOfPoints()>0) {
							//there is intersection, create a structure to host the interaction data
							massProp2->SetInput( triangFilter2->GetOutput() ); massProp2->Update();							
							massProp3->SetInput( filter->GetOutput() ); massProp3->Update();
							InteractionDataType interactionData;
							interactionData.volume = massProp3->GetVolume();
							interactionData.interactionCost = m_costFunction->Evaluate(massProp1->GetVolume(), massProp2->GetVolume(), massProp3->GetVolume());//massProp3->GetVolume() / (massProp1->GetVolume()+massProp2->GetVolume());
							interactionData.interactionCostFlag = true;
							interactionData.intersectionPolyData = vtkSmartPointer<vtkPolyData>::New();
							interactionData.intersectionPolyData->DeepCopy(filter->GetOutput());
							//intersection is bilateral, store the interaction information in both objects
							m_scene->GetObject(objectPtr->id)->interactionData.insert( ObjectInteractionPairType(it.GetID(), interactionData) );
							it.GetObjectInScene()->interactionData.insert( ObjectInteractionPairType(objectPtr->id, interactionData) );
						}
					}
				}
				triangFilter1->Delete(); triangFilter2->Delete(); filter->Delete(); massProp1->Delete(); massProp2->Delete(); massProp3->Delete();
				break;
			default: throw DeformableModelException("VTKObjectOverlap : unexpected dimension ; should never happen!"); 
		}
*/

	}

protected:	
	VTKObjectOverlap() : ObjectInteractionManager() { 
		m_triangFilter1 = vtkTriangleFilter::New();	m_triangFilter2 = vtkTriangleFilter::New();
		m_filter = vtkBooleanOperationPolyDataFilter::New();
		m_massProp1 = vtkMassProperties::New(); m_massProp2 = vtkMassProperties::New(); m_massProp3 = vtkMassProperties::New();
	};
	virtual ~VTKObjectOverlap() {
		m_triangFilter1->Delete();	m_triangFilter2->Delete();
		m_filter->Delete();;
		m_massProp1->Delete(); m_massProp2->Delete(); m_massProp3->Delete();

	};	

	vtkTriangleFilter *m_triangFilter1, *m_triangFilter2; 
	vtkBooleanOperationPolyDataFilter *m_filter; 
	vtkMassProperties *m_massProp1, *m_massProp2, *m_massProp3;


	/** given two 2D objects, compute what their interaction cost would be */
	inline void ComputePairWiseObjectInteractionData_2D(DeformableObjectType *object1, DeformableObjectType *object2, InteractionDataType &interactionData) {
		double areas[3];
		//compute the intersection between the 2 meshes
		vtkSmartPointer<vtkPolyData> intersect = vtkSmartPointer<vtkPolyData>::New(); //keep it here as a smart pointer: it will be incorporated into the interactionData object.
		BooleanOperation2DPolygons_WithAreaComputation(object1->GetObjectAsVTKPolyData(), object2->GetObjectAsVTKPolyData(), intersect, areas, SETINTERSECTION);
		if (areas[2]!=0) {
			//there is intersection, create a structure to host the interaction data
			interactionData.volume = areas[2];
			interactionData.interactionCost = m_costFunction->Evaluate(areas[0], areas[1], areas[2]);//areas[2] / (areas[0]+areas[1]);
			interactionData.interactionCostFlag = true;
			interactionData.intersectionPolyData = intersect;
		}
		else interactionData.interactionCostFlag = false; //internally, this flag indicates that NO interaction is present between the objects.
	}


	/** given two 2D objects, compute what their interaction cost would be */
	inline void ComputePairWiseObjectInteractionData_3D(DeformableObjectType *object1, DeformableObjectType *object2, InteractionDataType &interactionData) {
		m_triangFilter1->SetInput(object1->GetObjectAsVTKPolyData());
		m_triangFilter2->SetInput(object2->GetObjectAsVTKPolyData());
		m_filter->SetInput(0, m_triangFilter1->GetOutput() ); m_filter->SetInput(1, m_triangFilter2->GetOutput() ); m_filter->Update();
		if (m_filter->GetOutput()->GetNumberOfPoints()>0) {
			//there is intersection, create a structure to host the interaction data
			m_massProp1->SetInput( m_triangFilter1->GetOutput() ); m_massProp1->Update();
			m_massProp2->SetInput( m_triangFilter2->GetOutput() ); m_massProp2->Update();							
			m_massProp3->SetInput( m_filter->GetOutput() );        m_massProp3->Update();
			interactionData.volume = m_massProp3->GetVolume();
			interactionData.interactionCost = m_costFunction->Evaluate(m_massProp1->GetVolume(), m_massProp2->GetVolume(), m_massProp3->GetVolume());
			interactionData.interactionCostFlag = true;
			interactionData.intersectionPolyData = vtkSmartPointer<vtkPolyData>::New();
			interactionData.intersectionPolyData->DeepCopy(m_filter->GetOutput());
		}
		else interactionData.interactionCostFlag = false; //internally, this flag indicates that NO interaction is present between the objects.
	}

private:
	VTKObjectOverlap(const Self&);	//purposely not implemented
	const Self & operator=( const Self & );	//purposely not implemented
};


} // namespace psciob

#endif /* __VTKOBJECTOVERLAP_H_ */


