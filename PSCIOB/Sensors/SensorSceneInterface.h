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
* \file SensorSceneInterface.h
* \author Rémi Blanc 
* \date 17. August 2012
*/

#ifndef __SENSORSCENEINTERFACE_H_
#define __SENSORSCENEINTERFACE_H_

#include "BaseScene.h"

namespace psciob {

/**
 * \class SensorSceneInterface
 * \brief SensorSceneInterface
 * this is a simple class used by all sensor to communicate with the scene
 * in particular to inform the sensor which parts of the scene have been updated since the sensor has been updated last.
 */


//ABSTRACT CLASS
template<class TScene>
class SensorSceneInterface : public itk::LightObject {
public:
	/** Standard class typedefs. */
	typedef SensorSceneInterface          Self;
	typedef itk::LightObject              Superclass;
	typedef itk::SmartPointer<Self>	      Pointer;
	typedef itk::SmartPointer<const Self> ConstPointer;
	/** Base Types to which all inherited classes can refer */
	typedef Self                          BaseClass;
	typedef Pointer                       BaseClassPointer;
	/** Run-time type information (and related methods). */
	itkTypeMacro(SensorSceneInterface, itk::LightObject);
	itkNewMacro(Self);

	static const unsigned int Dimension = TScene::Dimension;

	typedef TScene                             SceneType;
	typedef typename SceneType::ObjectInScene  ObjectInScene;
	typedef typename SceneType::IDType         LabelType;

	typedef typename SceneType::DeformableObjectType::LabelObjectType OriginalLabelObjectType;
	typedef typename SceneType::DeformableObjectType::LabelMapType    OriginalLabelMapType;
	typedef typename SceneType::DeformableObjectType::LabelMapType    ObjectLabelMapType;

	typedef	typename SceneType::LabelObjectType    InputLabelObjectType;
	typedef	typename SceneType::LabelMapType       InputLabelMapType;
	typedef	typename SceneType::LabelImageType     InputLabelImageType;
	typedef	typename SceneType::BinaryImageType    InputBinaryImageType;
	typedef	typename SceneType::TexturedImageType  InputTexturedImageType;

	typedef typename SceneType::BinaryImageType::RegionType SceneRegionType;
	typedef typename std::vector<SceneRegionType>           SceneRegionListType;

	/** Set the scene to observe
	* By default, the sensor is connected to the scene, to get informed of the scene regions that are being modified
	* so it can update itself only on modified regions
	*/
	void SetScene(SceneType *scene) { 
		m_scene = scene; TrackSceneModifications(true); 
	}
	/** Get a pointer to the observed scene */
	SceneType* GetScene() { return m_scene;}

	/** Asks the scene to inform the sensor of its modifications */
	void TrackSceneModifications(bool b = true) { 
		m_connectedToScene=b;
		if (m_connectedToScene) {
			m_scene->ConnectSensor(this);
			m_sceneModifiedRegions.clear();
		}
		else { 
			m_scene->DisconnectSensor(this);
			m_sceneModifiedRegions.clear();
		}
	}

	/** Informs the sensor that a particular region of the scene has been modified and will require an update 
	* \todo: the code is a bit suboptimal - check code for optimization hints
	*/
	void AddModifiedSceneRegion(SceneRegionType sceneRegion) {
		if ( m_sceneModifiedRegions.empty() ) {m_sceneModifiedRegions.push_back(sceneRegion);}
		else { 
			//TODO: this is fairly suboptimal..., currently, I change m_sceneModifiedRegions to the axis-aligned bbox that contains both the old and new regions.
			//OPTIMIZATION: if the regions don't intersect, add a new region to the vector, otherwise merge some regions 
			// this can become a bit complex because different existing elements of the vector may need to be merged after a while, etc...
			// other possibility is, on the contrary, to increase the number of regions, extracting minimal rectangular parts
			//  _______             _______
			//  |  1  |             |  1| |
			//  |   __|__           |   | |__
			//  |___|_|  |     =>   |___|2| 3|
			//      |  2 |              | |  |
			//      |____|              |_|__|
			InputLabelImageType::IndexType modifiedStart = m_sceneModifiedRegions[0].GetIndex();
			InputLabelImageType::SizeType  modifiedSize = m_sceneModifiedRegions[0].GetSize();		
			const InputLabelImageType::IndexType &currentStart = sceneRegion.GetIndex();
			const InputLabelImageType::SizeType  &currentSize  = sceneRegion.GetSize();
			unsigned int modifiedEnd;
			for (unsigned i=0 ; i<Dimension ; i++) {
				modifiedEnd = std::max<unsigned>(modifiedStart[i] + modifiedSize[i], currentStart[i] + currentSize[i]);
				modifiedStart[i] = std::min<unsigned>( modifiedStart[i], currentStart[i] );
				modifiedSize[i] = modifiedEnd - modifiedStart[i];
			}
			m_sceneModifiedRegions[0].SetIndex(modifiedStart); m_sceneModifiedRegions[0].SetSize(modifiedSize);
		}
	}


	SceneRegionListType GetListSceneRegionsToUpdate() { return m_sceneModifiedRegions; }

	void ClearListRegionsToUpdate() {
		m_sceneModifiedRegions.clear();
	}

	bool m_connectedToScene;
protected:
	SensorSceneInterface() : m_connectedToScene(false) { 

	}
	~SensorSceneInterface() {
		m_scene->DisconnectSensor(this);
	};

	typename SceneType::Pointer	m_scene;
	typename SceneRegionListType m_sceneModifiedRegions;

private:
	SensorSceneInterface(const Self&);		//purposely not implemented
	const Self & operator=( const Self & );	//purposely not implemented
};





} // namespace psciob

#endif /* __SENSORSCENEINTERFACE_H_ */
