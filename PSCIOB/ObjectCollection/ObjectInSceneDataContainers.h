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
 * \file ObjectInSceneDataContainers.h
 * \author Rémi Blanc 
 * \date 29. February 2012
 */

#ifndef OBJECTINSCENEDATACONTAINERS_H
#define OBJECTINSCENEDATACONTAINERS_H

//#include <CommonTypes.h>
#include "ObjectPixelSet.h"

namespace psciob {


/**\class ObjectData_BaseContainer
* \brief ObjectData_BaseContainer - abstract base container, defining the interface for all object data storage
*/
class ObjectData_BaseContainer {
public:
	ObjectData_BaseContainer() {}
	virtual void InvalidateData() = 0;
};

/**\class EmptyContainer
 * \brief EmptyContainer basic container - essentially for testing purposes
*/
class EmptyObjectContainer : public ObjectData_BaseContainer {
public:
	EmptyObjectContainer() {}
	inline void InvalidateData() {}
};


/**\class ObjectNbPixelsContainer
 * \brief ObjectNbPixelsContainer basic container - essentially for testing purposes
*/
class ObjectNbPixelsContainer : public ObjectData_BaseContainer {
public:
	ObjectNbPixelsContainer() : nbPixels(0) {}
	unsigned int nbPixels;
	inline void InvalidateData() {nbPixels=0;}
};

/**\class ObjectPriorCostContainer
 * \brief ObjectPriorCostContainer container with storage space for a prior cost
*/
class ObjectPriorCostContainer : public ObjectData_BaseContainer {
public:
	ObjectPriorCostContainer() : priorCostFlag(false) {}
	inline void InvalidateData() { priorCostFlag = false; }

	bool priorCostFlag;
	double priorCost;
};


/**\class ObjectPriorCostAndPixelSetContainer
 * \brief ObjectPriorCostAndPixelSetContainer: container for a prior cost and an list of offContext pixel addresses
*/
class ObjectPriorCostAndPixelSetContainer : public ObjectPriorCostContainer {
public:
	ObjectPriorCostAndPixelSetContainer() : ObjectPriorCostContainer(), offContextPixelSetFlag(false) {}
	inline void InvalidateData() { ObjectPriorCostContainer::InvalidateData(); offContextPixelSetFlag=false;}
	std::vector<unsigned long> offContextPixelSet;	bool offContextPixelSetFlag;
};

/**\class ObjectCostsContainer
 * \brief CostsContainer container with storage space for various cost terms, and flags to indicate whether they are uptodate
 * priorCost ; dataCost ; offContextDataCost
 * warning: data costs for an object may be invalidated by modifications of other objects in the scene, with respect to e.g. overlaps, shadowing, etc...
*/
class ObjectCostsContainer : public ObjectData_BaseContainer {
public:
	ObjectCostsContainer() : priorCostFlag(false), dataCostFlag(false), offContextDataCostFlag(false) {}
	inline void InvalidateData() { priorCostFlag = false; dataCostFlag=false; offContextDataCostFlag=false; }
	inline void InvalidateInContextData() { dataCostFlag=false; }

	bool priorCostFlag, dataCostFlag, offContextDataCostFlag;
	double priorCost, dataCost, offContextDataCost;
};

/**\class ObjectCostsAndPixelSetContainer
 * \brief ObjectCostsAndPixelSetContainer: container with storage space for various cost terms, and flags to indicate whether they are uptodate
 * as well as pixel sets for the off-context scene, 
*/
class ObjectCostsAndPixelSetContainer : public ObjectCostsContainer {
public:
	ObjectCostsAndPixelSetContainer() : ObjectCostsContainer(), offContextPixelSetFlag(false) {}
	inline void InvalidateData() { ObjectCostsContainer::InvalidateData(); offContextPixelSetFlag=false;}
	std::vector<unsigned long> offContextPixelSet; bool offContextPixelSetFlag;
};


//other ideas would be vectors of parameters
//speed
//etc...






/**\class PixelSetIntersectionContainer
 * \brief PixelSetIntersectionContainer container containing the full set of overlapping voxels, and a cost of interaction
*/
class PixelSetIntersectionContainer {
public:
	PixelSetIntersectionContainer() : nbOverlappingPixels(0), interactionCostFlag(false) {}
	inline void InvalidateData() { interactionCostFlag = false; }

	unsigned int nbOverlappingPixels;
	std::vector<unsigned long> intersectionPixels;
	bool interactionCostFlag;
	double interactionCost;	
};

/**\class LabelObjectIntersectionContainer
 * \brief LabelObjectIntersectionContainer container containing the full set of overlapping voxels in the form of a LabelObject representation
 * and a cost of interaction
*/
template<class TLabel, unsigned int VDimension>
class LabelObjectIntersectionContainer {
public:
	LabelObjectIntersectionContainer() : nbOverlappingPixels(0), interactionCostFlag(false), intersectionObject(NULL) {}
	inline void InvalidateData() { interactionCostFlag = false; }

	unsigned int nbOverlappingPixels;
	typedef itk::LabelObject<TLabel, VDimension> LabelObjectType;
	typename LabelObjectType::Pointer intersectionObject;
	bool interactionCostFlag;
	double interactionCost;	
};


/**\class VTKIntersectionContainer
 * \brief VTKIntersectionContainer container containing the area/volume of intersection, the corresponding vtkPolyData, and a cost of interaction
*/
class VTKIntersectionContainer {
public:
	VTKIntersectionContainer() : volume(0), interactionCostFlag(false) {}
	inline void InvalidateData() { interactionCostFlag = false; }

	double volume;
	vtkSmartPointer<vtkPolyData> intersectionPolyData;
	bool interactionCostFlag;
	double interactionCost;	
};


} // namespace psciob

#endif /* OBJECTINSCENEDATACONTAINERS_H */
