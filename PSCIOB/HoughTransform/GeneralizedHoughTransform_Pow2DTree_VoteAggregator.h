/*
 * This file is part of the psciob library.
 *
 * Copyright (c) 2013, Remi Blanc
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
 * \file GeneralizedHoughTransform_Pow2DTree_VoteAggregator.h
 * \author Rémi Blanc 
 * \date 8. June 2012
 */

#ifndef __GeneralizedHoughTransform_Pow2DTree_VoteAggregator_H_
#define __GeneralizedHoughTransform_Pow2DTree_VoteAggregator_H_

#include "ObjectInteractionManager.h"

namespace psciob {

//NOT IMPLEMENTED YET
//just copied the code from ObjectOverlap_Pow2DTree.h and renamed things
//much needs to be done to do it...

/**\class GeneralizedHoughTransform_Pow2DTree_VoteAggregator 
 * \brief GeneralizedHoughTransform_Pow2DTree_VoteAggregator: class for the quad/oc-tree structure for aggregating votes
 * It is templated against the type of Vote, which should 
 *    the scene dimensionality, maximum number of levels in the tree, minimum and maximum number of objects in a node are fixed
 * 
 * This is an helper class for GeneralizedHoughTransform_Pow2DTree_VoteAggregator, and is not intended to be instanciated by the user
 * 
 * To determine to which node(s) an object belongs to, the axis-aligned boundings boxes are compared. Only in case of non-intersection of the 2 bboxes, the object does not belong there.
*/
template<class TScene>
class GeneralizedHoughTransform_Pow2DTree_VoteAggregator {
public:
	typedef TScene SceneType;
	static const unsigned int D = TScene::Dimension;
	static const unsigned int D2= 2*D;

	//TODO: make it possible to modify these variables (at least if the tree is empty...
	//one possibility to keep it safe would be to set these exclusively through the constructor.
	static const unsigned int m_maxTreeDepth    = 5; //maximum number of levels in the tree		
	static const unsigned int m_minNbObjPerNode = 3; //if there are fewer object than this in a node, its children are suppressed (if any).
	static const unsigned int m_maxNbObjPerNode = 6; //if there are more than this number of objects in a node, it is decomposed into children nodes
	
	typedef typename SceneType::ObjectInScene ObjectType;
	typedef typename SceneType::IDType        IDType;
	typedef typename std::set<IDType>         IDSetType;
	typedef typename IDSetType::iterator      IDSetIteratorType;

	typedef GeneralizedHoughTransform_Pow2DTree_VoteAggregator Self;
	typedef typename std::vector<Self *>     NodeListType;
	typedef typename NodeListType::iterator  NodeIteratorType;

	typedef typename SceneType::DeformableObjectType DeformableObjectType;
	/** Constructor requires the corresponding bounding box, and the depth of the node to be provided */
	GeneralizedHoughTransform_Pow2DTree_VoteAggregator(vnl_vector<double> bbox, unsigned int d, TScene *scene) : m_depth(d), m_nbObjects(0), m_hasChildren(false) {
		if (bbox.size()!=D2) throw DeformableModelException("GeneralizedHoughTransform_Pow2DTree_VoteAggregator::GeneralizedHoughTransform_Pow2DTree_VoteAggregator() -- invalid dimensionality");
		m_scene = scene;
		m_bbox = bbox;		
		m_center.set_size(D); 
		m_nbChildren = 1;
		for (unsigned i=0 ; i<D ; i++) {
			m_center(i) = (m_bbox(2*i)+m_bbox(2*i+1))/2.0;
			m_nbChildren*=2; //there are 2^D children
		}
//std::cout<<"CREATING TREE NODE, depth: "<<d<<", dim = "<<D<<", bbox = "<<m_bbox<<", nb children: "<<m_nbChildren<<", center = "<<m_center<<std::endl;
	}
	
	/** Destructor */
	~GeneralizedHoughTransform_Pow2DTree_VoteAggregator() {
//std::cout<<"DESTOYING node at depth: "<<m_depth<<", bbox = "<<m_bbox<<" has Children: "<<m_hasChildren<<std::endl;
		if (m_hasChildren) DestroyChildren();
	}

	/** Clears the tree: destroy all children, and clear the list of objects, but keeps the bounding box */
	void Reset() {
		if (m_hasChildren) DestroyChildren();
		m_objects.clear();
		m_nbObjects = 0;
	}

	/** Add an object to the tree 
	* also collects the set of potentially overlapping objects (elements of the node to which the new object is added)
	*/
	void AddVote();

	/** Remove an object from the tree */ 
	void RemoveObject(ObjectType *obj);
	
	/** Modify an object 
	* also collects the set of potentially overlapping objects (elements of the node to which the new object is added)
	*/
	inline void ModifyObject(ObjectType *oldObj, ObjectType *newObj, IDSetType &potentiallyOverlappingObjects) {
		RemoveObject(oldObj);
		AddObject(newObj, potentiallyOverlappingObjects);
	}
	
protected:

	typename SceneType::Pointer m_scene;
	vnl_vector<double> m_bbox;             // m_bbox(0),m_bbox(1) = min, max coordinates in the first dimension, etc.
	vnl_vector<double> m_center; //coordinates of the box center (can be useful for splitting...)
	
	unsigned int m_nbChildren;        //this is defined as 2^D
	NodeListType m_children;          //list of children
	bool m_hasChildren;			      //flag indicating whether this is a leaf node, or if it has children
	unsigned int m_depth;		      //depth of this node
	unsigned int m_nbObjects;         //number of objects in this node and all its childs
	
	IDSetType m_objects; //set of objects in this node (empty if there are children)
	
	/** Add an object to the relevant child(ren) and collect the list of objects that were present in the modified nodes. */
	void AddObjectToChildren(ObjectType *obj, IDSetType &potentiallyOverlappingObjects);

	/** Just add the object to children, without caring about other objects */
	void PassObjectToChildren(ObjectType *obj);

	/** Pass an object to the relevant child(ren), either for insertion or deletion - and collect the list of objects that were present in the modified nodes. */
	void RemoveObjectFromChildren(ObjectType *obj);

	/** Create a set of children, and distribute the current objects among them */
	void CreateChildren();

	/** Collect all balls from the children, store them in the current node, and kill the children */
	void DestroyChildren();
	
	/** Collect all the objects from the children, and add them to the provided set */
	void CollectObjects(IDSetType &objSet);
	
	/** Get the bounding box of this node */
	inline vnl_vector<double> GetBoundingBox() { return m_bbox; }

public:
	/** Collect the list of objects which share the same node(s) this object would occupy */
	void CollectPotentiallyOverlappingObjects(DeformableObjectType *obj, IDSetType &potentiallyOverlappingObjects);
	/** Collect the list of objects which share the same children node(s) this object would occupy */
	void CollectPotentiallyOverlappingObjectsFromChildren(DeformableObjectType *obj, IDSetType &potentiallyOverlappingObjects);
};
