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
 * \file ObjectOverlap_Pow2DTree.h
 * \author Rémi Blanc 
 * \date 8. June 2012
 */

#ifndef __OBJECTOVERLAP_POW2DTREE_H_
#define __OBJECTOVERLAP_POW2DTREE_H_

#include "ObjectInteractionManager.h"

namespace psciob {

/**\class Pow2DTree_Object 
 * \brief Pow2DTree_Object: class for the quad/oc-tree structure for collision detection
 * It is templated against the object type (expected to be a ObjectInScene object coherent with the scene)
 *    the scene dimensionality, maximum number of levels in the tree, minimum and maximum number of objects in a node are fixed
 * 
 * This is an helper class for ObjectOverlap_Pow2DTree, and is not intended to be instanciated by the user
 * 
 * To determine to which node(s) an object belongs to, the axis-aligned boundings boxes are compared. Only in case of non-intersection of the 2 bboxes, the object does not belong there.
*/
template<class TScene>
class Pow2DTree_Object {
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

	typedef Pow2DTree_Object                 Self;
	typedef typename std::vector<Self *>     NodeListType;
	typedef typename NodeListType::iterator  NodeIteratorType;

	typedef typename SceneType::DeformableObjectType DeformableObjectType;
	/** Constructor requires the corresponding bounding box, and the depth of the node to be provided */
	Pow2DTree_Object(vnl_vector<double> bbox, unsigned int d, TScene *scene) : m_depth(d), m_nbObjects(0), m_hasChildren(false) {
		if (bbox.size()!=D2) throw DeformableModelException("Pow2DTree_Object::Pow2DTree_Object() -- invalid dimensionality");
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
	~Pow2DTree_Object() {
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
	void AddObject(ObjectType *obj, typename IDSetType &potentiallyOverlappingObjects);

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

	

/**\class ObjectOverlap_Pow2DTree 
 * \brief ObjectOverlap_Pow2DTree: Wrapper class for managing interactions between objects using a tree structure (quad/oc-tree)
 * The tree structure allows to sort out which pairs of object need to be tested for intersections, directly reducing the associated computational burden.
 * \sa Pow2DTree_Object_Base
 * The second template parameter is the type of ObjectInteractionManager actually implementing the object interactions (e.g. PixelSetObjectOverlap, ...)
 * 
*/

//IDEA: instead of checking all pairs of objects, I should partition the scene space with an quad/oc-tree
//check e.g. http://www.videotutorialsrock.com/opengl_tutorial/collision_detection/text.php

//CONCRETE CLASS
template<class TScene, class TInteractionCalculator>
class ObjectOverlap_Pow2DTree : public ObjectInteractionManager<TScene> {
public:
	/** Standard class typedefs. */
	typedef ObjectOverlap_Pow2DTree       Self;
	typedef itk::LightObject              Superclass;
	typedef itk::SmartPointer<Self>       Pointer;
	typedef itk::SmartPointer<const Self> ConstPointer;
	/** Run-time type information (and related methods). */
	itkTypeMacro(ObjectOverlap_Pow2DTree, Superclass);
	itkNewMacro(ObjectOverlap_Pow2DTree);

	static const unsigned int Dimension = SceneType::Dimension;

	typedef TInteractionCalculator		  InteractionCalculatorType;
	typedef Pow2DTree_Object<SceneType>   TreeType;
	
	/** Attach the scene to the InteractionManager ; the user doesn't need to call this function
	 * \todo make it protected and use friendship to enable usage by the Scene
	 */
	void SetScene(SceneType* scene) { 
		m_scene = scene; 
		m_tree = new TreeType(m_scene->GetSceneBoundingBox(), 1, m_scene);
		m_interactionCalculator = InteractionCalculatorType::New();
		m_interactionCalculator->SetScene(m_scene);
	}

	/** Clear any scene-related cached information, as the scene is getting cleared itself 
	* use friendship to hide this function from the public interface...
	*/
	void Clear() { m_tree->Reset(); }

	/** Specifies a cost function to be used to compute the cost of the intersection 
	* \warning this requires the scene to be set before-hand, so that the internal object actually computing the interactions is properly defined.
	*/
	void SetIntersectionCostFunction(PairWiseIntersectionCostFunction *f) { 
		if (!m_interactionCalculator) throw DeformableModelException("ObjectOverlap_Pow2DTree::SetIntersectionCostFunction : the scene has to be set before"); 
		m_costFunction = f; m_interactionCalculator->SetIntersectionCostFunction(m_costFunction); 
	}


	/** Add an object to the Interaction Manager -> look for interactions with other objects of the scene 
	* 1: position the object in the tree, and collect potentially overlapping objects from there
	* 2: check these potential overlaps, and fill the structures.
	*/
	inline void AddObject(ObjectInScene *objectPtr) {
		//add the object to the tree, and collect the potentially overlapping objects
		std::set<IDType> setPotentialOverlappingObjects;
		m_tree->AddObject(objectPtr, setPotentialOverlappingObjects);
		//for all potentially overlapping objects, compute the interaction.
		if (!m_monitorInteractionsOnTheFly) return;
		for (std::set<IDType>::iterator it = setPotentialOverlappingObjects.begin() ; it != setPotentialOverlappingObjects.end() ; it++) {
			if (objectPtr->id == *it) { continue; }
			if ( !TestBoundingBoxesIntersection_NoCheck(objectPtr->obj->GetPhysicalBoundingBox(), m_scene->GetObject(*it)->obj->GetPhysicalBoundingBox()) ) continue;
			InteractionDataType interactionData;
			ComputePairWiseObjectInteractionData(objectPtr, m_scene->GetObject(*it), interactionData);
			if ( !interactionData.interactionCostFlag ) continue;
			//Register the interaction, bilaterally: store the information in both objects

			objectPtr->interactionData[*it] = interactionData;
			m_scene->GetObject(*it)->interactionData[objectPtr->id] = interactionData;
		}
	}

	/** Remove an object, from the tree, and clean the corresponding interactions. */
	inline void RemoveObject(ObjectInScene *objectPtr) {
		m_tree->RemoveObject(objectPtr); 
		if (!m_monitorInteractionsOnTheFly) return;
		for (ObjectInteractionMapType::iterator it = objectPtr->interactionData.begin() ; it != objectPtr->interactionData.end() ; ++it) {
			//??keep this sanity check??

			//if ( m_scene->GetObject( it->first )->interactionData.erase( objectPtr->id ) ==0 ) { throw DeformableModelException( "ObjectInteractionManager::RemoveObject found a non-bilateral interaction... should never happen!" + stringify(objectPtr->id) + " and " + stringify(it->first) ); }
			m_scene->GetObject( it->first )->interactionData.erase( objectPtr->id ); //this would be enough if I remove the sanity check.
		}
		//objectPtr->interactionData.clear(); //probably not necessary...
	}

	/** Modifies the parameter of an object	*/
	inline void ModifyObjectParameters(ObjectInScene *objectPtr, ObjectInScene *newObject) {
		//this may be optimized a bit, by only updating the structures which can be updated, instead of deleting and re-creating them
		//std::cout<<"Removing interaction between obj "<<objectPtr->id<<" and "<<it->first<<std::endl;
		this->RemoveObject(objectPtr);
		this->AddObject(newObject);
	}


	/** Given an arbitrary object, look in the scene with which object may interact with it, based on their axes aligned bounding boxes*/
	virtual std::vector<IDType> IdentifyInteractingObjectsInScene(DeformableObjectType *object, IDType ignoredLabel = 0) {
		std::vector<IDType> listPotentiallyInteractingObjects;
		//compute the set
		std::set<IDType> setPotentialOverlappingObjects;
		m_tree->CollectPotentiallyOverlappingObjects(object, setPotentialOverlappingObjects);
		
		for (std::set<IDType>::iterator it = setPotentialOverlappingObjects.begin() ; it!=setPotentialOverlappingObjects.end() ; ++it) {
			if ( *it != ignoredLabel ) {
				if ( TestBoundingBoxesIntersection_NoCheck(object->GetPhysicalBoundingBox(), m_scene->GetObject(*it)->obj->GetPhysicalBoundingBox()) )
					listPotentiallyInteractingObjects.push_back(*it);
			}
		}
		return listPotentiallyInteractingObjects;
	}
	

	/** given two arbitrary objects, compute how they interact
	* does not perform a bounding box check, which should therefore be made prior to calling this method...
	* ( see the function: TestBoundingBoxesIntersection_NoCheck (or TestBoundingBoxesIntersection for a safer version) )
	*/
	inline void ComputePairWiseObjectInteractionData(DeformableObjectType *object1, DeformableObjectType *object2, InteractionDataType &interactionData) {
	   m_interactionCalculator->ComputePairWiseObjectInteractionData(object1, object2, interactionData);
	}

	/** given two objects from the scene, compute how they interact 
	* does not perform a bounding box check, which should therefore be made prior to calling this method...
	* ( see the function: TestBoundingBoxesIntersection_NoCheck (or TestBoundingBoxesIntersection for a safer version) )
	* \warning the scene should be set before calling this function (no checks are performed)
	*/
	inline void ComputePairWiseObjectInteractionData(ObjectInScene *object1, ObjectInScene *object2, InteractionDataType &interactionData) {
		m_interactionCalculator->ComputePairWiseObjectInteractionData(object1, object2, interactionData);
	}


protected:	
	ObjectOverlap_Pow2DTree() : ObjectInteractionManager() {
		m_tree = NULL;
		m_interactionCalculator = NULL;
	};
	virtual ~ObjectOverlap_Pow2DTree() {
		if (m_tree!=NULL) { m_tree->Reset(); delete m_tree; m_tree=0; }
	};	

	TreeType *m_tree;
	typename InteractionCalculatorType::Pointer m_interactionCalculator;
		
private:
	ObjectOverlap_Pow2DTree(const Self&);	//purposely not implemented
	const Self & operator=( const Self & );	//purposely not implemented
};


} // namespace psciob


#include "ObjectOverlap_Pow2DTree.txx"

#endif /* __OBJECTOVERLAP_POW2DTREE_H_ */
