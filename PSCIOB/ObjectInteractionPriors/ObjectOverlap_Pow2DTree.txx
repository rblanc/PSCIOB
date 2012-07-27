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
 * \file ObjectOverlap_Pow2DTree.txx
 * \author Rémi Blanc 
 * \date 8. June 2012
 */

#ifndef __OBJECTOVERLAP_POW2DTREE_TXX_
#define __OBJECTOVERLAP_POW2DTREE_TXX_


#include "ObjectOverlap_Pow2DTree.h"

namespace psciob {


//MODIFY THE MECHANISM FOR PassObjectToChildren
//there should be one version which ADDS objects (and collect the potential interactions)
// and another for remove which does not care about it
// and another which adds without caring about the others...

/** Add an object to the tree */
template<class TScene>
void Pow2DTree_Object<TScene>::AddObject(ObjectType *obj, IDSetType &potentiallyOverlappingObjects) {
	m_nbObjects++;
	if (m_hasChildren) AddObjectToChildren(obj, potentiallyOverlappingObjects);
	else {
		if (m_depth < m_maxTreeDepth && m_nbObjects > m_maxNbObjPerNode) {
			CreateChildren();
			AddObjectToChildren(obj, potentiallyOverlappingObjects);
		}
		else {
			for (IDSetIteratorType it = m_objects.begin() ; it!=m_objects.end() ; it++) potentiallyOverlappingObjects.insert(*it);
			//std::cout<<"  inserting object with id = "<<obj->id<<" in tree, depth = "<<m_depth<<", new nb obj = "<<m_nbObjects<<", bbox = "<<m_bbox<<" & obj bbox = "<<obj->obj->GetPhysicalBoundingBox()<<std::endl;
			//std::cout<<"     potentially overlapping objects: "<<potentiallyOverlappingObjects.size()<<", list: ";
			//for (IDSetIteratorType it = m_objects.begin() ; it!=m_objects.end() ; it++) std::cout<<*it<<" ";
			//std::cout<<std::endl;
			m_objects.insert(obj->id);	
		}
	}
}

/** Remove an object from the tree */
template<class TScene>
void Pow2DTree_Object<TScene>::RemoveObject(ObjectType *obj) {
	m_nbObjects--;
			
	if (m_hasChildren && m_nbObjects < m_minNbObjPerNode) DestroyChildren();
			
	if (m_hasChildren) RemoveObjectFromChildren(obj);
	else {
		//m_objects.erase(obj->id);
		if (!m_objects.erase(obj->id)) std::cout<<"WARNING: Pow2DTree_Object<TScene>::RemoveObject -- could not find the requested object "<<obj->id<<" in the tree, at depth "<<m_depth<<std::endl;
		//else std::cout<<"  removed object with id = "<<obj->id<<" from the tree, at depth = "<<m_depth<<", bbox = "<<m_bbox<<std::endl;
	}
}

/** Pass an object to the relevant child(ren), either for insertion or deletion */
template<class TScene>
void Pow2DTree_Object<TScene>::AddObjectToChildren(ObjectType *obj, IDSetType &potentiallyOverlappingObjects) {
	//std::cout<<"  trying to add object with id "<<obj->id<<" to the children node."<<std::endl;
	//1: find out with which children does the obj interacts ; there is interaction if the bounding box is not fully outside that of the child
	const vnl_vector<double> &bbox = obj->obj->GetPhysicalBoundingBox(); bool intersect;
	//std::cout<<"             AddObjectToChildren, the object bbox is "<<bbox<<std::endl;
	//for (NodeIteratorType it = m_children.begin() ; it != m_children.end() ; it++) { //check intersection between bbox of the object, and that of the child
	for (unsigned i=0 ; i<m_nbChildren ; i++) {
		//const vnl_vector<double> &childBox = (*it)->GetBoundingBox(); intersect = true;
		const vnl_vector<double> &childBox = m_children[i]->GetBoundingBox(); intersect = true;
		for (unsigned d=0 ; d<D ; d++) {
			if ( bbox(2*d+1)<childBox(2*d) ) { intersect = false; break; }
			if ( bbox(2*d)>childBox(2*d+1) ) { intersect = false; break; }
		}		
		//if (intersect) (*it)->AddObject(obj, potentiallyOverlappingObjects);
		if (intersect) m_children[i]->AddObject(obj, potentiallyOverlappingObjects);
	}
}

/** Pass an object to the relevant child(ren), either for insertion or deletion */
template<class TScene>
void Pow2DTree_Object<TScene>::RemoveObjectFromChildren(ObjectType *obj) {
	//1: find out with which children does the obj interacts ; there is interaction if the bounding box is not fully outside that of the child
	const vnl_vector<double> &bbox = obj->obj->GetPhysicalBoundingBox(); bool intersect;
	for (unsigned i=0 ; i<m_nbChildren ; i++) {
		const vnl_vector<double> &childBox = m_children[i]->GetBoundingBox(); intersect = true;
		for (unsigned d=0 ; d<D ; d++) {
			if ( bbox(2*d+1)<childBox(2*d) ) { intersect = false; break; }
			if ( bbox(2*d)>childBox(2*d+1) ) { intersect = false; break; }
		}
		if (intersect) m_children[i]->RemoveObject(obj);
	}
}

/** Pass an object to the relevant child(ren), either for insertion or deletion */
template<class TScene>
void Pow2DTree_Object<TScene>::PassObjectToChildren(ObjectType *obj) {
	//1: find out with which children does the obj interacts ; there is interaction if the bounding box is not fully outside that of the child
	const vnl_vector<double> &bbox = obj->obj->GetPhysicalBoundingBox();
	
	bool intersect;
	for (unsigned i=0 ; i<m_nbChildren ; i++) {//for (NodeIteratorType it = m_children.begin() ; it != m_children.end() ; it++) {
		//check intersection between bbox of the object, and that of the child
		const vnl_vector<double> &childBox = m_children[i]->GetBoundingBox();
		intersect = true;
		for (unsigned d=0 ; d<D ; d++) {
			if ( bbox(2*d+1)<childBox(2*d) ) { intersect = false; break; }
			if ( bbox(2*d)>childBox(2*d+1) ) { intersect = false; break; }
		}
		IDSetType voidSet;
		if (intersect) m_children[i]->AddObject(obj, voidSet);		
	}
}
	
/** Create a set of children, and distribute the current objects among them */
template<class TScene>
void Pow2DTree_Object<TScene>::CreateChildren() {
	vnl_vector<double> childBox(D2), selector(D); selector.fill(0); int dd;
	for (unsigned i=0 ; i<m_nbChildren ; i++) {
		//compute the bbox of the child
		//std::cout<<"child "<<i<<" current selector: "<<selector<<std::endl;
		for (unsigned d=0 ; d<D ; d++) {
			if (selector(d)==0) { childBox(2*d) = m_bbox(2*d); childBox(2*d+1) = m_center(d); selector(d)=1; }
			else { 
				childBox(2*d) = m_center(d); childBox(2*d+1) = m_bbox(2*d+1); selector(d)=0; 
				dd=d-1; 
				while(dd>=0) {
					if (selector(dd)==1) { selector(dd--)=0; }
					else                 { selector(dd)=1; break; }
				}
			}
		}
		//std::cout<<"child "<<i<<" final selector: "<<selector<<std::endl;
		//now, create a new child
		Self *node = (Self *)(new Self(childBox, m_depth+1, m_scene));
		m_children.push_back( node );
		//m_children.push_back = new Self(childBox, m_depth+1, m_scene);
	}
	
	//now distribute the objects of this node to the children
	for (IDSetIteratorType it = m_objects.begin() ; it != m_objects.end() ; it++) { PassObjectToChildren(m_scene->GetObject(*it)); }
	//std::cout<<"   Distributed the current object to the children OK"<<std::endl;	
	m_objects.clear();
	//std::cout<<"   cleared local objects"<<std::endl;	

	m_hasChildren = true;	
	//std::cout<<"   indicate this is no longer a leaf"<<std::endl;	
}

/** Collect all balls from the children, store them in the current node, and kill the children */
template<class TScene>
void Pow2DTree_Object<TScene>::DestroyChildren() {
	CollectObjects(m_objects);
	
	for (unsigned i=0 ; i<m_nbChildren ; i++) { delete m_children[i]; }
	//for (unsigned i=0 ; i<m_children.size() ; i++) { delete m_children[i]; }
	m_children.clear();
	
	m_hasChildren = false;
}
	
/** Collect all the objects from the children, and add them to the provided set */
template<class TScene>
void Pow2DTree_Object<TScene>::CollectObjects(IDSetType &objSet) {
	if (m_hasChildren) for (unsigned i=0 ; i<m_nbChildren ; i++)                                     m_children[i]->CollectObjects(objSet);
	else               for (IDSetIteratorType it = m_objects.begin() ; it != m_objects.end() ; it++) objSet.insert(*it);
}
	
	
	

/** Collect the list of objects which share the same node(s) this object would occupy */
template<class TScene>
void Pow2DTree_Object<TScene>::CollectPotentiallyOverlappingObjects(DeformableObjectType *obj, IDSetType &potentiallyOverlappingObjects) {
	if (m_hasChildren) CollectPotentiallyOverlappingObjectsFromChildren(obj, potentiallyOverlappingObjects);
	else { for (IDSetIteratorType it = m_objects.begin() ; it!=m_objects.end() ; it++) potentiallyOverlappingObjects.insert(*it); }
}

/** Collect the list of objects which share the same children node(s) this object would occupy */
template<class TScene>
void Pow2DTree_Object<TScene>::CollectPotentiallyOverlappingObjectsFromChildren(DeformableObjectType *obj, IDSetType &potentiallyOverlappingObjects) {
	const vnl_vector<double> &bbox = obj->GetPhysicalBoundingBox(); bool intersect;
	for (unsigned i=0 ; i<m_nbChildren ; i++) {
		const vnl_vector<double> &childBox = m_children[i]->GetBoundingBox(); intersect = true;
		for (unsigned d=0 ; d<D ; d++) {
			if ( bbox(2*d+1)<childBox(2*d) ) { intersect = false; break; }
			if ( bbox(2*d)>childBox(2*d+1) ) { intersect = false; break; }
		}		
		if (intersect) m_children[i]->CollectPotentiallyOverlappingObjects(obj, potentiallyOverlappingObjects);
	}	
}


} // namespace psciob

#endif /* __OBJECTOVERLAP_POW2DTREE_TXX_ */