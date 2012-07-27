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

#include "GeneralUtils.h"
#include "SetsUtils.h"
#include "time.h"

#include "MultivariatePDF.h"

using namespace psciob;

void TestMultivariatePDF() {
	std::cout<<"\n\nTEST Multivariate PDFs\n\n"<<std::endl;

	MVN_PDF::Pointer pdf = MVN_PDF::New();
	std::cout<<"random sample: "<<pdf->DrawSample()<<std::endl;
}

void TestSet1() {
	std::cout<<"\n\nTEST 1\n\n"<<std::endl;
	clock_t t0 = clock();

	SetOfMutuallyRelatedLabels<unsigned int> labelSet;
	for (unsigned i=0 ; i<5000 ; i++) {
		labelSet.AddLabel(i);
	}

	for (unsigned i=2, j=3 ; i<1000 ; i+=2, j+=3) {
		labelSet.RelateLabels(i,j);
	}

	for (unsigned i=5 ; i<4500 ; i+=7 ) { labelSet.RemoveLabel(i); }

	for (unsigned i=0 ; i<1000 ; i++) { labelSet.AddLabel(i); }

	std::cout<<"time to add object, create some relations, remove objects, etc...: "<<(clock()-t0)/((double)CLOCKS_PER_SEC)<<" s."<<std::endl;

	std::cout<<"number of objects: "<<labelSet.GetNumberOfObjects()<<std::endl;
	std::cout<<"object with label 2: "<<labelSet.GetEntry(2)->id<<", numbner of interactions: "<<labelSet.GetEntry(2)->relatedIDs.size()<<std::endl;
	if ( labelSet.GetEntry(2)->relatedIDs.size() > 0 ) {
		std::cout<<"interaction is: "<<labelSet.GetEntry(2)->id<<" with "<<*(labelSet.GetEntry(2)->relatedIDs.begin())<<std::endl;
	}

}

void TestSet2() {
	std::cout<<"\n\nTEST 2\n\n"<<std::endl;
	clock_t t0 = clock();

	class RegionData {
	public:
		RegionData() : nbPixels(0), maxDepth(0) {}
		unsigned nbPixels;
		double maxDepth;
	};

	RegionData data;
	std::cout<<"data.nbPixels  = "<<data.nbPixels<<std::endl;

	data.nbPixels = 3;
	std::cout<<"data.nbPixels  = "<<data.nbPixels<<std::endl;

	RegionData data2 = data;
	std::cout<<"data2.nbPixels  = "<<data2.nbPixels<<std::endl;

	class RelationData {
	public:
		RelationData() : contactDepth(0) {}
		double contactDepth;
	};



	SetMutuallyRelatedObjects<unsigned int, RegionData, RelationData> labelSet;
	for (unsigned i=0 ; i<5000 ; i++) {
		labelSet.AddObject(data);
	}

	RelationData rData;
	for (unsigned i=2, j=3 ; i<1000 ; i+=2, j+=3) {
		rData.contactDepth = rand();
		labelSet.RelateObjects(i,j, rData);
	}

	for (unsigned i=5 ; i<4500 ; i+=7 ) { labelSet.RemoveObject(i); }

	for (unsigned i=0 ; i<1000 ; i++) { labelSet.AddObject(data); }

	std::cout<<"time to add object, create some relations, remove objects, etc...: "<<(clock()-t0)/((double)CLOCKS_PER_SEC)<<" s."<<std::endl;

	std::cout<<"number of objects: "<<labelSet.GetNumberOfObjects()<<std::endl;
	std::cout<<"object with label 2: "<<labelSet.GetEntry(2)->id<<", numbner of interactions: "<<labelSet.GetEntry(2)->relatedIDs.size()<<std::endl;
	if ( labelSet.GetEntry(2)->relatedIDs.size() > 0 ) {
		std::cout<<"interaction is: "<<labelSet.GetEntry(2)->id<<" with "<<labelSet.GetEntry(2)->relatedIDs.begin()->first<<", interaction data: "<<labelSet.GetEntry(2)->relatedIDs.begin()->second.contactDepth<<std::endl;
	}
	if ( labelSet.GetEntry(4)->relatedIDs.size() > 0 ) {
		std::cout<<"interaction is: "<<labelSet.GetEntry(4)->id<<" with "<<labelSet.GetEntry(4)->relatedIDs.begin()->first<<", interaction data: "<<labelSet.GetEntry(4)->relatedIDs.begin()->second.contactDepth<<std::endl;
	}

}

int main(int argc, char** argv) {

	TestMultivariatePDF();

	TestSet1();

	TestSet2();

	return 1;
}
