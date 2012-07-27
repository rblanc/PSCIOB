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

//..\Release\GeneralUtilsLib.lib


#include "GeneralUtils.h"
#include "SetsUtils.h"
#include "ITKUtils.h"
#include "time.h"
#include <vector>
#include <map>
#include <typeinfo>

using namespace psciob;


//
//void Test1(const vnl_vector<double> &v = vnl_vector<double>()) {
//	if (v.empty()) std::cout<<"empty input"<<std::endl;
//	else std::cout<<"non empty input"<<std::endl;
//}

void Test3() {
	unsigned i,j;
	std::vector<unsigned> v;
	v.push_back(0);v.push_back(1);v.push_back(2);v.push_back(3);v.push_back(4);v.push_back(5);

	j=0;
	std::cout<<"j="<<j<<std::endl;
	std::cout<<"v[++j] = "<<v[++j]<<std::endl; //updates j from 0 to 1, and return v[1]
	std::cout<<"j="<<j<<std::endl;				
	std::cout<<"v[j++] = "<<v[j++]<<std::endl; //return v[1] then updates j from 1 to 2, and 
	std::cout<<"j="<<j<<std::endl;
	std::cout<<"v[++j] = "<<v[++j]<<std::endl; //updates j from 2 to 3, and return v[3]
	std::cout<<"j="<<j<<std::endl;

	std::cout<<std::endl;
	j=0;
	while ( v[++j]==3 ) {
		std::cout<<"j="<<j<<std::endl;
	}
	std::cout<<"out... v[j] = "<<v[j]<<", j="<<j<<std::endl;


	std::map<double, bool> mymap;
	std::map<double, bool>::iterator it = mymap.begin();
	
}


//
//void Test4() {
//
//	class titi {
//	public:
//
//		struct OBJ {
//			double a;
//			std::vector<double> v;
//		};
//
//		titi() {
//			data.a = 5;
//			data.v.push_back(1);
//			data.v.push_back(2);
//		}
//
//		const OBJ* GetTitiByConstRef() {return (&data);}
//		OBJ GetTiti() {return data;}
//		OBJ* GetTitiByRef() {return (&data);}
//		OBJ* GetTitiByInvalidMethod() {return (NULL);}
//		void PrintData() { std::cout<<"titi::data.a = "<<data.a<<", data.v = "<<data.v[0]<<", "<<data.v[1]<<std::endl;}
//	private:
//		OBJ data;
//	};
//
//
//	titi a;
//	a.PrintData();
//
//	std::cout<<"\n GetTiti()"<<std::endl;
//	titi::OBJ b = a.GetTiti();
//	std::cout<<"b.a = "<<b.a<<", b.v = "<<b.v[0]<<", "<<b.v[1]<<std::endl;
//	a.PrintData();
//	
//	std::cout<<"\n c=GetTitiByRef()"<<std::endl;
//	titi::OBJ *c = a.GetTitiByRef();
//	std::cout<<"c->a = "<<c->a<<", c->v = "<<c->v[0]<<", "<<c->v[1]<<std::endl;
//	a.PrintData();
//
//	std::cout<<"\n *c++"<<std::endl;
//	c->a++; 
//	std::cout<<"c->a = "<<c->a<<", c->v = "<<c->v[0]<<", "<<c->v[1]<<std::endl;
//	a.PrintData(); // => this indeed modifies the private data of titi !!
//
//
//	std::cout<<"\n d=GetTitiByConstRef()"<<std::endl;
//	const titi::OBJ *d = a.GetTitiByConstRef(); //The compiler requests me to declare d as const 
//	std::cout<<"d->a = "<<d->a<<", d->v = "<<d->v[0]<<", "<<d->v[1]<<std::endl;
//	a.PrintData();
//
//	std::cout<<"\n *d++"<<std::endl;
////	d->a++; // the compiler forbids this operation
//	std::cout<<"d->a = "<<d->a<<", d->v = "<<d->v[0]<<", "<<d->v[1]<<std::endl;
//	a.PrintData();
//
//	std::cout<<"test invalid method..."<<std::endl;
//	titi::OBJ *e = a.GetTitiByInvalidMethod();
//	if (!e) std::cout<<"e is indeed invalid"<<std::endl;
//
//}


void TestCopyVectors() {

	std::vector<unsigned long> v1, v2;
	v1.push_back(1); v1.push_back(2); v1.push_back(3);
	v2.push_back(5);

	std::cout<<"v1 :"; for (unsigned i=0 ; i<v1.size() ; i++) std::cout<<v1[i]<<" "; std::cout<<std::endl;
	std::cout<<"v2 :"; for (unsigned i=0 ; i<v2.size() ; i++) std::cout<<v2[i]<<" "; std::cout<<std::endl;

	v2 = v1;

	std::cout<<"v1 :"; for (unsigned i=0 ; i<v1.size() ; i++) std::cout<<v1[i]<<" "; std::cout<<std::endl;
	std::cout<<"v2 :"; for (unsigned i=0 ; i<v2.size() ; i++) std::cout<<v2[i]<<" "; std::cout<<std::endl;


}


class ObjectCosts { 
public: 
	ObjectCosts(unsigned int i, double c) : id(i), localCost(c) {};
	unsigned int id; double localCost; 
};

void TestRemoveHolesInVector() {
	std::vector<ObjectCosts> listObject;
	listObject.push_back( ObjectCosts( 1, 10) );
	listObject.push_back( ObjectCosts( 0, 11) );
	listObject.push_back( ObjectCosts( 2, 12) );
	listObject.push_back( ObjectCosts( 3, 13) );
	listObject.push_back( ObjectCosts( 0, 14) );
	listObject.push_back( ObjectCosts( 0, 15) );
	//listObject.push_back( ObjectCosts( 0, 16) );

	std::cout<<"\nBEFORE:"<<std::endl;
	for (unsigned i=0 ; i<listObject.size() ; i++) {
		std::cout<<"element "<<i<<", label: "<<listObject[i].id<<", value = "<<listObject[i].localCost<<std::endl;
	}

	unsigned nbRemovedObjects = 0;
	for (unsigned i=0, j=listObject.size()-1 ; i<=j ; i++) {
		if (listObject[i].id==0) { //this entry is invalid -> replace it when
			while ( (listObject[j].id==0) && (j>i) ) { //find the next valid entry that can
				listObject.pop_back(); j--;
			}
			if (j<=i) { listObject.pop_back(); break; } //if I reach the up-front, then remove that last invalid element and exit.
			else { //replace the invalid by the last, remove the last, and continue
				listObject[i].id = listObject[j].id;
				listObject[i].localCost = listObject[j].localCost;
				listObject.pop_back(); j--;
			}
		}
	}

	std::cout<<"\nAFTER:"<<std::endl;
	for (unsigned i=0 ; i<listObject.size() ; i++) {
		std::cout<<"element "<<i<<", label: "<<listObject[i].id<<", value = "<<listObject[i].localCost<<std::endl;
	}


}

//
//
#include "signal.h"
void do_sig_interupt(int dummy) {
	std::cout<<"pressed ctrl-c"<<std::endl;
}

void TestSignal() {
	signal(SIGINT, do_sig_interupt);

	std::set<double> testset;
	std::cout<<"start insertion loop"<<std::endl;
	for (unsigned i=0 ; i<10000000 ; i++) {
		testset.insert(i);
	}
	std::cout<<"finished naturally."<<std::endl;
}

int main(int argc, char** argv) {

	//Test1();
	//vnl_vector<double> bla(1); bla(0) = 1;
	//Test1(bla);

	//Test3();

	//Test4();

	//TestCopyVectors();

	//TestRemoveHolesInVector();

	TestSignal();

	return 1;
}
