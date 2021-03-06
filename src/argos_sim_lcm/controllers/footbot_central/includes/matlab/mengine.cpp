/*
 * mEngine.cpp
 *
 *  Created on: 13/10/2014
 *      Author: roberto
 */

#include "mengine.h"

#include <stdio.h>
#include <string.h>
#include <iostream>
#include <math.h>

using namespace std;

MEngine::MEngine() {
	// TODO Auto-generated constructor stub

}

vector<double> MEngine::runEgine(double un_positions[][2], double an_positions[][2], double h_pred[][2], double horizont, double e_weight, double time, double debug ) {

	mxArray *orig_UN = NULL, *orig_AN, *result;

	mxArray * H, *edge_w, *t, *dbg;

	std::vector<double> AN;


	Engine *ep;
	if (!(ep = engOpen("\0"))) {
		fprintf(stderr, "\nCan't start MATLAB engine\n");
	} else {
		fprintf(stderr, "\n MATLAB engine OK!\n");
	}

	printf("UN (%f,%f)\n",un_positions[0][0],un_positions[0][1]);
	printf("AN (%f,%f)\n",an_positions[0][0],an_positions[0][1]);

	// UN locations
	orig_UN = mxCreateDoubleMatrix(2, 2, mxREAL);
	mxSetData(orig_UN,un_positions);
	//memcpy(mxGetPr(orig_UN), un_positions, sizeof(un_positions));

	engPutVariable(ep, "orig_UN", orig_UN);

	//AN locations
	orig_AN = mxCreateDoubleMatrix(1, 2, mxREAL);
	mxSetData(orig_AN,an_positions);
	//memcpy(mxGetPr(orig_AN), an_positions, sizeof(an_positions));

	engPutVariable(ep, "orig_AN", orig_AN);

	// UN locations prediction
	engPutVariable(ep, "h_pred", orig_UN);

	// H
	H = mxCreateDoubleScalar(horizont);
	engPutVariable(ep, "H", H);

	// edge_w
	edge_w = mxCreateDoubleScalar(e_weight);
	engPutVariable(ep, "edge_w", edge_w);

	// t
	t = mxCreateDoubleScalar(time);
	engPutVariable(ep, "t", t);

	// dbg
	dbg = mxCreateDoubleScalar(debug);
	engPutVariable(ep, "dbg", dbg);

	engEvalString(ep,
			"addpath('/home/roberto/RMAGAN/Dropbox/THESIS/ESTANCIA/MATLAB/CODE/')");
	engEvalString(ep,
			"addpath('/home/roberto/RMAGAN/Dropbox/THESIS/ESTANCIA/MATLAB/CODE/gaimc/')");
	engEvalString(ep,
			"[AN_t1, APs] = entry_point(orig_UN,orig_AN, h_pred,H,edge_w,t,dbg)");

	//engGetVariable(ep, "output");

	if ((result = engGetVariable(ep, "AN_t1")) == NULL)
		printf("Oops! You didn't create a variable output.\n\n");
	else {
		//printf();
	}

	AN.assign(mxGetPr(result), mxGetPr(result)+mxGetNumberOfElements(result));


//	printf("AN %i",AN.size());
//	cout << AN.size();

	for (unsigned i=0;i<AN.size();i++){
		printf("AN %d, %f\n",i,AN.at(i));
	}


	mxDestroyArray(result);
	engClose(ep);

	return AN;

}

MEngine::~MEngine() {
	// TODO Auto-generated destructor stub
}

