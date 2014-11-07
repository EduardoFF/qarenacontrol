/*
 * mEngine.h
 *
 *  Created on: 13/10/2014
 *      Author: roberto
 */

#include "engine.h"
#include <vector>

#ifndef MENGINE_H_
#define MENGINE_H_

using namespace std;

class mEngine {
public:
	mEngine();

	std::vector<double> runEgine(double un_positions[][2], double an_positions[][2], double h_pred[][2], double horizont, double e_weight, double time, double debug );

	virtual ~mEngine();
};

#endif /* MENGINE_H_ */
