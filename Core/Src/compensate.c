/*
 * compensate.c
 *
 *  Created on: 5 May 2022
 *      Author: adamf
 */


#include "compensate.h"
#include "math.h"

float	TrianglePhaseOffset =   311.4118313;
float    TrianglePeriod      =   3.200669785;
float    TriangleAmplitude   =   0.210105524;

float    SinPhaseOffset  =       13.22661143;
float    SinAmplitude    =       0.233605166;
float    SinPeriod       =       23.19141246;

float    Sin2PhaseOffset =       117.1516361;
float    Sin2Amplitude   =       0.021459276;
float    Sin2Period      =       2.78447308;

int    comp_offset          =       29;

float C1;
float C2;
float C3;

int compensate(int x){
	C1 = 100*TriangleAmplitude*(2/M_PI)*asin(sin((M_PI/180)*(TrianglePhaseOffset+(3.6*x)/TrianglePeriod)));
	C2 = 100*SinAmplitude*sin((M_PI/180)*(SinPhaseOffset+(3.6*x)/SinPeriod));
	C3 = 100*Sin2Amplitude*sin((M_PI/180)*(Sin2PhaseOffset+(3.6*x)/Sin2Period));

	return x + C1 + C2 + C3 + comp_offset;
	//return x;
}
