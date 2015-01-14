//**********************************************************************************
// RoboPiLib based Diagnostic program for RoboPi
//
// Copyright 2014 William Henning
//
// http://Mikronauts.com
//
//**********************************************************************************

#include <stdio.h>
#include "RoboPiLib.h"

#define HC_SR04 16

char buffer[256];

void main(int argc, char **argv) {	

	RoboPiInit("/dev/ttyAMA0", 115200);

	getProductID(buffer);

	printf("%s\n",buffer);
	fflush(stdout);

	while (1) {
		printf("Distance = %dmm\n", readDistance(HC_SR04));
	}

	RoboPiExit();

}


