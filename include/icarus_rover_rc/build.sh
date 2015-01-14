#!/bin/sh
#gcc -o resetRoboPi  RoboPi_reset.c -lwiringPi
g++ -o analogRead   RoboPi_analogRead.c RoboPiLib.o
#g++ -o analogWrite  RoboPi_analogWrite.c RoboPiLib.o
#g++ -o digitalRead  RoboPi_digitalRead.c RoboPiLib.o
#g++ -o digitalWrite RoboPi_digitalWrite.c RoboPiLib.o
#g++ -o servoWrite   RoboPi_servoWrite.c RoboPiLib.o
#g++ -o readDistance RoboPi_readDistance.c RoboPiLib.o
