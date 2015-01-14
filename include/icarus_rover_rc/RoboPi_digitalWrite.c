#include <stdio.h>
#include "RoboPiLib.h"

#define  LEFT_BUMPER   22
#define  RIGHT_BUMPER  23

#define  LEFT_LED      8
#define  RIGHT_LED     9

#define  PRESSED  0

int main(int argc, char *argv[]) {

   RoboPiInit("/dev/ttyAMA0",115200);
  
   pinMode(LEFT_BUMPER,  INPUT);
   pinMode(RIGHT_BUMPER, INPUT);

   pinMode(LEFT_LED,     OUTPUT);
   pinMode(RIGHT_LED,    OUTPUT);
      
   while (1) {
        
      digitalWrite(LEFT_LED,  (digitalRead(LEFT_BUMPER)==PRESSED));
      digitalWrite(RIGHT_LED, (digitalRead(RIGHT_BUMPER)==PRESSED));
                                    
   }
                                   
}