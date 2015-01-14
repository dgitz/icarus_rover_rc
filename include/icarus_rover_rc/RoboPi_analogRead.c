#include <stdio.h>
#include "RoboPiLib.h"

#define  POT  0

int main(int argc, char *argv[]) {

   RoboPiInit("/dev/ttyAMA0",115200);
  
   while (1) {
        
      printf("Potentiometer value is %d\n", analogRead(POT));                                
      sleep(1); // only check once per second
                                    
   }
                                   
}