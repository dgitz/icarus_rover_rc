#include <wiringPi.h>

#define ROBOPI_RESET 17

int main (void) {

    wiringPiSetupGpio() ;

    pinMode (ROBOPI_RESET, OUTPUT) ;

    digitalWrite(ROBOPI_RESET, LOW); 
    digitalWrite(ROBOPI_RESET, HIGH); 
    delay(500);
    
    return 0 ;
}