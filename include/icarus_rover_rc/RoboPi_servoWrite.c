#include <stdio.h>
#include "RoboPiLib.h"

#define  LEFT_SERVO     0
#define  RIGHT_SERVO    1

#define SERVO_MIN	500
#define SERVO_MAX	2500
#define SERVO_REV	(SERVO_MIN+SERVO_MAX)

int main(int argc, char *argv[]) {

   int i;

   RoboPiInit("/dev/ttyAMA0",115200);
  
   pinMode(LEFT_SERVO,  SERVO);
   pinMode(RIGHT_SERVO, SERVO);

   while (1) {

      for(i=SERVO_MIN;i<=SERVO_MAX;i+=100) {

         servoWrite(LEFT_SERVO, i);
         servoWrite(RIGHT_SERVO, SERVO_REV-i);
         
         printf("LEFT  SERVO = %d\n", servoRead(LEFT_SERVO));
         printf("RIGHT SERVO = %d\n", servoRead(RIGHT_SERVO));
         sleep(1);

      }
              
   }
                                   
}