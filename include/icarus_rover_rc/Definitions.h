#ifndef __DEFINITIONS_INCLUDED__   
#define __DEFINITIONS_INCLUDED__

//Diagnostic Definitions

//Field 1: System
#define ROVER 1
#define GROUND_STATION 5
#define REMOTE_CONTROL 7

//Field 2: Subsystem
#define ENTIRE_SYSTEM 0
#define ROBOT_CONTROLLER 1
#define MOTION_CONTROLLER 2
#define SONIC_CONTROLLER 3

//Field 3: Component
#define ENTIRE_SUBSYSTEM 0
#define DIAGNOSTIC_NODE 1
#define NAVIGATION_NODE 2
#define MOTION_CONTROLLER_NODE 3
#define SONIC_CONTROLLER_NODE 4
#define ROBOT_CONTROLLER_NODE 5
#define MAPPING_NODE 7
#define EVOLUTION_NODE 8
#define TARGETING_NODE 9


//Field 4: Diagnostic Type
#define NO_ERROR 0
#define ELECTRICAL 1
#define SOFTWARE 2
#define COMMUNICATIONS 3
#define SENSORS 4
#define ACTUATORS 5
#define DATA_STORAGE 6
#define REMOTE_CONTROL 7
#define GENERAL_ERROR 9

//Field 5: Level
//#define NO_ERROR 0  Already defined above, just leaving here for completeness.
#define DEBUG 1
#define INFORMATION 2
#define MINIMAL 3
#define CAUTION 4
#define SEVERE 5
#define FATAL 6

//Field 6: Diagnostic_Message
//#define NO_ERROR 0  Already defined above, just leaving here for completeness.
#define INITIALIZING 1
#define INITIALIZING_ERROR 2
#define DROPPING_PACKETS 4
#define MISSING_HEARTBEATS 5
#define DEVICE_NOT_AVAILABLE 6
//#define GENERAL_ERROR 9  Already defined above, just leaving here for completeness.

#define EXTENDED_SWITCH_PIN -1
#define RETRACTED_SWITCH_PIN -1
#define WINCH_MOTOR_PIN -1
#define FORWARD_TIME_LIMIT -1
#define REVERSE_TIME_LIMIT -1
#define WINCH_MOTOR_FORWARD -1
#define WINCH_MOTOR_REVERSE -1
#define WINCH_MOTOR_NEUTRAL -1
#define SONAR1_PIN -1
#define SONAR2_PIN -1
#define SONAR3_PIN -1
#define SONAR4_PIN -1
#define SONAR5_PIN -1
#define SONAR6_PIN -1
#define SONAR7_PIN -1
#define SONAR8_PIN -1

#define SONAR9_PIN -1
#define SONAR10_PIN -1
#define SONAR11_PIN -1
#define SONAR12_PIN -1
#define SONAR13_PIN -1
#define SONAR14_PIN -1
#define SONAR15_PIN -1
#define SONAR16_PIN -1

#define PROBE_MOVING_FORWARD 1
#define PROBE_MOVING_REVERSE 2
#define PROBE_EXTENDED 3
#define PROBE_RETRACTED 4

#define RECHARGE_START 1
#define RECHARGE_STOP 2
#define RECHARGE_WAIT 3

#define NO_ERROR 0
#define EXTENSION_ERROR 1
#define RETRACTION_ERROR 2

#define TARGET_NONE 1
#define TARGET_TRACKING 2


//Armed State Definitions
#define ARMED 1
#define DISARMED 0


//Gear Definitions
#define GEAR_FORWARD 1
#define GEAR_PARK    0
#define GEAR_REVERSE -1

//Naming Definitions
#define DIAGNOSTICS_NODE_NAME "Diagnostics"
#define DIAGNOSTICS_NODE_KEY "D"
#define ROBOTCONTROLLER_NODE_NAME "Robot"
#define ROBOTCONTROLLER_NODE_KEY "R"
#define MOTIONCONTROLLER_NODE_NAME "Motion"
#define MOTIONCONTROLLER_NODE_KEY "M"
#define SONICCONTROLLER_NODE_NAME "Sonic"
#define SONICCONTROLLER_NODE_KEY "S"
#define NAVIGATIONCONTROLLER_NODE_NAME "Navigation"
#define NAVIGATIONCONTROLLER_NODE_KEY "N"
#define MAPPING_NODE_NAME "maPping"
#define MAPPING_NODE_KEY "P"
#define LEARNING_NODE_NAME "Learning"
#define LEARNING_NODE_KEY "L"
#endif



