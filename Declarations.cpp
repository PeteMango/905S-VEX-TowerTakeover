#include "main.h"
#include "Declarations.hpp"
#define G06 MOTOR_GEARSET_06
#define G18 MOTOR_GEARSET_18
#define G36 MOTOR_GEARSET_36
#define t true
#define f false

using namespace pros;

Controller M (CONTROLLER_MASTER);
Motor LB (13,G06,t), RB (16,G06,f), RF (5,G06,f), LF (17,G06,t), A (11,G36,f), T (10,G36,f), L (9,G18,f), R (1,G18,t);
ADIPotentiometer PA (8), PT (7);
ADIEncoder LT(1, 2, true), RT(3, 4, true);
ADIAnalogIn LS(6);  
